#include "usb_task.hpp"
#include "shared.hpp"
#include "Tasks/MQTT/mqtt_task.hpp"
#include "Tasks/Stepper/stepper_task.hpp"
#include "Tasks/Stepper/tracker_state.hpp"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

// -- log_print -----------------------------------------------------------------
// Sole caller of printf() in the whole project.  All other tasks call
// log_print() to keep stdio off the FreeRTOS SMP shared stack.
// Non-blocking: message silently dropped if the queue is full.
void log_print( const char* fmt, ... )
{
    LogMessage msg;
    va_list args;
    va_start( args, fmt );
    vsnprintf( msg.buf, sizeof( msg.buf ), fmt, args );
    va_end( args );
    xQueueSend( g_log_queue, &msg, 0 );
}

// -- Console output flag -------------------------------------------------------
// Default true for ground station — all log_print() output is visible without
// needing to type "log on".  Use "log off" to suppress if needed.
static bool s_log_enabled = true;

static constexpr char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t base64_encode( const uint8_t* in,
                             size_t len,
                             char* out,
                             size_t out_len )
{
    const size_t needed = ( ( len + 2u ) / 3u ) * 4u;
    if ( out_len < needed + 1u ) return 0;

    size_t i = 0;
    size_t o = 0;
    while ( i < len ) {
        const uint32_t a = in[i++];
        const uint32_t b = ( i < len ) ? in[i++] : 0u;
        const uint32_t c = ( i < len ) ? in[i++] : 0u;
        const uint32_t triple = ( a << 16 ) | ( b << 8 ) | c;

        out[o++] = kBase64Alphabet[( triple >> 18 ) & 0x3Fu];
        out[o++] = kBase64Alphabet[( triple >> 12 ) & 0x3Fu];
        out[o++] = ( i - 1u <= len ) ? kBase64Alphabet[( triple >> 6 ) & 0x3Fu] : '=';
        out[o++] = ( i <= len ) ? kBase64Alphabet[triple & 0x3Fu] : '=';
    }

    const size_t rem = len % 3u;
    if ( rem != 0u ) {
        out[o - 1u] = '=';
        if ( rem == 1u ) out[o - 2u] = '=';
    }
    out[o] = '\0';
    return o;
}

static void print_usb_mqtt_frame( const MqttMessage& msg )
{
    static char b64[ ( ( sizeof(msg.payload) + 2u ) / 3u ) * 4u + 1u ];
    const size_t n = base64_encode( msg.payload, msg.payload_len, b64, sizeof(b64) );
    if ( n == 0u ) return;

    printf( "MQTTB64,%s,%u,%s\n",
            msg.topic,
            (unsigned)msg.payload_len,
            b64 );
}

// -- Commands ------------------------------------------------------------------

static void cmd_help()
{
    printf(
        "Commands:\n"
        "  help           show this list\n"
        "  status         print heap, queue depths, tracker state\n"
        "  log   [on|off] toggle/set task log message output (default: off)\n"
        "  ALT,<m>[,<boot_ms>,<rssi>,<snr>]  update rocket baro altitude MSL\n"
        "  alt <m>        update rocket baro altitude MSL\n"
        "  base <m> [fs]  set base altitude MSL and full-scale AGL metres\n"
        "  arm|disarm     arm/disarm tracker\n"
        "  auto|manual|stop  set tracker mode\n"
        "  elcal [deg]    mark elevation calibrated at deg (default 0)\n"
        "  el <deg> [spd] command zenith/elevation axis in manual mode\n"
        "  clear          clear terminal screen\n"
    );
}

static void cmd_status()
{
    printf( "heap free  : %u B (min %u B)\n",
            ( unsigned ) xPortGetFreeHeapSize(),
            ( unsigned ) xPortGetMinimumEverFreeHeapSize() );
    printf( "wifi       : %s\n",
            ( xEventGroupGetBits( g_net_events ) & EVT_WIFI_CONNECTED ) ? "up" : "down" );
    printf( "mqtt       : %s\n",
            mqtt_is_connected() ? "up" : "down" );
    printf( "log_q      : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_log_queue  ), ( unsigned ) LOG_QUEUE_DEPTH );
    printf( "mqtt_q     : %u / %u\n",
            ( unsigned ) uxQueueMessagesWaiting( g_mqtt_queue ), ( unsigned ) MQTT_QUEUE_DEPTH );
    printf( "tracker    : %s armed=%u el_cal=%u\n",
            tracker_mode_name( tracker_mode() ),
            tracker_is_armed() ? 1u : 0u,
            tracker_elevation_calibrated() ? 1u : 0u );
    const TrackerConfig cfg = tracker_config_snapshot();
    printf( "altitude   : base=%.1f m full_scale=%.1f m altitude_only=%u\n",
            (double)cfg.base_altitude_m,
            (double)cfg.altitude_full_scale_m,
            cfg.altitude_only_tracking ? 1u : 0u );
    printf( "log output : %s\n", s_log_enabled ? "on" : "off" );
}

static bool parse_float_arg( const char* text, float* out )
{
    if ( !text || !out ) return false;
    char* end = nullptr;
    const float value = strtof( text, &end );
    if ( end == text ) return false;
    while ( *end != '\0' ) {
        if ( !isspace( (unsigned char)*end ) && *end != ',' ) return false;
        end++;
    }
    *out = value;
    return true;
}

static void cmd_altitude( float alt_m,
                          uint32_t boot_ms = 0,
                          float rssi = 0.0f,
                          float snr = 0.0f )
{
    if ( !g_rocket_altitude_q ) {
        printf( "altitude queue not ready\n" );
        return;
    }

    AltitudeMsg msg = {};
    msg.alt_m = alt_m;
    msg.source_boot_ms = boot_ms;
    msg.timestamp_us = time_us_64();
    msg.rssi = rssi;
    msg.snr = snr;
    msg.valid = true;
    xQueueOverwrite( g_rocket_altitude_q, &msg );
    printf( "altitude: %.2f m MSL\n", (double)alt_m );
}

static bool dispatch_alt_line( const char* line )
{
    if ( strncmp( line, "ALT,", 4 ) != 0 ) return false;

    float alt_m = 0.0f;
    unsigned long boot_ms = 0;
    float rssi = 0.0f;
    float snr = 0.0f;
    const int count = sscanf( line + 4, "%f,%lu,%f,%f",
                              &alt_m, &boot_ms, &rssi, &snr );
    if ( count < 1 ) {
        printf( "bad ALT line\n" );
        return true;
    }

    cmd_altitude( alt_m, (uint32_t)boot_ms, rssi, snr );
    return true;
}

static void cmd_base( const char* args )
{
    float base_m = 880.0f;
    float full_scale_m = tracker_config_snapshot().altitude_full_scale_m;
    int consumed = 0;
    if ( sscanf( args, "%f %f%n", &base_m, &full_scale_m, &consumed ) < 1 ) {
        printf( "usage: base <m> [full_scale_m]\n" );
        return;
    }

    tracker_set_altitude_profile( base_m, full_scale_m );
    const TrackerConfig cfg = tracker_config_snapshot();
    printf( "altitude profile: base=%.1f m full_scale=%.1f m\n",
            (double)cfg.base_altitude_m,
            (double)cfg.altitude_full_scale_m );
}

static void cmd_el_cal( const char* args )
{
    float ref = 0.0f;
    if ( args && *args ) parse_float_arg( args, &ref );
    if ( !g_stepper_zen_cal_q ) {
        printf( "elevation calibration queue not ready\n" );
        return;
    }

    StepperCalibrationCmd cal = {};
    cal.set_current_angle = true;
    cal.current_angle_deg = ref;
    xQueueOverwrite( g_stepper_zen_cal_q, &cal );
    printf( "elevation calibrated at %.2f deg\n", (double)ref );
}

static void cmd_el( const char* args )
{
    float target = 0.0f;
    float speed = tracker_config_snapshot().default_speed_dps;
    if ( sscanf( args, "%f %f", &target, &speed ) < 1 ) {
        printf( "usage: el <deg> [speed_dps]\n" );
        return;
    }
    if ( !g_stepper_zen_cmd_q ) {
        printf( "elevation command queue not ready\n" );
        return;
    }

    tracker_set_armed( true );
    tracker_set_mode( TrackerMode::Manual );
    tracker_set_manual_target( false, target, false );
    StepperCmd cmd = {};
    cmd.target_angle_deg = target;
    cmd.speed_dps = speed;
    cmd.stop = false;
    xQueueOverwrite( g_stepper_zen_cmd_q, &cmd );
    printf( "elevation command: %.2f deg speed=%.2f dps\n",
            (double)target, (double)speed );
}

// -- Command dispatch ----------------------------------------------------------

static void dispatch( const char* line, size_t len )
{
    if ( len == 0 ) return;

    if ( dispatch_alt_line( line ) ) return;

    if ( strncmp( line, "help",   4 ) == 0 ) { cmd_help();   return; }
    if ( strncmp( line, "status", 6 ) == 0 ) { cmd_status(); return; }
    if ( strncmp( line, "arm",    3 ) == 0 ) { tracker_set_armed( true );  printf( "armed\n" ); return; }
    if ( strncmp( line, "disarm", 6 ) == 0 ) { tracker_set_armed( false ); printf( "disarmed\n" ); return; }
    if ( strncmp( line, "auto",   4 ) == 0 ) { tracker_set_mode( TrackerMode::Auto ); printf( "mode: auto\n" ); return; }
    if ( strncmp( line, "manual", 6 ) == 0 ) { tracker_set_mode( TrackerMode::Manual ); printf( "mode: manual\n" ); return; }
    if ( strncmp( line, "stop",   4 ) == 0 ) {
        tracker_set_mode( TrackerMode::Stop );
        StepperCmd stop = {};
        stop.stop = true;
        if ( g_stepper_az_cmd_q ) xQueueOverwrite( g_stepper_az_cmd_q, &stop );
        if ( g_stepper_zen_cmd_q ) xQueueOverwrite( g_stepper_zen_cmd_q, &stop );
        printf( "stopped\n" );
        return;
    }

    if ( strncmp( line, "clear",  5 ) == 0 ) {
        printf( "\x1b[2J\x1b[H" );
        stdio_flush();
        return;
    }

    if ( strncmp( line, "log", 3 ) == 0 ) {
        const char* arg = line + 3;
        while ( *arg == ' ' ) arg++;
        if      ( strncmp( arg, "on",  2 ) == 0 ) s_log_enabled = true;
        else if ( strncmp( arg, "off", 3 ) == 0 ) s_log_enabled = false;
        else                                       s_log_enabled = !s_log_enabled;
        printf( "log output: %s\n", s_log_enabled ? "on" : "off" );
        return;
    }

    if ( strncmp( line, "alt ", 4 ) == 0 ) {
        float alt_m = 0.0f;
        if ( parse_float_arg( line + 4, &alt_m ) ) cmd_altitude( alt_m );
        else printf( "usage: alt <m>\n" );
        return;
    }

    if ( strncmp( line, "base ", 5 ) == 0 ) { cmd_base( line + 5 ); return; }
    if ( strncmp( line, "elcal", 5 ) == 0 ) {
        const char* arg = line + 5;
        while ( *arg == ' ' ) arg++;
        cmd_el_cal( arg );
        return;
    }
    if ( strncmp( line, "el ", 3 ) == 0 ) { cmd_el( line + 3 ); return; }

    printf( "unknown command: '%s'  (type 'help')\n", line );
}

// -- USB / serial console task -------------------------------------------------
// Pinned to core 0: TinyUSB IRQ fires on core 0; printf() must live there too.
//
// Loop:
//   1. Drain g_mqtt_queue — mirror protobuf MQTT payloads over USB CDC.
//   2. Drain g_log_queue — print only when s_log_enabled.
//   3. Non-blocking char read — echo, backspace, dispatch on CR/LF.
//      Prints "# " as a prompt after every dispatched line.
static void usb_task( void* )
{
    char       line[ 128 ] = { 0 };
    size_t     line_len    = 0;
    bool       was_connected = false;

    for ( ;; ) {
        if ( !stdio_usb_connected() ) {
            was_connected = false;
            vTaskDelay( pdMS_TO_TICKS( 100 ) );
            continue;
        }

        if ( !was_connected ) {
            was_connected = true;
            printf( "\r\n# " );
            stdio_flush();
        }

        // -- 1. Drain MQTT queue -------------------------------------------
        {
            MqttMessage msg;
            bool flushed = false;
            while ( xQueueReceive( g_mqtt_queue, &msg, 0 ) == pdTRUE ) {
                print_usb_mqtt_frame( msg );
                flushed = true;
            }
            if ( flushed ) stdio_flush();
        }

        // -- 2. Drain log queue --------------------------------------------
        {
            LogMessage msg;
            if ( xQueueReceive( g_log_queue, &msg, pdMS_TO_TICKS( 10 ) ) == pdTRUE ) {
                bool flushed = false;
                if ( s_log_enabled ) { printf( "%s", msg.buf ); flushed = true; }
                while ( xQueueReceive( g_log_queue, &msg, 0 ) == pdTRUE )
                    if ( s_log_enabled ) { printf( "%s", msg.buf ); flushed = true; }
                if ( flushed ) stdio_flush();
            }
        }

        // -- 3. Read characters from USB CDC ------------------------------
        {
            int c;
            while ( ( c = getchar_timeout_us( 0 ) ) != PICO_ERROR_TIMEOUT && c < 256 ) {
                if ( c == '\r' || c == '\n' ) {
                    printf( "\r\n" );
                    line[ line_len ] = '\0';
                    dispatch( line, line_len );
                    line_len  = 0;
                    line[ 0 ] = '\0';
                    printf( "# " );
                    stdio_flush();

                } else if ( ( c == '\b' || c == 127 ) && line_len > 0 ) {
                    line[ --line_len ] = '\0';
                    printf( "\b \b" );
                    stdio_flush();

                } else if ( c >= 0x20 && line_len < sizeof( line ) - 1 ) {
                    line[ line_len++ ] = ( char ) c;
                    stdio_putchar( c );
                    stdio_flush();
                }
            }
        }
    }
}

static StaticTask_t s_usb_tcb;
static StackType_t  s_usb_stack[ 2048 ];

void usb_task_init()
{
    // TinyUSB IRQ fires on core 0; printf must live on the same core.
    TaskHandle_t h = task_create( usb_task, "usb", 2048, nullptr,
                                   tskIDLE_PRIORITY + 1,
                                   s_usb_stack, &s_usb_tcb );
    vTaskCoreAffinitySet( h, 0x01 );
}
