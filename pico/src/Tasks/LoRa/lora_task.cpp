#include "lora_task.hpp"
#include "shared.hpp"
#include "Tasks/MQTT/mqtt_task.hpp"
#include "Proto/mqtt_proto.hpp"

#include "sx1276/SX1276.hpp"
#include "SIGMA.hpp"
#include "SIGMA2/SIGMA2.hpp"
#include "SIGMA2/packets/gps_packets.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"

#include <math.h>
#include <string.h>

static const radio::sx1276::Config s_lora0_cfg {
    .freq_mhz  = LoRa0Cfg::FREQ_MHZ,
    .bw_khz    = LoRa0Cfg::BW_KHZ,
    .sf        = LoRa0Cfg::SF,
    .cr        = LoRa0Cfg::CR,
    .sync_word = LoRa0Cfg::SYNC_WORD,
    .tx_dbm    = LoRa0Cfg::TX_POWER,
    .preamble  = LoRa0Cfg::PREAMBLE,
};

static radio::sx1276::SX1276 s_radio(
    spi0,
    Pins::LORA0_SCK,
    Pins::LORA0_MOSI,
    Pins::LORA0_MISO,
    Pins::LORA0_NSS,
    Pins::LORA0_DIO0,
    Pins::LORA0_RST,
    s_lora0_cfg );

static const char* state_name( FlightState s )
{
    switch ( s ) {
        case FlightState::GROUND_IDLE:    return "GROUND_IDLE";
        case FlightState::ARMED:          return "ARMED";
        case FlightState::POWERED_ASCENT: return "POWERED_ASCENT";
        case FlightState::COAST_ASCENT:   return "COAST_ASCENT";
        case FlightState::APOGEE:         return "APOGEE";
        case FlightState::DESCENT_DROGUE: return "DESCENT_DROGUE";
        case FlightState::DESCENT_MAIN:   return "DESCENT_MAIN";
        case FlightState::LANDED:         return "LANDED";
        case FlightState::FAULT:          return "FAULT";
        default:                          return "UNKNOWN";
    }
}

static groundstation_FlightState map_sigma2_state( SIGMA2::FLIGHT_STATE state )
{
    switch ( state ) {
        case SIGMA2::FLIGHT_STATE::PAD:
        case SIGMA2::FLIGHT_STATE::UNKOWN:
            return groundstation_FlightState_FLIGHT_STATE_GROUND_IDLE;
        case SIGMA2::FLIGHT_STATE::BOOST:
            return groundstation_FlightState_FLIGHT_STATE_POWERED_ASCENT;
        case SIGMA2::FLIGHT_STATE::COAST:
            return groundstation_FlightState_FLIGHT_STATE_COAST_ASCENT;
        case SIGMA2::FLIGHT_STATE::APOGEE:
            return groundstation_FlightState_FLIGHT_STATE_APOGEE;
        case SIGMA2::FLIGHT_STATE::DESCENT:
            return groundstation_FlightState_FLIGHT_STATE_DESCENT_DROGUE;
        case SIGMA2::FLIGHT_STATE::LANDED:
            return groundstation_FlightState_FLIGHT_STATE_LANDED;
        default:
            return groundstation_FlightState_FLIGHT_STATE_FAULT;
    }
}

static float speed_from_cms( int16_t vn_cms, int16_t ve_cms, int16_t vd_cms )
{
    const float vn = static_cast<float>( vn_cms ) * 0.01f;
    const float ve = static_cast<float>( ve_cms ) * 0.01f;
    const float vd = static_cast<float>( vd_cms ) * 0.01f;
    return sqrtf( vn * vn + ve * ve + vd * vd );
}

static float cms_to_mps( int16_t cms )
{
    return static_cast<float>( cms ) * 0.01f;
}

static float cm_to_m( uint16_t cm )
{
    return static_cast<float>( cm ) * 0.01f;
}

static float q15_to_float( int16_t q )
{
    return static_cast<float>( q ) / 32768.0f;
}

static void publish_lora_sample( const groundstation_RocketLoRaSample& pb )
{
    MqttMessage m = {};
    if ( mqtt_encode_proto( m, "rocket/lora0",
                            groundstation_RocketLoRaSample_fields, &pb ) )
        xQueueSend( g_mqtt_queue, &m, 0 );
}

static void update_rocket_location( double lat,
                                    double lon,
                                    double alt_m,
                                    uint32_t boot_ms,
                                    float vel_n_mps = 0.0f,
                                    float vel_e_mps = 0.0f,
                                    float vel_d_mps = 0.0f,
                                    bool have_velocity = false,
                                    float h_acc_m = 0.0f,
                                    float v_acc_m = 0.0f,
                                    float s_acc_mps = 0.0f,
                                    bool have_accuracy = false )
{
    LocationMsg loc = {};
    loc.lat = lat;
    loc.lon = lon;
    loc.alt_m = alt_m;
    loc.timestamp_us = time_us_64();
    loc.vel_n_mps = vel_n_mps;
    loc.vel_e_mps = vel_e_mps;
    loc.vel_d_mps = vel_d_mps;
    loc.h_acc_m = h_acc_m;
    loc.v_acc_m = v_acc_m;
    loc.s_acc_mps = s_acc_mps;
    loc.source_boot_ms = boot_ms;
    loc.have_velocity = have_velocity;
    loc.have_accuracy = have_accuracy;
    xQueueOverwrite( g_rocket_location_q, &loc );
}

static bool handle_sigma2_gps_nav( const radio::Packet& pkt,
                                   const SIGMA2::DecodedFrame& frame )
{
    SIGMA2::TRANSMIT_PACKETS::GPSNav gps = {};
    if ( !SIGMA2::deserialize_packet_payload( frame, gps ) ) return false;

    const double lat = static_cast<double>( gps.lat_deg_e7 ) * 1.0e-7;
    const double lon = static_cast<double>( gps.lon_deg_e7 ) * 1.0e-7;
    const float alt_m = static_cast<float>( gps.alt_msl_cm ) * 0.01f;
    const float vel_n = cms_to_mps( gps.vel_n_cms );
    const float vel_e = cms_to_mps( gps.vel_e_cms );
    const float vel_d = cms_to_mps( gps.vel_d_cms );

    groundstation_RocketLoRaSample pb = groundstation_RocketLoRaSample_init_zero;
    pb.has_boot_ms = true;     pb.boot_ms = frame.header.timestamp_ms;
    pb.has_sats = true;        pb.sats = gps.num_sv;
    pb.has_flags = true;       pb.flags = gps.flags;
    pb.has_lat = true;         pb.lat = lat;
    pb.has_lon = true;         pb.lon = lon;
    pb.has_alt_gps_m = true;   pb.alt_gps_m = alt_m;
    pb.has_speed_ms = true;    pb.speed_ms = speed_from_cms( gps.vel_n_cms,
                                                             gps.vel_e_cms,
                                                             gps.vel_d_cms );
    pb.has_rssi = true;        pb.rssi = pkt.rssi;
    pb.has_snr = true;         pb.snr = pkt.snr;

    publish_lora_sample( pb );

    if ( ( gps.flags & SIGMA2::DATA_VALID_FLAG::GPS_VALID ) || gps.fix_type >= 2u ) {
        update_rocket_location( lat, lon, static_cast<double>( alt_m ),
                                frame.header.timestamp_ms,
                                vel_n, vel_e, vel_d, true,
                                cm_to_m( gps.h_acc_cm ),
                                cm_to_m( gps.v_acc_cm ),
                                cm_to_m( gps.s_acc_cms ),
                                true );
    }
    return true;
}

static bool handle_sigma2_nav_state( const radio::Packet& pkt,
                                     const SIGMA2::DecodedFrame& frame )
{
    SIGMA2::TRANSMIT_PACKETS::NavState nav = {};
    if ( !SIGMA2::deserialize_packet_payload( frame, nav ) ) return false;

    const double lat = static_cast<double>( nav.lat_deg_e7 ) * 1.0e-7;
    const double lon = static_cast<double>( nav.lon_deg_e7 ) * 1.0e-7;
    const float alt_m = static_cast<float>( nav.alt_fused_cm ) * 0.01f;
    const float vel_n = cms_to_mps( nav.vel_n_cms );
    const float vel_e = cms_to_mps( nav.vel_e_cms );
    const float vel_d = cms_to_mps( nav.vel_d_cms );

    groundstation_RocketLoRaSample pb = groundstation_RocketLoRaSample_init_zero;
    pb.has_boot_ms = true;      pb.boot_ms = frame.header.timestamp_ms;
    pb.has_state = true;        pb.state = map_sigma2_state( nav.state );
    pb.has_flags = true;        pb.flags = nav.flags;
    pb.has_lat = true;          pb.lat = lat;
    pb.has_lon = true;          pb.lon = lon;
    pb.has_alt_gps_m = true;    pb.alt_gps_m = alt_m;
    pb.has_alt_baro_m = true;   pb.alt_baro_m = alt_m;
    pb.has_speed_ms = true;     pb.speed_ms = speed_from_cms( nav.vel_n_cms,
                                                              nav.vel_e_cms,
                                                              nav.vel_d_cms );
    pb.q_count = 4;
    pb.q[0] = q15_to_float( nav.q[0] );
    pb.q[1] = q15_to_float( nav.q[1] );
    pb.q[2] = q15_to_float( nav.q[2] );
    pb.q[3] = q15_to_float( nav.q[3] );
    pb.has_rssi = true;         pb.rssi = pkt.rssi;
    pb.has_snr = true;          pb.snr = pkt.snr;

    publish_lora_sample( pb );

    const bool has_gps_pos =
        ( nav.flags & SIGMA2::DATA_VALID_FLAG::GPS_VALID ) ||
        ( nav.nav_source & SIGMA2::TRANSMIT_PACKETS::NavSource::GPS_POS );
    if ( has_gps_pos ) {
        update_rocket_location( lat, lon, static_cast<double>( alt_m ),
                                frame.header.timestamp_ms,
                                vel_n, vel_e, vel_d, true );
    }
    return true;
}

static bool handle_sigma2_packet( const radio::Packet& pkt )
{
    SIGMA2::DecodedFrame frame = {};
    const SIGMA2::DecodeStatus status =
        SIGMA2::deserialize_frame( pkt.data, pkt.len, frame );
    if ( status != SIGMA2::DecodeStatus::Ok ) return false;

    switch ( frame.header.type ) {
        case SIGMA2::PacketType::GPS:
            return handle_sigma2_gps_nav( pkt, frame );
        case SIGMA2::PacketType::NAV_STATE:
            return handle_sigma2_nav_state( pkt, frame );
        default:
            log_print( "[lora0] SIGMA2 frame type %u ignored (%u B RSSI %.1f SNR %.1f)\n",
                       static_cast<unsigned>( frame.header.type ),
                       static_cast<unsigned>( pkt.len ),
                       static_cast<double>( pkt.rssi ),
                       static_cast<double>( pkt.snr ) );
            return true;
    }
}

static bool handle_legacy_sigma_packet( const radio::Packet& pkt )
{
    SigmaLoRaData d;
    if ( !SigmaLoRaData::deserialize( pkt.data, pkt.len, d ) ) return false;

    groundstation_RocketLoRaSample pb = groundstation_RocketLoRaSample_init_zero;
    pb.has_boot_ms = true;    pb.boot_ms = d.boot_ms;
    pb.has_state = true;      pb.state = (groundstation_FlightState)d.state;
    pb.has_sats = true;       pb.sats = d.satellites;
    pb.has_flags = true;      pb.flags = d.flags;
    pb.has_lat = true;        pb.lat = d.lat;
    pb.has_lon = true;        pb.lon = d.lon;
    pb.has_alt_gps_m = true;  pb.alt_gps_m = d.alt_gps_m;
    pb.has_alt_baro_m = true; pb.alt_baro_m = d.alt_baro_m;
    pb.has_speed_ms = true;   pb.speed_ms = d.speed_ms;
    pb.q_count = 4;
    pb.q[0] = d.q[0]; pb.q[1] = d.q[1];
    pb.q[2] = d.q[2]; pb.q[3] = d.q[3];
    pb.has_rssi = true;       pb.rssi = pkt.rssi;
    pb.has_snr = true;        pb.snr = pkt.snr;

    publish_lora_sample( pb );

    if ( d.flags & SIGMA_FLAG_GPS_VALID ) {
        update_rocket_location( d.lat, d.lon, static_cast<double>( d.alt_gps_m ),
                                d.boot_ms );
    }
    return true;
}

static uint8_t diag_spi_read_reg( bool miso_pullup )
{
    spi_init( spi0, 1'000'000u );
    spi_set_format( spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );
    gpio_set_function( Pins::LORA0_SCK,  GPIO_FUNC_SPI );
    gpio_set_function( Pins::LORA0_MOSI, GPIO_FUNC_SPI );
    gpio_set_function( Pins::LORA0_MISO, GPIO_FUNC_SPI );

    if ( miso_pullup ) gpio_pull_up( Pins::LORA0_MISO );
    else               gpio_disable_pulls( Pins::LORA0_MISO );

    gpio_init( Pins::LORA0_NSS );
    gpio_set_dir( Pins::LORA0_NSS, GPIO_OUT );
    gpio_put( Pins::LORA0_NSS, 1 );

    gpio_init( Pins::LORA0_RST );
    gpio_set_dir( Pins::LORA0_RST, GPIO_OUT );
    gpio_put( Pins::LORA0_RST, 0 );
    sleep_ms( 10 );
    gpio_put( Pins::LORA0_RST, 1 );
    sleep_ms( 10 );

    gpio_put( Pins::LORA0_NSS, 0 );
    const uint8_t addr = 0x42u;
    uint8_t val = 0xAAu;
    spi_write_blocking( spi0, &addr, 1 );
    spi_read_blocking( spi0, 0x00u, &val, 1 );
    gpio_put( Pins::LORA0_NSS, 1 );

    gpio_disable_pulls( Pins::LORA0_MISO );
    return val;
}

static void diag_spi()
{
    const uint8_t no_pull = diag_spi_read_reg( false );
    const uint8_t pullup  = diag_spi_read_reg( true );

    log_print( "[lora0] SX1276 SPI diag: RegVersion=0x%02X (pulled-up=0x%02X)\n",
               no_pull, pullup );

    if ( no_pull == 0x12 ) {
        log_print( "[lora0] diag: RFM9x/SX1276 OK\n" );
    } else if ( no_pull == 0x00 && pullup == 0xFF ) {
        log_print( "[lora0] diag: MISO not connected - check GPIO%u\n", Pins::LORA0_MISO );
    } else if ( no_pull == 0x00 && pullup == 0x00 ) {
        log_print( "[lora0] diag: MISO shorted to GND\n" );
    } else if ( no_pull == 0xFF && pullup == 0xFF ) {
        log_print( "[lora0] diag: chip in reset / no power - check GPIO%u and 3.3V\n",
                   Pins::LORA0_RST );
    } else {
        log_print( "[lora0] diag: unexpected - NSS=%u SCK=%u MOSI=%u MISO=%u RST=%u\n",
                   Pins::LORA0_NSS, Pins::LORA0_SCK,
                   Pins::LORA0_MOSI, Pins::LORA0_MISO, Pins::LORA0_RST );
    }
}

static void lora0_task( void* )
{
    gpio_init( Pins::LORA0_EN );
    gpio_set_dir( Pins::LORA0_EN, GPIO_OUT );
    gpio_put( Pins::LORA0_EN, 1 );

    log_print( "[lora0] RFM9x init after 10 s radio power settle\n" );
    vTaskDelay( pdMS_TO_TICKS( 10000 ) );

    diag_spi();

    int state = s_radio.begin();
    if ( state != 0 ) {
        log_print( "[lora0] RFM9x/SX1276 init failed %d - task halting\n", state );
        while ( true ) vTaskDelay( portMAX_DELAY );
    }

    log_print( "[lora0] RFM9x/SX1276 ready - %.1f MHz SF%u BW%.0f kHz sync=0x%02X\n",
               LoRa0Cfg::FREQ_MHZ, (unsigned)LoRa0Cfg::SF,
               LoRa0Cfg::BW_KHZ, (unsigned)LoRa0Cfg::SYNC_WORD );

    s_radio.start_receive();

    for ( ;; ) {
        if ( s_radio.packet_available() ) {
            radio::Packet pkt;
            state = s_radio.read_packet( pkt );

            if ( state == 0 ) {
                if ( !handle_sigma2_packet( pkt ) &&
                     !handle_legacy_sigma_packet( pkt ) ) {
                    log_print( "[lora0] rx %u B RSSI %.1f dBm SNR %.1f dB - bad SIGMA2/SIGMA frame\n",
                               (unsigned)pkt.len, (double)pkt.rssi, (double)pkt.snr );
                }
            } else {
                log_print( "[lora0] read_packet error %d\n", state );
            }

            s_radio.start_receive();
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

static StaticTask_t s_lora0_tcb;
static StackType_t  s_lora0_stack[ 2048 ];

void lora0_task_init()
{
    TaskHandle_t h = task_create( lora0_task, "lora0", 2048,
                                  nullptr, tskIDLE_PRIORITY + 4,
                                  s_lora0_stack, &s_lora0_tcb );
    vTaskCoreAffinitySet( h, 1u << 0 );
}
