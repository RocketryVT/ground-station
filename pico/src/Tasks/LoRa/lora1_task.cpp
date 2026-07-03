#include "lora1_task.hpp"
#include "shared.hpp"
#include "Tasks/MQTT/mqtt_task.hpp"
#include "Proto/mqtt_proto.hpp"

#include "rf69/RF69.hpp"
#include "SIGMA.hpp"
#include "SIGMA2/SIGMA2.hpp"
#include "SIGMA2/packets/gps_packets.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>

static const radio::rf69::Config s_lora1_cfg {
    .freq_mhz   = LoRa1Cfg::FREQ_MHZ,
    .br_kbps    = LoRa1Cfg::BR_KBPS,
    .fdev_khz   = LoRa1Cfg::FREQ_DEV_KHZ,
    .rx_bw_khz  = LoRa1Cfg::RX_BW_KHZ,
    .tx_dbm     = LoRa1Cfg::TX_POWER,
    .high_power = LoRa1Cfg::HIGH_POWER,
    .preamble   = LoRa1Cfg::PREAMBLE,
};

static radio::rf69::RF69 s_radio(
    spi1,
    Pins::LORA1_SCK,
    Pins::LORA1_MOSI,
    Pins::LORA1_MISO,
    Pins::LORA1_NSS,
    Pins::LORA1_DIO0,
    Pins::LORA1_RST,
    s_lora1_cfg );

static uint8_t diag_spi_read_reg( bool miso_pullup )
{
    spi_init( spi1, 1'000'000u );
    spi_set_format( spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );
    gpio_set_function( Pins::LORA1_SCK,  GPIO_FUNC_SPI );
    gpio_set_function( Pins::LORA1_MOSI, GPIO_FUNC_SPI );
    gpio_set_function( Pins::LORA1_MISO, GPIO_FUNC_SPI );

    if ( miso_pullup ) gpio_pull_up( Pins::LORA1_MISO );
    else               gpio_disable_pulls( Pins::LORA1_MISO );

    gpio_init( Pins::LORA1_NSS );
    gpio_set_dir( Pins::LORA1_NSS, GPIO_OUT );
    gpio_put( Pins::LORA1_NSS, 1 );

    gpio_init( Pins::LORA1_RST );
    gpio_set_dir( Pins::LORA1_RST, GPIO_OUT );
    gpio_put( Pins::LORA1_RST, 1 );
    sleep_ms( 1 );
    gpio_put( Pins::LORA1_RST, 0 );
    sleep_ms( 10 );

    gpio_put( Pins::LORA1_NSS, 0 );
    const uint8_t addr = 0x10u;
    uint8_t val = 0xAAu;
    spi_write_blocking( spi1, &addr, 1 );
    spi_read_blocking( spi1, 0x00u, &val, 1 );
    gpio_put( Pins::LORA1_NSS, 1 );

    gpio_disable_pulls( Pins::LORA1_MISO );
    return val;
}

static void diag_spi()
{
    const uint8_t no_pull = diag_spi_read_reg( false );
    const uint8_t pullup  = diag_spi_read_reg( true );

    log_print( "[lora1] RF69 SPI diag: RegVersion=0x%02X (pulled-up=0x%02X)\n",
               no_pull, pullup );

    if ( no_pull == 0x24 ) {
        log_print( "[lora1] diag: RFM69HCW/RF69 OK\n" );
    } else if ( no_pull == 0x00 && pullup == 0xFF ) {
        log_print( "[lora1] diag: MISO not connected - check GPIO%u\n", Pins::LORA1_MISO );
    } else if ( no_pull == 0x00 && pullup == 0x00 ) {
        log_print( "[lora1] diag: MISO shorted to GND\n" );
    } else if ( no_pull == 0xFF && pullup == 0xFF ) {
        log_print( "[lora1] diag: chip in reset / no power - check GPIO%u and 3.3V\n",
                   Pins::LORA1_RST );
    } else {
        log_print( "[lora1] diag: unexpected - NSS=%u SCK=%u MOSI=%u MISO=%u RST=%u\n",
                   Pins::LORA1_NSS, Pins::LORA1_SCK,
                   Pins::LORA1_MOSI, Pins::LORA1_MISO, Pins::LORA1_RST );
    }
}

static void publish_hex_packet( const radio::Packet& pkt )
{
    MqttMessage m = {};
    groundstation_Lora1Rf69Packet pb = groundstation_Lora1Rf69Packet_init_zero;
    pb.has_data = true;
    pb.data.size = pkt.len < sizeof(pb.data.bytes) ? pkt.len : sizeof(pb.data.bytes);
    memcpy( pb.data.bytes, pkt.data, pb.data.size );
    pb.has_rssi = true; pb.rssi = pkt.rssi;
    pb.has_snr = true;  pb.snr = pkt.snr;

    if ( mqtt_encode_proto( m, "rocket/lora1/rf69",
                            groundstation_Lora1Rf69Packet_fields, &pb ) )
        xQueueSend( g_mqtt_queue, &m, 0 );
}

static void publish_lora_sample( const groundstation_RocketLoRaSample& pb )
{
    MqttMessage m = {};
    if ( mqtt_encode_proto( m, "rocket/lora1",
                            groundstation_RocketLoRaSample_fields, &pb ) )
        xQueueSend( g_mqtt_queue, &m, 0 );
}

static SIGMA2::DecodeStatus deserialize_sigma2_packet( const radio::Packet& pkt,
                                                       SIGMA2::DecodedFrame& frame )
{
    SIGMA2::DecodeStatus status =
        SIGMA2::deserialize_frame( pkt.data, pkt.len, frame );
    if ( status != SIGMA2::DecodeStatus::LengthMismatch ) return status;

    SIGMA2::HEADER header = {};
    if ( !SIGMA2::HEADER::deserialize( pkt.data, pkt.len, header ) )
        return SIGMA2::DecodeStatus::TooShort;
    if ( !header.valid_start() ) return SIGMA2::DecodeStatus::BadStart;
    if ( header.len > SIGMA2::MAX_PAYLOAD )
        return SIGMA2::DecodeStatus::PayloadTooLarge;

    const size_t total = SIGMA2::HEADER::WIRE_SIZE + header.len +
                         SIGMA2::FOOTER::WIRE_SIZE;
    if ( total > pkt.len ) return SIGMA2::DecodeStatus::TooShort;
    return SIGMA2::deserialize_frame( pkt.data, total, frame );
}

static void update_altitude( float alt_m,
                             uint32_t boot_ms,
                             float rssi,
                             float snr )
{
    if ( !g_rocket_altitude_q ) return;

    AltitudeMsg msg = {};
    msg.alt_m = alt_m;
    msg.source_boot_ms = boot_ms;
    msg.timestamp_us = time_us_64();
    msg.rssi = rssi;
    msg.snr = snr;
    msg.valid = true;
    xQueueOverwrite( g_rocket_altitude_q, &msg );
}

static bool handle_sigma2_baro_packet( const radio::Packet& pkt )
{
    SIGMA2::DecodedFrame frame = {};
    if ( deserialize_sigma2_packet( pkt, frame ) != SIGMA2::DecodeStatus::Ok ) {
        return false;
    }
    if ( frame.header.type != SIGMA2::PacketType::BARO ) return false;

    SIGMA2::TRANSMIT_PACKETS::Barometer baro = {};
    if ( !SIGMA2::deserialize_packet_payload( frame, baro ) ) return false;

    const float alt_m = static_cast<float>( baro.altitude_cm ) * 0.01f;
    update_altitude( alt_m, frame.header.timestamp_ms, pkt.rssi, pkt.snr );

    groundstation_RocketLoRaSample pb = groundstation_RocketLoRaSample_init_zero;
    pb.has_boot_ms = true;    pb.boot_ms = frame.header.timestamp_ms;
    pb.has_alt_baro_m = true; pb.alt_baro_m = alt_m;
    pb.has_rssi = true;       pb.rssi = pkt.rssi;
    pb.has_snr = true;        pb.snr = pkt.snr;
    publish_lora_sample( pb );

    log_print( "[lora1] SIGMA2 BARO alt=%.1f m RSSI=%.0f\n",
               (double)alt_m, (double)pkt.rssi );
    return true;
}

static bool handle_legacy_sigma_packet( const radio::Packet& pkt )
{
    SIGMA::LoRaData d;
    if ( !SIGMA::LoRaData::deserialize( pkt.data, pkt.len, d ) ) return false;

    update_altitude( d.alt_baro_m, d.boot_ms, pkt.rssi, pkt.snr );

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

    log_print( "[lora1] SIGMA baro=%.1f m gps=%.1f m RSSI=%.0f\n",
               (double)d.alt_baro_m, (double)d.alt_gps_m, (double)pkt.rssi );
    return true;
}

static void lora1_task( void* )
{
    gpio_init( Pins::LORA1_EN );
    gpio_set_dir( Pins::LORA1_EN, GPIO_OUT );
    gpio_put( Pins::LORA1_EN, 1 );

    log_print( "[lora1] RFM69HCW init after 12 s radio power settle\n" );
    vTaskDelay( pdMS_TO_TICKS( 12000 ) );

    diag_spi();

    int state = s_radio.begin();
    if ( state != 0 ) {
        log_print( "[lora1] RFM69HCW init failed %d - task halting\n", state );
        while ( true ) vTaskDelay( portMAX_DELAY );
    }

    log_print( "[lora1] RFM69HCW ready - %.1f MHz %.1f kbps fdev=%.1f kHz rx_bw=%.0f kHz\n",
               LoRa1Cfg::FREQ_MHZ, LoRa1Cfg::BR_KBPS,
               LoRa1Cfg::FREQ_DEV_KHZ, LoRa1Cfg::RX_BW_KHZ );

    s_radio.start_receive();

    for ( ;; ) {
        if ( s_radio.packet_available() ) {
            radio::Packet pkt;
            state = s_radio.read_packet( pkt );

            if ( state == 0 && pkt.len > 0 ) {
                if ( handle_sigma2_baro_packet( pkt ) ||
                     handle_legacy_sigma_packet( pkt ) ) {
                    publish_hex_packet( pkt );
                    s_radio.start_receive();
                    continue;
                }
                publish_hex_packet( pkt );
            } else if ( state != 0 ) {
                log_print( "[lora1] read_packet error %d\n", state );
            }

            s_radio.start_receive();
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

static StaticTask_t s_lora1_tcb;
static StackType_t  s_lora1_stack[ 1024 ];

void lora1_task_init()
{
    TaskHandle_t h = task_create( lora1_task, "lora1", 1024,
                                  nullptr, tskIDLE_PRIORITY + 4,
                                  s_lora1_stack, &s_lora1_tcb );
    vTaskCoreAffinitySet( h, 1u << 1 );
}
