// Ground Station — Primary Pico
// Azimuth:  STEP1 GPIO 4/5, 10:1 gearbox × 32T:14T = 22.857:1 total
// Elevation: STEP2 GPIO 6/7, 10:1 gearbox × 20T:14T = 14.286:1 total

#include "shared.hpp"

// WiFi/MQTT/NTP are intentionally not started for the no-WiFi field profile.
// USB CDC is the command/status path.
// #include "Tasks/WiFi/wifi_task.hpp"
// #include "Tasks/NTP/ntp_task.hpp"
// #include "Tasks/MQTT/mqtt_task.hpp"
#include "Tasks/I2C/i2c_task.hpp"
#include "Tasks/IMU/imu_task.hpp"
#include "Tasks/YawIMU/yaw_imu_task.hpp"
#include "Tasks/Mag/mag_task.hpp"
// #include "Tasks/Baro/baro_task.hpp"
#include "Tasks/Fusion/fusion_task.hpp"
#include "Tasks/GPS/gps_task.hpp"
#include "Tasks/LoRa/lora_task.hpp"
#include "Tasks/LoRa/lora1_task.hpp"
#include "Tasks/LoRa/lora_task.hpp"
// #include "Tasks/UDP/udp_recv_task.hpp"
#include "Tasks/Stepper/stepper_task.hpp"
#include "Tasks/USB/usb_task.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/stdlib.h"
#include <stdio.h>

// -- Shared FreeRTOS handles ---------------------------------------------------
EventGroupHandle_t g_net_events        = nullptr;
QueueHandle_t      g_mqtt_queue        = nullptr;
QueueHandle_t      g_log_queue         = nullptr;
QueueHandle_t      g_gs_location_q     = nullptr;
QueueHandle_t      g_rocket_location_q = nullptr;
QueueHandle_t      g_rocket_altitude_q = nullptr;
QueueHandle_t      g_imu_q             = nullptr;
QueueHandle_t      g_icm_q             = nullptr;
QueueHandle_t      g_mag_q             = nullptr;
QueueHandle_t      g_yaw_imu_q         = nullptr;
QueueHandle_t      g_baro_q            = nullptr;

// -- Static backing storage ----------------------------------------------------
static StaticEventGroup_t s_net_events_buf;

static StaticQueue_t s_mqtt_queue_buf;
static uint8_t       s_mqtt_queue_storage[ MQTT_QUEUE_DEPTH * sizeof(MqttMessage) ];

static StaticQueue_t s_log_queue_buf;
static uint8_t       s_log_queue_storage[ LOG_QUEUE_DEPTH  * sizeof(LogMessage) ];

static StaticQueue_t s_gs_location_buf;
static uint8_t       s_gs_location_storage[ 1 * sizeof(LocationMsg) ];

static StaticQueue_t s_rocket_location_buf;
static uint8_t       s_rocket_location_storage[ 1 * sizeof(LocationMsg) ];

static StaticQueue_t s_rocket_altitude_buf;
static uint8_t       s_rocket_altitude_storage[ 1 * sizeof(AltitudeMsg) ];

static StaticQueue_t s_imu_buf;
static uint8_t       s_imu_storage[ 1 * sizeof(ImuMsg) ];

static StaticQueue_t s_icm_buf;
static uint8_t       s_icm_storage[ 1 * sizeof(IcmMsg) ];

static StaticQueue_t s_mag_buf;
static uint8_t       s_mag_storage[ 1 * sizeof(MagMsg) ];

static StaticQueue_t s_yaw_imu_buf;
static uint8_t       s_yaw_imu_storage[ 1 * sizeof(YawImuMsg) ];

static StaticQueue_t s_baro_buf;
static uint8_t       s_baro_storage[ 1 * sizeof(BaroMsg) ];

// -- FreeRTOS static-allocation callbacks --------------------------------------
extern "C" {

void vApplicationGetIdleTaskMemory( StaticTask_t** ppxIdleTaskTCBBuffer,
                                     StackType_t**  ppxIdleTaskStackBuffer,
                                     uint32_t*      pulIdleTaskStackSize )
{
    static StaticTask_t idle_tcb;
    static StackType_t  idle_stack[ configMINIMAL_STACK_SIZE ];
    *ppxIdleTaskTCBBuffer   = &idle_tcb;
    *ppxIdleTaskStackBuffer =  idle_stack;
    *pulIdleTaskStackSize   =  configMINIMAL_STACK_SIZE;
}

void vApplicationGetPassiveIdleTaskMemory( StaticTask_t** ppxIdleTaskTCBBuffer,
                                            StackType_t**  ppxIdleTaskStackBuffer,
                                            uint32_t*      pulIdleTaskStackSize,
                                            BaseType_t     xPassiveIdleTaskIndex )
{
    static StaticTask_t passive_tcb  [ configNUMBER_OF_CORES - 1 ];
    static StackType_t  passive_stack[ configNUMBER_OF_CORES - 1 ]
                                     [ configMINIMAL_STACK_SIZE ];
    *ppxIdleTaskTCBBuffer   = &passive_tcb  [ xPassiveIdleTaskIndex ];
    *ppxIdleTaskStackBuffer =  passive_stack[ xPassiveIdleTaskIndex ];
    *pulIdleTaskStackSize   =  configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t** ppxTimerTaskTCBBuffer,
                                      StackType_t**  ppxTimerTaskStackBuffer,
                                      uint32_t*      pulTimerTaskStackSize )
{
    static StaticTask_t timer_tcb;
    static StackType_t  timer_stack[ configTIMER_TASK_STACK_DEPTH ];
    *ppxTimerTaskTCBBuffer   = &timer_tcb;
    *ppxTimerTaskStackBuffer =  timer_stack;
    *pulTimerTaskStackSize   =  configTIMER_TASK_STACK_DEPTH;
}

} // extern "C"

// -- Entry point ---------------------------------------------------------------
int main()
{
    stdio_init_all();

    sleep_ms( 3000 );
    printf( "Ground station primary starting...\n" );

    // -- Create all shared queues and event groups ------------------------------
    g_net_events = xEventGroupCreateStatic( &s_net_events_buf );

    g_mqtt_queue = xQueueCreateStatic( MQTT_QUEUE_DEPTH, sizeof(MqttMessage),
                                        s_mqtt_queue_storage, &s_mqtt_queue_buf );
    g_log_queue  = xQueueCreateStatic( LOG_QUEUE_DEPTH,  sizeof(LogMessage),
                                        s_log_queue_storage,  &s_log_queue_buf );

    g_gs_location_q     = xQueueCreateStatic( 1, sizeof(LocationMsg),
                                               s_gs_location_storage, &s_gs_location_buf );
    g_rocket_location_q = xQueueCreateStatic( 1, sizeof(LocationMsg),
                                               s_rocket_location_storage, &s_rocket_location_buf );
    g_rocket_altitude_q = xQueueCreateStatic( 1, sizeof(AltitudeMsg),
                                               s_rocket_altitude_storage, &s_rocket_altitude_buf );

    g_imu_q  = xQueueCreateStatic( 1, sizeof(ImuMsg),  s_imu_storage,  &s_imu_buf  );
    g_icm_q  = xQueueCreateStatic( 1, sizeof(IcmMsg),  s_icm_storage,  &s_icm_buf  );
    g_mag_q  = xQueueCreateStatic( 1, sizeof(MagMsg),  s_mag_storage,  &s_mag_buf  );
    g_yaw_imu_q = xQueueCreateStatic( 1, sizeof(YawImuMsg),
                                      s_yaw_imu_storage, &s_yaw_imu_buf );
    g_baro_q = xQueueCreateStatic( 1, sizeof(BaroMsg), s_baro_storage, &s_baro_buf );

    // -- Init tasks (order matters: I2C before sensors, WiFi before MQTT/NTP) --
    i2c0_task_init();   // I2C0: ISM330DLC IMU + LIS3MDL magnetometer
    i2c1_task_init();   // I2C1: yaw-body ICM-20948 9-axis IMU

    imu_task_init();
    mag_task_init();
    yaw_imu_task_init();
    // baro_task_init();
    fusion_task_init();

    // No WiFi at the pad. Commands and telemetry move over USB CDC instead.
    // wifi_task_init();
    // ntp_task_init();
    // mqtt_task_init();
    gps_task_init();

    lora0_task_init();
    lora1_task_init();
    // udp_recv_task_init();

    stepper_az_task_init();
    stepper_zen_task_init();
    stepper_ctrl_task_init();
    stepper_state_task_init();

    usb_task_init();

    vTaskStartScheduler();
    for ( ;; ) {}
}
