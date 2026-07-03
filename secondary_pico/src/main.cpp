// Secondary Ground Station Pico — 433 MHz receiver + USB altitude forwarder
// FreeRTOS / RP2350 (Pico 2W)
//
// Task layout:
//   lora2     (pri 4) – RFM69HCW / 433 MHz receive -> USB ALT lines
//   usb       (pri 1) – USB CDC logger + serial console
//
// Each decoded altitude-bearing packet is printed as:
//   ALT,<alt_m>,<boot_ms>,<rssi>,<snr>
// A laptop-side bridge can forward those lines to the primary Pico USB console.
//
// All FreeRTOS objects are statically allocated — no heap usage for scheduler
// infrastructure.

#include "shared.hpp"

// WiFi/UDP and the 915 MHz receiver are intentionally disabled for the no-WiFi
// field profile. USB CDC is the forwarding path.
// #include "Tasks/WiFi/wifi_task.hpp"
#include "Tasks/LoRa/lora1_task.hpp"
#include "Tasks/LoRa/lora2_task.hpp"
// #include "Tasks/UDP/udp_send_task.hpp"
#include "Tasks/USB/usb_task.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

// -- Shared FreeRTOS handles ---------------------------------------------------
EventGroupHandle_t g_net_events  = nullptr;
QueueHandle_t      g_log_queue   = nullptr;
QueueHandle_t      g_udp_queue   = nullptr;

// -- Static backing storage ----------------------------------------------------
static StaticEventGroup_t s_net_events_buf;

static StaticQueue_t s_log_queue_buf;
static uint8_t       s_log_queue_storage[ LOG_QUEUE_DEPTH * sizeof(LogMessage) ];

static StaticQueue_t s_udp_queue_buf;
static uint8_t       s_udp_queue_storage[ UDP_QUEUE_DEPTH * sizeof(UdpFrame) ];

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

    printf( "Secondary Pico starting...\n" );
    for ( int i = 0; i < 10; ++i ) { printf( "." ); sleep_ms( 500 ); }
    printf( "\n" );

    // Shared synchronisation objects
    g_net_events = xEventGroupCreateStatic( &s_net_events_buf );

    g_log_queue = xQueueCreateStatic( LOG_QUEUE_DEPTH, sizeof(LogMessage),
                                       s_log_queue_storage, &s_log_queue_buf );
    g_udp_queue = xQueueCreateStatic( UDP_QUEUE_DEPTH, sizeof(UdpFrame),
                                       s_udp_queue_storage, &s_udp_queue_buf );

    printf( "Initializing tasks...\n" );

    // wifi_task_init();
    lora1_task_init();
    lora2_task_init();
    // udp_send_task_init();

    printf( "Initializing USB task...\n" );
    usb_task_init();

    log_print( "Secondary Pico initialized.\n" );

    { void* p = pvPortMalloc(1); vPortFree(p); }

    log_print( "Starting scheduler...\n" );
    vTaskStartScheduler();
    for ( ;; ) {}
}
