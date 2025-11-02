/*
    main.c (ESP-IDF v5.0.x)
 *
    FREQUENCY METER USING ESP32-C3 AND ESP-NOW COMMUNICATION
    This code configures an ESP32-C3 to measure frequency on a specified GPIO pin
    using Pulse Counting and General Purpose Timer. The measured frequency is sent
    to another ESP32-C3 device via ESP-NOW protocol: INICIATOR-RESPONDER.
*/

/*
    Frequency Measurement Libraries
 */
#include "driver/pulse_cnt.h"     // Pulse Counter driver (pcnt_*) - REQUIRED for pulse counting APIs
#include "hal/pcnt_types.h"       // PCNT HAL types (may be redundant with driver header on some IDF versions)
#include "driver/gptimer.h"       // GPTimer driver (gptimer_*) - REQUIRED for high-resolution timer

/*
    WiFi and ESP-NOW Libraries
*/
#include "esp_now.h"              // ESP-NOW protocol functions
#include "esp_wifi.h"             // WiFi functions
#include "esp_netif.h"            // Network interface functions
#include "esp_mac.h"              // MAC address functions

/*
    Other libraries
*/
#include "sdkconfig.h"             // IDF project configuration (keep)
#include "freertos/portmacro.h"    // FreeRTOS port macros (usually included indirectly by FreeRTOS.h)
#include "freertos/FreeRTOS.h"     // FreeRTOS core functions
#include "freertos/task.h"         // FreeRTOS task management
#include "driver/gpio.h"           // GPIO definitions (EDGE_GPIO value present; explicit gpio API not used here)
#include "nvs_flash.h"             // Non-volatile storage functions
#include <stdio.h>                 // Standard I/O functions
#include <string.h>                // String manipulation functions
#include "esp_attr.h"              // Attribute macros (e.g., IRAM_ATTR)
#include "esp_event.h"             // Event loop functions
#include "esp_log.h"               // ESP logging functions


/*
    Definitions of constants and macros
*/
#define ESP_CHANNEL 1              // WiFi channel for ESP-NOW communication
#define COUNT_UNIT_L_LIM -32767    // Lower limit for pulse counter (16-bit signed max negative value)
#define COUNT_UNIT_H_LIM 32767     // Upper limit for pulse counter (16-bit signed max positive value)
#define EDGE_GPIO 22               // GPIO pin used for frequency measurement input
#define RESOLUTION_HZ 10000000     // Timer resolution in Hz (10 MHz, gives 0.1μs precision)

static uint8_t peer_mac [ESP_NOW_ETH_ALEN] = {0x68,0x67,0x25,0x4f,0xc9,0x78}; // MAC address of the target ESP32-C3 device


/* 
    INITIALIZE WiFi IN STATION MODE FOR ESP-NOW COMMUNICATION
 */
static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    
    esp_netif_init();                         // Initialize network interface
    esp_event_loop_create_default();          // Create default event loop
    nvs_flash_init();                         // Initialize non-volatile storage
    esp_wifi_init(&wifi_init_config);         // Initialize WiFi with default config
    esp_wifi_set_mode(WIFI_MODE_STA);         // Set station mode
    esp_wifi_set_storage(WIFI_STORAGE_FLASH); // Store WiFi config in flash
    esp_wifi_start();                         // Start WiFi with current configuration

    return ESP_OK;
}

/* Callback function for ESP-NOW receive events - currently empty as we only send data,
    needed to receive the MAC address of the peer device when first connecting
*/
void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {  }

/* Callback function for ESP-NOW send status - currently empty as we don't need to send confirmations,
    needed in the first connection with the peer device
*/
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {  }

/*
    INITIALIZE ESP-NOW PROTOCOL AND REGISTER CALLBACKS
 */
static esp_err_t init_esp_now(void)
{
    esp_now_init();                           // Initialize ESP-NOW protocol
    esp_now_register_recv_cb(recv_cb);        // Register callback for receiving data
    esp_now_register_send_cb(send_cb);        // Register callback for send status

    return ESP_OK;
}   

/*
    REGISTER A PEER DEVICE FOR ESP-NOW COMMUNICATION
 */
static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t esp_now_peer_info = {};                       // Initialize peer info structure
    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);  // Copy MAC address
    esp_now_peer_info.channel = ESP_CHANNEL;                          // Set communication channel
    esp_now_peer_info.ifidx = ESP_IF_WIFI_STA;                        // Use Station interface
    
    esp_now_add_peer(&esp_now_peer_info);                             // Register the peer

    return ESP_OK;
}

/* 
    SEND DATA TO A REGISITERED PEER USING ESP-NOW:
 * @param peer_addr: MAC address of the target peer
 * @param data: Pointer to the data buffer to send
 * @param len: Length of the data in bytes
 */
static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);

    return ESP_OK;
}

/*
    MAIN APPLICATION:
 * Initializes wireless communication and sets up frequencimeter
 */
void app_main(void)
{
    // Initialize wireless communication
    ESP_ERROR_CHECK(init_wifi());             // Set up WiFi in station mode
    ESP_ERROR_CHECK(init_esp_now());          // Initialize ESP-NOW protocol
    ESP_ERROR_CHECK(register_peer(peer_mac)); // Register the target device

    // Configure pulse counter unit
    pcnt_unit_config_t unit_config = {
        .low_limit = COUNT_UNIT_L_LIM,        // Set lower counting limit
        .high_limit = COUNT_UNIT_H_LIM        // Set upper counting limit
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit)); // Create new pulse counter unit

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = EDGE_GPIO   ,        // GPIO pin for pulse input
    };
    pcnt_channel_handle_t pcnt_chan = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action (pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    // Configure general purpose timer, GPTimer
    gptimer_handle_t gptimer = NULL; 
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,   // Use default clock source
        .direction = GPTIMER_COUNT_UP,        // Count upwards
        .resolution_hz = RESOLUTION_HZ        // 10MHz, 1 tick=1us
    };

    // Initialize and start the timer
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));    // Create new timer instance
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                       // Enable the timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));                       // Start the timer
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));                  // Enable pulse counter
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));                  // Start pulse counting

    // Variables for time and pulse measurements
    uint64_t timer_count1;    // First timer reading
    uint64_t timer_count2;    // Second timer reading
    int pulse_count1;         // First pulse count reading
    int pulse_count2;         // Second pulse count reading

    while (1)
    {
        pcnt_unit_get_count(pcnt_unit, &pulse_count1);  // Get initial pulse count
        vTaskDelay(pdMS_TO_TICKS(15));                  // Wait for 15 milliseconds
        pcnt_unit_get_count(pcnt_unit, &pulse_count2);  // Get pulse count after delay

        int v0 = pulse_count2-pulse_count1;             // Calculate number of pulses in the interval
        long double v = (long double) v0;               // Convert pulse count to long double for precision

        gptimer_get_raw_count(gptimer, &timer_count1);  // Get initial timer count
        vTaskDelay(pdMS_TO_TICKS(15));                  // Wait for 15 milliseconds
        gptimer_get_raw_count(gptimer, &timer_count2);  // Get timer count after delay

        long double w = timer_count2 - timer_count1;    // Calculate elapsed time in timer ticks

        int32_t frequencia = 0;                         // Variable to hold calculated frequency

        if (w != 0 && v >= 0)                           // Prevent division by zero and negative pulse counts
        {
            frequencia = (v / w) * RESOLUTION_HZ;       // Calculate frequency in Hz
        } else {
            frequencia = 0;                             // Set frequency to zero if invalid
        }

        if (frequencia != 0) // Filter to prevent glitches and counter resets from affecting measurements
        {
            char f[sizeof(frequencia)+4];               // Buffer to hold formatted frequency string
            sprintf(f, "2,%lu", frequencia);            // Format string for sending: "2" indicates Sensor 2
            esp_now_send_data(peer_mac, (uint8_t *)&f, sizeof(f));  // Send frequency data via ESP-NOW
            printf("%s\n", f);                          // Print frequency to console
            vTaskDelay(pdMS_TO_TICKS(100));             // Wait for 100 milliseconds before next measurement
        }      
    }
}