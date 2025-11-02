/*
    main.c (ESP-IDF v5.0.x)
 *
    ESP-NOW RESPONDER FOR RECEIVING FREQUENCY FROM THE PEER DEVICE,ESP32-C3
    This code initializes WiFi in station mode, configures ESP-NOW,
    and registers callbacks to receive the registered frequency.
 */


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
    Definitions of constants
*/
#define ESP_CHANNEL 1


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


/*
    ESP-NOW RECEIVE CALLBACK
 * @esp_now_info: metadata about the received packet (including source MAC)
 * @data: pointer to the received payload bytes
 * @data_len: number of bytes in the payload
 */
void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    printf("Received (%d bytes): ");                    // Indicate number of bytes received
    fflush(stdout);                                     // Ensure prompt output  
    printf("%.*s\n", data_len, (const char *)data);     // Print received data as string
}


/*
    INITIALIZE ESP-NOW AND REGISTER CALLBACKS
 */
static esp_err_t init_esp_now(void)
{
    esp_now_init();                         // Initialize ESP-NOW protocol
    esp_now_register_recv_cb(recv_cb);      // Register callback for receiving data
    esp_now_register_send_cb(send_cb);      // Register callback for send status

    return ESP_OK;
}


/*
    INITIALIZE ESP-NOW RESPONDER APPLICATION
 */
void app_main(void)
{
        ESP_ERROR_CHECK(init_esp_now());

}