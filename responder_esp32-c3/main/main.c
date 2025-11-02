/*
 * main.c
 *
 * Minimal ESP-NOW responder example for the ESP32-C3.
 * This file initializes WiFi in station mode, configures ESP-NOW,
 * and registers callbacks to receive and (optionally) send data.
 *
 * Notes:
 * - This file purposefully keeps initialization simple. In production
 *   you should check returned esp_err_t values and handle errors.
 * - recv_cb prints received bytes using its length to avoid buffer
 *   issues when the incoming packet is not NUL-terminated.
 */

#include <stdio.h>
#include <math.h> // kept for potential frequency calculations (commented usage)
#include <string.h>
#include "sdkconfig.h" // project config
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "hal/pcnt_types.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_now.h"    // ESP-NOW API
#include "esp_wifi.h"   // WiFi control (used by ESP-NOW)
#include "esp_netif.h"  // network interface
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"  // non-volatile storage (WiFi storage)

#define ESP_CHANNEL 1

/* Optional: broadcast/default peer MAC address (commented out).
 * Use this if you want to add a peer and send messages to a specific MAC.
 */
// static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};

/*
 * Initialize WiFi in station mode and start the TCP/IP stack.
 * Returns ESP_OK on success. For brevity this example ignores and
 * does not propagate individual initialization error codes.
 */
static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();

    /* Initialize TCP/IP stack and default event loop. */
    esp_netif_init();
    esp_event_loop_create_default();

    /* Initialize NVS flash used by WiFi for provisioning/storage. */
    nvs_flash_init();

    /* Initialize WiFi driver and put it into station mode. ESP-NOW
     * uses the underlying WiFi hardware but does not require a
     * full WiFi connection to an AP.
     */
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();

    /* In a production app check and handle each function's return value. */
    return ESP_OK;
}


/*
 * recv_cb - ESP-NOW receive callback
 * @esp_now_info: metadata about the received packet (including source MAC)
 * @data: pointer to the received payload bytes
 * @data_len: number of bytes in the payload
 *
 * Note: incoming data is not guaranteed to be NUL-terminated. Use the
 * length parameter when printing or copying the payload.
 */
void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    /* Example logging (commented out): prints source MAC and raw payload.
     * Use ESP_LOGI with a TAG string if you prefer structured logging.
     */
    // ESP_LOGI(TAG, "Received from: " MACSTR, MAC2STR(esp_now_info->src_addr));

    /* Safely print the received payload using the provided length so we
     * don't rely on the sender to NUL-terminate the buffer. If the payload
     * is binary, printing as a string may produce non-printable output.
     */
    printf("Received (%d bytes): ");
    fflush(stdout);
    /* Print exactly data_len bytes as characters. Using %.*s ensures
     * we only print the specified number of bytes. Cast to char* for
     * printf formatting. If the payload is binary, consider printing
     * a hex dump instead.
     */
    printf("%.*s\n", data_len, (const char *)data);
}


/*
 * send_cb - ESP-NOW send status callback
 * @mac_addr: destination MAC address for the packet that was sent
 * @status: send result (ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL)
 *
 * This callback is optional but useful for monitoring whether outgoing
 * packets were transmitted successfully.
 */
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    /* Example logging (commented out). Use ESP_LOGI/ESP_LOGW with a TAG
     * for production diagnostics.
     */
    // if (status == ESP_NOW_SEND_SUCCESS) { ESP_LOGI(TAG, "Send success"); }
    // else { ESP_LOGW(TAG, "Send failed"); }
}


/*
 * Initialize ESP-NOW and register callbacks.
 * Returns ESP_OK on success. As before, this example keeps error handling
 * minimal for clarity.
 */
static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    // ESP_LOGI(TAG, "ESP-NOW initialization complete");
    return ESP_OK;
}


/*
 * Entry point called by ESP-IDF. Initialize WiFi and ESP-NOW.
 */
void app_main(void)
{
    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());

}