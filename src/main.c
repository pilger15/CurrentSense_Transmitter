// #TODO: https://github.com/aliengreen/esp32_uart_bridge/blob/main/main/esp32_uart_bridge.c
// #TODO dublebuffer/ringbuffer

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_private/wifi.h"
#include "esp_private/esp_wifi_types_private.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "hal/usb_serial_jtag_ll.h"
#include "driver/usb_serial_jtag.h"

#define BLUE_LED GPIO_NUM_3
// #define BIT_BLUE_LED BIT3
#define ORANGE_LED GPIO_NUM_2
// #define BIT_ORANGE_LED BIT2

#define CONFIG_ESPNOW_CHANNEL 1
esp_now_peer_info_t *peer;

#define BUFFER_USB_SZ 1024
#define USB_PACKET_SZ 256
char terminator[] = "ENDATA";
char stream[BUFFER_USB_SZ];
int stream_idx = 0;
char *terminator_p;

RingbufferType_t espnow_rx_handle;
RingbufferType_t usb_rx_handle;

#define RINGBUFFER_SZ (128 * 4)
uint8_t buffer_usb_rx[BUFFER_USB_SZ];

#define USB_SEND_MESSAGE_LEN 16
uint8_t buffer_usb_tx[USB_SEND_MESSAGE_LEN];
uint8_t buffer_espnow_rx = sizeof(buffer_usb_tx);
;

// a0:76:4e:44:51:38
esp_now_peer_info_t *peer_dongle;
bool dongleRegistered = false;
typedef enum
{
    TRANSFER_USB,
    TRANSFER_WIFI
} transfertype_t;
const transfertype_t transfertype = TRANSFER_WIFI;
typedef enum
{
    espcommand_empty,
    espcommand_start
} esp_command_t;

// static const uint64_t espcommand_channel = 0x74797380ec3cce13ULL;
static const uint64_t espcommand_channel_bigendian = 0x13CE3CEC80737974ULL; // reverse due to little endian when casting

bool measure = false;

// static uint8_t sensor_mac[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x44, 0x51, 0x38};
//  a0:76:4e:41:bd:dc
// static uint8_t sensor_mac1[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x41, 0xBD, 0xDC};
//  a0:76:4e:41:e6:70
// static uint8_t sensor_mac2[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x41, 0xE6, 0x70};
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// a0 : 76 : 4e : 44 : 51 : 38

void receive_usb_send_esp_now(void *pvParameters)
{
    /* Configure USB-CDC */
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = BUFFER_USB_SZ,
        .rx_buffer_size = BUFFER_USB_SZ};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));

    while (true)
    {
        stream_idx += usb_serial_jtag_read_bytes(stream, 256, 1 / portTICK_PERIOD_MS);
        stream[stream_idx] = '\0';
        terminator_p = strstr(stream, terminator);

        // Check if header is found and if 256 bytes have been transmitted
        if (terminator_p != NULL)
        {
            const uint8_t *data = (const uint8_t *)(terminator_p)-ESP_NOW_MAX_DATA_LEN;
            esp_err_t ret = esp_now_send(peer->peer_addr, data, ESP_NOW_MAX_DATA_LEN);

            // Remove data from stream and shift the rest of the stream to the beginning
            memcpy(&stream[0], terminator_p + 256, ((int)(&stream[stream_idx]) - (int)&terminator_p) - USB_PACKET_SZ);
        }
    }
}
void IRAM_ATTR receive_esp_now_send_usb(void *pvParameters)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // esp_wifi_internal_set_fix_rate(CONFIG_ESPNOW_CHANNEL, 1, WIFI_PHY_RATE_MCS7_SGI);

    ESP_ERROR_CHECK(esp_wifi_start());
    // ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_MCS7_SGI));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Initialize ESPNOW and register sending and receiving callback function.

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_MODE_STA, WIFI_PHY_RATE_MCS7_SGI));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    uint8_t mac_address[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_address);
    ESP_LOGI("MAC", "%02x:%02x:%02x:%02x:%02x:%02x\n",
             mac_address[0], mac_address[1], mac_address[2],
             mac_address[3], mac_address[4], mac_address[5]);
    // esp_log_level_set("ESP-NOW", ESP_LOG_DEBUG);

    while (true)
    {

        // check ringbuffer wrap
        if (dongleRegistered && (((buffer_read_idx + RINGBUFFER_READ_LEN_16) < buffer_write_idx) // this assumes that read+read_len will not wrap around
                                 || (buffer_write_idx < buffer_read_idx)))
        { // this assumes the write has wrapped around so the read is bigger

            // Send data over ESP-NOW
            // REG_WRITE(GPIO_OUT_W1TS_REG, 1 <<BLUE_LED); // initialise conversion
            memcpy(&transfer_buffer[1], &adc_ringbuffer[buffer_read_idx], RINGBUFFER_READ_LEN_16 * sizeof(uint16_t));
            transfer_buffer[0] = (packet_cnt++); // will overflow to 0
            esp_err_t ret = esp_now_send(peer_dongle->peer_addr, (uint8_t *)transfer_buffer, sizeof(transfer_buffer));
            buffer_read_idx = (buffer_read_idx + RINGBUFFER_READ_LEN_16) % RINGBUFFER_SIZE;
            // ESP_LOGE("CNT", "CNT = %d", espnow_buffer_tx[0]);
            // REG_WRITE(GPIO_OUT_W1TC_REG, 1 <<BLUE_LED); // initialise conversion
            if (ret != ESP_OK)
            {
                ESP_LOGE("ESP-NOW", "Error sending data: %s", esp_err_to_name(ret));
                REG_WRITE(GPIO_OUT_W1TC_REG, 1 << BLUE_LED); // Clear BLUE LED;
            }
        }
        taskYIELD();
    }
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}
static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    //
    if (dongleRegistered && esp_now_is_peer_exist(esp_now_info->src_addr)) // check if command is from registed Dongle
    {
        ESP_LogD("ESP-NOW", "Received command %d- entering Switch", data[0]);
        switch ((esp_command_t)data[0])
        {
        case espcommand_start:
            if (!measure) // start measure
            {
                ESP_LOGI("ESP-NOW", "Start measurement");
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMERPERIOD_US)); // in us
                                                                                           // garbage transmit
                ESP_LOGD("TIMER", "Measurement started");

                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << ORANGE_LED); // High
                packet_cnt = 0;
            }
            else
            {
                ESP_LOGI("ESP-NOW", "Stop measurement"); // stop
                // timer stopped in ISR
                REG_WRITE(GPIO_OUT_W1TC_REG, 1 << ORANGE_LED); // LOW
                                                               //  REG_WRITE(GPIO_OUT_W1TC_REG, BIT10); // LOW
                ESP_LOGD("TIMER", "Measurement stopped");
                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << BLUE_LED); // Set blue in case it was turned off as error indicator
            }

            measure = !measure;
            buffer_read_idx = 0;
            buffer_write_idx = 0;

            break;

        default:
            break;
        }
    }
    // register dongle
    else if (!dongleRegistered && (data_len == sizeof(uint64_t)) && *(uint64_t *)data == espcommand_channel_bigendian) // cast command an receive reverse endian
    {
        uint8_t espcommand_channel[8];
        memcpy(espcommand_channel, data, 8);
        ESP_LOGI("ESP-NOW", "Registration of channel %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 espcommand_channel[0],
                 espcommand_channel[1],
                 espcommand_channel[2],
                 espcommand_channel[3],
                 espcommand_channel[4],
                 espcommand_channel[5],
                 espcommand_channel[6],
                 espcommand_channel[7]);

        /* Add Dongle information to peer list. */

        memcpy(peer_dongle->peer_addr, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer_dongle));
        ESP_LOGI("ESP-NOW", "Dongle-MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 peer_dongle->peer_addr[0],
                 peer_dongle->peer_addr[1],
                 peer_dongle->peer_addr[2],
                 peer_dongle->peer_addr[3],
                 peer_dongle->peer_addr[4],
                 peer_dongle->peer_addr[5]);
        dongleRegistered = true;
        REG_WRITE(GPIO_OUT_W1TS_REG, 1 << BLUE_LED); // Set blue
    }
    else
    {
        ESP_LOGD("ESP-NOW", "Received message but not addressed to this channel");
    }
}

void app_main()
{
    RingbufferType_t espnow_rx_handle;
    RingbufferType_t usb_rx_handle;

    peer_dongle = malloc(sizeof(esp_now_peer_info_t));

    memset(peer_dongle, 0, sizeof(esp_now_peer_info_t));
    peer_dongle->channel = CONFIG_ESPNOW_CHANNEL;
    peer_dongle->ifidx = ESP_IF_WIFI_STA;
    peer_dongle->encrypt = false;

    xTaskCreate(receive_usb_send_esp_now, "receive_usb_send_esp_now", 4096, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(receive_esp_now_send_usb, "receive_esp_now_send_usb", 4096, NULL, tskIDLE_PRIORITY, NULL);
    while (0)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI("IDLE", "%02x:%02x:%02x:%02x:%02x:%02x\n",
                 mac_address[0], mac_address[1], mac_address[2],
                 mac_address[3], mac_address[4], mac_address[5]);
    }
}
/*
// a0:76:4e:44:51:38
//a0:76:4e:43:9b:88
a0:76:4e:41:e6:70
    // Cleanup Wi-Fi
    esp_wifi_stop();
    esp_wifi_deinit();
*/