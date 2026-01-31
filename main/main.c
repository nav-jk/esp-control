/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "saved.c"

#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT


/* PIN definitions */
#define MAX_DQ_SIZE 100
#define CS_PIN 5
#define MOSI_PIN 23
#define MISO_PIN 19
#define SCLK_PIN 18


/* Data sharing thingies */

static mpu9250_data_t buf_a[MAX_DQ_SIZE];
static mpu9250_data_t buf_b[MAX_DQ_SIZE];

static mpu9250_data_t *write_buf = buf_a;
static mpu9250_data_t *send_buf  = buf_b;

static uint16_t write_count = 0;
static uint16_t send_count  = 0;

static TaskHandle_t xTaskSensor = NULL, xTaskUDP = NULL;  




/*  Serialization starting here */
static inline void write_u16_be(uint8_t **p, uint16_t v) {
    // big endian unsigned write
    *(*p)++ = (uint8_t)(v >> 8);
    *(*p)++ = (uint8_t)(v);
}

static inline void write_i16_be(uint8_t **p, int16_t v) {
    // big endian signed write
    write_u16_be(p, (uint16_t)v);
}

static size_t serialize_mpu9250_batch(
    uint8_t *buf,
    const mpu9250_data_t *data,
    uint16_t count
) {
    uint8_t *p = buf;

    // batch size
    write_u16_be(&p, count);

    for (uint16_t i = 0; i < count; i++) {
        write_i16_be(&p, data[i].ax);
        write_i16_be(&p, data[i].ay);
        write_i16_be(&p, data[i].az);

        write_i16_be(&p, data[i].gx);
        write_i16_be(&p, data[i].gy);
        write_i16_be(&p, data[i].gz);

        write_i16_be(&p, data[i].mx);
        write_i16_be(&p, data[i].my);
        write_i16_be(&p, data[i].mz);
    }

    return (size_t)(p - buf); // total bytes written
}

/* Serialization ending here */

static void read_sensor_value(void *pvParameters) {
    mpu9250_config_t cfg = {
        .spi_host = SPI2_HOST,
        .mosi_pin = MOSI_PIN, // 
        .miso_pin = MISO_PIN, // 
        .sclk_pin = SCLK_PIN, // SCK
        .cs_pin   = CS_PIN, // NCS
        .clock_speed_hz = 1 * 1000 * 1000,
        .gyro_config  = 0x08,   // ±500 dps
        .accel_config = 0x08    // ±4g
    };

    if (!mpu9250_init(&cfg)) {
        ESP_LOGE(TAG, "MPU init failed");
    }

    mpu9250_data_t d;

    while (1) {
        if (mpu9250_get_data(&d)) {
            write_buf[write_count++] = d;

            if (write_count == MAX_DQ_SIZE) {
                
                // swap write and send

                
                mpu9250_data_t *tmp = write_buf;
                write_buf = send_buf;
                send_buf = tmp;

                send_count = write_count;
                write_count = 0;
                xTaskNotifyGiveIndexed( xTaskUDP, 0 );  
                ulTaskNotifyTakeIndexed( 0, pdTRUE, portMAX_DELAY );  
           }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}



static void udp_client_task(void *pvParameters)
{
    // char rx_buffer[128];
    // char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        uint8_t payload[2 + MAX_DQ_SIZE*18];
        while (1) {
            ulTaskNotifyTake( pdTRUE, portMAX_DELAY );  
            size_t len = serialize_mpu9250_batch(
                payload,
                send_buf,
                send_count
            ); // serialize buffer
            int err = sendto(sock, payload, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");
            
            xTaskNotifyGive( xTaskSensor );  
            vTaskDelay(2000 / portTICK_PERIOD_MS); // send every two seconds
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, &xTaskUDP);
    xTaskCreate(read_sensor_value, "sensor_value", 2048, NULL, 7, &xTaskSensor);
}

