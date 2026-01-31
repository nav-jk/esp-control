#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/spi_master.h"

/* ================== Registers ================== */

static const char *TAG = "MPU";

#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C
#define USER_CTRL       0x6A
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27

#define ACCEL_XOUT_H     0x3B
#define EXT_SENS_DATA_00 0x49

/* ================== Private State ================== */


typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
} mpu9250_data_t;




typedef struct {
    spi_host_device_t spi_host;   // SPI2_HOST or SPI3_HOST
    int mosi_pin;
    int miso_pin;
    int sclk_pin;
    int cs_pin;
    int clock_speed_hz;           // e.g. 1 MHz
    uint8_t gyro_config;          // FS_SEL bits (0x00,0x08,0x10,0x18)
    uint8_t accel_config;         // AFS_SEL bits (0x00,0x08,0x10,0x18)
} mpu9250_config_t;


static spi_device_handle_t mpu;
static SemaphoreHandle_t data_lock;
static mpu9250_config_t cfg_cached;

/* ================== SPI Helpers ================== */

static void mpu_write(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg & 0x7F, data };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_transmit(mpu, &t);
}

static void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0, sizeof(tx));

    tx[0] = reg | 0x80;

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(mpu, &t);
    memcpy(buf, &rx[1], len);
}

/* ================== Hardware Init ================== */

static void mpu_hw_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write(PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write(PWR_MGMT_1, 0x01);
    mpu_write(PWR_MGMT_2, 0x00);

    mpu_write(USER_CTRL, 0x10);
    mpu_write(CONFIG, 0x02);
    mpu_write(GYRO_CONFIG, cfg_cached.gyro_config);
    mpu_write(ACCEL_CONFIG, cfg_cached.accel_config);
    mpu_write(ACCEL_CONFIG2, 0x02);

    /* Enable I2C master */
    mpu_write(USER_CTRL, 0x30);
    mpu_write(I2C_MST_CTRL, 0x0D);

    /* AK8963 power down */
    mpu_write(I2C_SLV0_ADDR, 0x0C);
    mpu_write(I2C_SLV0_REG, 0x0A);
    mpu_write(I2C_SLV0_CTRL, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Continuous mode 2 (100 Hz) */
    mpu_write(I2C_SLV0_ADDR, 0x0C);
    mpu_write(I2C_SLV0_REG, 0x0A);
    mpu_write(I2C_SLV0_CTRL, 0x96);
}

/* ================== Public API ================== */

bool mpu9250_init(const mpu9250_config_t *cfg)
{
    if (!cfg) return false;
    cfg_cached = *cfg;

    spi_bus_config_t buscfg = {
        .mosi_io_num = cfg->mosi_pin,
        .miso_io_num = cfg->miso_pin,
        .sclk_io_num = cfg->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = cfg->clock_speed_hz,
        .mode = 0,
        .spics_io_num = cfg->cs_pin,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(spi_bus_initialize(cfg->spi_host, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(cfg->spi_host, &devcfg, &mpu));

    uint8_t who;
    mpu_read(WHO_AM_I, &who, 1);
    // if (who != 0x71) {
    //     ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X", who);
    //     return false;
    // }

    data_lock = xSemaphoreCreateMutex();
    if (!data_lock) return false;

    mpu_hw_init();
    ESP_LOGI(TAG, "MPU-9250 initialized");

    return true;
}

bool mpu9250_get_data(mpu9250_data_t *out)
{
    if (!out) return false;

    uint8_t raw[14];
    uint8_t mag[6];

    xSemaphoreTake(data_lock, portMAX_DELAY);

    mpu_read(ACCEL_XOUT_H, raw, 14);
    mpu_read(EXT_SENS_DATA_00, mag, 6);

    out->ax = (raw[0] << 8) | raw[1];
    out->ay = (raw[2] << 8) | raw[3];
    out->az = (raw[4] << 8) | raw[5];

    out->gx = (raw[8] << 8) | raw[9];
    out->gy = (raw[10] << 8) | raw[11];
    out->gz = (raw[12] << 8) | raw[13];

    out->mx = (mag[1] << 8) | mag[0];
    out->my = (mag[3] << 8) | mag[2];
    out->mz = (mag[5] << 8) | mag[4];

    xSemaphoreGive(data_lock);
    return true;
}