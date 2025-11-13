/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU6050 inertial measurement unit.
*/

#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

static const char *TAG = "mpu6050";

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU6050_SENSOR_ADDR 0x68         /*!< Address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR 0x75   /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power management register */
#define MPU6050_RESET_BIT 7

// float AccX, AccY, AccZ;
// float GyroX, GyroY, GyroZ;
// float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
// float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
// int c = 0;

struct DataSample
{
    float X;
    float Y;
    float Z;
};

// Returns the number of microseconds since the ESP-IDF program started
uint32_t millis()
{
    return esp_timer_get_time() / 1000;
}

// struct DataSample AccelSample;
//  struct DataSample GyroSample;

/**
 * @brief Read a sequence of bytes from a sensor registers
 */
static esp_err_t sensor_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a sensor register
 */
static esp_err_t sensor_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *mpu6050_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t mpu6050_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &mpu6050_config, mpu6050_handle));
}

void calculate_IMU_error()
{
    int count = 0;
    int iterations = 200;
    uint8_t buffer[6];
    struct DataSample AccelSample, GyroSample, AccelError, GyroError;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t mpu6050_handle;
    i2c_master_init(&bus_handle, &mpu6050_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    while (count++ < iterations)
    {
        // === Read acceleromter data === //
        ESP_ERROR_CHECK(sensor_register_read(mpu6050_handle, 0x3B, buffer, 6));
        AccelSample.X = ((int16_t)((buffer[0] << 8) + (buffer[1])) / 16384.0);
        AccelSample.Y = ((int16_t)((buffer[2] << 8) + (buffer[3])) / 16384.0);
        AccelSample.Z = ((int16_t)((buffer[4] << 8) + (buffer[5])) / 16384.0);

        // Sum all readings
        AccelError.X = AccelError.X + ((atan((AccelSample.Y) / sqrt(pow((AccelSample.X), 2) + pow((AccelSample.Z), 2))) * 180 / M_PI));
        AccelError.Y = AccelError.Y + ((atan(-1 * (AccelSample.X) / sqrt(pow((AccelSample.Y), 2) + pow((AccelSample.Z), 2))) * 180 / M_PI));
    }

    AccelError.X = AccelError.X / iterations;
    AccelError.Y = AccelError.Y / iterations;

    count = 0;

    while (count++ < iterations)
    {
        ESP_ERROR_CHECK(sensor_register_read(mpu6050_handle, 0x43, buffer, 6));
        GyroSample.X = (int16_t)((buffer[0] << 8) + (buffer[1]));
        GyroSample.Y = (int16_t)((buffer[2] << 8) + (buffer[3]));
        GyroSample.Z = (int16_t)((buffer[4] << 8) + (buffer[5]));

        GyroError.X = GyroError.X + (GyroSample.X / 131.0);
        GyroError.Y = GyroError.Y + (GyroSample.Y / 131.0);
        GyroError.Z = GyroError.Z + (GyroSample.Z / 131.0);
    }

    GyroError.X = GyroError.X / iterations;
    GyroError.Y = GyroError.Y / iterations;
    GyroError.Z = GyroError.Z / iterations;

    ESP_LOGI(TAG, "AccelErrorX= %f, AccelErrorY= %f, GyroErrorX= %f, GyroErrorY= %f, GyroErrorZ= %f",
             AccelError.X, AccelError.Y, GyroError.X, GyroError.Y, GyroError.Z);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(mpu6050_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void sensor_task(void *pvParameters)
{
    /* Block for 500ms. */
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    // calculate_IMU_error();
    struct DataSample AccelSample, GyroSample, AccelAngle, GyroAngle;
    // struct DataSample GyroSample;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t mpu6050_handle;
    i2c_master_init(&bus_handle, &mpu6050_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    // uint8_t data[2];
    //  ESP_ERROR_CHECK(sensor_register_read(mpu6050_handle, MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    //  ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by resetting the MPU9250 */
    // ESP_ERROR_CHECK(sensor_register_write_byte(mpu6050_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));

    /* Resetting the MPU6050 */
    ESP_ERROR_CHECK(sensor_register_write_byte(mpu6050_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00));
    // ESP_ERROR_CHECK(sensor_register_write_byte(mpu6050_handle, 0x6A, 0x00));
    // ESP_ERROR_CHECK(sensor_register_write_byte(mpu6050_handle, 0X37, 0x02));

    GyroAngle.X = 0.0;
    GyroAngle.Y = 0.0;
    yaw = 0.0;

    uint8_t buffer[6];

    currentTime = millis();

    for (;;)
    {
        // === Read acceleromter data === //
        ESP_ERROR_CHECK(sensor_register_read(mpu6050_handle, 0x3B, buffer, sizeof(buffer)));
        AccelSample.X = ((int16_t)((buffer[0] << 8) + (buffer[1])) / 16384.0);
        AccelSample.Y = ((int16_t)((buffer[2] << 8) + (buffer[3])) / 16384.0);
        AccelSample.Z = ((int16_t)((buffer[4] << 8) + (buffer[5])) / 16384.0);

        // ESP_LOGI(TAG, "X-Axis= %f, Y-Axis= %f, Z-Axis= %f", AccelSample.X, AccelSample.Y, AccelSample.Z);

        // Calculating Roll and Pitch from the accelerometer data
        AccelAngle.X = (atan(AccelSample.Y / sqrt(pow(AccelSample.X, 2) + pow(AccelSample.Z, 2))) * 180 / M_PI) - 0.236268;      // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
        AccelAngle.Y = (atan(-1 * AccelSample.X / sqrt(pow(AccelSample.Y, 2) + pow(AccelSample.Z, 2))) * 180 / M_PI) + 2.937030; // AccErrorY ~(-1.58)

        // === Read gyroscope data === //
        previousTime = currentTime;                        // Previous time is stored before the actual time read
        currentTime = millis();                            // Current time actual time read
        elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

        ESP_ERROR_CHECK(sensor_register_read(mpu6050_handle, 0x43, buffer, 6));
        GyroSample.X = ((int16_t)((buffer[0] << 8) + (buffer[1])) / 131.0);
        GyroSample.Y = ((int16_t)((buffer[2] << 8) + (buffer[3])) / 131.0);
        GyroSample.Z = ((int16_t)((buffer[4] << 8) + (buffer[5])) / 131.0);

        GyroSample.X = GyroSample.X + 2.948621;
        GyroSample.Y = GyroSample.Y - 1.575840;
        GyroSample.Z = GyroSample.Z - 0.323206;

        // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
        GyroAngle.X = GyroAngle.X + GyroSample.X * elapsedTime; // deg/s * s = deg
        GyroAngle.Y = GyroAngle.Y + GyroSample.Y * elapsedTime;
        yaw = yaw + GyroSample.Z * elapsedTime;
        // Complementary filter - combine acceleromter and gyro angle values
        roll = 0.96 * GyroAngle.X + 0.04 * AccelAngle.X;
        pitch = 0.96 * GyroAngle.Y + 0.04 * AccelAngle.Y;

        // Print the values on the serial monitor
        ESP_LOGI(TAG, "/%.2f/%.2f/%.2f", roll, pitch, yaw);

        vTaskDelay( xDelay );
    }
}

void app_main(void)
{

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(mpu6050_handle));
    //ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    //ESP_LOGI(TAG, "I2C de-initialized successfully");
}