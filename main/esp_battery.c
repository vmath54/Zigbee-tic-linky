#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_battery.h"

/* call back function pointer */
static esp_battery_sensor_callback_t func_ptr;
/* update interval in seconds */
static uint16_t interval = 1;
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

static const char *TAG = "ESP_TEMP_SENSOR_DRIVER";

float read_battery_voltage(void)
{
    int adc_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc1_handle, ADC_CHANNEL_6, &adc_value));
    float voltage = adc_value * (3.3f / 4095.0f);  // Convert ADC value to voltage
    ESP_LOGI(TAG, "ADC Value: %d, Voltage: %.2fV", adc_value, voltage);
    return voltage;
}

/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void battery_sensor_driver_value_update(void *arg)
{
    for (;;) {
    float voltage = read_battery_voltage();
        if (func_ptr) {
            func_ptr(voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(interval * 1000));
    }
}

/**
 * @brief init temperature sensor
 *
 * @param config      pointer of temperature sensor config.
 */
static esp_err_t battery_sensor_driver_sensor_init(battery_sensor_config_t *config)
{
    return (xTaskCreate(battery_sensor_driver_value_update, "sensor_update", 2048, NULL, 10, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

esp_err_t init_battery_sensor(battery_sensor_config_t *config, uint16_t update_interval,
                             esp_battery_sensor_callback_t cb)
{
    if (ESP_OK != battery_sensor_driver_sensor_init(config)) {
        return ESP_FAIL;
    }
    func_ptr = cb;
    interval = update_interval;
    return ESP_OK;
}

void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_6, &chan_config)); // ADC_CHANNEL_6 correspond au GPIO34
}

