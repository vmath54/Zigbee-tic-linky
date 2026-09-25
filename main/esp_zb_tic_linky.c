/*
We are going to read the battery voltage, decode in UART the TIC signal from Linky and
extact informations Power , intensité etc ....
Mode Standard
ADSC: Adresse du compteur.
VTIC: Version de la TIC.
DATE: Date et heure de l'enregistrement.
NGTF: Nom de la grille tarifaire.
LTARF: Libellé tarifaire en cours.
EAST: Énergie active soutirée totale (en Wh).
EASF01 à EASF10: Énergies soutirées par périodes tarifaires (en Wh).
IRMS1: Courant efficace en phase 1 (en A).
URMS1: Tension efficace en phase 1 (en V).
PREF: Puissance de référence (en kVA).
PCOUP: Puissance de coupure (en kVA).
SINSTS: Puissance apparente instantanée soutirée (en VA).
SMAXSN: Puissance maximale soutirée du mois en cours (en VA).
CCASN: Consommation soutirée actuelle (en Wh).
UMOY1: Tension moyenne en phase 1 (en V).
STGE: Statut général du compteur.
MSG1: Message court (information de l'opérateur).
PRM: Point de référence de mesure.
RELAIS: État des relais.
NTARF: Numéro de l'index tarifaire en cours.
NJOURF: Nombre de jours restant avant le prochain changement de tarif.
PJOURF+1: Prévision de changement de tarif pour le jour suivan
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_tic_linky.h"
#include "esp_zb_uart.h"
#include "light_driver.h"
#include "blinker.h"
#include "esp_zigbee_core.h"

// #include "esp_battery.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_TIC_LINKY";
static spinlock_t zigbee_lock = SPINLOCK_INITIALIZER;

// Définir les priorités des tâches
#define TASK_ZIGBEE_MAIN_PRIO       5
#define TASK_ZIGBEE_DATA_PRIO       4
#define TASK_TELEINFO_GEN_PRIO      3

/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t deferred_driver_init(void)
{

    light_driver_init(false);
    return ESP_OK;
}

static void esp_app_voltage_sensor_handler(float voltage)
{
    /* Update voltage sensor measured value */
    // esp_zb_lock_acquire(portMAX_DELAY);
    //  esp_zb_zcl_set_attribute_val(HA_ESP_LINKY_ENDPOINT,
    //      ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    // ESP_ZB_ZCL_ATTR_ELECTRONICAL_MEASUREMENT_VALUE_ID, &voltage, false);
    // esp_zb_lock_release();
}

// Init the battery voltage sensor to handle the measurement task in background
// static esp_err_t deferred_driver_init(void)
// {
//     battery_sensor_config_t temp_sensor_config =
//         BATTERY_SENSOR_CONFIG_DEFAULT(ESP_BATTERY_SENSOR_MIN_VALUE, ESP_BATTERY_SENSOR_MAX_VALUE);
//     ESP_RETURN_ON_ERROR(init_battery_sensor(&temp_sensor_config, ESP_BATTERY_SENSOR_UPDATE_INTERVAL, esp_app_voltage_sensor_handler), TAG,
//                         "Failed to initialize temperature sensor");
//     return ESP_OK;
// }

void send_data_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    ESP_LOGI(TAG, "Send data task started");
    while (1) {
        TeleinfoValues *data = teleinfo_measure();
        float current = 0.0f;
        float power = 0.0f;
        
        // Prendre le mutex avant d'accéder aux données
        if (xSemaphoreTake(teleinfo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current = (float)atoi(data->iinst);
            power = (float)atoi(data->papp);
            xSemaphoreGive(teleinfo_mutex);
            
            // N'envoyer que si les valeurs sont valides
            if (current > 0 || power > 0) {
                ESP_LOGI(TAG, "Données mises à jour via Zigbee : PAPP=%.1f VA, IINST=%.1f A", power, current);
                
                // Prendre le mutex Zigbee
                spinlock_acquire(&zigbee_lock, portMAX_DELAY);
                
                esp_zb_zcl_set_attribute_val(HA_ESP_LINKY_ENDPOINT,
                                           ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                           ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
                                           &current,
                                           false);

                esp_zb_zcl_set_attribute_val(HA_ESP_LINKY_ENDPOINT,
                                           ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE,
                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                           ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID,
                                           &power,
                                           false);
                                           
                spinlock_release(&zigbee_lock);
            }
        }
        
        // Utiliser vTaskDelayUntil pour une temporisation plus précise
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "First start");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Device rebooted");
            // After connexion working, handle measurments tasks
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            
            esp_zb_bdb_commissioning_status_t status = esp_zb_get_bdb_commissioning_status();
            ESP_LOGI(TAG, "Network status: %d", status);
            
            if (status == ESP_ZB_BDB_STATUS_NOT_ON_A_NETWORK) {
                ESP_LOGI(TAG, "Not on network, trying to join network");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else if (status == ESP_ZB_BDB_STATUS_ON_A_NETWORK) {
                ESP_LOGI(TAG, "Already on network");
                start_blinking();
            } else {
                ESP_LOGW(TAG, "Unexpected commissioning status: %d", status);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            bdb_start_top_level_commissioning_cb(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "Start network steering");
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            start_blinking();
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        ESP_LOGW(TAG, "Connection lost - attempting to rejoin network");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        start_blinking();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    // ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    uint8_t ManufacturerName[] = {5, 'S', 'h', 'i', 'f', 't'}; // warning: this is in format {length, 'string'} :
    uint8_t ModelIdentifier[] = {5, 'l', 'i', 'n', 'k', 'y'};
    uint8_t DateCode[] = {8, '2', '0', '2', '4', '0', '5', '1', '5'};
    ESP_LOGI(TAG, "ModelIdentifier: %s)", ModelIdentifier);
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, DateCode);

    // ------------------------------ Cluster IDENTIFY ------------------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);
    // ----------------------------- Cluster UART -----------------------------
    esp_zb_analog_output_cluster_cfg_t current_cluster_cfg = {
        .out_of_service = false,
        .present_value = 0.0,
        .status_flags = 0};
    esp_zb_attribute_list_t *esp_zb_electric_cluster = esp_zb_analog_output_cluster_create(&current_cluster_cfg);
    esp_zb_analog_value_cluster_cfg_t power_cluster_cfg = {
        .out_of_service = false,
        .present_value = 0.0,
        .status_flags = 0};
    esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_analog_value_cluster_create(&power_cluster_cfg);
    // // // ------------------------------ Cluster BATTERY VOLTAGE ------------------------------
    // esp_zb_analog_input_cluster_cfg_t battery_voltage = {
    //     .out_of_service = 0xFFFF,   /*!< This attribute indicates whether or not the physical input that the cluster represents is in service */
    //     .present_value = 0, /*!< This attribute indicates the current value of the input as appropriate for the cluster */
    //     .status_flags = 0,  /*!< This attribute indicates the general "health" of the analog sensor */
    // };
    // esp_zb_attribute_list_t *esp_zb_analog_input_baterry_voltage_cluster = esp_zb_analog_input_cluster_create(&battery_voltage);

    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_output_cluster(esp_zb_cluster_list, esp_zb_electric_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_value_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, 
                         esp_zb_cluster_list,
                         (esp_zb_endpoint_config_t){
                             .endpoint = HA_ESP_LINKY_ENDPOINT,
                             .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                             .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
                             .app_device_version = 0,
                         });

    // ------------------------------ Register Device ------------------------------
    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    xTaskCreate(send_data_task, "send_data_task", 8096, NULL, TASK_ZIGBEE_DATA_PRIO, NULL);
    xTaskCreate(teleinfo_generator_task, "teleinfo_gen", 8096, NULL, TASK_TELEINFO_GEN_PRIO, NULL);
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    uart_init();
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, TASK_ZIGBEE_MAIN_PRIO, NULL);
}
