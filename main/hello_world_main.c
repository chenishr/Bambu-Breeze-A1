/*
 * Bambu A1 Printer Smart Chamber Fan Controller
 * PWM Fan Speed control based on material type parsed from Bambu MQTT status.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/ledc.h"
#include "mqtt_client.h"
#include "cJSON.h"

// Wi-Fi Configurations (Please modify these macros to match your local Wi-Fi router)
#define WIFI_SSID           "ChinaNet-Pxp6"
#define WIFI_PASS           "an6ughx2"
#define WIFI_MAX_RETRY      10

// Bambu Lab A1 Printer MQTT Configurations
#define BAMBU_PRINTER_IP      "192.168.1.54"
#define BAMBU_MQTT_PORT       "8883"
#define BAMBU_USERNAME        "bblp"
#define BAMBU_ACCESS_CODE     "bb96681d"
#define BAMBU_PRINTER_SERIAL  "03919D520902254" // 请在此处替换为您的拓竹 A1 打印机序列号 (可在打印机屏幕上查看，例如: 01P00A...)
#define BAMBU_MQTT_TOPIC      "device/" BAMBU_PRINTER_SERIAL "/report"

// Fan PWM (LEDC) Configurations
#define FAN_PWM_GPIO        5
#define FAN_LEDC_TIMER      LEDC_TIMER_0
#define FAN_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL    LEDC_CHANNEL_0
#define FAN_LEDC_RES        LEDC_TIMER_8_BIT      // 8-bit resolution (0-255)
#define FAN_LEDC_FREQ       20000                 // 20kHz PWM frequency

// FreeRTOS Event Group for Wi-Fi Connection
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0

// Timer handle for delayed fan shutdown
static TimerHandle_t s_fan_timer = NULL;
#define FAN_SHUTDOWN_DELAY_MS  (5 * 60 * 1000)    // 5 minutes (300 seconds)

static const char *TAG = "BambuFanCtrl";
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static int s_wifi_retry_num = 0;
static uint32_t s_current_fan_duty = 0; // 记录当前风扇占空比

// Function Prototypes
static void init_ledc_pwm(void);
static void set_fan_duty(uint32_t duty);
static void wifi_init_sta(void);
static void mqtt_app_start(void);
static void parse_printer_status(const char *json_str);
static void fan_timer_callback(TimerHandle_t xTimer);

static void init_ledc_pwm(void)
{
    // LEDC Timer Configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = FAN_LEDC_MODE,
        .timer_num        = FAN_LEDC_TIMER,
        .duty_resolution  = FAN_LEDC_RES,
        .freq_hz          = FAN_LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // LEDC Channel Configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = FAN_LEDC_MODE,
        .channel        = FAN_LEDC_CHANNEL,
        .timer_sel      = FAN_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = FAN_PWM_GPIO,
        .duty           = 0, // initially off
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "LEDC PWM initialized on GPIO %d (Freq: %d Hz, Res: 8-bit).", FAN_PWM_GPIO, FAN_LEDC_FREQ);
}

static void set_fan_duty(uint32_t duty)
{
    if (duty > 255) {
        duty = 255;
    }
    s_current_fan_duty = duty;
    ESP_LOGI(TAG, "Changing Fan PWM duty cycle to: %lu/255 (%.1f%%)", duty, (duty * 100.0) / 255.0);
    ESP_ERROR_CHECK(ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL));
}

static void fan_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Fan shutdown timer expired. Turning fan OFF.");
    set_fan_duty(0);
}

static void parse_printer_status(const char *json_str)
{
    static char s_last_gcode_state[32] = "UNKNOWN";
    static char s_last_tray_type[32] = "UNKNOWN";

    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse printer JSON status message.");
        return;
    }

    cJSON *print_obj = cJSON_GetObjectItem(root, "print");
    if (!print_obj) {
        // Message does not contain "print" object (e.g., cmd response or other telemetry)
        cJSON_Delete(root);
        return;
    }

    cJSON *gcode_state_item = cJSON_GetObjectItem(print_obj, "gcode_state");
    if (gcode_state_item && cJSON_IsString(gcode_state_item)) {
        const char *gcode_state = gcode_state_item->valuestring;
        strncpy(s_last_gcode_state, gcode_state, sizeof(s_last_gcode_state) - 1);
        s_last_gcode_state[sizeof(s_last_gcode_state) - 1] = '\0';
        ESP_LOGI(TAG, "G-code State: %s", gcode_state);

        if (strcmp(gcode_state, "RUNNING") == 0) {
            // Stop delay timer if running
            if (s_fan_timer != NULL && xTimerIsTimerActive(s_fan_timer) == pdTRUE) {
                xTimerStop(s_fan_timer, 0);
                ESP_LOGI(TAG, "Stopped fan shutdown timer because state is RUNNING.");
            }

            const char *tray_type = NULL;
            int tray_now_val = -1;

            cJSON *tray_now_item = cJSON_GetObjectItem(print_obj, "tray_now");
            if (tray_now_item) {
                if (cJSON_IsNumber(tray_now_item)) {
                    tray_now_val = tray_now_item->valueint;
                } else if (cJSON_IsString(tray_now_item)) {
                    tray_now_val = atoi(tray_now_item->valuestring);
                }
            }

            // Read active tray material type
            if (tray_now_val == 255 || tray_now_val == 254 || tray_now_val == -1) {
                // Use external spool / virtual tray (vt_tray)
                cJSON *vt_tray = cJSON_GetObjectItem(print_obj, "vt_tray");
                if (vt_tray) {
                    cJSON *tray_type_item = cJSON_GetObjectItem(vt_tray, "tray_type");
                    if (tray_type_item && cJSON_IsString(tray_type_item)) {
                        tray_type = tray_type_item->valuestring;
                    }
                }
            } else {
                // Use AMS active slot
                cJSON *ams_obj = cJSON_GetObjectItem(print_obj, "ams");
                if (ams_obj) {
                    cJSON *ams_arr = cJSON_GetObjectItem(ams_obj, "ams");
                    if (ams_arr && cJSON_IsArray(ams_arr)) {
                        int ams_idx = tray_now_val / 4;
                        int slot_idx = tray_now_val % 4;
                        if (ams_idx < cJSON_GetArraySize(ams_arr)) {
                            cJSON *ams_item = cJSON_GetArrayItem(ams_arr, ams_idx);
                            if (ams_item) {
                                cJSON *tray_arr = cJSON_GetObjectItem(ams_item, "tray");
                                if (tray_arr && cJSON_IsArray(tray_arr)) {
                                    if (slot_idx < cJSON_GetArraySize(tray_arr)) {
                                        cJSON *tray_item = cJSON_GetArrayItem(tray_arr, slot_idx);
                                        if (tray_item) {
                                            cJSON *tray_type_item = cJSON_GetObjectItem(tray_item, "tray_type");
                                            if (tray_type_item && cJSON_IsString(tray_type_item)) {
                                                tray_type = tray_type_item->valuestring;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Fallback directly to print.ams.ams[0].tray[0].tray_type as requested
            if (tray_type == NULL) {
                cJSON *ams_obj = cJSON_GetObjectItem(print_obj, "ams");
                if (ams_obj) {
                    cJSON *ams_arr = cJSON_GetObjectItem(ams_obj, "ams");
                    if (ams_arr && cJSON_IsArray(ams_arr) && cJSON_GetArraySize(ams_arr) > 0) {
                        cJSON *ams_item = cJSON_GetArrayItem(ams_arr, 0);
                        if (ams_item) {
                            cJSON *tray_arr = cJSON_GetObjectItem(ams_item, "tray");
                            if (tray_arr && cJSON_IsArray(tray_arr) && cJSON_GetArraySize(tray_arr) > 0) {
                                cJSON *tray_item = cJSON_GetArrayItem(tray_arr, 0);
                                if (tray_item) {
                                    cJSON *tray_type_item = cJSON_GetObjectItem(tray_item, "tray_type");
                                    if (tray_type_item && cJSON_IsString(tray_type_item)) {
                                        tray_type = tray_type_item->valuestring;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Update fan duty based on tray type
            if (tray_type != NULL) {
                ESP_LOGI(TAG, "Active tray material: %s", tray_type);
                strncpy(s_last_tray_type, tray_type, sizeof(s_last_tray_type) - 1);
                s_last_tray_type[sizeof(s_last_tray_type) - 1] = '\0';

                if (strcmp(tray_type, "PLA") == 0) {
                    set_fan_duty(255); // 100% full speed
                } else if (strcmp(tray_type, "PETG") == 0) {
                    set_fan_duty(85);  // ~33% low speed
                } else if (strcmp(tray_type, "ABS") == 0 || strcmp(tray_type, "ASA") == 0) {
                    set_fan_duty(0);   // Off
                } else {
                    ESP_LOGI(TAG, "Material '%s' not explicitly handled. Setting PWM to 128 (50%%) by default.", tray_type);
                    set_fan_duty(128);
                }
            } else {
                ESP_LOGW(TAG, "Active material type could not be resolved. Keeping current fan speed.");
            }

        } else if (strcmp(gcode_state, "FINISH") == 0 || strcmp(gcode_state, "IDLE") == 0) {
            strncpy(s_last_tray_type, "NONE", sizeof(s_last_tray_type) - 1);
            if (s_fan_timer != NULL) {
                if (xTimerIsTimerActive(s_fan_timer) == pdFALSE) {
                    ESP_LOGI(TAG, "G-code state is %s. Starting 5-minute (300s) fan shutdown timer.", gcode_state);
                    xTimerStart(s_fan_timer, 0);
                } else {
                    ESP_LOGD(TAG, "Shutdown timer already running.");
                }
            }
        } else {
            ESP_LOGI(TAG, "State is %s. No action taken.", gcode_state);
        }
    }

    // 打印状态汇总日志 (包含打印机当前状态、耗材类型、以及风扇 PWM 状态)
    ESP_LOGI(TAG, "Status Summary -> G-code: %s, Material: %s | Fan PWM: %lu/255 (%.1f%%)",
             s_last_gcode_state, s_last_tray_type, s_current_fan_duty, (s_current_fan_duty * 100.0) / 255.0);

    cJSON_Delete(root);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_wifi_retry_num++;
            ESP_LOGI(TAG, "Retrying connection to AP...");
        } else {
            ESP_LOGW(TAG, "Connection to AP failed. Retrying in 5 seconds...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            s_wifi_retry_num = 0;
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta complete.");
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            {
                ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
                msg_id = esp_mqtt_client_subscribe(client, BAMBU_MQTT_TOPIC, 0);
                ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", BAMBU_MQTT_TOPIC, msg_id);

                // 发送状态查询指令 (pushall) 促使打印机主动推送包含 gcode_state 等完整信息的数据包
                char request_topic[100];
                snprintf(request_topic, sizeof(request_topic), "device/%s/request", BAMBU_PRINTER_SERIAL);
                const char *request_payload = "{\"pushing\":{\"sequence_id\":\"0\",\"command\":\"pushall\"}}";
                msg_id = esp_mqtt_client_publish(client, request_topic, request_payload, 0, 1, 0);
                ESP_LOGI(TAG, "Sent status request (pushall) to %s, msg_id=%d", request_topic, msg_id);
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA received: data_len=%d, total_len=%d", event->data_len, event->total_data_len);
            if (event->data_len > 0) {
                char *json_str = malloc(event->data_len + 1);
                if (json_str) {
                    memcpy(json_str, event->data, event->data_len);
                    json_str[event->data_len] = '\0';
                    ESP_LOGI(TAG, "Payload: %s", json_str);
                    parse_printer_status(json_str);
                    free(json_str);
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGD(TAG, "Other MQTT event id: %d", event->event_id);
            break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtts://" BAMBU_PRINTER_IP ":" BAMBU_MQTT_PORT,
            },
            .verification = {
                .skip_cert_common_name_check = true,
            },
        },
        .credentials = {
            .username = BAMBU_USERNAME,
            .client_id = "esp32_bambu_fan_ctrl",
            .authentication = {
                .password = BAMBU_ACCESS_CODE,
            },
        },
        .buffer = {
            .size = 24576, // 24KB buffer to hold the large Bambu Lab JSON payload
        },
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(TAG, "MQTTS client initialization complete.");
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LEDC PWM for fan control
    init_ledc_pwm();

    // Create FreeRTOS Software Timer for 5-minute delayed shutdown
    s_fan_timer = xTimerCreate("FanShutdownTimer", pdMS_TO_TICKS(FAN_SHUTDOWN_DELAY_MS), pdFALSE, NULL, fan_timer_callback);
    if (s_fan_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create fan shutdown timer.");
    }

    // Initialize Wi-Fi
    wifi_init_sta();

    // Wait for Wi-Fi connection
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection to be established...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi connection established. Starting MQTT client...");

    // Start MQTTS Client
    mqtt_app_start();

    // Keep the main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
