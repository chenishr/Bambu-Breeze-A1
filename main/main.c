/*
 * Bambu A1 Printer Smart Chamber Fan Controller
 * Wi-Fi AP Config Portal, Web Monitor Dashboard, and PWM Fan control.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/ledc.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_timer.h"

#define NVS_NAMESPACE "config"

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
static uint32_t s_current_fan_duty = 0; // Current fan duty cycle (0-255)
static bool s_manual_fan_control = false; // Ignore A1 automatic fan speed adjustments if true

// Application configuration struct stored in NVS
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char bambu_ip[32];
    char bambu_pass[32];
    char bambu_serial[32];
} app_config_t;

static app_config_t s_config;
static bool s_config_loaded = false;

// Printer status structure for dashboard JSON serialization
typedef struct {
    char gcode_state[32];
    char subtask_name[128];
    int mc_percent;
    int mc_remaining_time;
    int layer_num;
    int total_layer_num;
    float nozzle_temper;
    float nozzle_target_temper;
    float bed_temper;
    float bed_target_temper;
    char cooling_fan_speed[16];
    char wifi_signal[16];
    int tray_now;
    struct {
        char type[16];
        char color[16]; // HEX color, e.g. "F6DA5AFF"
        int remain;
    } ams_trays[4];
    struct {
        char type[16];
        char color[16];
    } vt_tray;
    
    // ESP32 System metrics
    int esp_fan_duty;
    bool printer_online;
    int spd_lvl;
} printer_status_t;

static printer_status_t s_printer_status;
static SemaphoreHandle_t s_status_mutex = NULL;
static esp_timer_handle_t s_wifi_reconnect_timer = NULL;

// Binary data references to embedded index.html
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

// Function Prototypes
static void init_ledc_pwm(void);
static void set_fan_duty(uint32_t duty);
static void wifi_init(void);
static void mqtt_app_start(void);
static void parse_printer_status(const char *json_str);
static void fan_timer_callback(TimerHandle_t xTimer);
static void init_printer_status(void);
static httpd_handle_t start_webserver(void);
static void update_fan_from_current_status(void);

// NVS Helper Functions
static esp_err_t load_app_config(app_config_t *cfg)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        return err;
    }
    size_t required_size;
    
    required_size = sizeof(cfg->wifi_ssid);
    err = nvs_get_str(my_handle, "wifi_ssid", cfg->wifi_ssid, &required_size);
    if (err == ESP_OK) {
        required_size = sizeof(cfg->wifi_pass);
        nvs_get_str(my_handle, "wifi_pass", cfg->wifi_pass, &required_size);
        required_size = sizeof(cfg->bambu_ip);
        nvs_get_str(my_handle, "bambu_ip", cfg->bambu_ip, &required_size);
        required_size = sizeof(cfg->bambu_pass);
        nvs_get_str(my_handle, "bambu_pass", cfg->bambu_pass, &required_size);
        required_size = sizeof(cfg->bambu_serial);
        nvs_get_str(my_handle, "bambu_serial", cfg->bambu_serial, &required_size);
    }
    nvs_close(my_handle);
    return err;
}

static esp_err_t save_app_config(const app_config_t *cfg)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(my_handle, "wifi_ssid", cfg->wifi_ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "wifi_pass", cfg->wifi_pass);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "bambu_ip", cfg->bambu_ip);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "bambu_pass", cfg->bambu_pass);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(my_handle, "bambu_serial", cfg->bambu_serial);
    }
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }
    nvs_close(my_handle);
    return err;
}

// LEDC PWM Fan Controller Functions
static void init_ledc_pwm(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = FAN_LEDC_MODE,
        .timer_num        = FAN_LEDC_TIMER,
        .duty_resolution  = FAN_LEDC_RES,
        .freq_hz          = FAN_LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = FAN_LEDC_MODE,
        .channel        = FAN_LEDC_CHANNEL,
        .timer_sel      = FAN_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = FAN_PWM_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "LEDC PWM initialized on GPIO %d (Freq: %d Hz).", FAN_PWM_GPIO, FAN_LEDC_FREQ);
}

static void set_fan_duty(uint32_t duty)
{
    if (duty > 255) {
        duty = 255;
    }
    s_current_fan_duty = duty;
    if (s_status_mutex != NULL && xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_printer_status.esp_fan_duty = duty;
        xSemaphoreGive(s_status_mutex);
    }
    ESP_LOGI(TAG, "Changing Fan PWM duty cycle to: %lu/255 (%.1f%%)", duty, (duty * 100.0) / 255.0);
    ESP_ERROR_CHECK(ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL));
}

static void fan_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Fan shutdown timer expired. Turning fan OFF.");
    set_fan_duty(0);
}

// Printer Status parsing (from Bambu Lab MQTT report)
static void init_printer_status(void)
{
    memset(&s_printer_status, 0, sizeof(s_printer_status));
    strcpy(s_printer_status.gcode_state, "OFFLINE");
    strcpy(s_printer_status.subtask_name, "未连接到打印机状态数据...");
    s_printer_status.tray_now = -1;
    s_printer_status.printer_online = false;
    for (int i = 0; i < 4; i++) {
        strcpy(s_printer_status.ams_trays[i].type, "Empty");
        strcpy(s_printer_status.ams_trays[i].color, "FFFFFFFF");
        s_printer_status.ams_trays[i].remain = -1;
    }
    strcpy(s_printer_status.vt_tray.type, "Empty");
    strcpy(s_printer_status.vt_tray.color, "FFFFFFFF");
}

static void parse_printer_status(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        return;
    }

    cJSON *print_obj = cJSON_GetObjectItem(root, "print");
    if (!print_obj) {
        cJSON_Delete(root);
        return;
    }

    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_printer_status.printer_online = true;

        cJSON *item;

        item = cJSON_GetObjectItem(print_obj, "gcode_state");
        if (item && cJSON_IsString(item)) {
            strncpy(s_printer_status.gcode_state, item->valuestring, sizeof(s_printer_status.gcode_state) - 1);
        }

        item = cJSON_GetObjectItem(print_obj, "subtask_name");
        if (item && cJSON_IsString(item)) {
            strncpy(s_printer_status.subtask_name, item->valuestring, sizeof(s_printer_status.subtask_name) - 1);
        }

        item = cJSON_GetObjectItem(print_obj, "mc_percent");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.mc_percent = item->valueint;
        }

        item = cJSON_GetObjectItem(print_obj, "mc_remaining_time");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.mc_remaining_time = item->valueint;
        }

        item = cJSON_GetObjectItem(print_obj, "layer_num");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.layer_num = item->valueint;
        }

        item = cJSON_GetObjectItem(print_obj, "total_layer_num");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.total_layer_num = item->valueint;
        }

        item = cJSON_GetObjectItem(print_obj, "nozzle_temper");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.nozzle_temper = item->valuedouble;
        }

        item = cJSON_GetObjectItem(print_obj, "nozzle_target_temper");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.nozzle_target_temper = item->valuedouble;
        }

        item = cJSON_GetObjectItem(print_obj, "bed_temper");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.bed_temper = item->valuedouble;
        }

        item = cJSON_GetObjectItem(print_obj, "bed_target_temper");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.bed_target_temper = item->valuedouble;
        }

        item = cJSON_GetObjectItem(print_obj, "cooling_fan_speed");
        if (item) {
            if (cJSON_IsString(item)) {
                strncpy(s_printer_status.cooling_fan_speed, item->valuestring, sizeof(s_printer_status.cooling_fan_speed) - 1);
            } else if (cJSON_IsNumber(item)) {
                snprintf(s_printer_status.cooling_fan_speed, sizeof(s_printer_status.cooling_fan_speed), "%d", item->valueint);
            }
        }

        item = cJSON_GetObjectItem(print_obj, "wifi_signal");
        if (item && cJSON_IsString(item)) {
            strncpy(s_printer_status.wifi_signal, item->valuestring, sizeof(s_printer_status.wifi_signal) - 1);
        }

        item = cJSON_GetObjectItem(print_obj, "spd_lvl");
        if (item && cJSON_IsNumber(item)) {
            s_printer_status.spd_lvl = item->valueint;
        }

        int tray_now_val = -1;
        item = cJSON_GetObjectItem(print_obj, "tray_now");
        if (item) {
            if (cJSON_IsNumber(item)) {
                tray_now_val = item->valueint;
            } else if (cJSON_IsString(item)) {
                tray_now_val = atoi(item->valuestring);
            }
        }
        s_printer_status.tray_now = tray_now_val;

        // Parse AMS trays
        cJSON *ams_obj = cJSON_GetObjectItem(print_obj, "ams");
        if (ams_obj) {
            cJSON *ams_arr = cJSON_GetObjectItem(ams_obj, "ams");
            if (ams_arr && cJSON_IsArray(ams_arr) && cJSON_GetArraySize(ams_arr) > 0) {
                cJSON *ams_item = cJSON_GetArrayItem(ams_arr, 0);
                if (ams_item) {
                    cJSON *tray_arr = cJSON_GetObjectItem(ams_item, "tray");
                    if (tray_arr && cJSON_IsArray(tray_arr)) {
                        int count = cJSON_GetArraySize(tray_arr);
                        for (int i = 0; i < 4 && i < count; i++) {
                            cJSON *t_item = cJSON_GetArrayItem(tray_arr, i);
                            if (t_item) {
                                cJSON *type_item = cJSON_GetObjectItem(t_item, "tray_type");
                                if (type_item && cJSON_IsString(type_item)) {
                                    strncpy(s_printer_status.ams_trays[i].type, type_item->valuestring, sizeof(s_printer_status.ams_trays[i].type) - 1);
                                }
                                cJSON *color_item = cJSON_GetObjectItem(t_item, "tray_color");
                                if (color_item && cJSON_IsString(color_item)) {
                                    strncpy(s_printer_status.ams_trays[i].color, color_item->valuestring, sizeof(s_printer_status.ams_trays[i].color) - 1);
                                }
                                cJSON *rem_item = cJSON_GetObjectItem(t_item, "remain");
                                if (rem_item && cJSON_IsNumber(rem_item)) {
                                    s_printer_status.ams_trays[i].remain = rem_item->valueint;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Parse vt_tray
        cJSON *vt_tray = cJSON_GetObjectItem(print_obj, "vt_tray");
        if (vt_tray) {
            cJSON *type_item = cJSON_GetObjectItem(vt_tray, "tray_type");
            if (type_item && cJSON_IsString(type_item)) {
                strncpy(s_printer_status.vt_tray.type, type_item->valuestring, sizeof(s_printer_status.vt_tray.type) - 1);
            }
            cJSON *color_item = cJSON_GetObjectItem(vt_tray, "tray_color");
            if (color_item && cJSON_IsString(color_item)) {
                strncpy(s_printer_status.vt_tray.color, color_item->valuestring, sizeof(s_printer_status.vt_tray.color) - 1);
            }
        }



        // Update fan duty from parsed status (respects manual override inside the function)
        update_fan_from_current_status();

        xSemaphoreGive(s_status_mutex);
    }
    
    cJSON_Delete(root);
}

static void update_fan_from_current_status(void)
{
    if (s_manual_fan_control) {
        return;
    }

    const char *active_material = NULL;
    int tray_now_val = s_printer_status.tray_now;
    if (tray_now_val == 255 || tray_now_val == 254 || tray_now_val == -1) {
        if (s_printer_status.vt_tray.type[0] != '\0' && strcmp(s_printer_status.vt_tray.type, "Empty") != 0) {
            active_material = s_printer_status.vt_tray.type;
        }
    } else if (tray_now_val >= 0 && tray_now_val < 4) {
        if (s_printer_status.ams_trays[tray_now_val].type[0] != '\0' && strcmp(s_printer_status.ams_trays[tray_now_val].type, "Empty") != 0) {
            active_material = s_printer_status.ams_trays[tray_now_val].type;
        }
    }

    if (strcmp(s_printer_status.gcode_state, "RUNNING") == 0) {
        if (s_fan_timer != NULL && xTimerIsTimerActive(s_fan_timer) == pdTRUE) {
            xTimerStop(s_fan_timer, 0);
        }
        if (active_material != NULL) {
            if (strcmp(active_material, "PLA") == 0) {
                set_fan_duty(255);
            } else if (strcmp(active_material, "PETG") == 0) {
                set_fan_duty(85);
            } else if (strcmp(active_material, "ABS") == 0 || strcmp(active_material, "ASA") == 0) {
                set_fan_duty(0);
            } else {
                set_fan_duty(128);
            }
        }
    } else if (strcmp(s_printer_status.gcode_state, "FINISH") == 0 || strcmp(s_printer_status.gcode_state, "IDLE") == 0) {
        if (s_fan_timer != NULL && xTimerIsTimerActive(s_fan_timer) == pdFALSE) {
            xTimerStart(s_fan_timer, 0);
        }
    }
}

// Wi-Fi Configuration Portal & Event Handlers
static void wifi_reconnect_timer_callback(void* arg)
{
    ESP_LOGI(TAG, "Reconnecting to Wi-Fi STA...");
    esp_wifi_connect();
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi STA Disconnected.");
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        if (s_status_mutex != NULL && xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_printer_status.printer_online = false;
            xSemaphoreGive(s_status_mutex);
        }

        // Retry connection in 5 seconds without blocking the event queue
        if (s_config.wifi_ssid[0] != '\0') {
            if (s_wifi_reconnect_timer == NULL) {
                esp_timer_create_args_t timer_args = {
                    .callback = &wifi_reconnect_timer_callback,
                    .name = "wifi_reconnect"
                };
                esp_timer_create(&timer_args, &s_wifi_reconnect_timer);
            }
            esp_timer_start_once(s_wifi_reconnect_timer, 5000000); // 5 seconds in microseconds
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Wi-Fi STA Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // Start MQTT client now that network is available
        mqtt_app_start();
    }
}

static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Config AP Hotspot (always visible to configure or monitor directly)
    wifi_config_t ap_config = {
        .ap = {
            .ssid = "Bambu-Breeze-AP",
            .ssid_len = strlen("Bambu-Breeze-AP"),
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };

    // Config STA Connection if SSID is stored in NVS
    wifi_config_t sta_config = {0};
    if (s_config.wifi_ssid[0] != '\0') {
        strncpy((char*)sta_config.sta.ssid, s_config.wifi_ssid, sizeof(sta_config.sta.ssid));
        strncpy((char*)sta_config.sta.password, s_config.wifi_pass, sizeof(sta_config.sta.password));
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_LOGI(TAG, "Starting Wi-Fi in AP+STA mode. SSID: %s", s_config.wifi_ssid);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
        ESP_LOGI(TAG, "Starting Wi-Fi in AP mode (Configuration Portal open). SSID: Bambu-Breeze-AP");
    }

    ESP_ERROR_CHECK(esp_wifi_start());
}

// MQTT client operations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            {
                ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
                char topic_buf[64];
                snprintf(topic_buf, sizeof(topic_buf), "device/%s/report", s_config.bambu_serial);
                msg_id = esp_mqtt_client_subscribe(client, topic_buf, 0);
                ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", topic_buf, msg_id);

                char request_topic[64];
                snprintf(request_topic, sizeof(request_topic), "device/%s/request", s_config.bambu_serial);
                const char *request_payload = "{\"pushing\":{\"sequence_id\":\"0\",\"command\":\"pushall\"}}";
                msg_id = esp_mqtt_client_publish(client, request_topic, request_payload, 0, 1, 0);
                ESP_LOGI(TAG, "Sent status request (pushall) to %s, msg_id=%d", request_topic, msg_id);
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_printer_status.printer_online = false;
                xSemaphoreGive(s_status_mutex);
            }
            break;
        case MQTT_EVENT_DATA:
            if (event->data_len > 0) {
                char *json_str = malloc(event->data_len + 1);
                if (json_str) {
                    memcpy(json_str, event->data, event->data_len);
                    json_str[event->data_len] = '\0';
                    parse_printer_status(json_str);
                    free(json_str);
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            break;
    }
}

static void mqtt_app_start(void)
{
    if (s_mqtt_client != NULL) {
        return;
    }
    if (s_config.bambu_ip[0] == '\0' || s_config.bambu_serial[0] == '\0') {
        ESP_LOGW(TAG, "Bambu printer configuration is incomplete. MQTT client not started.");
        return;
    }

    char uri_buf[64];
    snprintf(uri_buf, sizeof(uri_buf), "mqtts://%s:8883", s_config.bambu_ip);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = uri_buf,
            },
            .verification = {
                .skip_cert_common_name_check = true,
            },
        },
        .credentials = {
            .username = "bblp",
            .client_id = "esp32_bambu_fan_ctrl",
            .authentication = {
                .password = s_config.bambu_pass,
            },
        },
        .buffer = {
            .size = 24576,
        },
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(TAG, "MQTT client initialization complete.");
}

// HTTP Server and API Handling
static esp_err_t get_root_handler(httpd_req_t *req)
{
    size_t size = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, index_html_start, size);
    return ESP_OK;
}

static esp_err_t get_status_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mutex timeout");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "gcode_state", s_printer_status.gcode_state);
    cJSON_AddStringToObject(root, "subtask_name", s_printer_status.subtask_name);
    cJSON_AddNumberToObject(root, "mc_percent", s_printer_status.mc_percent);
    cJSON_AddNumberToObject(root, "mc_remaining_time", s_printer_status.mc_remaining_time);
    cJSON_AddNumberToObject(root, "layer_num", s_printer_status.layer_num);
    cJSON_AddNumberToObject(root, "total_layer_num", s_printer_status.total_layer_num);
    cJSON_AddNumberToObject(root, "nozzle_temper", s_printer_status.nozzle_temper);
    cJSON_AddNumberToObject(root, "nozzle_target_temper", s_printer_status.nozzle_target_temper);
    cJSON_AddNumberToObject(root, "bed_temper", s_printer_status.bed_temper);
    cJSON_AddNumberToObject(root, "bed_target_temper", s_printer_status.bed_target_temper);
    cJSON_AddStringToObject(root, "cooling_fan_speed", s_printer_status.cooling_fan_speed);
    cJSON_AddStringToObject(root, "wifi_signal", s_printer_status.wifi_signal);
    cJSON_AddNumberToObject(root, "tray_now", s_printer_status.tray_now);
    cJSON_AddNumberToObject(root, "spd_lvl", s_printer_status.spd_lvl);

    // AMS Trays list
    cJSON *ams_arr = cJSON_CreateArray();
    for (int i = 0; i < 4; i++) {
        cJSON *tray = cJSON_CreateObject();
        cJSON_AddStringToObject(tray, "type", s_printer_status.ams_trays[i].type);
        cJSON_AddStringToObject(tray, "color", s_printer_status.ams_trays[i].color);
        cJSON_AddNumberToObject(tray, "remain", s_printer_status.ams_trays[i].remain);
        cJSON_AddItemToArray(ams_arr, tray);
    }
    cJSON_AddItemToObject(root, "ams_trays", ams_arr);

    // Virtual Tray
    cJSON *vt = cJSON_CreateObject();
    cJSON_AddStringToObject(vt, "type", s_printer_status.vt_tray.type);
    cJSON_AddStringToObject(vt, "color", s_printer_status.vt_tray.color);
    cJSON_AddItemToObject(root, "vt_tray", vt);

    // ESP32 system metrics
    cJSON_AddNumberToObject(root, "esp_fan_duty", s_printer_status.esp_fan_duty);
    cJSON_AddBoolToObject(root, "esp_fan_manual", s_manual_fan_control);
    cJSON_AddBoolToObject(root, "printer_online", s_printer_status.printer_online);

    // Wi-Fi Connection
    bool wifi_connected = (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
    cJSON_AddBoolToObject(root, "esp_wifi_connected", wifi_connected);

    // Config references
    cJSON_AddStringToObject(root, "config_wifi_ssid", s_config.wifi_ssid);
    cJSON_AddStringToObject(root, "config_bambu_ip", s_config.bambu_ip);
    cJSON_AddStringToObject(root, "config_bambu_serial", s_config.bambu_serial);
    cJSON_AddBoolToObject(root, "config_wifi_pass_set", s_config.wifi_pass[0] != '\0');
    cJSON_AddBoolToObject(root, "config_bambu_pass_set", s_config.bambu_pass[0] != '\0');

    xSemaphoreGive(s_status_mutex);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    free(json_str);

    return ESP_OK;
}

static void reboot_timer_callback(void* arg)
{
    esp_restart();
}

static esp_err_t post_config_handler(httpd_req_t *req)
{
    char buf[256];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload too large");
        return ESP_FAIL;
    }

    int cur = 0;
    while (remaining > 0) {
        ret = httpd_req_recv(req, buf + cur, remaining);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        cur += ret;
        remaining -= ret;
    }
    buf[cur] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    app_config_t temp_config = s_config;
    cJSON *item;

    item = cJSON_GetObjectItem(root, "wifi_ssid");
    if (item && cJSON_IsString(item) && strlen(item->valuestring) > 0) {
        strncpy(temp_config.wifi_ssid, item->valuestring, sizeof(temp_config.wifi_ssid) - 1);
    }
    
    item = cJSON_GetObjectItem(root, "wifi_pass");
    if (item && cJSON_IsString(item) && strlen(item->valuestring) > 0) {
        strncpy(temp_config.wifi_pass, item->valuestring, sizeof(temp_config.wifi_pass) - 1);
    }
    
    item = cJSON_GetObjectItem(root, "bambu_ip");
    if (item && cJSON_IsString(item) && strlen(item->valuestring) > 0) {
        strncpy(temp_config.bambu_ip, item->valuestring, sizeof(temp_config.bambu_ip) - 1);
    }
    
    item = cJSON_GetObjectItem(root, "bambu_pass");
    if (item && cJSON_IsString(item) && strlen(item->valuestring) > 0) {
        strncpy(temp_config.bambu_pass, item->valuestring, sizeof(temp_config.bambu_pass) - 1);
    }
    
    item = cJSON_GetObjectItem(root, "bambu_serial");
    if (item && cJSON_IsString(item) && strlen(item->valuestring) > 0) {
        strncpy(temp_config.bambu_serial, item->valuestring, sizeof(temp_config.bambu_serial) - 1);
    }

    cJSON_Delete(root);

    esp_err_t err = save_app_config(&temp_config);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config");
        return ESP_FAIL;
    }

    s_config = temp_config;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");

    ESP_LOGI(TAG, "Configuration updated. Restarting system in 2 seconds...");
    esp_timer_create_args_t timer_args = {
        .callback = &reboot_timer_callback,
        .name = "reboot"
    };
    esp_timer_handle_t reboot_timer;
    esp_timer_create(&timer_args, &reboot_timer);
    esp_timer_start_once(reboot_timer, 2000000); // 2 seconds

    return ESP_OK;
}

static esp_err_t post_control_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (cmd_item && cJSON_IsString(cmd_item)) {
        const char *cmd = cmd_item->valuestring;
        ESP_LOGI(TAG, "Post control trigger: %s", cmd);

        if (strcmp(cmd, "fan_mode") == 0) {
            cJSON *mode_item = cJSON_GetObjectItem(root, "mode");
            if (mode_item && cJSON_IsString(mode_item)) {
                const char *mode = mode_item->valuestring;
                if (strcmp(mode, "auto") == 0) {
                    s_manual_fan_control = false;
                    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        update_fan_from_current_status();
                        xSemaphoreGive(s_status_mutex);
                    }
                } else if (strcmp(mode, "manual") == 0) {
                    cJSON *duty_item = cJSON_GetObjectItem(root, "duty");
                    if (duty_item && cJSON_IsNumber(duty_item)) {
                        s_manual_fan_control = true;
                        set_fan_duty(duty_item->valueint);
                    }
                }
            }
            cJSON_Delete(root);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
            return ESP_OK;
        }

        if (strcmp(cmd, "disconnect_wifi") == 0) {
            memset(s_config.wifi_ssid, 0, sizeof(s_config.wifi_ssid));
            memset(s_config.wifi_pass, 0, sizeof(s_config.wifi_pass));
            save_app_config(&s_config);
            
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
            
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            cJSON_Delete(root);
            return ESP_OK;
        }

        if (s_mqtt_client != NULL && s_config.bambu_serial[0] != '\0') {
            char req_topic[100];
            snprintf(req_topic, sizeof(req_topic), "device/%s/request", s_config.bambu_serial);
            char payload[256];

            if (strcmp(cmd, "pause") == 0) {
                bool is_paused = false;
                if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    is_paused = (strcmp(s_printer_status.gcode_state, "PAUSE") == 0);
                    xSemaphoreGive(s_status_mutex);
                }
                if (is_paused) {
                    snprintf(payload, sizeof(payload), "{\"print\":{\"sequence_id\":\"0\",\"command\":\"resume\"}}");
                } else {
                    snprintf(payload, sizeof(payload), "{\"print\":{\"sequence_id\":\"0\",\"command\":\"pause\"}}");
                }
                esp_mqtt_client_publish(s_mqtt_client, req_topic, payload, 0, 1, 0);
            } else if (strcmp(cmd, "stop") == 0) {
                snprintf(payload, sizeof(payload), "{\"print\":{\"sequence_id\":\"0\",\"command\":\"stop\"}}");
                esp_mqtt_client_publish(s_mqtt_client, req_topic, payload, 0, 1, 0);
            } else if (strcmp(cmd, "speed") == 0) {
                cJSON *param_item = cJSON_GetObjectItem(root, "param");
                if (param_item && cJSON_IsString(param_item)) {
                    snprintf(payload, sizeof(payload), "{\"print\":{\"sequence_id\":\"0\",\"command\":\"print_speed\",\"param\":\"%s\"}}", param_item->valuestring);
                    esp_mqtt_client_publish(s_mqtt_client, req_topic, payload, 0, 1, 0);
                }
            }
        } else {
            ESP_LOGW(TAG, "Cannot dispatch control - MQTT not active.");
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "MQTT inactive");
            return ESP_FAIL;
        }
    }

    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;
    config.stack_size = 8192; // 8KB stack for robust serialization task execution

    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = get_root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t status_uri = {
            .uri       = "/api/status",
            .method    = HTTP_GET,
            .handler   = get_status_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t config_uri = {
            .uri       = "/api/config",
            .method    = HTTP_POST,
            .handler   = post_config_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &config_uri);

        httpd_uri_t control_uri = {
            .uri       = "/api/control",
            .method    = HTTP_POST,
            .handler   = post_control_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &control_uri);

        return server;
    }
    ESP_LOGE(TAG, "Failed to initialize Web Server!");
    return NULL;
}

// Application Entry Point
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize mutex for printer status synchronization
    s_status_mutex = xSemaphoreCreateMutex();
    if (s_status_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create status mutex. System halted.");
        return;
    }

    // Initialize global status struct
    init_printer_status();

    // Load configurations from NVS
    esp_err_t config_err = load_app_config(&s_config);
    if (config_err != ESP_OK) {
        ESP_LOGI(TAG, "App configuration not found in NVS. Booting into AP configure mode.");
        memset(&s_config, 0, sizeof(s_config));
    } else {
        s_config_loaded = true;
        ESP_LOGI(TAG, "Config loaded -> Wi-Fi SSID: '%s', Printer IP: '%s'", s_config.wifi_ssid, s_config.bambu_ip);
    }

    // Initialize LEDC PWM for fan control
    init_ledc_pwm();

    // Create FreeRTOS Software Timer for 5-minute delayed shutdown
    s_fan_timer = xTimerCreate("FanShutdownTimer", pdMS_TO_TICKS(FAN_SHUTDOWN_DELAY_MS), pdFALSE, NULL, fan_timer_callback);
    if (s_fan_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create fan shutdown timer.");
    }

    // Initialize and Start Wi-Fi (AP or AP_STA depending on config presence)
    wifi_init();

    // Start Web configuration portal and status server
    start_webserver();

    // Keep the main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
