#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "oled.h"
#include "wifi_connect.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"

#define FOOD_DISPENSER_DIR_PIN 21
#define FOOD_DISPENSER_STEP_PIN 23
#define FOOD_DISPENSER_SLEEP_PIN GPIO_NUM_19
#define FOOD_DISPENSER_MANUAL_PIN GPIO_NUM_18
#define FOOD_DISPENSER_PIN_SEL  ((1ULL<<FOOD_DISPENSER_DIR_PIN) | (1ULL<<FOOD_DISPENSER_STEP_PIN) | (1ULL<<FOOD_DISPENSER_SLEEP_PIN))
#define ESP_INTR_FLAG_DEFAULT 0

#define WEIGHT_SENSOR_ZERO_PIN GPIO_NUM_5
#define WEIGHT_SENSOR_CLK GPIO_NUM_33
#define WEIGHT_SENSOR_DT GPIO_NUM_25
#define WEIGHT_SENSOR_CALIBRATE_PIN GPIO_NUM_32
#define BUTTON_CHECK_PERIOD 500

#define OLED_I2C_MASTER_FREQ_HZ 400000  
#define OLED_CLK GPIO_NUM_27
#define OLED_SDA GPIO_NUM_26
#define OLED_PORT_NUM I2C_NUM_0

#define WIFI_PASS "S@mpleN3twork"
#define WIFI_SSID "Acme_Corp"
#define INDEX_HTML_PATH "/spiffs/index.html"

typedef enum {
    HX711_GAIN_128,
	HX711_GAIN_32,
	HX711_GAIN_64
} HX711_GAIN_T;

static TickType_t weight_sensor_zero_offset_next;
static TickType_t weight_sensor_calibrate_next;
static TickType_t food_dispenser_manual_next;
static TaskHandle_t zero_task_handle;
static TaskHandle_t dispense_food_handle;
static TaskHandle_t calibrate_task_handle;
static TaskHandle_t get_weight_readings_task_handle;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t calibration_weight = 130; // 17HS4023 stepper motor weights 130 g 
static uint8_t weight = 0;
static tcpip_adapter_ip_info_t ip_info;
static uint32_t zero = 0;
static int32_t offset = 0;
static float scale = 0;
static float max_weight = 50;
static float min_weight = 5;
static HX711_GAIN_T default_gain = HX711_GAIN_128;
static httpd_handle_t server = NULL;
static int web_socket_descriptor = -1;
static char index_html[4096];
static bool calibration_complete = false;

const char * TAG = "food_dispenser";

static void weight_sensor_init(void);

static esp_err_t handle_socket_opened(httpd_handle_t hd, int sockfd) {
    web_socket_descriptor = sockfd;
    return ESP_OK;
}

static void handle_socket_closed(httpd_handle_t hd, int sockfd) {
    if (sockfd == web_socket_descriptor) {
        web_socket_descriptor = -1;
    }
}

static void send_async(void *arg) {
    if(web_socket_descriptor < 0) {
        return;
    }

    char buffer[512];
    memset(buffer, 0, sizeof(buffer));
    sprintf(buffer, 
        "{\"weight\": %d, \"ip_address\": \""IPSTR"\", \"calibration_weight\": %d, \"max_weight\": %f, \"min_weight\": %f}",
        weight, IP2STR(&ip_info.ip), calibration_weight, max_weight, min_weight);
    
    httpd_ws_frame_t ws_packet;
    memset(&ws_packet, 0, sizeof(httpd_ws_frame_t));

    ws_packet.payload = (uint8_t*)buffer;
    ws_packet.len = strlen(buffer);
    ws_packet.type = HTTPD_WS_TYPE_TEXT;
    httpd_ws_send_frame_async(server, web_socket_descriptor, &ws_packet);
}

static esp_err_t handle_ws_req(httpd_req_t *request) {
    httpd_ws_frame_t ws_packet;
    uint8_t buffer[16];

    memset(&ws_packet, 0, sizeof(httpd_ws_frame_t));
    ws_packet.payload = buffer;
    ws_packet.type = HTTPD_WS_TYPE_BINARY;

    httpd_queue_work(server, send_async, NULL);
    return ESP_OK;
}

static esp_err_t handle_dispense_food_req(httpd_req_t *request) {
    xTaskNotifyGive(dispense_food_handle);
    httpd_resp_set_status(request, HTTPD_200);
    return httpd_resp_send(request, "", 0);
}

static esp_err_t handle_zero_offset_req(httpd_req_t *request) {
    xTaskNotifyGive(zero_task_handle);
    httpd_resp_set_status(request, HTTPD_200);
    return httpd_resp_send(request, "", 0);
}

static esp_err_t handle_calibrate_req(httpd_req_t *request) {
    xTaskNotifyGive(calibrate_task_handle);
    httpd_resp_set_status(request, HTTPD_200);
    return httpd_resp_send(request, "", 0);
}

static esp_err_t handle_http_get(httpd_req_t *request) {
    if (index_html[0] == 0) {
        httpd_resp_set_status(request, HTTPD_500);
        return httpd_resp_send(request, "There is no index.html file to read from this device.", HTTPD_RESP_USE_STRLEN);
    }

    return httpd_resp_send(request, index_html, HTTPD_RESP_USE_STRLEN);
}

static void web_server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.open_fn = handle_socket_opened;
    config.close_fn = handle_socket_closed;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_get = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = handle_http_get,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &uri_get);

        httpd_uri_t uri_ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = handle_ws_req,
            .user_ctx = NULL,
            .is_websocket = true
        };

        httpd_register_uri_handler(server, &uri_ws);

        httpd_uri_t dispense_food_uri = {
            .uri = "/dispense_food",
            .method = HTTP_POST,
            .handler = handle_dispense_food_req,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &dispense_food_uri);

        httpd_uri_t zero_offset_uri = {
            .uri = "/zero_offset",
            .method = HTTP_POST,
            .handler = handle_zero_offset_req,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &zero_offset_uri);

        httpd_uri_t calibrate_uri = {
            .uri = "/calibrate",
            .method = HTTP_POST,
            .handler = handle_calibrate_req,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &calibrate_uri);
    }
}

static void web_server_init(void) {
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&config));
    memset((void*)index_html, 0, sizeof(index_html));
    struct stat st;

    if (stat(INDEX_HTML_PATH, &st)) {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen(INDEX_HTML_PATH, "r");
    fread(index_html, st.st_size, 1, fp);

    fclose(fp);
}

static void wifi_connect_callback(void) {
    ESP_LOGI(TAG, "Wifi connected");

    char buffer[30] = {0};

    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
    sprintf(buffer, "IP Address:" IPSTR, IP2STR(&ip_info.ip));
    oled_setXY(0x00, 0x7F, 0x04, 0x02);
    oled_string(buffer);
    web_server_init();
    weight_sensor_init();
    web_server_start();
}

static void wifi_failed_callback(void) {
    ESP_LOGE(TAG, "Wifi intialization failed");
}

static void wifi_init(void) {
    connect_wifi_params_t params =  {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
        .on_connected = wifi_connect_callback,
        .on_failed = wifi_failed_callback
    };

    wifi_connect(params);
}

static uint32_t weight_sensor_get_raw_data(HX711_GAIN_T gain) {
    uint32_t raw_data = 0;
    uint8_t gain_clock_cycles = 0;

    taskENTER_CRITICAL(&spinlock);
    gpio_set_level(WEIGHT_SENSOR_CLK, false);
    ets_delay_us(1);

    while (gpio_get_level(WEIGHT_SENSOR_DT));
    
    for (uint8_t index = 0; index < 24; index++) {
        gpio_set_level(WEIGHT_SENSOR_CLK, true);
        ets_delay_us(1);
        raw_data = raw_data << 1;
        gpio_set_level(WEIGHT_SENSOR_CLK, false);
        ets_delay_us(1);
        
        if (gpio_get_level(WEIGHT_SENSOR_DT)) {
            raw_data++;
        }
    }
    
    switch (gain) {
        case HX711_GAIN_128:
			gain_clock_cycles = 1;
			break;
			
		case HX711_GAIN_32:
			gain_clock_cycles = 2;
			break;
			
		case HX711_GAIN_64:
			gain_clock_cycles = 3;
			break;
    }

    for (uint8_t index = 0; index < gain_clock_cycles; index++) {
        gpio_set_level(WEIGHT_SENSOR_CLK, true);
        ets_delay_us(1);
        gpio_set_level(WEIGHT_SENSOR_CLK, false);
        ets_delay_us(1);
    }

    taskEXIT_CRITICAL(&spinlock);

    raw_data = raw_data ^ 0x800000;	
    return raw_data;
}

static uint32_t weight_sensor_get_average_raw_data(uint8_t times, HX711_GAIN_T gain) {
    uint32_t raw_data = 0;

    for (uint8_t index = 0; index < times; index++) {
        raw_data += weight_sensor_get_raw_data(gain);
    }    

    return raw_data / times;
}

static void weight_sensor_zero_offset_task(void *arg) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Setting zero offset for weight sensor...");

        zero = weight_sensor_get_average_raw_data(10, default_gain);

        ESP_LOGI(TAG, "Zero value for weight sensor: %d...", zero);
    }         
}

static float weight_sensor_get_value(void) {
    int32_t data = (weight_sensor_get_average_raw_data(3, default_gain) - zero);
	return ((float)(data)) * scale;
}

static void weight_sensor_readings_task(void *arg) {
    char buffer[25] = {0};

    for(;;) {
        if (calibration_complete) {
            weight = (int32_t)weight_sensor_get_value();
    
            sprintf(buffer, "Weight %d g          ", weight);
            printf("Weight: %d\n", weight);
            oled_setXY(0x00, 0x7F, 0x02, 0x02);
            oled_string(buffer);
            httpd_queue_work(server, send_async, NULL);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            printf("Waiting for calibration...\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

static void weight_sensor_trigger_low_food_task(void *arg) {
    for(;;) {
        ESP_LOGI(TAG, "Checking if there is enough food...");
        if (weight < min_weight && calibration_complete) {
            xTaskNotifyGive(dispense_food_handle);
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

static void weight_sensor_calibrate_task(void *arg) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    
        
        ESP_LOGI(TAG, "Calibrating...");
        uint32_t raw_data = weight_sensor_get_average_raw_data(10, default_gain);

        offset = raw_data - zero;
        scale = ((float)calibration_weight) / ((float)offset);
        uint32_t w = (int32_t)weight_sensor_get_value();

        if (w > calibration_weight - 2 && w < calibration_weight + 2) {
            calibration_complete = true;
            vTaskResume(&get_weight_readings_task_handle);
        } else {
            ESP_LOGE(TAG, "Calibration process failed...");
        }

        ESP_LOGI(TAG, "Scale value: %f", scale);
    }  
}

static void IRAM_ATTR weight_sensor_zero_offset_isr(void *arg) {
    TickType_t current = xTaskGetTickCountFromISR();

    if (current > weight_sensor_zero_offset_next) {
        vTaskNotifyGiveFromISR(zero_task_handle, NULL);

        weight_sensor_zero_offset_next = current + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);
    }
}

static void IRAM_ATTR weight_sensor_calibrate_isr(void *arg) {
    TickType_t current = xTaskGetTickCountFromISR();

    if (current > weight_sensor_calibrate_next) {
        vTaskNotifyGiveFromISR(calibrate_task_handle, NULL);

        weight_sensor_calibrate_next = current + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);
    }
}

static void weight_sensor_init(void) {
    gpio_config_t config_buttons = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << WEIGHT_SENSOR_ZERO_PIN) | (1ULL << WEIGHT_SENSOR_CALIBRATE_PIN))
    };

    gpio_config_t config_sck = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << WEIGHT_SENSOR_CLK)
    };

    gpio_config_t config_dt = {
        .pin_bit_mask = (1ULL << WEIGHT_SENSOR_DT),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_INPUT
    };

    gpio_config(&config_buttons);
    gpio_config(&config_sck);
    gpio_config(&config_dt);
    gpio_isr_handler_add(WEIGHT_SENSOR_ZERO_PIN, weight_sensor_zero_offset_isr, NULL);
    gpio_isr_handler_add(WEIGHT_SENSOR_CALIBRATE_PIN, weight_sensor_calibrate_isr, NULL);
    xTaskCreate(weight_sensor_zero_offset_task, "zero_offset_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &zero_task_handle);
    xTaskCreate(weight_sensor_calibrate_task, "calibrate_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &calibrate_task_handle);
    xTaskCreate(weight_sensor_readings_task, "readings_task", configMINIMAL_STACK_SIZE * 5, NULL, 8, &get_weight_readings_task_handle);
    xTaskCreate(weight_sensor_trigger_low_food_task, "trigger_low_food_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}

static void IRAM_ATTR food_dispenser_manual_isr(void *arg) {
    TickType_t current = xTaskGetTickCountFromISR();

    if (current > food_dispenser_manual_next) {
        vTaskNotifyGiveFromISR(dispense_food_handle, NULL);

        food_dispenser_manual_next = current + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);
    }
}

static void food_dispenser_move(bool opening) {
    for (uint8_t index = 0; index < 50; index++) {
        gpio_set_level(FOOD_DISPENSER_STEP_PIN, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(FOOD_DISPENSER_STEP_PIN, 0); 
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if(weight > max_weight && opening) {
            ESP_LOGI(TAG, "Too much food for your cats!");
            return;
        }
    }
}

static void food_dispenser_open(void) {
    gpio_set_level(FOOD_DISPENSER_SLEEP_PIN, 1);
    gpio_set_level(FOOD_DISPENSER_DIR_PIN, 1);    
    food_dispenser_move(true);
    gpio_set_level(FOOD_DISPENSER_SLEEP_PIN, 0);
}

static void food_dispenser_close(void) {
    gpio_set_level(FOOD_DISPENSER_SLEEP_PIN, 1);
    gpio_set_level(FOOD_DISPENSER_DIR_PIN, 0);    
    food_dispenser_move(false);
    gpio_set_level(FOOD_DISPENSER_SLEEP_PIN, 0);
}

static void food_dispenser_task(void *arg) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "Giving food to cats...");

        if(weight > max_weight) {
            ESP_LOGI(TAG, "Too much food for your cats!");
        } else {
            food_dispenser_open();
            food_dispenser_close();
        }
    }         
}

static void food_dispenser_init(void) {
    gpio_config_t config_outputs = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = FOOD_DISPENSER_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config_t config_inputs = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FOOD_DISPENSER_MANUAL_PIN)
    };

    gpio_config(&config_outputs);
    gpio_config(&config_inputs);
    gpio_isr_handler_add(FOOD_DISPENSER_MANUAL_PIN, food_dispenser_manual_isr, NULL);
    xTaskCreate(food_dispenser_task, "food_dispenser_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &dispense_food_handle);
}

static void display_init(void) {
    oled_params_t params = {
        .sda_io_num = OLED_SDA,
        .scl_io_num = OLED_CLK,
        .i2c_port_num = OLED_PORT_NUM,
        .clk_speed = OLED_I2C_MASTER_FREQ_HZ
    };

    oled_init(params);

    oled_clear();
    oled_setXY(0x00, 0x7F, 0x00, 0x07);
    oled_string("Food Dispenser");
}

void app_main(void) {
    TickType_t current = xTaskGetTickCount() + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);
    
    weight_sensor_zero_offset_next = current;
    food_dispenser_manual_next = current;

    gpio_install_isr_service(0);
    food_dispenser_init();    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    display_init();
    wifi_init();
    vTaskSuspend(NULL);
}
