#include <stdio.h>
#include <stdlib.h>
#include "espressif/esp_common.h"
#include <string.h>
#include <ssid_config.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <semphr.h>
#include "bme680/bme680.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "i2c/i2c.h"
#include "queue.h"
#define I2C_BUS       0
#define I2C_SCL   14
#define I2C_SDA   12
#define I2C_FREQ      I2C_FREQ_100K
#define PCF_ADDRESS    0x38
#define led1         0xfe
#define black         0xff
#define threshold 100
float gas_baseline = 0;
#define TASK_STACK_DEPTH 256
#define MQTT_HOST ("test.mosquitto.org")
#define MQTT_PORT 1883
#define MQTT_USER NULL
#define MQTT_PASS NULL
SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
static bme680_sensor_t* sensor;

void write_byte_pcf(uint8_t data) {
    i2c_slave_write(I2C_BUS, PCF_ADDRESS, NULL, &data, 1);
}

void user_task(void *pvParameters) {
    bme680_values_float_t values;
    TickType_t last_wakeup = xTaskGetTickCount();
    // as long as sensor configuration isn't changed, duration is constant
    float gas_offset, hum_offset;
    float hum_weighting = 0.25;
    float hum_baseline = 40.0;
    float hum_score, gas_score;
    float air_quality_index;
    float air_quality_score;
    int count = 0;

    uint32_t duration = bme680_get_measurement_duration(sensor);
    while (1) {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(sensor)) {
            // passive waiting until measurement results are available
            vTaskDelay(duration);


            // get the results and do something with them
            if (bme680_get_results_float(sensor, &values)) {
                if (count < 20) {
                    gas_baseline = gas_baseline + values.gas_resistance;
                    count++;
                    printf("%d \n", count);
                    vTaskDelay(pdMS_TO_TICKS(300));
                } else {
                    gas_baseline = gas_baseline / 20.0;
                    printf("else %d \n", count);
                    gas_offset = gas_baseline - values.gas_resistance;
                    hum_offset = values.humidity - hum_baseline;
                    if (hum_offset > 0)
                        hum_score = (100 - hum_baseline - hum_offset) / (100 - hum_baseline) * (hum_weighting * 100);
                    else
                        hum_score = (hum_baseline + hum_offset) / hum_baseline * (hum_weighting * 100);
                    if (gas_offset > 0)
                        gas_score = (values.gas_resistance / gas_baseline) * (100 - (hum_weighting * 100));
                    else
                        gas_score = 100 - (hum_weighting * 100);

                    air_quality_score = hum_score + gas_score;
                    air_quality_index = (100 - air_quality_score) * 5;

                    if (air_quality_index >= 301)
                        printf("Hazardous\n");
                    if (air_quality_index >= 201 && air_quality_index <= 300)
                        printf("Very unhealthy\n");
                    if (air_quality_index >= 176 && air_quality_index <= 200)
                        printf("Unhealthy\n");
                    if (air_quality_index >= 151 && air_quality_index <= 175)
                        printf("Unhealthy for Sensitive Groups\n");
                    if (air_quality_index >= 51 && air_quality_index <= 150)
                        printf("Moderate\n");
                    if (air_quality_index >= 0 && air_quality_index <= 50)
                        printf("Good\n");
                    printf("Air Quality Index: %f\n", air_quality_index);
                    printf("Air Quality Score: %f\n", air_quality_score);
                    printf("hum Score: %f\n", hum_score);
                    printf("gas Score: %f\n", gas_score);
                    printf("gas resistance: %.2f\n", values.gas_resistance);
                    printf("humidity: %.2f\n", values.humidity);
                    printf("gas baseline: %f\n", gas_baseline);
                    gas_baseline = gas_baseline * 20.0;

                    if (air_quality_index > threshold) {
                        write_byte_pcf(led1);
                        vTaskDelay(pdMS_TO_TICKS(300));
                        write_byte_pcf(black);
                    }
                    vTaskDelay(pdMS_TO_TICKS(200));
                    char m[8];
                    snprintf(m, 8, "%.2f \r\n", air_quality_index);
                    printf("%s", m);
                    xQueueSend(publish_queue,(void*)m, 0);
                }
            }
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }

}

static const char * get_my_id(void) {
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *) my_id))
        return NULL;
    for (i = 5; i >= 0; --i) {
        x = my_id[i] & 0x0F;
        if (x > 9)
            x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9)
            x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static void mqtt_task(void *pvParameters) {
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new(&network);
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while (1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ", __func__, MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if (ret) {
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf,
                100);

        data.willFlag = 0;
        data.MQTTVersion = 3;
        data.clientID.cstring = mqtt_client_id;
        data.username.cstring = MQTT_USER;
        data.password.cstring = MQTT_PASS;
        data.keepAliveInterval = 10;
        data.cleansession = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if (ret) {
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\r\n");

        while (1) {

            char msg[8] = "\0";
            while (xQueueReceive(publish_queue, (void *) msg, portMAX_DELAY)
                    == pdTRUE) {
                // printf("%s",msg);
                // printf("got message to publish\r\n");
                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = 9;
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, "/beat", &message);
                xQueueReset(publish_queue);
                if (ret != MQTT_SUCCESS) {
                    printf("error while publishing message: %d\n", ret);
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}

static void wifi_task(void *pvParameters) {
    uint8_t status = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = { .ssid = WIFI_SSID, .password =
            WIFI_PASS, };

    printf("WiFi: connecting to WiFi\n\r");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while (1) {
        while ((status != STATION_GOT_IP) && (retries)) {
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status);
            if (status == STATION_WRONG_PASSWORD) {
                printf("WiFi: wrong password\n\r");
                break;
            } else if (status == STATION_NO_AP_FOUND) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if (status == STATION_CONNECT_FAIL) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status())
                == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void user_init(void) {
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(1, sizeof(char)*8);
    // Give the UART some time to settle
    vTaskDelay(1);
    i2c_init(I2C_BUS, I2C_SCL, I2C_SDA, I2C_FREQ);
    gpio_enable(I2C_SCL, GPIO_OUTPUT);

    // init the sensor with slave address BME680_I2C_ADDRESS_2 connected to I2C_BUS.
    sensor = bme680_init_sensor(I2C_BUS, BME680_I2C_ADDRESS_2, 0);
    int count2 = 0;
    uint32_t duration = bme680_get_measurement_duration(sensor);
    bme680_values_float_t values;

    while (count2 < 9000) {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(sensor)) {
            // passive waiting until measurement results are available
            vTaskDelay(duration);

            // get the results and do something with them
            if (bme680_get_results_float(sensor, &values)) {
                count2++;
                printf("Waiting for sensor %d \n", count2);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }



    xTaskCreate(&wifi_task, "wifi_task", 256, NULL, 1, NULL);
    xTaskCreate(&user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 3, NULL);
    
}

