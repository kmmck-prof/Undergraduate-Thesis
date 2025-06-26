/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Border Router Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_border_router.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_cli_extension.h"
#include "esp_ot_config.h"
#include "esp_ot_wifi_cmd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "openthread/error.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"

#include "openthread/thread.h"
#include "openthread/udp.h"
#include "esp_sntp.h"
#include "mqtt_client.h"

#define TAG "OT BR"

#define UDP_PORT 2222
#define UDP_PORT_nodes 1234

otUdpSocket udpSocket;

char Current_Date_Time[20];
esp_mqtt_client_handle_t MQTtest;
// SemaphoreHandle_t semaphore;

// char json_payload[500] = "";
// char send_payload[500] = "";

char payload_1[500] = "";
char payload_2[500] = "";
char payload_3[500] = "";
char payload_4[500] = "";

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipseprojects_io_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
#endif
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

static const char *TAG_MQTT = "MQTTS_EXAMPLE";

#if CONFIG_EXTERNAL_COEX_ENABLE
static void ot_br_external_coexist_init(void)
{
    esp_external_coex_gpio_set_t gpio_pin = ESP_OPENTHREAD_DEFAULT_EXTERNAL_COEX_CONFIG();
    esp_external_coex_set_work_mode(EXTERNAL_COEX_LEADER_ROLE);
    ESP_ERROR_CHECK(esp_enable_extern_coex_gpio_pin(CONFIG_EXTERNAL_COEX_WIRE_TYPE, gpio_pin));
}
#endif /* CONFIG_EXTERNAL_COEX_ENABLE */

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

//RETURNS the final time in string form
void Get_current_date_time(char *date_time){
	char strftime_buf[100];
	time_t now;
	    struct tm timeinfo;
	    time(&now);
	    localtime_r(&now, &timeinfo);

	    	// Set timezone to Indian Standard Time
				setenv("TZ", "UTC-08:00", 1);
	    	    tzset();
	    	    localtime_r(&now, &timeinfo);

	    	    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	    	    //strftime(date_time, 100, "%Y-%m-%dT%H:%M:%S", &timeinfo);
	    	    //ESP_LOGI(TAG, "The current date/time in Philippines is: %s", strftime_buf);

	    	    // Extract the time values manually
	    	    int year = timeinfo.tm_year + 1900;  // Years since 1900, so add 1900
	    	    int month = timeinfo.tm_mon + 1;     // Months are 0-based, so add 1
	    	    int day = timeinfo.tm_mday;
	    	    int hour = timeinfo.tm_hour;
	    	    int minute = timeinfo.tm_min;
	    	    int second = timeinfo.tm_sec;

	    	    //concatenate the string manually
	    	    sprintf(strftime_buf, "%d-%02d-%02dT%02d:%02d:%02d", year, month, day, hour, minute, second);
                strcpy(date_time,strftime_buf);
}

//initializes SNTP server settings
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.google.com");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    esp_sntp_init();
}

// helper function that obtains timezone
static void obtain_time(void)
{
    initialize_sntp();
    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}

//function that actually syncs with NTP server
 void Set_SystemTime_SNTP()  {

	 time_t now;
	    struct tm timeinfo;
	    time(&now);
	    localtime_r(&now, &timeinfo);
	    // Is time set? If not, tm_year will be (1970 - 1900).
	    if (timeinfo.tm_year < (2016 - 1900)) {
	        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
	        obtain_time();
	        // update 'now' variable with current time
	        time(&now);
	    }
}

void mqtt_publish(esp_mqtt_client_handle_t client, char *mqtt_msg){
	const char *Topic_Name = "UPCARE/UNDERGRAD/ECE199_PUB2324";
	esp_mqtt_client_publish(client, Topic_Name, mqtt_msg, 0, 2, 0);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
    	ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG_MQTT, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG_MQTT, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG_MQTT, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG_MQTT, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(TAG_MQTT, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(esp_mqtt_client_handle_t client)
{
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void upload_data (void *params)
{
    while(1){
        if (strcmp(payload_1, "") != 0) { 
            // printf("payload 1\n");
            mqtt_publish(MQTtest, payload_1);
            strcpy(payload_1, "");
        }
        else if (strcmp(payload_2, "") != 0) { 
            // printf("payload 2\n");
            mqtt_publish(MQTtest, payload_2);
            strcpy(payload_2, "");
        } 
        else if (strcmp(payload_3, "") != 0) { 
            // printf("payload 3\n");
            mqtt_publish(MQTtest, payload_3);
            strcpy(payload_3, "");
        } 
        else if (strcmp(payload_4, "") != 0) { 
            // printf("payload 4\n");
            mqtt_publish(MQTtest, payload_4);
            strcpy(payload_4, "");
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

// source: https://www.esp32.com/viewtopic.php?t=38516
static void udp_send(void) // ff02:1, 64000, ff03:1, 2222
{
	otMessageInfo messageInfo;
    otMessageSettings msgSettings;
    msgSettings.mLinkSecurityEnabled = true;
    msgSettings.mPriority = 1;
    otIp6AddressFromString("ff03::1", &messageInfo.mPeerAddr);
    messageInfo.mPeerPort = UDP_PORT_nodes;
    otMessage * message = otUdpNewMessage(esp_openthread_get_instance(), &msgSettings);
    // const char * buf = "id1,18,5.90,6,5.90,32.03,49.86,0,400.00,0,0.00,2,1.77,0,0.07,14.699734,121.723847,23.55";
    const char * buf = Current_Date_Time;
    // char * buf = "";
    // snprintf(buf, sizeof(data_payload), "%s", data_payload);

    otError error = otMessageAppend(message, buf, (uint16_t) strlen(buf));
    if (error != OT_ERROR_NONE){
        ESP_LOGE(TAG, "UDP message creation fail (error %d : %s)", error, otThreadErrorToString(error));
    }
	// else {
	// 	uint16_t payloadLength = otMessageGetLength(message) - otMessageGetOffset(message);
	// 	char buf1[payloadLength+1];
	// 	otMessageRead(message, otMessageGetOffset(message),buf1, payloadLength);
	// 	buf1[payloadLength]='\0';
	// 	printf("UDP Message created: %s\n",buf1);
	// 	ESP_LOGI(TAG, "UDP message created.");
	// }

    error = otUdpSend(esp_openthread_get_instance(), &udpSocket, message, &messageInfo);
    if (error != OT_ERROR_NONE){
        ESP_LOGE(TAG, "UDP send fail (error %d : %s)", error, otThreadErrorToString(error));
    }
    // else {
    // 	ESP_LOGI(TAG, "UDP sent. Message: %s\n", buf);
    // }
}
static void udp_send_task()
{
    while(1){
        Get_current_date_time(Current_Date_Time);
        udp_send();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void udp_rx_cb(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo) // receive callback for sensor nodes
{
    uint16_t payloadLength = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
	char buf[payloadLength+1];
	otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, payloadLength);
	buf[payloadLength]='\0';
	if (strcmp(payload_1, "") == 0){
        snprintf(payload_1, 500, "%s", buf);
        // printf("payload_1\n");
    } else if ( strcmp(payload_2, "") == 0){
        snprintf(payload_2, 500, "%s", buf);
        // printf("payload_2\n");
    } else if ( strcmp(payload_3, "") == 0){
        snprintf(payload_3, 500, "%s", buf);
        // printf("payload_3\n");
    } else{
        snprintf(payload_4, 500, "%s", buf);
        // printf("payload_4\n");
    }
    ESP_LOGI(TAG, "UDP received successfully");
    // snprintf(json_payload, 500, "%s", buf);
    
    // snprintf(json_payload, 500, "%s", buf);
    // printf("%s\n", json_payload);
    // mqtt_publish(MQTtest, buf); // BR Hangs eventually
}

static void udp_init(void)
{
    otInstance * thread_instance = esp_openthread_get_instance();
    otSockAddr bind_info;
//    otUdpSocket udpSocket; // Declare this globally
    otNetifIdentifier netif = OT_NETIF_THREAD;
    memset(&bind_info, 0, sizeof(otSockAddr));
    otIp6AddressFromString("::", &bind_info.mAddress);
    bind_info.mPort = UDP_PORT;

    otError error = otUdpOpen(thread_instance, &udpSocket, udp_rx_cb, NULL);
    if (error != OT_ERROR_NONE)
    {
        ESP_LOGE(TAG, "UDP open error (error %d:%s)", error, otThreadErrorToString(error));
    } else
    {
        ESP_LOGI(TAG, "UDP initialized");
    }

    error = otUdpBind(thread_instance, &udpSocket, &bind_info, netif);
    if (error != OT_ERROR_NONE)
    {
        ESP_LOGE(TAG, "UDP bind error (error %d:%s)", error, otThreadErrorToString(error));
    }else{
        ESP_LOGI(TAG, "UDP binded");
    }
}

static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t       *openthread_netif = esp_netif_new(&cfg);
    assert(openthread_netif != NULL);

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));

    // Initialize border routing features
    esp_openthread_lock_acquire(portMAX_DELAY);
    ESP_ERROR_CHECK(esp_netif_attach(openthread_netif, esp_openthread_netif_glue_init(&config)));

    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
    esp_openthread_cli_init();

//    esp_cli_custom_command_init();

//    esp_openthread_cli_create_task();

#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(esp_openthread_border_router_init());
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif // CONFIG_OPENTHREAD_BR_AUTO_START

    esp_cli_custom_command_init();
//    esp_openthread_lock_release();

    // Run the main loop
    esp_openthread_cli_create_task();
    udp_init();
    esp_openthread_lock_release();
    esp_openthread_launch_mainloop();

    // Clean up
    esp_netif_destroy(openthread_netif);
    esp_openthread_netif_glue_deinit();
    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        esp_wifi_connect();
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP...\n\n");
        break;
    default:
        break;
    }
}
/*
 * UPCARE WIFI: UPCARE203 2point4GHz, wfwfm66699
 * Home WIFI: CMFG, 131823cbl
 * Phone WIFI: UPCARE PUB AQ, team1h_esp32s3
 */
void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation
    esp_netif_create_default_wifi_sta(); // WiFi station
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // s1.4
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
			.ssid = "EEE192-429",
			.password = "EEE192_Room429"}};
    esp_wifi_set_mode(WIFI_MODE_STA);


    esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    printf("WiFi Starting...\n");
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

void app_main(void)
{
    // Used eventfds:
    // * netif
    // * task queue
    // * border router
    esp_vfs_eventfd_config_t eventfd_config = {
#if CONFIG_OPENTHREAD_RADIO_NATIVE
        // * radio driver (A native radio device needs a eventfd for radio driver.)
        .max_fds = 4,
#else
        .max_fds = 3,
#endif
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_EXAMPLE_CONNECT_WIFI
#if CONFIG_OPENTHREAD_BR_AUTO_START
    ESP_ERROR_CHECK(example_connect());
    // wifi_connection();
    vTaskDelay(8000 / portTICK_PERIOD_MS);
    Set_SystemTime_SNTP();
    const esp_mqtt_client_config_t mqtt_cfg = {
    	//broker URL or URI
        .broker = {
            //.address.uri = "mqtts://a182f6d4db414543bbd712a57a4b1290.s1.eu.hivemq.cloud:8883/mqtt",
    		.address.uri = "mqtts://ed7632329e6e4fbcbe77b1fa917585a1.s1.eu.hivemq.cloud:8883/mqtt",
            .verification.certificate = (const char *)mqtt_eclipseprojects_io_pem_start
        },
        //this is where I put the credentials
    	//I added this
    	.credentials = {
            .username="cayanan.k2",
    		.authentication = {
                .password="LUZ0Po5YK3hZnA3"
            }
        },

    };
    MQTtest = esp_mqtt_client_init(&mqtt_cfg);
    mqtt_app_start(MQTtest);
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE && CONFIG_OPENTHREAD_RADIO_NATIVE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_coex_wifi_i154_enable());
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_EXTERNAL_COEX_ENABLE
    ot_br_external_coexist_init();
#endif // CONFIG_EXTERNAL_COEX_ENABLE

#endif
    esp_openthread_set_backbone_netif(get_example_netif());
#else
    esp_ot_wifi_netif_init();
    esp_openthread_set_backbone_netif(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));
#endif // CONFIG_OPENTHREAD_BR_AUTO_START
#elif CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(example_connect());
    esp_openthread_set_backbone_netif(get_example_netif());
#else
    ESP_LOGE(TAG, "ESP-Openthread has not set backbone netif");
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
    
    // semaphore = xSemaphoreCreateCounting(1,0);
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));
    xTaskCreate(ot_task_worker, "ot_br_main", 20480, xTaskGetCurrentTaskHandle(), 5, NULL);
    vTaskDelay(15000/portTICK_PERIOD_MS);
    xTaskCreate(udp_send_task, "udp_send_task", 4096, NULL, 5, NULL);
    xTaskCreate(upload_data, "mqtt_upload", 4096, NULL, 5, NULL);
}