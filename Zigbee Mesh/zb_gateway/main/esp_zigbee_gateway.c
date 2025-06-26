#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist_internal.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_vfs_eventfd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_zigbee_gateway.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "esp_partition.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <stdlib.h>
#include <time.h>
#include <esp_wifi_types.h>
#include "driver/uart.h"
#include <time.h>
#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "driver/gpio.h"

#if (!defined ZB_MACSPLIT_HOST && defined ZB_MACSPLIT_DEVICE)
#error Only Zigbee gateway host device should be defined
#endif

SemaphoreHandle_t node3_semaphore, node1_semaphore, node4_semaphore, mqtt_semaphore, semaphore;
#define TASK_COUNT 9
TaskHandle_t zigbee_handle, upload_handle, timer_handle, node1_upload_handle, node4_upload_handle;


//global variables
int test_count=0;
char payload[50]="Payload sample text"; //50 characters max
esp_mqtt_client_handle_t MQTtest;
char sample_char[50];
uint16_t SENSOR_DATA_CLUSTER_ID = 0xFFF1;
uint16_t TIME_CLUSTER_ID = 0xFFF2;
uint8_t SENSOR_DATA_ATTR_ID = 0;
uint8_t SENSOR_DATA_ATTR_ID_2 = 1;
uint8_t TIME_ATTR_ID = 3;

const uart_port_t uart_num = UART_NUM_2;

int node_id;

int node1_data1_flag,node1_data2_flag,node1_data3_flag,node2_data1_flag,node2_data2_flag;
int node3_data1_flag,node3_data2_flag,node3_data3_flag,node4_data1_flag,node4_data2_flag,node4_data3_flag;

float pm25_in, pm10_in, co_in, no2_in, co2_in, voc_in, co2_in3, voc_in3;
float pm25_out, pm10_out, co_out, no2_out, co2_out, voc_out;
float speed;

float lat_decimal = 0;
float long_decimal = 0;


char Current_Date_Time[100];
char Upload_Time[20];
char json_payload[701];
char json_payload_1[701];
char json_payload_3[701];
char json_payload_4[701];

char n11_buffer[76], n12_buffer[76], n13_buffer[76], n41_buffer[76], n42_buffer[76], n43_buffer[76], n21_buffer[76], n22_buffer[76], n31_buffer[76], n32_buffer[76], n33_buffer[76];

double lat_deg, long_deg;
char* parsed_data1[16];
char* parsed_data3[16];
char* parsed_data4[18];
char* data[25];
char count[2];

//UART2 for Occupancy Sensor
#define TX_pin 1
#define RX_pin 2

//Drive Alert LEDs
#define RECIRC_LED 15 //WHITE LED
#define INTAKE_LED 16 //BLUE LED

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipseprojects_io_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
#endif
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

static const char *TAG = "ESP_ZB_GATEWAY";
static const char *TAG_MQTT = "MQTTS_EXAMPLE";

/* Note: Please select the correct console output port based on the development board in menuconfig */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
esp_err_t esp_zb_gateway_console_init(void)
{
    esp_err_t ret = ESP_OK;
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    esp_vfs_usb_serial_jtag_use_driver();
    esp_vfs_dev_uart_register();
    return ret;
}
#endif

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

void init_uart(void){
	uart_driver_delete(UART_NUM_0);
	uart_config_t uart_config = {
	    .baud_rate = 9600, //GPS baud rate according to datasheet
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
	// Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
	ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_pin, RX_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); //RTS and CTS pins will not be used
	// Setup UART buffered IO with event queue
	const int uart_buffer_size = (1024*4);
	QueueHandle_t uart_queue;
	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

//global float
float speed = 1.00;
char occu_cleaned[8];
char occu_count[8] = {};

void cleanData(char *raw_data, char *cleaned_data) {
    int i;
    for (i = 0; i < strlen(raw_data); i++) {
        if (raw_data[i] == '\n') {
            break;  // Stop iterating if newline is detected
        }
        cleaned_data[i] = raw_data[i]; // Copy character to cleaned_data
    }
    cleaned_data[i] = '\0'; // Null-terminate the cleaned string
}


void occu_task(void){
	//Occupancy Exchange
	int length;
	char buffer[10]; // Adjust size as needed
	snprintf(buffer, sizeof(buffer), "%.0f", speed);
	//Send UART Command
	uart_write_bytes(uart_num, buffer, strlen(buffer));
	//uart_flush(uart_num);
	//Receive Passenger Count
	ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
	while(length == 0){
		//wait here
		printf("WAITING...\n");
		vTaskDelay(1000/portTICK_PERIOD_MS);
		ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length)); //check every1s for update
	}
	char occu_in[length];
	uart_read_bytes(uart_num, occu_in, length, 1000);

	//call data cleaner
	cleanData(occu_in,occu_cleaned);
   	//snprintf(occu_count, 8, "%s.00", occu_cleaned);
	printf("COUNT: %s\n", occu_cleaned);
	vTaskDelay(1000/portTICK_PERIOD_MS);
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
			.ssid = "CMFG",
			.password = "131823cbl"}};
    esp_wifi_set_mode(WIFI_MODE_STA);


    esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    printf("WiFi Starting...\n");
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

void mqtt_publish(esp_mqtt_client_handle_t client){
	const char *HiveMQ = "UPCARE/UNDERGRAD/ECE199_PUB2324";
	esp_mqtt_client_publish(client, HiveMQ, json_payload, 0, 2, 0);
}

void mqtt_publish_node_specific(esp_mqtt_client_handle_t client, int node_num){
	const char *Topic_Name = "UPCARE/UNDERGRAD/ECE199_PUB2324";
	if(node_num == 1){
		esp_mqtt_client_publish(client, Topic_Name, json_payload_1, 0, 2, 0);
	}
	else if (node_num == 3){
		esp_mqtt_client_publish(client, Topic_Name, json_payload_3, 0, 2, 0);
	}
	else if (node_num == 4){
		esp_mqtt_client_publish(client, Topic_Name, json_payload_4, 0, 2, 0);
	}

}

void driver_alert_init (void){
	gpio_reset_pin(RECIRC_LED);
	gpio_set_direction(RECIRC_LED, GPIO_MODE_OUTPUT);
	gpio_reset_pin(INTAKE_LED);
	gpio_set_direction(INTAKE_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(INTAKE_LED, 0);
	gpio_set_level(RECIRC_LED, 1);
}
void driver_alert(void){ // (Switch to INTAKE when Above ORANGE Indoor) AND (In_Score > Out_Score)
	if((co_in > 76 && co_in > co_out) || (no2_in > 76 && no2_in > no2_out) || (co2_in > 76 && no2_in > co2_out) || (voc_in > 76 && voc_in > voc_out)){
			gpio_set_level(INTAKE_LED, 1);
			gpio_set_level(RECIRC_LED, 0);
	}
	else{
		gpio_set_level(INTAKE_LED, 0);
		gpio_set_level(RECIRC_LED, 1);
	}
}

void upload_node1 (void* params){
	while(1){
		if(strcmp(n11_buffer, "") && strcmp(n12_buffer, "")){

			int node_num = 1;
			//UPLOAD NODE 1 DATA
			char* token11 = strtok(n11_buffer, "K");
			token11 = strtok(token11, ",");
			for (int i=0; i<12; i++){
				parsed_data1[i] = token11;
				token11 = strtok(NULL, ",");
			}

			char* token12 = strtok(n12_buffer, "K");
			token12 = strtok(token12, ",");
			for (int i=0; i<3; i++){
				parsed_data1[i+12] = token12;
				token12 = strtok(NULL, ",");
			}
//			co_in = atof(data[10]);
//			co2_in = atof(data[6]);
//			no2_in = atof(data[12]);
//			pm25_in = atof(data[0]);
//			pm10_in = atof(data[2]);
//			voc_in = atof(data[8]);
			//driver_alert(); //UPDATE DRIVER ALERT
			//snprintf(json_payload_1, 601, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}",data[11],data[10],data[7],data[6],data[13],data[12],data[1],data[0],data[3],data[2],data[9],data[8],data[4],data[5],n13_buffer);

			printf("SENDING NODE 1\n");
			snprintf(json_payload_1, 650, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}","0.01","1",parsed_data1[7],parsed_data1[6],"0.01","1",parsed_data1[1],parsed_data1[0],parsed_data1[3],parsed_data1[2],parsed_data1[9],parsed_data1[8],parsed_data1[4],parsed_data1[5],parsed_data1[14]);
			mqtt_publish_node_specific(MQTtest, node_num);
			strcpy(n11_buffer, "");
			strcpy(n12_buffer, "");
		}
		vTaskDelay(500/portTICK_PERIOD_MS);

	}
}

void upload_node3 (void* params){
	while(1){
		if(strcmp(n31_buffer, "") && strcmp(n32_buffer, "")){

			int node_num = 3;
			//UPLOAD NODE 3 DATA
			char* token31 = strtok(n31_buffer, "K");
			token31 = strtok(token31, ",");
			for (int i=0; i<12; i++){
				parsed_data3[i] = token31;
				token31 = strtok(NULL, ",");
			}

			char* token32 = strtok(n32_buffer, "K");
			token32 = strtok(token32, ",");
			for (int i=0; i<3; i++){
				parsed_data3[i+12] = token32;
				token32 = strtok(NULL, ",");
			}
//			co_in = atof(data[10]);
//			co2_in = atof(data[6]);
//			no2_in = atof(data[12]);
//			pm25_in = atof(data[0]);
//			pm10_in = atof(data[2]);
//			voc_in = atof(data[8]);
			//driver_alert(); //UPDATE DRIVER ALERT
			//snprintf(json_payload_1, 601, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}",data[11],data[10],data[7],data[6],data[13],data[12],data[1],data[0],data[3],data[2],data[9],data[8],data[4],data[5],n13_buffer);

			printf("SENDING NODE 3\n");
			snprintf(json_payload_3, 650, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 3\",\"local_time\":\"%s\",\"type\":\"data\"}","0.01","1",parsed_data3[7],parsed_data3[6],"0.01","1",parsed_data3[1],parsed_data3[0],parsed_data3[3],parsed_data3[2],parsed_data3[9],parsed_data3[8],parsed_data3[4],parsed_data3[5],parsed_data3[14]);
			mqtt_publish_node_specific(MQTtest, node_num);
			strcpy(n31_buffer, "");
			strcpy(n32_buffer, "");
		}
		vTaskDelay(500/portTICK_PERIOD_MS);
	}
}

void upload_node4 (void* params){
	while(1){
		if(strcmp(n41_buffer, "") && strcmp(n42_buffer, "")){

			//UPLOAD NODE 4 DATA
			int node_num = 4;
			char* token41 = strtok(n41_buffer, "K");
			token41 = strtok(token41, ",");
			for (int i=0; i<12; i++){
				parsed_data4[i] = token41;
				token41 = strtok(NULL, ",");
			}

			char* token42 = strtok(n42_buffer, "K");
			token42 = strtok(token42, ",");
			for (int i=0; i<6; i++){
				parsed_data4[i+12] = token42;
				token42 = strtok(NULL, ",");
			}
			//data4[16] is SPEED - REMOVED for now
/*			co_out = atof(data4[10]);
			co2_out = atof(data4[6]);
			no2_out = atof(data4[12]);
			pm25_out = atof(data4[0]);
			pm10_out = atof(data4[2]);
			voc_out = atof(data4[8]);*/
			//speed = atof(data4[16]);
			//lat_decimal = atof(data4[14]);
			//long_decimal = atof(data4[15]);
			//occu_task();

			snprintf(json_payload_4, 650, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"Lat\":\"%s\",\"Long\":\"%s\",\"speed\":\"%s\",\"Occu\":\"%s\",\"source\":\"Node 4\",\"local_time\":\"%s\",\"type\":\"data\"}",parsed_data4[11],parsed_data4[10],parsed_data4[7],parsed_data4[6],parsed_data4[13],parsed_data4[12],parsed_data4[1],parsed_data4[0],parsed_data4[3],parsed_data4[2],parsed_data4[9],parsed_data4[8],parsed_data4[4],parsed_data4[5],parsed_data4[14],parsed_data4[15],parsed_data4[16],"10",parsed_data4[17]);

			printf("SENDING NODE 4\n");
			mqtt_publish_node_specific(MQTtest, node_num);
			strcpy(n41_buffer, "");
			strcpy(n42_buffer, "");
		}
		vTaskDelay(500/portTICK_PERIOD_MS);
	}
}

void upload_task(void *params){
	while(1){
		if(uxSemaphoreGetCount(semaphore) == TASK_COUNT){ //TASK_COUNT
			xSemaphoreTake(semaphore, portMAX_DELAY);
			xSemaphoreTake(semaphore, portMAX_DELAY);
			xSemaphoreTake(semaphore, portMAX_DELAY);

			xSemaphoreTake(semaphore, portMAX_DELAY);
			xSemaphoreTake(semaphore, portMAX_DELAY);
			xSemaphoreTake(semaphore, portMAX_DELAY);

			xSemaphoreTake(semaphore, portMAX_DELAY);
			xSemaphoreTake(semaphore, portMAX_DELAY);
			xSemaphoreTake(semaphore, portMAX_DELAY);

			printf("UPLOAD TASK\n");
			//UPLOAD NODE 1 DATA
			printf("DECODING NODE 1.1\n");
			int node_num = 1;
			char* token11 = strtok(n11_buffer, ",");
			for (int i=0; i<8; i++){
				data[i] = token11;
				token11 = strtok(NULL, ",");
			}
			printf("DECODING NODE 1.2\n");
			char* token12 = strtok(n12_buffer, ",");
			for (int i=0; i<6; i++){
				data[i+8] = token12;
				token12 = strtok(NULL, ",");
			}


//			co_in = atof(data[10]);
//			co2_in = atof(data[6]);
//			no2_in = atof(data[12]);
//			pm25_in = atof(data[0]);
//			pm10_in = atof(data[2]);
//			voc_in = atof(data[8]);
			//driver_alert(); //UPDATE DRIVER ALERT
			//snprintf(json_payload_1, 601, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}",data[11],data[10],data[7],data[6],data[13],data[12],data[1],data[0],data[3],data[2],data[9],data[8],data[4],data[5],n13_buffer);

			printf("SENDING NODE 1\n");
			snprintf(json_payload_1, 650, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}","0.01","1",data[7],data[6],"0.01","1",data[1],data[0],data[3],data[2],data[9],data[8],data[4],data[5],n13_buffer);
			printf("MQTT\n");
			mqtt_publish_node_specific(MQTtest, node_num);
			printf("MQTT-P\n");
			vTaskDelay(500/portTICK_PERIOD_MS);


			//UPLOAD NODE 3 DATA
			printf("DECODING NODE 3.1\n");
			char* token31 = strtok(n31_buffer, ",");
			node_num = 3;
			for (int i=0; i<8; i++){
				parsed_data3[i] = token31;
				token31 = strtok(NULL, ",");
			}
			printf("DECODING NODE 3.2\n");
			char* token32 = strtok(n32_buffer, ",");
			for (int i=0; i<6; i++){
				parsed_data3[i+8] = token32;
				token32 = strtok(NULL, ",");
			}
/*			co_in_3 = atof(data3[10]);
			co2_in = atof(data3[6]);
			no2_in = atof(data3[12]);
			pm25_in = atof(data3[0]);
			pm10_in = atof(data3[2]);
			voc_in = atof(data3[8]);*/
			printf("SENDING NODE 3\n");
			//snprintf(json_payload_3, 700, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 3\",\"local_time\":\"%s\",\"type\":\"data\"}",data3[11],data3[10],data3[7],data3[6],data3[13],data3[12],data3[1],data3[0],data3[3],data3[2],data3[9],data3[8],data3[4],data3[5],n33_buffer);
			snprintf(json_payload_3, 650, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"source\":\"Node 3\",\"local_time\":\"%s\",\"type\":\"data\"}","0.01","1",parsed_data3[7],parsed_data3[6],"0.01","1",parsed_data3[1],parsed_data3[0],parsed_data3[3],parsed_data3[2],parsed_data3[9],parsed_data3[8],parsed_data3[4],parsed_data3[5],n33_buffer);
			printf("MQTT\n");
			mqtt_publish_node_specific(MQTtest, node_num);
			printf("MQTT-P\n");
			vTaskDelay(500/portTICK_PERIOD_MS);


			//UPLOAD NODE 4 DATA
			node_num = 4;
			printf("DECODING NODE 4.1\n");
			char* token41 = strtok(n41_buffer, ",");
			char* data4[20];
			for (int i=0; i<8; i++){
				data4[i] = token41;
				token41 = strtok(NULL, ",");
			}
			printf("DECODING NODE 4.2\n");
			char* token42 = strtok(n42_buffer, ",");
			for (int i=0; i<6; i++){
				data4[i+8] = token42;
				token42 = strtok(NULL, ",");
			}
			printf("DECODING NODE 4.3\n");
			char* token43 = strtok(n43_buffer, ",");
			for (int i=0; i<4 ; i++){
				data4[i+14] = token43;
				token43 = strtok(NULL, ",");
			}
			//data4[16] is SPEED - REMOVED for now
/*			co_out = atof(data4[10]);
			co2_out = atof(data4[6]);
			no2_out = atof(data4[12]);
			pm25_out = atof(data4[0]);
			pm10_out = atof(data4[2]);
			voc_out = atof(data4[8]);*/
			//speed = atof(data4[16]);
			//lat_decimal = atof(data4[14]);
			//long_decimal = atof(data4[15]);
			//occu_task();

			printf("With GPS");
			snprintf(json_payload_4, 650, "{\"COraw\":\"%s\",\"COindex\":\"%s\",\"CO2raw\":\"%s\",\"CO2index\":\"%s\",\"NO2raw\":\"%s\",\"NO2index\":\"%s\",\"PM25raw\":\"%s\",\"PM25index\":\"%s\",\"PM10raw\":\"%s\",\"PM10index\":\"%s\",\"VOCraw\":\"%s\",\"VOCindex\":\"%s\",\"T\":\"%s\",\"RH\":\"%s\",\"Lat\":\"%s\",\"Long\":\"%s\",\"speed\":\"%s\",\"Occu\":\"%s\",\"source\":\"Node 4\",\"local_time\":\"%s\",\"type\":\"data\"}",data4[11],data4[10],data4[7],data4[6],data4[13],data4[12],data4[1],data4[0],data4[3],data4[2],data4[9],data4[8],data4[4],data4[5],data4[14],data4[15],data4[16],occu_cleaned,data4[17]);

			printf("MQTT\n");
			mqtt_publish_node_specific(MQTtest, node_num);
			printf("MQTT-P\n");
			vTaskDelay(500/portTICK_PERIOD_MS);

			//RESET FLAGS (CONSIDER CHANGING TO BOOL)
			node1_data1_flag = 0;
			node1_data2_flag = 0;
			node1_data3_flag = 0;
			node3_data1_flag = 0;
			node3_data2_flag = 0;
			node3_data3_flag = 0;
			node4_data1_flag = 0;
			node4_data2_flag = 0;
			node4_data3_flag = 0;
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
		printf("%d %d %d %d %d %d %d %d %d %d\n", uxSemaphoreGetCount(semaphore), node1_data1_flag, node1_data2_flag, node1_data3_flag, node3_data1_flag, node3_data2_flag, node3_data3_flag, node4_data1_flag, node4_data2_flag, node4_data3_flag);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
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

void update_time_attr (void *params){
	while(1){
		Get_current_date_time(Current_Date_Time);
		//UPDATE TIME ATTRIBUTE
		esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(COORDINATOR_ENDPOINT, TIME_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, TIME_ATTR_ID);
		memcpy(value_r->data_p, &Current_Date_Time, sizeof(Current_Date_Time));
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    esp_zb_zdo_signal_macsplit_dev_boot_params_t *rcp_version = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_MACSPLIT_DEVICE_BOOT:
        ESP_LOGI(TAG, "Zigbee rcp device booted");
        rcp_version = (esp_zb_zdo_signal_macsplit_dev_boot_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Running RCP Version: %s", rcp_version->version_str);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network formation");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee_address;
            esp_zb_get_long_address(ieee_address);
            ESP_LOGI(TAG, "Formed network successfully (ieee_address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
                     ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            ESP_LOGI(TAG, "Starting time cluster");
            xTaskCreate(update_time_attr, "Time Cluster", 4096, NULL, 5, &timer_handle);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
            ESP_LOGI(TAG, "Uploading to CARE Database started");
            //xTaskCreate(upload_task, "uploading", 1024*9, NULL, 5, &upload_handle);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Report from address(0x%x) src_ep(%d) to dst_ep(%d) cluster(0x%x)", message->src_address.u.short_addr,
             message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Information: attribute(0x%x), type(0x%x), value(%s)\n", message->attribute.id, message->attribute.data.type,
             (char *)message->attribute.data.value);
    if (message->src_endpoint == 1) { //INDOOR SOURCE NODE 1
		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2, VOC, CO
			snprintf(n11_buffer, sizeof(n11_buffer), "%s", (char* )message->attribute.data.value);
		}
		else if (message->attribute.id == 1){ //NO2, DateTime
			snprintf(n12_buffer, sizeof(n12_buffer), "%s", (char* )message->attribute.data.value);
		}
    }
    else if (message->src_endpoint == 3) { //"OUTDOOR" SOURCE NODE 4
		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2, VOC, CO
			snprintf(n31_buffer, sizeof(n31_buffer), "%s", (char* )message->attribute.data.value);
		}
		else if (message->attribute.id == 1){ //NO2, DateTime
			snprintf(n32_buffer, sizeof(n32_buffer), "%s", (char* )message->attribute.data.value);
		}
    }
    else if (message->src_endpoint == 4) { //"OUTDOOR" SOURCE NODE 4
		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2, VOC, CO
			snprintf(n41_buffer, sizeof(n41_buffer), "%s", (char* )message->attribute.data.value);
		}
		else if (message->attribute.id == 1){ //NO2, LAT, LONG, SPEED, DateTime
			snprintf(n42_buffer, sizeof(n42_buffer), "%s", (char* )message->attribute.data.value);
		}
    }
//    if (message->src_endpoint == 1) { //INDOOR SOURCE NODE 1
//    	if (node1_data1_flag == 1 && node1_data2_flag == 1 && node1_data3_flag == 1){
//    		return ESP_OK; //SKIP
//    	}
//    	else{
//    		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2
//				if (node1_data1_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n11_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node1_semaphore);
//					node1_data1_flag = 1;
//				}
//			}
//			else if (message->attribute.id == 1){ //VOC, CO, NO2
//				if (node1_data2_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n12_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node1_semaphore);
//					node1_data2_flag = 1;
//				}
//			}
//			else if(message->attribute.id == 2){ //TIME
//				if (node1_data3_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n13_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node1_semaphore);
//					node1_data3_flag = 1;
//				}
//			}
//			else{
//				ESP_LOGI(TAG, "Unrecognized Attribute ID: %d", message->attribute.id);
//			}
//    	}
//    }    else if (message->src_endpoint == 2){
//    	if (node2_data1_flag == 1 && node2_data2_flag == 1){
//    		return ESP_OK; //SKIP
//    	}
//    	else{
//    		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2
//				if (node2_data1_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n21_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					node2_data1_flag = 1;
//
//				}
//			}
//			else if (message->attribute.id == 1){ //VOC, CO, NO2, LAT, LONG
//				if (node2_data2_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n22_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					node2_data2_flag = 1;
//				}
//			}
//			else{
//				ESP_LOGI(TAG, "Unrecognized Attribute ID: %d", message->attribute.id);
//			}
//    	}
//    }
//    else if (message->src_endpoint == 3){
//    	if (node3_data1_flag == 1 && node3_data2_flag == 1 && node3_data3_flag == 1){
//    		return ESP_OK; //SKIP
//    	}
//    	else{
//    		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2
//				if (node3_data1_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n31_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node3_semaphore);
//					node3_data1_flag = 1;
//
//				}
//			}
//			else if (message->attribute.id == 1){ //VOC, CO, NO2
//				if (node3_data2_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n32_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node3_semaphore);
//					node3_data2_flag = 1;
//				}
//			}
//			else if(message->attribute.id == 2){ //TIME
//				if (node3_data3_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n33_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node1_semaphore);
//					node3_data3_flag = 1;
//				}
//			}
//			else{
//				ESP_LOGI(TAG, "Unrecognized Attribute ID: %d", message->attribute.id);
//			}
//    	}
//    }
//    else if (message->src_endpoint == 4){
//    	if (node4_data1_flag == 1 && node4_data2_flag == 1 && node4_data3_flag == 1){
//    		return ESP_OK; //SKIP
//    	}
//    	else{
//    		if (message->attribute.id == 0){ //PM2.5, PM10, TEMPERATURE, REL. HUMIDITY, CO2
//				if (node4_data1_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n41_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node4_semaphore);
//					node4_data1_flag = 1;
//
//				}
//			}
//			else if (message->attribute.id == 1){ //VOC, CO, NO2
//				if (node4_data2_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n42_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node4_semaphore);
//					node4_data2_flag = 1;
//				}
//			}
//			else if (message->attribute.id == 2){ //LAT, LONG, SPEED
//				if (node4_data3_flag == 1){
//					return ESP_OK; //SKIP
//				}
//				else{
//					snprintf(n43_buffer, 50, "%s", (char* )message->attribute.data.value);
//					xSemaphoreGive(semaphore);
//					//xSemaphoreGive(node4_semaphore);
//					node4_data3_flag = 1;
//				}
//			}
//			else{
//				ESP_LOGI(TAG, "Unrecognized Attribute ID: %d", message->attribute.id);
//			}
//    	}
//    }
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    float light_state = 0;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == COORDINATOR_ENDPOINT) {
        if (message->info.cluster == SENSOR_DATA_CLUSTER_ID) {
            if (message->attribute.id == SENSOR_DATA_ATTR_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING) {
                light_state = message->attribute.data.value ? *(char *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Data received: %.2f", light_state);
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                //light_driver_set_power(light_state);
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee coordinator config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    const uint8_t attr_access = ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE;

	esp_zb_attribute_list_t *esp_zb_sensordata_cluster = esp_zb_zcl_attr_list_create(SENSOR_DATA_CLUSTER_ID);
	ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(esp_zb_sensordata_cluster, SENSOR_DATA_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, attr_access, &sample_char));
	esp_zb_custom_cluster_add_custom_attr(esp_zb_sensordata_cluster, SENSOR_DATA_ATTR_ID_2, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, attr_access, &sample_char);

	esp_zb_attribute_list_t *esp_zb_time_cluster = esp_zb_zcl_attr_list_create(TIME_CLUSTER_ID);
	esp_zb_custom_cluster_add_custom_attr(esp_zb_time_cluster, TIME_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, attr_access, &sample_char);

	esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_sensordata_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
	esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_time_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


	esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
	esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, COORDINATOR_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);
	esp_zb_device_register(esp_zb_ep_list);

	//esp_zb_device_register(esp_zb_ep_node2_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    /* initiate Zigbee Stack start without zb_send_no_autostart_signal auto-start */
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
	init_uart();
    semaphore = xSemaphoreCreateCounting(TASK_COUNT, 0);
    node1_semaphore = xSemaphoreCreateCounting(4, 0);
    node3_semaphore = xSemaphoreCreateCounting(4, 0);
    node4_semaphore = xSemaphoreCreateCounting(4, 0);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

	esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif
#if CONFIG_EXAMPLE_CONNECT_WIFI
    wifi_connection();
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
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    coex_enable();
    coex_schm_status_bit_set(1, 1);
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif
#endif
    //xTaskCreate(occu_task, "uart test", 4096, NULL, 5, NULL);
    driver_alert_init();
    //xTaskCreate(upload_task, "uploading", 1024*9, NULL, 5, &upload_handle);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096*2, NULL, 5, &zigbee_handle);
    xTaskCreate(upload_node1, "Node 1 Upload", 4096, NULL, 5, NULL);
    xTaskCreate(upload_node3, "Node 3 Upload", 4096, NULL, 5, NULL);
    xTaskCreate(upload_node4, "Node 4 Upload", 4096, NULL, 5, NULL);

}
