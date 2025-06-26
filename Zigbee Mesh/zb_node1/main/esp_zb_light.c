#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "esp_zigbee_core.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include <stdlib.h>
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include <math.h>
#include "driver/uart.h"

#include "led_strip.h"
#define BLINK_GPIO 8
static uint8_t s_led_state = 1;
static led_strip_handle_t led_strip;

uint16_t SENSOR_DATA_CLUSTER_ID = 0xFFF1;
uint16_t SENSOR_DATA_ATTR_ID = 0;
uint16_t SENSOR_DATA_ATTR_ID_2 = 1;
uint16_t SENSOR_DATA_ATTR_ID_3 = 2;
char Current_Date_Time[30];

SemaphoreHandle_t semaphore;
TaskHandle_t sen_handle, mics_handle, dummy, sgp_handle, finish_handle, uart_handle, zb_handle;

static const char *TAG_SDCARD = "SD_CARD";
int heated = 0;
#define TASK_COUNT 2

//GPIO Pins for LED
#define RED_LED 22
#define ORANGE_LED 23
#define YELLOW_LED 27
#define GREEN_LED 26
#define RECIRC_LED 30 //CHANGE ACCORDINGLY
#define INTAKE_LED 30

//I2C0 for SGP30
#define I2C_PORT_0 0
#define I2C0_SCL 10
#define I2C0_SDA 11

//I2C1 for SEN55 and MiCS-4514
#define I2C_PORT_1 1
#define I2C1_SCL 12
#define I2C1_SDA 8

//For Sensor I2C Addresses
#define SEN55_ADDR 0x69
#define SGP30_ADDR 0x58

//General I2C parameters
#define I2C_FREQUENCY 100000

//Global Variables for Sensor Readings
static float pm25, pm10, rh, temp, co, no2, co2, voc;
int no2_adc, co_adc;
float pm25_score, pm10_score, co_score, no2_score, co2_score, voc_score, iaq;
char* location[4];
double lat_deg, long_deg;

//SPI for SD Card Module
#define PIN_NUM_MISO  0
#define PIN_NUM_MOSI  5
#define PIN_NUM_CLK   4
#define PIN_NUM_CS    1
#define MAX_SIZE 220
#define MOUNT_POINT "/sdcard"
int first_write = 1;

//ADC1 for MiCS-4514
adc_channel_t ADC1_CH1_RED = ADC_CHANNEL_1; // For RED ADC Readings
adc_channel_t ADC1_CH2_NOX = ADC_CHANNEL_2; // For NOX ADC Readings
#define PREHEAT_PIN 13 // Pin to pre-heat the module

//UART1 for Ublox Neo 6M V2
#define TX_pin 26
#define RX_pin 27

static const char *TAG = "ESP_ZB_CLIENT_1";
char data1[80] = "K150,1000.00,150,1000.00,85.00,100.00,150,60000.00,150,10000.00,150,1000.00";
char data2[80] = "K150,10.00,00.000000,000.000000,00.00,2020-01-01T12:59:59";
char data3[50] = "K (75 characters) is the max";
//https://github.com/espressif/esp-zigbee-sdk/issues/244

/* Zigbee configuration */
/* T H I S     I S      F O R       E N D       D E V I C E S*/
#define INSTALLCODE_POLICY_ENABLE       false    	/* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    	/* 3000 millisecond */
#define ZB_CLIENT_ENDPOINT_1 			1          /* esp device endpoint */
#define ZB_CLIENT_ENDPOINT_2 			2
#define ZB_CLIENT_ENDPOINT_3 			3
#define ZB_CLIENT_ENDPOINT_4 			4
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 11)  /* Zigbee primary channel mask use in the example */
#define ESP_ZB_SECONDARY_CHANNEL_MASK   (1l << 13)  /* Zigbee primary channel mask use in the example */

uint16_t GATEWAY_TIME_CLUSTER_ID = 0xFFF2;
uint8_t GATEWAY_TIME_ATTR_ID = 3;

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }
#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = RADIO_MODE_NATIVE,                        \
    }
#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,      \
    }

typedef struct zb_coordinator_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} zb_coordinator_t;
zb_coordinator_t zb_coord_info;

typedef struct zdo_info_ctx_s {
    uint8_t endpoint;
    uint16_t short_addr;
} zdo_info_user_ctx_t;

void ReportAttribute (uint16_t attr_id){
	//DELIERY DETAILS
	esp_zb_zcl_report_attr_cmd_t report_here;
	report_here.zcl_basic_cmd.dst_addr_u.addr_short = zb_coord_info.short_addr;
	report_here.zcl_basic_cmd.dst_endpoint = zb_coord_info.endpoint;
	report_here.zcl_basic_cmd.src_endpoint = ZB_CLIENT_ENDPOINT_1;
	report_here.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	report_here.clusterID = SENSOR_DATA_CLUSTER_ID;
	report_here.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE; //SERVER ACTUALLY MEANS I HAVE THE DATA, NOT CLIENT WILL SEND DATA
	report_here.attributeID = attr_id;
	esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(ZB_CLIENT_ENDPOINT_1, SENSOR_DATA_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id);
	if (attr_id == SENSOR_DATA_ATTR_ID){
		memcpy(value_r->data_p, &data1, sizeof(data1));
	}
	else if (attr_id == SENSOR_DATA_ATTR_ID_2){
		memcpy(value_r->data_p, &data2, sizeof(data2));
	}
	else if (attr_id == SENSOR_DATA_ATTR_ID_3){
		memcpy(value_r->data_p, &data3, sizeof(data3));
	}
	esp_zb_zcl_report_attr_cmd_req(&report_here);
}

void ReadAttribute (void){
	esp_zb_zcl_read_attr_cmd_t read_here;
	read_here.zcl_basic_cmd.dst_addr_u.addr_short = zb_coord_info.short_addr;
	read_here.zcl_basic_cmd.dst_endpoint = zb_coord_info.endpoint;
	read_here.zcl_basic_cmd.src_endpoint = ZB_CLIENT_ENDPOINT_1;
	read_here.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
	read_here.clusterID = GATEWAY_TIME_CLUSTER_ID;
	read_here.attributeID = GATEWAY_TIME_ATTR_ID;
	esp_zb_zcl_read_attr_cmd_req(&read_here);
}

static void blink_led(void)
{
	/* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
	led_strip_set_pixel(led_strip, 0, 255, 16, 16);
	/* Refresh the strip to send data */
	led_strip_refresh(led_strip);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

//SD Card R/W Functions
static esp_err_t sd_card_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG_SDCARD, "Opening file %s", path);
    FILE *f = fopen(path, "a+");
    if (f == NULL) {
        ESP_LOGE(TAG_SDCARD, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG_SDCARD, "File written");

    return ESP_OK;
}
static esp_err_t sd_card_read_file(const char *path)
{
    ESP_LOGI(TAG_SDCARD, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG_SDCARD, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[MAX_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG_SDCARD, "Read from file: '%s'", line);

    return ESP_OK;
}
//SD Card Main Functions
void sd_card_write(void){
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG_SDCARD, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    ESP_LOGI(TAG_SDCARD, "Using SPI peripheral");

    // For setting a SD card frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SDCARD, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT(); //SD Card Module has no CS and WP i/o
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG_SDCARD, "Mounting to SD Card...");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG_SDCARD, "Failed to mount");
        } else {
            ESP_LOGE(TAG_SDCARD, "Failed to initialize the card (%s).", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG_SDCARD, "SD Card mounted");

    // Card has been initialized, print its properties
    //sdmmc_card_print_info(stdout, card);

    // Directory for "aq_log.txt"
    const char *aq_log_file = MOUNT_POINT"/aq_log.csv";
    char log[MAX_SIZE];

    if (first_write == 1){
    	//Write Headers
    	snprintf(log, MAX_SIZE, "Time, CO (ppb), CO Index, CO2 (ppm), CO2 Index, NO2 (ppb), NO2 Index, PM2.5 (ug/m3), PM2.5 Index, PM10 (ug/m3), PM10 Index, VOC (ppb), VOC Index, Temperature (C), RH (%s)\n", "%%");
    	ret = sd_card_write_file(aq_log_file, log);
        if (ret != ESP_OK) {
            return;
        }
    	first_write = 0;
    }
    // Set up data to be logged
    snprintf(log, MAX_SIZE, "%s, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.2f\n", Current_Date_Time, co, co_score, co2, co2_score, no2, no2_score, pm25, pm25_score, pm10, pm10_score, voc, voc_score, temp, rh);
    //printf("%s\n", log);
    // Write data log to aq_log.txt
    ret = sd_card_write_file(aq_log_file, log);
    if (ret != ESP_OK) {
        return;
    }
    //Open file for reading
/*    ret = sd_card_read_file(aq_log_file);
    if (ret != ESP_OK) {
        return;
    }*/

    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG_SDCARD, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}

//Functions for LED Alert System
void led_alert_init (void){
	gpio_reset_pin(RED_LED);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
	gpio_reset_pin(ORANGE_LED);
	gpio_set_direction(ORANGE_LED, GPIO_MODE_OUTPUT);
	gpio_reset_pin(YELLOW_LED);
	gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
	gpio_reset_pin(GREEN_LED);
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(RED_LED, 1);
	gpio_set_level(ORANGE_LED, 1);
	gpio_set_level(YELLOW_LED, 1);
	gpio_set_level(GREEN_LED, 1);
}
void update_led_alert (float pm25, float pm10, float rh, float temp, float co, float no2, float co2, float voc){
	//Condition for RED Level AQI
	if (pm25 > 30 || pm10 > 100 || co > 70 || co2 > 1500 || no2 > 250 || voc > 800){
		gpio_set_level(RED_LED, 1);
		gpio_set_level(ORANGE_LED, 0);
		gpio_set_level(YELLOW_LED, 0);
		gpio_set_level(GREEN_LED, 0);
	}
	//Condition for ORANGE Level AQI
	else if (pm25 > 20 || pm10 > 75 || co > 50 || co2 > 1150 || no2 > 175 || voc > 600){
		gpio_set_level(RED_LED, 0);
		gpio_set_level(ORANGE_LED, 1);
		gpio_set_level(YELLOW_LED, 0);
		gpio_set_level(GREEN_LED, 0);
	}
	//Condition for YELLOW Level AQI
	else if (pm25 > 15 || pm10 > 50 || co > 35 || co2 > 800 || no2 > 100 || voc > 400){
		gpio_set_level(RED_LED, 0);
		gpio_set_level(ORANGE_LED, 0);
		gpio_set_level(YELLOW_LED, 1);
		gpio_set_level(GREEN_LED, 0);
	}
	//Condition for GREEN Level AQI
	else if (pm25 > 0 || pm10 > 0 || co > 0 || co2 > 0 || no2 > 0 || voc > 0){
		gpio_set_level(RED_LED, 0);
		gpio_set_level(ORANGE_LED, 0);
		gpio_set_level(YELLOW_LED, 0);
		gpio_set_level(GREEN_LED, 1);
	}
	//negative values are encountered, possible code error
	else{
		gpio_set_level(RED_LED, 0.4);
		gpio_set_level(ORANGE_LED, 0.4);
		gpio_set_level(YELLOW_LED, 0.4);
		gpio_set_level(GREEN_LED, 0.4);
	}
}
void iaq_score(float pm25, float pm10, float co, float no2, float co2, float voc){
	float index[4][2] = {{0,50}, {51,75}, {76,100}, {101,150}};
	float co_bp[4][2] = {{0,35}, {36,50}, {51,70}, {70,1000}};
	float co2_bp[4][2] = {{0,800}, {801,1150}, {1151,1500}, {1501,60000}};
	float pm25_bp[4][2] = {{0,15}, {16,20}, {21,30}, {31,1000}};
	float pm10_bp[4][2] = {{0,50}, {51,75}, {76,100}, {101,1000}};
	float no2_bp[4][2] = {{0,100}, {101,175}, {176,250}, {251,10000}};
	float voc_bp[4][2] = {{0,400}, {401,600}, {601,800}, {801,60000}};
	//PM2.5
	for (int j=3; j>=0; j--){
		if(pm25 >= pm25_bp[j][0]){
			pm25_score = (((index[j][1]-index[j][0])/(pm25_bp[j][1]-pm25_bp[j][0]))*(pm25-pm25_bp[j][0]))+(index[j][0]);
			break;
		}
	}
	//PM10
	for (int j=3; j>=0; j--){
		if(pm10 >= pm10_bp[j][0]){
			pm10_score = (((index[j][1]-index[j][0])/(pm10_bp[j][1]-pm10_bp[j][0]))*(pm10-pm10_bp[j][0]))+(index[j][0]);
			break;
		}
	}
	//CO
	for (int j=3; j>=0; j--){
		if(co >= co_bp[j][0]){
			co_score = (((index[j][1]-index[j][0])/(co_bp[j][1]-co_bp[j][0]))*(co-co_bp[j][0]))+(index[j][0]);
			break;
		}
	}
	//NO2
	for (int j=3; j>=0; j--){
		if(no2 >= no2_bp[j][0]){
			no2_score = (((index[j][1]-index[j][0])/(no2_bp[j][1]-no2_bp[j][0]))*(no2-no2_bp[j][0]))+(index[j][0]);
			break;
		}
	}
	//CO2
	for (int j=3; j>=0; j--){
		if(co2 >= co2_bp[j][0]){
			co2_score = (((index[j][1]-index[j][0])/(co2_bp[j][1]-co2_bp[j][0]))*(co2-co2_bp[j][0]))+(index[j][0]);
			break;
		}
	}
	//VOC
	for (int j=3; j>=0; j--){
		if(voc >= voc_bp[j][0]){
			voc_score = (((index[j][1]-index[j][0])/(voc_bp[j][1]-voc_bp[j][0]))*(voc-voc_bp[j][0]))+(index[j][0]);
			break;
		}
	}

}
void driver_alert_init (void){
	gpio_reset_pin(RECIRC_LED);
	gpio_set_direction(RECIRC_LED, GPIO_MODE_OUTPUT);
	gpio_reset_pin(INTAKE_LED);
	gpio_set_direction(INTAKE_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(INTAKE_LED, 0);
	gpio_set_level(RECIRC_LED, 0);
}
void driver_alert(float pm25_in, float pm10_in, float co_in, float no2_in, float co2_in, float voc_in,
				  float pm25_out, float pm10_out, float co_out, float no2_out, float co2_out, float voc_out){
	if (co_in > co_out || no2_in > no2_out || co2_in > co2_out || voc_in > voc_out){
		gpio_set_level(INTAKE_LED, 1); // Accumulated gas concentrations (Orange level)
		gpio_set_level(RECIRC_LED, 0);
	}
	else{
		gpio_set_level(INTAKE_LED, 0);
		gpio_set_level(RECIRC_LED, 1);
	}
}

//Warm-Up for MiCS-4514 for 3 minutes
void sensor_init(int *heated_var){
	gpio_reset_pin(PREHEAT_PIN);
	gpio_set_direction(PREHEAT_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(PREHEAT_PIN, 1);

	int i2c_master_port0 = I2C_PORT_0;
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_SDA,
        .scl_io_num = I2C0_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };
    i2c_param_config(i2c_master_port0, &conf0);
    i2c_driver_install(i2c_master_port0, conf0.mode, 0, 0, 0);
    uint8_t SGP30_INIT[2] = {0x20, 0x03}; //Initialize SGP30
    i2c_master_write_to_device(i2c_master_port0, SGP30_ADDR, SGP30_INIT, 2, 1000/portTICK_PERIOD_MS);
	if(heated_var == 0){
		printf("Pre-heating MiCS-4514...\n");
		vTaskDelay(180000/portTICK_PERIOD_MS); // Preaheat MiCS for 3 minutes
	}
	else{
		printf("Waiting for SGP30...\n");
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
//For mapping MiCS-4514 raw readings
void process_mics4514(int D_co, int D_no2) {
	//Source https://myscope.net/auswertung-der-airpi-gas-sensoren/
	//For CO readings
	printf("D_OUTS: %d, %d\n", D_co, D_no2);
	double Vo_co = (D_co-2144) * (1.944)/(4081-2144);
	double Vo_no2 = (D_no2-2144) * (1.944)/(4081-2144);
	printf("VOLTAGE: %.5f  %.5f\n", Vo_co, Vo_no2);
	double Rs_co = (5-Vo_co) / (Vo_co/820);
	double Rs_no2 = (5-Vo_no2) / (Vo_no2/820);
	printf("RESISTANCE: %.2f  %.2f\n", Rs_co, Rs_no2);
	double R0_co = 2976.59; // FOR CALIBRATION (4.00 * R0 = Rs @ 0.8 ppm CO)
	co = pow(10, -1.1859 * (log(Rs_co/R0_co) / M_LN10) + 0.6201); //Curve Fitting Equation from Source
	//For NO2 readings
	double R0_no2 = 1440000; // FOR CALIBRATION (0.05*R0 = Rs @ 0.01 ppm NO2)
	no2 = pow(10, 0.9682 * (log(Rs_no2/R0_no2) / M_LN10) - 0.8108); //Curve Fitting Equation from Source
	//printf("CO: %.2f, NO2: %.2f\n", co, no2);
}
//RTOS Task for MiCS-4514 @ ADC1 Channel 1 & 2
void ADC1_Data_Task(void *params){
	//Initialize ADC1 Peripheral
	adc_oneshot_unit_handle_t adc1_handle;
	adc_oneshot_unit_init_cfg_t init_config1 = {
		    .unit_id = ADC_UNIT_1,
			.clk_src = ADC_DIGI_CLK_SRC_RC_FAST,
			.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	printf("ADC INIT\n");
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//Configure the 2 ADC Channels
	adc_oneshot_chan_cfg_t config1 = {
		     .atten = ADC_ATTEN_DB_11,
	};
	adc_oneshot_chan_cfg_t config2 = {
		     .atten = ADC_ATTEN_DB_11,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CH1_RED, &config1)); // For RED analog readings
	vTaskDelay(100/portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CH2_NOX, &config2)); // For NOX analog readings
    vTaskDelay(100/portTICK_PERIOD_MS);
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CH1_RED, &co_adc)); // Get readings from RED
        vTaskDelay(10/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CH2_NOX, &no2_adc)); // Get readings from NOX
        //printf("CHECK: co: %d, no2: %d\n", co_adc, no2_adc);
        //printf("V_CHECK: co: %d, no2: %d\n", co_v, no2_v);
        process_mics4514(co_adc, no2_adc);
        //printf("%d", uxTaskGetStackHighWaterMark(NULL));
        //printf("CO: %.2f, NO2: %.2f\n", co, no2);
        xSemaphoreGive(semaphore);
        vTaskSuspend(mics_handle);
    }
}
//Process SEN55 Data
void process_sen55(uint8_t data[24]){
	//Raw Bit Data
	//PM2.5 Bits
	uint16_t pm25_bits = data[3] << 8; //First 8 bits
			 pm25_bits |= data[4]; //Add last 8 bits
	//PM10 Bits
	uint16_t pm10_bits = data[9] << 8; //First 8 bits
			 pm10_bits |= data[10]; //Add last 8 bits
	//Humidity Bits
	int16_t rh_bits = data[12]; //First 8 bits
	 	 	rh_bits <<= 8; //Shift by 8 bits
	 	 	rh_bits |= data[13]; //Add last 8 bits
	//Temperature Bits
	 int16_t temp_bits = data[15]; //First 8 bits
	 	 	 temp_bits <<= 8; //Shift by 8 bits
	 	 	 temp_bits |= data[16]; //Add last 8 bits


	//Divide by Scaling Factors
	float pm25_raw = pm25_bits;
	float pm10_raw = pm10_bits;
	float rh_raw = rh_bits;
	float temp_raw = temp_bits;

	pm25 = pm25_raw /10;
	pm10 = pm10_raw /10;
	rh = rh_raw / 100;
	temp = temp_raw / 200;
	//Node 1
	pm25 = 1.9467002394827233*pm25 + (2.5926092705655464);
	pm10 = 2.3350992860750863*pm10 + (1.1038457405169595);
	temp = 0.7311396775389937*temp + (5.1184212550840265);
	rh = 0.8405511007613504*rh + (-1.3350502127248518);


}
//SEN55 and SGP30 CRC Calculation
uint8_t CalcCrc(uint8_t data[2]) {
	uint8_t crc = 0xFF;
	for(int i = 0; i < 2; i++) {
		crc ^= data[i];
		for(uint8_t bit = 8; bit > 0; --bit) {
			if(crc & 0x80) {
				crc = (crc << 1) ^ 0x31u;
			}
			else {
				crc = (crc << 1);
			}
		}
	 }
	 return crc;
}
//RTOS Task for SEN55 @ I2C1
void I2C1_Data_Task(void *params){
	//SEN55 Initialize
	i2c_config_t conf1 = {
			.mode = I2C_MODE_MASTER,
	        .sda_io_num = I2C1_SDA,
	        .scl_io_num = I2C1_SCL,
	        .sda_pullup_en = GPIO_PULLUP_ENABLE,
	        .scl_pullup_en = GPIO_PULLUP_ENABLE,
	        .master.clk_speed = I2C_FREQUENCY,
	};
	i2c_param_config(I2C_PORT_1, &conf1);
	i2c_driver_install(I2C_PORT_1, conf1.mode, 0, 0, 0);
	uint8_t SEN55_MEAS_INIT[2] = {0x00,0x21}; //Start Measurement Mode
    uint8_t sen55_data[24];
	uint8_t SEN55_READ_DATA[2] = {0x03,0xC4}; //Get Measured Values Command
	ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT_1, SEN55_ADDR, SEN55_MEAS_INIT, 2, 1000/portTICK_PERIOD_MS)); //Measurement Mode
	vTaskDelay(10/portTICK_PERIOD_MS);
    while (1) {
    	//For SEN55 (Starts at Idle Mode)
    	ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT_1, SEN55_ADDR, SEN55_READ_DATA, 2, 1000/portTICK_PERIOD_MS)); //Send Get-Measurement Command
    	vTaskDelay(20/portTICK_PERIOD_MS);
    	ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_PORT_1, SEN55_ADDR, sen55_data, 24, 1000/portTICK_PERIOD_MS)); //Read Measurements
    	vTaskDelay(10/portTICK_PERIOD_MS);
    	process_sen55(sen55_data); //Update Global Variables
    	//printf("%d", uxTaskGetStackHighWaterMark(NULL));
    	//printf("PM2.5: %.2f, PM10: %.2f, Temperature: %.02f, RH: %.2f\n", pm25, pm10, temp, rh);
        xSemaphoreGive(semaphore);
        vTaskSuspend(sen_handle);
    }
}
//RTOS Task for SGP30 @ I2C0
void I2C0_Data_Task(void *params){
    int i2c_master_port = I2C_PORT_0;
    uint8_t sgp30_data[6];
	uint8_t SGP30_MEAS[2] = {0x20, 0x08};
	while (true) {
		ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, SGP30_ADDR, SGP30_MEAS, 2, 1000/portTICK_PERIOD_MS)); //Send Measurement Command
    	vTaskDelay(15/portTICK_PERIOD_MS);
		i2c_master_read_from_device(i2c_master_port, SGP30_ADDR, sgp30_data, 6, 1000/portTICK_PERIOD_MS); //Read Measured Values
		//Process received bit sequence
		uint16_t co2_bits = sgp30_data[0] << 8;
		co2_bits |= sgp30_data[1];
		uint16_t voc_bits = sgp30_data[3] << 8;
		voc_bits |= sgp30_data[4];
		float co2_raw = co2_bits;
		float voc_raw = voc_bits;
		co2 = co2_raw;
		voc = voc_raw;
    	vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
//RTOS Task for All_data_collected
void Data_Complete_Task (void *params){
	while(1){
		if(uxSemaphoreGetCount(semaphore) == TASK_COUNT){
			for (int i=0; i == TASK_COUNT-1; i++){
				xSemaphoreTake(semaphore, portMAX_DELAY);
			}
			ReadAttribute();
			vTaskDelay(1000/portTICK_PERIOD_MS);

			update_led_alert(pm25, pm10, temp, rh, co, no2, co2, voc);
			iaq_score(pm25, pm10, co, no2, co2, voc);

			//snprintf(data1, sizeof(data1), "%.0f,%.2f,%.0f,%.2f,%.2f,%.2f,%.0f,%.2f", pm25_score, pm25, pm10_score, pm10, temp, rh, co2_score, co2);
		    //snprintf(data2, sizeof(data2), "%.0f,%.2f,%.0f,%.2f,%.0f,%.2f", voc_score, voc, co_score, co, no2_score, no2); //INDOOR VERSION
			//snprintf(data3, sizeof(data3), "%s", Current_Date_Time);


			// String Send (oK-Edition)
			snprintf(data1, sizeof(data1), "K%.0f,%.2f,%.0f,%.2f,%.2f,%.2f,%.0f,%.2f,%.0f,%.2f,%.0f,%.2f", pm25_score, pm25, pm10_score, pm10, temp, rh, co2_score, co2, voc_score, voc, co_score, co);
			snprintf(data2, sizeof(data2), "K%.0f,%.2f,%s", no2_score, no2, Current_Date_Time);


			printf("%s\n", data1);
			printf("%s\n", data2);
			ReportAttribute(SENSOR_DATA_ATTR_ID);
			vTaskDelay(50/portTICK_PERIOD_MS);
			ReportAttribute(SENSOR_DATA_ATTR_ID_2);
			vTaskDelay(50/portTICK_PERIOD_MS);

			sd_card_write();
			vTaskDelay(9900/portTICK_PERIOD_MS); //THROUGHPUT
			//vTaskResume(sgp_handle);
			vTaskResume(mics_handle);
			vTaskResume(sen_handle);
		}
		else{
			vTaskDelay(500/portTICK_PERIOD_MS);
		}
	}
}
 //GPS (UART), Driver Alert
void UART_Data_Task(void *params){
	//Start UART Here
	const uart_port_t uart_num = UART_NUM_1;
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

	const char* filter_str = "$GPGGA";
	char *p = NULL;
	char str1[100];
	int j=0;
	while(1){
		int length;
		ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
		char gps_out[length];

		uart_read_bytes(uart_num, gps_out, length, 1000);

		p = strstr(gps_out,filter_str);
		if(p){
			for (int i=0; p[i] != '\n'; i++){
				str1[j] = p[i];
				j++;
			}
			str1[j+1] = '\n';
			j = 0;
			char* token = strtok(str1, ",");
			token = strtok(NULL, ",");
			for (int i=0; i<4; i++){
				token = strtok(NULL,",");
				location[i] = token;
			}
			double latitude = atof(location[0]);
			double longitude = atof(location[2]);

			int latitude_decimal = floor(latitude/100);
			int longitude_decimal = floor(longitude/100);

			if(latitude_decimal == 0 && longitude_decimal == 0){
				printf("No GPGGA Signal\n");
				xSemaphoreGive(semaphore);
				vTaskSuspend(uart_handle);
			}
			else{
				double latitude_decimal_minutes = modf(latitude/100, &latitude);
				latitude_decimal_minutes = latitude_decimal_minutes*100;
				double longitude_decimal_minutes = modf(longitude/100, &longitude);
				longitude_decimal_minutes = longitude_decimal_minutes*100;

				lat_deg = latitude_decimal + latitude_decimal_minutes/60;
				long_deg = longitude_decimal + longitude_decimal_minutes/60;
				//printf("%.2f, %.2f\n", lat_deg, long_deg);
				xSemaphoreGive(semaphore);
				vTaskSuspend(uart_handle);
			}
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
		//Task Complete Procedure
	}
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bind response from address(0x%x), endpoint(%d) with status(%d)", ((zdo_info_user_ctx_t *)user_ctx)->short_addr,
                 ((zdo_info_user_ctx_t *)user_ctx)->endpoint, zdo_status);
/*         configure report attribute command*/
        esp_zb_zcl_config_report_cmd_t report_cmd;
        bool report_change = 1;
        report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = zb_coord_info.short_addr;
        report_cmd.zcl_basic_cmd.dst_endpoint = zb_coord_info.endpoint;
        report_cmd.zcl_basic_cmd.src_endpoint = ZB_CLIENT_ENDPOINT_1;
        report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        report_cmd.clusterID = SENSOR_DATA_CLUSTER_ID;                      /*!< Cluster ID to report */
        report_cmd.attributeID = SENSOR_DATA_ATTR_ID;                     /*!< Attribute ID to report */
        report_cmd.attrType = ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING;                              /*!< Attribute type to report refer to zb_zcl_common.h zcl_attr_type */
        report_cmd.reportable_change = &report_change;
        esp_zb_zcl_config_report_cmd_req(&report_cmd);
    }
}

static void ieee_cb(esp_zb_zdp_status_t zdo_status, esp_zb_ieee_addr_t ieee_addr, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        memcpy(&(zb_coord_info.ieee_addr), ieee_addr, sizeof(esp_zb_ieee_addr_t));
        ESP_LOGI(TAG, "IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
                 ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);


        /* BIND the on-off light to on-off switch */
        esp_zb_zdo_bind_req_param_t bind_req;
        memcpy(&(bind_req.src_address), zb_coord_info.ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req.src_endp = zb_coord_info.endpoint;
        bind_req.cluster_id = SENSOR_DATA_CLUSTER_ID;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
        bind_req.dst_endp = ZB_CLIENT_ENDPOINT_1;
        bind_req.req_dst_addr = zb_coord_info.short_addr;
        static zdo_info_user_ctx_t test_info_ctx;
        test_info_ctx.endpoint = ZB_CLIENT_ENDPOINT_1;
        test_info_ctx.short_addr = zb_coord_info.short_addr;
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *) & (test_info_ctx));
    }
}

static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Active endpoint response: status(%d) and endpoint count(%d)", zdo_status, ep_count);
        for (int i = 0; i < ep_count; i++) {
            ESP_LOGI(TAG, "Endpoint ID List: %d", ep_id_list[i]);
        }
    }
}

static void simple_desc_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Simple desc response: status(%d), device_id(%d), app_version(%d), profile_id(0x%x), endpoint_ID(%d)", zdo_status,
                 simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);

        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            ESP_LOGI(TAG, "Cluster ID list: 0x%x", *(simple_desc->app_cluster_list + i));
        }
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Match desc response: status(%d), address(0x%x), endpoint(%d)", zdo_status, addr, endpoint);
        /* save into remote device record structure for future use */
        zb_coord_info.endpoint = endpoint;
        zb_coord_info.short_addr = addr;
        /* find the active endpoint */
        esp_zb_zdo_active_ep_req_param_t active_ep_req;
        active_ep_req.addr_of_interest = zb_coord_info.short_addr;
        esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, NULL);
        /* get the node simple descriptor */
        esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
        simple_desc_req.addr_of_interest = addr;
        simple_desc_req.endpoint = endpoint;
        esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, NULL);
        /* get the light ieee address */
        esp_zb_zdo_ieee_addr_req_param_t ieee_req;
        ieee_req.addr_of_interest = zb_coord_info.short_addr;
        ieee_req.dst_nwk_addr = zb_coord_info.short_addr;
        ieee_req.request_type = 0;
        ieee_req.start_index = 0;
        esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_cb, NULL);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            /* device auto start successfully and on a formed network */
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_zdo_match_desc_req_param_t  find_req;
            find_req.addr_of_interest = 0x0000;
            find_req.dst_nwk_addr = 0x0000;
            /* find the match on-off light device */
            esp_zb_zdo_find_on_off_light(&find_req, user_find_cb, NULL);

        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
        }
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
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->src_address.u.short_addr,
             message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Received report information: attribute(0x%x), type(0x%x), value(%d)\n", message->attribute.id, message->attribute.data.type,
             message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);
    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message){
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received report information: attribute(0x%x), type(0x%x), value(%s)", message->attribute.id, message->attribute.data.type,
             (char *)message->attribute.data.value);
    char buffer[100];
    snprintf(buffer, 100, "%s", (char* )message->attribute.data.value);

    strcpy(Current_Date_Time, buffer);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
	//THESE ARE EXECUTED WHEN A CALLBACK ID IS RECEVIED BY THIS DEVICE
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    //IMPORTANT CASE FOR RECEIVING ATTRIBUTE REPORTS
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    //IMPORTANT CASE FOR CONFIGURING A REPORT (ATTRIBUTE REPORT) TO THE PROPER FORMAT
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    //IMPORTANT CASE WHEN SETTING ATTRIBUTE TO A SPECIFIC VALUE
    //case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        //ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        //break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
    	ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
    	break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    const uint8_t attr_access = ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE;

    esp_zb_attribute_list_t *esp_zb_sensordata_cluster = esp_zb_zcl_attr_list_create(SENSOR_DATA_CLUSTER_ID);
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(esp_zb_sensordata_cluster, SENSOR_DATA_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, attr_access, &data1));
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(esp_zb_sensordata_cluster, SENSOR_DATA_ATTR_ID_2, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, attr_access, &data2));
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(esp_zb_sensordata_cluster, SENSOR_DATA_ATTR_ID_3, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, attr_access, &data3));
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_sensordata_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    //Set Endpoint HERE
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, ZB_CLIENT_ENDPOINT_1, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID);

    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_secondary_network_channel_set(ESP_ZB_SECONDARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

void app_main(void)
{
	//INDOOR MODE SETUP -- LAST EDIT
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    semaphore = xSemaphoreCreateCounting(TASK_COUNT, 0);
	heated = 1;
	sensor_init(&heated);

	//configure_led();
	led_alert_init(); //Indoor

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, &zb_handle); //START ZIGBEE
    vTaskDelay(10000/portTICK_PERIOD_MS);
	xTaskCreate(ADC1_Data_Task, "ADC1 Data Task", 1024*3, NULL, 5, &mics_handle);
	xTaskCreate(I2C0_Data_Task, "I2C0 Data Task", 1024*3, NULL, 5, &sgp_handle);
	xTaskCreate(I2C1_Data_Task, "I2C1 Data Task", 1024*2, NULL, 5, &sen_handle);
    xTaskCreate(Data_Complete_Task, "Data Upload and Save", 4096, NULL, 5, &finish_handle);


}
