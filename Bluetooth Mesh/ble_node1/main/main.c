/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

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
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>
#include "driver/uart.h"

#define TAG "EXAMPLE"

#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

uint16_t my_net_idx;
uint16_t my_addr;

char Current_Date_Time[20];
int latitude_decimal;
int longitude_decimal;
// SemaphoreHandle_t semaphore;
TaskHandle_t sen_handle, mics_handle, dummy, sgp_handle, finish_handle, uart_handle, ot_handle;

static const char *TAG_SDCARD = "SD_CARD";
int heated = 0;
#define TASK_COUNT 3

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
float pm25_score, pm10_score, co_score, no2_score, co2_score, voc_score, iaq, speed_kmh;
float speed_kmh = 0;
char* location[4];
char* velocity[10];
double lat_deg = 0;
double long_deg = 0;
char* speed_str = "0";

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

char data_payload[500] = "{\"COraw\":\"1000.00\",\"COindex\":\"150\",\"CO2raw\":\"60000.00\",\"CO2index\":\"150\",\"NO2raw\":\"1000.00\",\"NO2index\":\"150\",\"PM25raw\":\"1000.00\",\"PM25index\":\"150\",\"PM10raw\":\"1000.00\",\"PM10index\":\"150\",\"VOCraw\":\"10000.00\",\"VOCindex\":\"150\",\"T\":\"100.00\",\"RH\":\"100.00\",\"source\":\"Node 1\",\"local_time\":\"2024-05-05T12:12:12\",\"type\":\"data\"}";

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
    	snprintf(log, MAX_SIZE, "Time, CO (ppb), CO Index, CO2 (ppm), CO2 Index, NO2 (ppb), NO2 Index, PM2.5 (ug/m3), PM2.5 Index, PM10 (ug/m3), PM10 Index, VOC (ppb), VOC Index, Temperature (C), RH (%s), Latitude, Longitude, Speed (km/h)\n", "%%");
    	ret = sd_card_write_file(aq_log_file, log);
        if (ret != ESP_OK) {
            return;
        }
    	first_write = 0;
    }
    // Set up data to be logged
    snprintf(log, MAX_SIZE, "%s, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.0f, %.2f, %.2f, %.7f, %.7f, %.2f\n", Current_Date_Time, co, co_score, co2, co2_score, no2, no2_score, pm25, pm25_score, pm10, pm10_score, voc, voc_score, temp, rh, lat_deg, long_deg, speed_kmh);
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
//		led_strip_set_pixel(led_strip, 0, 255, 0, 0);
//		led_strip_refresh(led_strip);
	}
	//Condition for ORANGE Level AQI
	else if (pm25 > 20 || pm10 > 75 || co > 50 || co2 > 1150 || no2 > 175 || voc > 600){
		gpio_set_level(RED_LED, 0);
		gpio_set_level(ORANGE_LED, 1);
		gpio_set_level(YELLOW_LED, 0);
		gpio_set_level(GREEN_LED, 0);
//		led_strip_set_pixel(led_strip, 0, 255, 165, 0);
//		led_strip_refresh(led_strip);
	}
	//Condition for YELLOW Level AQI
	else if (pm25 > 15 || pm10 > 50 || co > 35 || co2 > 800 || no2 > 100 || voc > 400){
		gpio_set_level(RED_LED, 0);
		gpio_set_level(ORANGE_LED, 0);
		gpio_set_level(YELLOW_LED, 1);
		gpio_set_level(GREEN_LED, 0);
//		led_strip_set_pixel(led_strip, 0, 255, 255, 0);
//		led_strip_refresh(led_strip);
	}
	//Condition for GREEN Level AQI
	else if (pm25 > 0 || pm10 > 0 || co > 0 || co2 > 0 || no2 > 0 || voc > 0){
		gpio_set_level(RED_LED, 0);
		gpio_set_level(ORANGE_LED, 0);
		gpio_set_level(YELLOW_LED, 0);
		gpio_set_level(GREEN_LED, 1);
//		led_strip_set_pixel(led_strip, 0, 0, 255, 0);
//		led_strip_refresh(led_strip);
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
		printf("Preheating MiCS-4514...\n");
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
	// printf("D_OUTS: %d, %d\n", D_co, D_no2);
	double Vo_co = 3.55 * (D_co-2113)/(4081-2113);
	double Vo_no2 = 3.55 * (D_no2-2113)/(4081-2113);
	// printf("VOLTAGE: %.5f  %.5f\n", Vo_co, Vo_no2);
	double Rs_co = (5-Vo_co) / (Vo_co/820);
	double Rs_no2 = (5-Vo_no2) / (Vo_no2/820);
	// printf("RESISTANCE: %.2f  %.2f\n", Rs_co, Rs_no2);
	double R0_co = 950; // FOR CALIBRATION (4.00 * R0 = Rs @ 0.8 ppm CO)
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
			.clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
			.ulp_mode = ADC_ULP_MODE_RISCV,
	};
	printf("ADC INIT\n");
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//Configure the 2 ADC Channels
	adc_oneshot_chan_cfg_t config1 = {
		     .atten = ADC_ATTEN_DB_11,
			 .bitwidth = ADC_BITWIDTH_DEFAULT
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CH1_RED, &config1)); // For RED analog readings
	vTaskDelay(100/portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CH2_NOX, &config1)); // For NOX analog readings
    vTaskDelay(100/portTICK_PERIOD_MS);
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CH1_RED, &co_adc)); // Get readings from RED
        vTaskDelay(10/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CH2_NOX, &no2_adc)); // Get readings from NOX
        process_mics4514(co_adc, no2_adc);
		if(co != co){
			co = 0;
		}
		if(no2 != no2){
			no2 = 0;
		}
        //printf("%d", uxTaskGetStackHighWaterMark(NULL));
        // printf("MiCS-4514 Readings - CO: %.2f, NO2: %.2f\n", co, no2);
        // xSemaphoreGive(semaphore);
        // vTaskSuspend(mics_handle);
        vTaskDelay(1000/portTICK_PERIOD_MS);
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
// CALIBRATION
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
    	// printf("SEN55 Readings : PM2.5: %.2f, PM10: %.2f, Temperature: %.02f, RH: %.2f\n", pm25, pm10, temp, rh);
        // xSemaphoreGive(semaphore);
        // vTaskSuspend(sen_handle);
        vTaskDelay(1000/portTICK_PERIOD_MS);
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
		// printf("SGP30 Readings - CO2: %.2f, VOC: %.2f\n", co2, voc);
    	// xSemaphoreGive(semaphore);
    	// vTaskSuspend(sgp_handle); // So only 1 semaphore is given in case a task is faster than the other
    	vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
//RTOS Task for All_data_collected
// void Data_Complete_Task (void *params){
// 	while(1){
// 		if(uxSemaphoreGetCount(semaphore) == TASK_COUNT){
// 			for (int i=0; i == TASK_COUNT; i++){
// 				xSemaphoreTake(semaphore, portMAX_DELAY);
// 			}
// 			//printf("Data Upload/Network and Alerts Task\n\n");
// 			//printf("SGP30 Task State: %d\n", eTaskGetState(sgp_handle));
// 			//printf("MiCS-4514 Task State: %d\n", eTaskGetState(mics_handle));
// 			//printf("SEN55 Task State: %d\n\n", eTaskGetState(sen_handle));


// 			// ReadAttribute(); // For Zigbee
// 			// vTaskDelay(1000/portTICK_PERIOD_MS); // For Zigbee
// 			// snprintf(Current_Date_Time, sizeof(Current_Date_Time), "%s", Rx_Date_Time);
// 			// printf("%s\n", Current_Date_Time);
// 			// Current_Date_Time = Rx_Date_Time;
// 			update_led_alert(pm25, pm10, temp, rh, co, no2, co2, voc);
// 			iaq_score(pm25, pm10, co, no2, co2, voc);
			
// 		// ZIGBEE
// 			// snprintf(data1, sizeof(data1), "%.0f,%.2f,%.0f,%.2f,%.2f,%.2f,%.0f,%.2f", pm25_score, pm25, pm10_score, pm10, temp, rh, co2_score, co2);
// 			// snprintf(data2, sizeof(data2), "%.0f,%.2f,%.0f,%.2f,%.0f,%.2f", voc_score, voc, co_score, co, no2_score, no2);
// 			// snprintf(data3, sizeof(data3), "%.7f,%.7f,%.2f,%s", lat_deg, long_deg, speed_kmh, Current_Date_Time); // Lat, Long, Speed, Time
// 		    // esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(ZB_CLIENT_ENDPOINT_4, SENSOR_DATA_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, SENSOR_DATA_ATTR_ID_3);
// 		    // snprintf(data3, sizeof(data3), "%s%s", (char *)value_r->data_p, Current_Date_Time);
// 			// printf("%s\n", data1);
// 			// printf("%s\n", data2);
// 			// printf("%s\n", data3);
// 			// vTaskDelay(100/portTICK_PERIOD_MS);
// 			// ReportAttribute(SENSOR_DATA_ATTR_ID);
// 			// vTaskDelay(100/portTICK_PERIOD_MS);
// 			// ReportAttribute(SENSOR_DATA_ATTR_ID_2);
// 			// vTaskDelay(100/portTICK_PERIOD_MS);
// 			// ReportAttribute(SENSOR_DATA_ATTR_ID_3);

// 		//Thread
// 			// snprintf(data_payload, 500, "{\"COraw\":\"%.2f\",\"COindex\":\"%.0f\",\"CO2raw\":\"%.2f\",\"CO2index\":\"%.0f\",\"NO2raw\":\"%.2f\",\"NO2index\":\"%.0f\",\"PM25raw\":\"%.2f\",\"PM25index\":\"%.0f\",\"PM10raw\":\"%.2f\",\"PM10index\":\"%.0f\",\"VOCraw\":\"%.2f\",\"VOCindex\":\"%.0f\",\"T\":\"%.2f\",\"RH\":\"%.2f\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}",co,co_score,co2,co2_score,no2,no2_score,pm25,pm25_score,pm10,pm10_score,voc,voc_score,temp,rh,Current_Date_Time);
// 			// snprintf(data_payload, 150, "1,%.0f,%.2f,%.0f,%.2f,%.2f,%.2f,%.0f,%.2f,%.0f,%.2f,%.0f,%.2f,%.0f,%.2f,%s", pm25_score, pm25, pm10_score, pm10, temp, rh, co2_score, co2,voc_score, voc, co_score, co, no2_score, no2, Current_Date_Time);
// 			// printf("%s\n", data_payload);
// 			// printf("Data String: %s\n", data_payload);
// 			// udp_send();

//             snprintf(data_payload, 500, "{\"COraw\":\"%.2f\",\"COindex\":\"%.0f\",\"CO2raw\":\"%.2f\",\"CO2index\":\"%.0f\",\"NO2raw\":\"%.2f\",\"NO2index\":\"%.0f\",\"PM25raw\":\"%.2f\",\"PM25index\":\"%.0f\",\"PM10raw\":\"%.2f\",\"PM10index\":\"%.0f\",\"VOCraw\":\"%.2f\",\"VOCindex\":\"%.0f\",\"T\":\"%.2f\",\"RH\":\"%.2f\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}",co,co_score,co2,co2_score,no2,no2_score,pm25,pm25_score,pm10,pm10_score,voc,voc_score,temp,rh,Current_Date_Time);

// 			sd_card_write();
// 			vTaskDelay(30000/portTICK_PERIOD_MS);
// 			vTaskResume(sgp_handle);
// 			vTaskResume(mics_handle);
// 			vTaskResume(sen_handle);
// 			//vTaskResume(uart_handle);
// 		}
// 		else{
// 			vTaskDelay(1000/portTICK_PERIOD_MS);
// 		}
// 	}
// }
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
	const int uart_buffer_size = (1024*3);
	QueueHandle_t uart_queue;
	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
	vTaskDelay(100/portTICK_PERIOD_MS);
	const char* filter_str_loc = "$GPGGA";
	const char* filter_str_speed = "$GPVTG";
	char *loc = NULL;
	char *speed = NULL;
	char str1[100];
	char str2[45];
	int j=0;
	while(1){
		int length;

		ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
		char gps_out[length];

		uart_read_bytes(uart_num, gps_out, length, 1000);

		loc = strstr(gps_out,filter_str_loc);
	    if (loc == NULL) {
	        printf("GPGGA not found in the sentence\n");
	    }
	    else{
		    char *newline = strchr(loc, '\n');
		    if (newline == NULL) {
		        printf("Incomplete NMEA Sentence\n");
		    }
		    else if(loc){
				for (int i=0; loc[i] != '\n'; i++){
					str1[j] = loc[i];
					j++;
				}
				str1[j+1] = '\n';
				j = 0;
				printf("%s\n\n", str1);
				char* token = strtok(str1, ",");
				token = strtok(NULL, ",");
				for (int i=0; i<4; i++){
					token = strtok(NULL,",");
					location[i] = token;
				}
				double latitude = atof(location[0]);
				double longitude = atof(location[2]);

				 latitude_decimal = floor(latitude/100);
				 longitude_decimal = floor(longitude/100);

				if(latitude_decimal == 0 && longitude_decimal == 0){
					printf("No Signal\n");
				}
				else{
					double latitude_decimal_minutes = modf(latitude/100, &latitude);
					latitude_decimal_minutes = latitude_decimal_minutes*100;
					double longitude_decimal_minutes = modf(longitude/100, &longitude);
					longitude_decimal_minutes = longitude_decimal_minutes*100;

					lat_deg = latitude_decimal + latitude_decimal_minutes/60;
					long_deg = longitude_decimal + longitude_decimal_minutes/60;
					//printf("%.8f, %.8f\n", lat_deg, long_deg);
				}
			}
	    }
	    //printf("%s\n\n", gps_out);
	    speed_str = "0";
		speed = strstr(gps_out,filter_str_speed);
		//printf("%s", speed);
	    if (speed == NULL) {
	        printf("GPVTG not found in the sentence\n");
	    }
	    else{
	 	    char *newline2 = strchr(speed, '\n');
		    if (newline2 == NULL) {
		        printf("Incomplete NMEA Sentence\n");
		    }
		    else if (newline2 != NULL && latitude_decimal != 0 && longitude_decimal != 0){
		    	for (int i=0; speed[i] != '\n'; i++){
		    		str2[j] = speed[i];
		    		j++;
		    	}
		    	str2[j+1] = '\n';
		    	j = 0;
		    	printf("%s\n\n", str2);
		    	if(strlen(str2) > 20){
					char* token2 = strtok(str2, ",");
					token2 = strtok(NULL, ",");
					for (int i=0; i<6; i++){
						token2 = strtok(NULL,",");
						velocity[i] = token2;
						if (strcmp(token2, "N") == 0){
							token2 = strtok(NULL,",");
							speed_str = token2;
							break;
						}
					}
		    	}
				printf("SPEED STRING: %s\n", speed_str);
				printf("%.7f, %.7f\n", lat_deg, long_deg);
				if(speed_str == NULL){
					strcpy(speed_str, "0");
				}
				speed_kmh = atof(speed_str);
				printf("%.2f\n\n", speed_kmh);
				if (speed_kmh < 1){
					speed_kmh = 1;
				}

		    }
	    }

	    //xSemaphoreGive(semaphore);
		//vTaskSuspend(uart_handle);
	    //sd_card_write();
	    char gps_data_format[30] = "00.0000000,000.0000000,00.00,";
	    snprintf(gps_data_format, 30, "%.7f,%.7f,%.2f,", lat_deg, long_deg, speed_kmh);
//	    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(ZB_CLIENT_ENDPOINT_4, SENSOR_DATA_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, SENSOR_DATA_ATTR_ID_3);
//	    memcpy(value_r->data_p, &gps_data_format, sizeof(gps_data_format));
	    vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        
        my_net_idx = param->node_prov_complete.net_idx;
        my_addr = param->node_prov_complete.addr;
        
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        // if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
        //     uint16_t tid = *(uint16_t *)param->model_operation.msg;
        //     ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x", param->model_operation.opcode, tid);
        //     esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
        //             param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
        //             sizeof(tid), (uint8_t *)&tid);
        //     if (err) {
        //         ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
        //     }
        // }
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
            update_led_alert(pm25, pm10, temp, rh, co, no2, co2, voc);
			iaq_score(pm25, pm10, co, no2, co2, voc);
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            // char *mydata = "id1,18,5.90,6,5.90,32.03,49.86,0,400.00,0,0.00,2,1.77,0,0.07,14.699734,121.723847,23.55"; //"id1,18,5.90,6,5.90,32.03,49.86,0,400.00,0,0.00,2,1.77,0,0.07,14.699734,121.723847,23.55"
            // char *mydata = "{\"COraw\":\"0000.00\",\"COindex\":\"000\",\"CO2raw\":\"0000.00\",\"CO2index\":\"000\",\"NO2raw\":\"0000.00\",\"NO2index\":\"000\",\"PM25raw\":\"0000.00\",\"PM25index\":\"000\",\"PM10raw\":\"0000.00\",\"PM10index\":\"000\",\"VOCraw\":\"0000.00\",\"VOCindex\":\"000\",\"T\":\"0000.00\",\"RH\":\"0000.00\",\"Lat\":\"000.0000000\",\"Long\":\"000.0000000\",\"speed\":\"000.00\"\"source\":\"Node 4\",\"local_time\":\"2024-06-01T15:15:15\",\"type\":\"data\"}";

            ESP_LOGI(TAG, "Recv 0x%06" PRIx32 ", tid 0x%04x", param->model_operation.opcode, tid);
            snprintf(Current_Date_Time, 20, "%s", (char *)param->model_operation.msg);
            ESP_LOGI(TAG, "%s", Current_Date_Time);
            snprintf(data_payload, 500, "{\"COraw\":\"%.2f\",\"COindex\":\"%.0f\",\"CO2raw\":\"%.2f\",\"CO2index\":\"%.0f\",\"NO2raw\":\"%.2f\",\"NO2index\":\"%.0f\",\"PM25raw\":\"%.2f\",\"PM25index\":\"%.0f\",\"PM10raw\":\"%.2f\",\"PM10index\":\"%.0f\",\"VOCraw\":\"%.2f\",\"VOCindex\":\"%.0f\",\"T\":\"%.2f\",\"RH\":\"%.2f\",\"source\":\"Node 1\",\"local_time\":\"%s\",\"type\":\"data\"}",co,co_score,co2,co2_score,no2,no2_score,pm25,pm25_score,pm10,pm10_score,voc,voc_score,temp,rh,Current_Date_Time);
			sd_card_write();
            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
//                    sizeof(tid), (uint8_t *)&tid);
            		strlen(data_payload)+1, (uint8_t *)data_payload);
            if (err) {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
            printf("string sent: %s\n", data_payload);
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "Send 0x%06" PRIx32, param->model_send_comp.opcode);
        break;
    // case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
    //     //time
    //     break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return ESP_OK;
}

void example_ble_mesh_send_vendor_message(void)
{
    esp_ble_mesh_msg_ctx_t ctx = {0};
    uint32_t opcode;
    esp_err_t err;

    ctx.net_idx = my_net_idx;
    ctx.app_idx = 0x0000;
    ctx.addr = 0x0001;
    ctx.send_ttl = 3;
    opcode = ESP_BLE_MESH_VND_MODEL_OP_SEND;

    char *mydata = "{\"PM25raw\":\"0000.00\",\"PM25index\":\"000\",\"}";

    err = esp_ble_mesh_server_model_send_msg(&vnd_models[0], &ctx, opcode, strlen(mydata)+1, (uint8_t *)mydata);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send vendor message 0x%06" PRIx32, opcode);
        return;
    }
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    // semaphore = xSemaphoreCreateCounting(TASK_COUNT, 0);
   	heated = 1;

	//configure_led();
   	led_alert_init();

	sensor_init(&heated);
    xTaskCreate(ADC1_Data_Task, "ADC1 Data Task", 1024*3, NULL, 5, &mics_handle);
    xTaskCreate(I2C0_Data_Task, "I2C0 Data Task", 1024*3, NULL, 5, &sgp_handle);
    xTaskCreate(I2C1_Data_Task, "I2C1 Data Task", 1024*2, NULL, 5, &sen_handle);
    // xTaskCreate(UART_Data_Task, "UART Data Task", 1024*8, NULL, 5, &uart_handle);
    // xTaskCreate(Data_Complete_Task, "Data Upload and Save", 4096, NULL, 5, &finish_handle);
}
