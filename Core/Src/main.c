/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "lwip/api.h"
#include "bma456.h"
#include "bma4_common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	REQ = 0, RESP = 1
} packet_type;

struct time_packet {
	uint8_t head; //0xAE
	uint8_t type; //0:REQ, 1:RESP
};

#pragma pack(1)
struct lora_packet_t {
	uint8_t device_id;
	float temp;
	float vocs;
	float smoke;
	float vibr;
	uint8_t fire;
	uint8_t fan_stat;
	uint8_t fext_stat;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEVICEID  31;
#define FAN_ON_TEMP 35

//#define LORA  1
//#define UDP  1

#define SERVER_IP1  192 //server ip address
#define SERVER_IP2  168
#define SERVER_IP3  1
//#define SERVER_IP4  152
#define SERVER_IP4  137
#define SERVER_PORT	8000 //server listen port

// lora define
#define REGION	"KR920"
#define TX_POWER "0"
//#define DEV_EUI	"60C5A8FFFE78AF6C"
#define APP_EUI	"28d810c5b5291877"
#define APP_KEY	"756be986bd0cf786c37fad995b8f4961"
#define CONFIRM	"1"
#define GRAVITY_EARTH      (9.80665f)
#define VIBR_LIMIT      (1.0f)
#define DIFF_ABS(X,Y) ((X)>(Y)?(X)-(Y):(Y)-(X))


//8000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
extern struct netif gnetif; //extern gnetif
osThreadId tcp_task;  //tcp client task handle
osThreadId i2c_task;
osThreadId collection_task;
osThreadId udp_send_task;
osThreadId lora_send_task;

osThreadId led_blink_task;

ip_addr_t server_addr; //server address
struct time_packet packet; //256 bytes time_packet structure

int t;
int h;

struct netconn *conn;
bool state; // lora join state
bool lora_init;
bool bma_init;
struct bma4_dev bma = { 0 };
struct bma4_accel sens_data = { 0 };

float vibr_x;
float vibr_y;
float vibr_z;
float vibr;
float temp;
float co;
float vocs;
float formaldehyde;
float toluene;
float benzene;
uint8_t fire;
uint8_t debug;
uint8_t fext_stat;
uint8_t fan_stat;
uint8_t i;



float c_vibr;
float c_temp;
float c_co;
float c_vocs;

float vibr_abs_new;
float vibr_abs_old;

uint8_t device_id;
char *device_eui;

uint8_t LORA;
uint8_t UDP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */
void StartClientTask(void const *argument); //tcp client task function
void StartI2CTask(void const *argument);
void startUdpSendTask(void const *argument);
void startLoRaSendTask(void const *argument);
void collectionTask(void const *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	uint8_t ch8 = ch;
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch8, 1, HAL_MAX_DELAY);
	return ch;
}

int __io_getchar() {
	uint8_t ch8;
	HAL_UART_Receive(&huart3, &ch8, 1, HAL_MAX_DELAY);
	return 0;
}

void reset(){
	vibr = c_vibr;
	temp = c_temp;
	co = c_co;
	vocs = 0;

}

void print_ip(unsigned int ip, unsigned int netmask, unsigned int gw_ip) {
	unsigned char bytes[4];
	bytes[0] = ip & 0xFF;
	bytes[1] = (ip >> 8) & 0xFF;
	bytes[2] = (ip >> 16) & 0xFF;
	bytes[3] = (ip >> 24) & 0xFF;
	printf("ip %d.%d.%d.%d\r\n", bytes[3], bytes[2], bytes[1], bytes[0]);

	bytes[0] = netmask & 0xFF;
	bytes[1] = (netmask >> 8) & 0xFF;
	bytes[2] = (netmask >> 16) & 0xFF;
	bytes[3] = (netmask >> 24) & 0xFF;
	printf("netmask %d.%d.%d.%d\r\n", bytes[3], bytes[2], bytes[1], bytes[0]);

	bytes[0] = gw_ip & 0xFF;
	bytes[1] = (gw_ip >> 8) & 0xFF;
	bytes[2] = (gw_ip >> 16) & 0xFF;
	bytes[3] = (gw_ip >> 24) & 0xFF;
	printf("gw_ip %d.%d.%d.%d\r\n", bytes[3], bytes[2], bytes[1], bytes[0]);
}

static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width) {
	float half_scale = (1 << bit_width) / 2.0f;
	return ((float) (GRAVITY_EARTH * val * g_range)) / half_scale;
}

const float LM75A_DEGREES_RESOLUTION = 0.125;
void collectionTask(void const *argument) {
	printf("----- StartCollectionTask ------\r\n");

	//adc
	uint32_t adc;
	float co_tmp;
	float V;

	//gpio
	uint8_t fext_no = HAL_GPIO_ReadPin(GPIOG, FEXT_NO_Pin); // FEXT EMPTY
	uint8_t fext_nc = HAL_GPIO_ReadPin(GPIOG, FEXT_NC_Pin); // FEXT NOT CONNECTED

	uint8_t fire0 = HAL_GPIO_ReadPin(GPIOD, FIRE0_Pin); // temp up
	uint8_t fire1 = HAL_GPIO_ReadPin(GPIOD, FIRE1_Pin); // fire up

	fext_stat = fext_no << 1 | fext_nc;
	fire = fire1 << 1 | fire0;

	//i2c
	HAL_StatusTypeDef ret;
	int8_t rslt;

	int16_t refactored_value;
	uint8_t *ptr = (uint8_t*) &refactored_value;

	uint8_t send_buf[1] = { 0x00 };
	uint8_t i2c_rx_buf[2] = { 0 };

	float real_value;

	while (1) {
		// ---------------------- START ADC ----------------------
		osDelay(200);

		//printf("========== ADC ==========\r\n");
		//printf("       ADC    V         Value\r\n");


		// co , PC2 , SMOKE_A
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		adc = HAL_ADC_GetValue(&hadc2);
		V = adc * (3.3f / 4096.0f);
		V = V * 5 / 3.3f;

		co_tmp = 113.26042 - (271.01187 * V) + (148.17739 * pow(V, 2)); // * 5/3.3

		if (V > 1.0 && co_tmp > 0) {
			if (co_tmp > 1000) {
				c_co = 1000;
			} else {
				c_co = co_tmp;
			}
		} else {
			c_co = 0;
		}

		co  = fmaxf(co,c_co);

		//printf("CO  : %d , %f, %f PPM\r\n", adc, V, co);

		// voc , PF4 , VOC_A
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
		adc = HAL_ADC_GetValue(&hadc3);
		V = adc * (3.3f / 4096.0f);
		V = V * 5 / 3.3;

		toluene =  pow(10, (-5.237f) + 4.848f * (V) + (-0.857f) * pow(V, 2));
		formaldehyde = pow(10,(-5.905f) + 6.996f * (V) + (-1.327f) * pow(V, 2));
		benzene = pow(10, (-11.207f) + 14.718f * (V) + (-3.829f) * pow(V, 2));

		if (V > 2.62) {
			toluene = 750;
		}

		if (V > 2.08) {
			formaldehyde = 750;
		}

		if (V > 1.39) {
			benzene = 70;
		}

		//formaldehyde = (-3.665) + (3.009 * V ) + ((-0.362) * pow(V, 2));
		//toluene = (-9.234) + (5.249 * V) + ((-0.557)* pow(V, 2));

		//printf("VOCS  : %d , %fV\r\n", adc, V);
		//printf("TOL  : %d , %f, %fppm \r\n", adc, toluene);
		//printf("FOR  : %d , %f, %fppm \r\n", adc, V, formaldehyde);
		//printf("BEZ  : %d , %f, %fppm \r\n", adc, V, benzene);
		c_vocs = formaldehyde;
		vocs = fmaxf(vocs,c_vocs);
		// ---------------------- END ADC ----------------------

		// ---------------------- START I2C ----------------------
		//printf("========== I2C ==========\r\n");

		ret = HAL_I2C_Master_Transmit(&hi2c1, 0x90, send_buf, 1, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			printf("LM75AD i2s send error\r\n");
		} else {
			ret = HAL_I2C_Master_Receive(&hi2c1, 0x90, i2c_rx_buf, 2,
			HAL_MAX_DELAY);

			if (ret != HAL_OK) {
				printf("LM75AD i2s receive error\r\n");
			} else {

				// Swap bytes
				*ptr = *((uint8_t*) &i2c_rx_buf + 1);
				*(ptr + 1) = *(uint8_t*) &i2c_rx_buf;

				// Shift data (left-aligned)
				refactored_value >>= 5;

				if (refactored_value & 0x0400) {
					// When sign bit is set, set upper unused bits, then 2's complement
					refactored_value |= 0xf800;
					refactored_value = ~refactored_value + 1;
					real_value = (float) refactored_value * (-1)
							* LM75A_DEGREES_RESOLUTION;
				} else {
					real_value = (float) refactored_value
							* LM75A_DEGREES_RESOLUTION;
				}
				c_temp = real_value;
				temp = fmaxf(temp,c_temp);
				//printf("temp : %f\r\n", temp);
			}
		}

		// ---------------------- START GPIO ----------------------
		//printf("========== GPIO ==========\r\n");

		// get Fire extinguisher status
		// 0 ->1 : GAS EMPTY , 1 -> 0 : WORKING or NOT CONNECT

		fext_no = HAL_GPIO_ReadPin(GPIOG, FEXT_NO_Pin); // FEXT EMPTY
		fext_nc = HAL_GPIO_ReadPin(GPIOG, FEXT_NC_Pin); // FEXT NOT CONNECTED
		fire0 = HAL_GPIO_ReadPin(GPIOD, FIRE0_Pin); // temp up
		fire1 = HAL_GPIO_ReadPin(GPIOD, FIRE1_Pin); // fire up

		fext_stat = fext_no << 1 | fext_nc;
		fire = fire1 << 1 | fire0;

		//printf(" FIRE0 %d\r\n",fire0 );
		//printf(" FIRE1 %d\r\n",fire1 );
		//printf("fext_stat : %d \r\n", fext_stat);
		//printf("fire : %d \r\n", fire);

		fan_stat = HAL_GPIO_ReadPin(GPIOG, FAN_Pin);
		//printf("fan_stat : %d\r\n", fan_stat);

		// control fan
		if(temp >= FAN_ON_TEMP && fan_stat == 0){
			printf(" FAN ON \r\n");
			HAL_GPIO_WritePin(GPIOG, FAN_Pin ,GPIO_PIN_SET);
		}else if(temp < FAN_ON_TEMP && fan_stat == 1){
			printf(" FAN OFF \r\n");
			HAL_GPIO_WritePin(GPIOG, FAN_Pin ,GPIO_PIN_RESET);
		}


		//HAL_GPIO_TogglePin(GPIOG, FAN_Pin);

		// ---------------------- END GPIO ----------------------
		/* Read the accel x, y, z data */
//		if (bma_init) {
//			rslt = bma4_read_accel_xyz(&sens_data, &bma);
//			bma4_error_codes_print_result("bma4_read_accel_xyz status", rslt);
//
//			if (rslt == BMA4_OK) {
//
//				/* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
//				vibr_x = lsb_to_ms2(sens_data.x, 2, bma.resolution);
//				vibr_y = lsb_to_ms2(sens_data.y, 2, bma.resolution);
//				vibr_z = lsb_to_ms2(sens_data.z, 2, bma.resolution);
//
//				/* Print the data in m/s2 */
//				/* Print the data in m/s2 */
//				//printf("ANGLE : x:%d, y:%d, z:%d\r\n", sens_data.x, sens_data.y, sens_data.z);
//				printf("ANGLE : x:%f, y:%f, z:%f , %f\r\n", vibr_x, vibr_y,
//						vibr_zsqrt(pow(vibr_x, 2.0) + pow(vibr_y, 2.0)+ pow(vibr_z, 2.0)));
//			}
//		}
		/*uint8_t send_buf[1] = { 0x00};
		 uint8_t i2c_rx_buf[65] = { 0 };

		 ret = HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf, sizeof(send_buf), HAL_MAX_DELAY);
		 if (ret != HAL_OK) {
		 printf("i2s send error\r\n");
		 } else {
		 ret = HAL_I2C_Master_Receive(&hi2c1, 0x30, i2c_rx_buf, sizeof(i2c_rx_buf),
		 HAL_MAX_DELAY);

		 if (ret != HAL_OK) {
		 printf("i2s receive error\r\n");
		 } else {
		 printf("i2c : !");
		 for (uint8_t i = 0; i < sizeof(i2c_rx_buf); i++) {
		 printf("%x ", i2c_rx_buf[i]);
		 }

		 printf("\r\n");

		 }
		 }
		 */
		// ---------------------- END I2C ----------------------
		// ---------------------- TOGGLE TEST--------------------
		//HAL_GPIO_TogglePin(GPIOB, RED_LED_Pin);
//		if (i % 2 == 0) {
//			printf("========== GPIO TOGGLE ==========\r\n");
//			HAL_GPIO_TogglePin(GPIOG, FAN_Pin); //toggle relay of fan
//		}
//		i++;
	}

}

// lora uart data
uint8_t lora_rv_data;
uint8_t lora_rv_buf[1024];
uint32_t lora_rv_buf_pos;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle) {

	if (uartHandle->Instance == huart3.Instance) {
//		if (pos == 0) {
//			memset(buf, 0, sizeof(buf));
//		}
//
//		buf[pos++] = data;
//
//		if (pos == sizeof(buf)) {
//			printf("%s\r\n", buf);
//			pos = 0;
//		}
//
//		HAL_UART_Receive_IT(&huart3, &lora_rv_data, 1);
	} else if (uartHandle->Instance == huart1.Instance) {

		lora_rv_buf[lora_rv_buf_pos++] = lora_rv_data;

		if (lora_rv_buf_pos == sizeof(lora_rv_buf)) {
			//printf("%s", lora_rv_buf);
			lora_rv_buf_pos = 0;
		}

		HAL_UART_Receive_IT(&huart1, &lora_rv_data, 1);
	}
}

void Lora_Buf_Clear() {
	memset(lora_rv_buf, 0x0, sizeof(lora_rv_buf));
	lora_rv_buf_pos = 0;
	osDelay(500);
	HAL_UART_Receive_IT(&huart1, &lora_rv_data, 1);

}

void Lora_Cmd_Send(UART_HandleTypeDef *huart, char *cmd) {
	HAL_UART_Transmit_IT(huart, cmd, strlen(cmd));
}

bool Lora_Str_Find(char *str) {
	return strstr((char*) lora_rv_buf, str) != NULL;
}

void Print_Lora_Buf() {
	printf("%s\r\n", lora_rv_buf);
}

bool Lora_INIT(UART_HandleTypeDef *huart, char *cmd) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, cmd);
	osDelay(1500);

	//check uart response
	//Print_Lora_Buf();
	if (Lora_Str_Find("OK")) {
		return true;
	} else {
		return false;
	}

}
bool c1, c2, c3, c4, c5, c6;
bool Lora_Config(UART_HandleTypeDef *huart) {

	char cmd[128];
	bool flag = true;
	while (flag) {
		// set KR920
		if (!c1) {
			sprintf(cmd, "at+set_config=lora:region:%s\r\n", REGION);
			if (Lora_INIT(huart, cmd)) {
				printf("Lora_INIT OK : %s", cmd);
				c1 = true;
			} else {
				printf("Lora_INIT FAIL : %s", cmd);
				continue;
			}
		}

		// set TX Power
// default : 0
//		if (!c2) {
//			sprintf(cmd, "at+set_config=lora:tx_power:%s\r\n", TX_POWER);
//			if (Lora_INIT(huart, cmd)) {
//				printf("Lora_INIT OK : %s", cmd);
//				c2 = true;
//			} else {
//				printf("Lora_INIT FAIL : %s", cmd);
//				continue;
//			}
//		}

		// set app_eui device_eui

		if (!c3) {
			sprintf(cmd, "at+set_config=lora:app_eui:%s\r\n", APP_EUI);
			if (Lora_INIT(huart, cmd)) {
				printf("Lora_INIT OK : %s", cmd);
				c3 = true;
			} else {
				printf("Lora_INIT FAIL : %s", cmd);
				continue;
			}
		}
		// set dev_eui

		if (!c4) {
			//sprintf(cmd, "at+set_config=lora:dev_eui:%s\r\n", DEV_EUI);
			sprintf(cmd, "at+set_config=lora:dev_eui:%s\r\n", device_eui);
			if (Lora_INIT(huart, cmd)) {
				printf("Lora_INIT OK : %s", cmd);
				c4 = true;
			} else {
				printf("Lora_INIT FAIL : %s", cmd);
				continue;
			}
		}

		if (!c5) {
			// set app_key
			sprintf(cmd, "at+set_config=lora:app_key:%s\r\n", APP_KEY);
			if (Lora_INIT(huart, cmd)) {
				printf("Lora_INIT OK : %s", cmd);
				c5 = true;
			} else {
				printf("Lora_INIT FAIL : %s", cmd);
				continue;
			}
		}

		if (!c6) {
			// set app_key
			sprintf(cmd, "at+set_config=lora:confirm:%s\r\n", CONFIRM);
			if (Lora_INIT(huart, cmd)) {
				printf("Lora_INIT OK : %s", cmd);
				c6 = true;
			} else {
				printf("Lora_INIT FAIL : %s", cmd);
				continue;
			}
		}
		break;

	}

}

bool Lora_Status(UART_HandleTypeDef *huart) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, "at+get_config=lora:status\r\n");
	osDelay(1000);
	//Print_Lora_Buf();
	if (Lora_Str_Find("Joined Network:true")) {
		//printf("LoRa Joined Network\r\n");
		return true;
	} else {
		//printf("LoRa Not Joined Network\r\n");
		return false;
	}
}

bool Lora_Join(UART_HandleTypeDef *huart) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, "at+join\r\n");
	osDelay(5000);
	//Print_Lora_Buf();
	if (Lora_Str_Find("Join Success")) {
		return true;
	} else {
		return false;
	}
}

char lora_send_msg[128];
char lora_send_payload[128];
uint32_t len;
struct lora_packet_t lora_packet_t;
void Lora_Send(UART_HandleTypeDef *huart) {

	len = sprintf((char*) lora_send_msg, "at+send=lora:10:");

//	struct lora_packet_t {
//		uint32_t device_id;
//		uint8_t temp;
//		uint8_t vocs;
//		short smoke;
//		float vibr_x;
//		float vibr_y;
//		float vibr_z;
//		uint8_t fire;
//		uint8_t fan_stat;
//		uint8_t fext_stat;
//	};

//	float vibr_x;
//	float vibr_y;
//	float vibr_z;
//	float temp;
//	float co;
//	float formaldehyde;
//	float toluene;
//	float vocs;
//	uint8_t fire0;
//	uint8_t fire1;
//	uint8_t fire;
//	uint8_t debug;
//	uint8_t fext_stat;
//	uint8_t fan_stat;
//	uint8_t i;

	lora_packet_t.device_id = device_id;
	lora_packet_t.temp = temp;
	lora_packet_t.vocs = vocs;
	lora_packet_t.smoke = co;
	lora_packet_t.vibr = vibr;
	lora_packet_t.fire = fire;
	lora_packet_t.fan_stat = fan_stat;
	lora_packet_t.fext_stat = fext_stat;

	printf(
			"#### SEND LoRa Msg : {\"device_id\":%d,\"temp\":%f,\"vocs\":%f,\"smoke\":%f,\"vibr\":%f,\"fire\":%x,\"fan\":%x,\"fext_stat\":%x}\r\n",
			lora_packet_t.device_id,
			lora_packet_t.temp, //
			lora_packet_t.vocs, //
			lora_packet_t.smoke, lora_packet_t.vibr, lora_packet_t.fire,
			lora_packet_t.fan_stat, lora_packet_t.fext_stat);

	for (uint8_t i = 0; i < sizeof(struct lora_packet_t); i++) {
		sprintf((char*) &lora_send_msg[len + (i * 2)], "%02X",
				((char*) &lora_packet_t)[i]);
	}

	len += sizeof(struct lora_packet_t) * 2;

	lora_send_msg[len++] = '\r';
	lora_send_msg[len++] = '\n';

//printf("%s",lora_send_msg, sizeof(struct lora_packet_t));
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, lora_send_msg);
	//osDelay(500);
	for (uint8_t wait_cnt = 0; wait_cnt < 60; wait_cnt++) { // wait 1 minute
		osDelay(1000);
		Print_Lora_Buf();
		if (Lora_Str_Find("OK") || Lora_Str_Find("ERROR")) {
			break;
		}
	}
	reset();
}

void StartI2CTask(void const *argument) {
	printf("----- startI2CTask ------\r\n");

	HAL_StatusTypeDef ret;

	uint8_t send_buf[1] = { 0x00 };
	uint8_t i2c_rx_buf[2] = { 0 };

//lora_rv_data = netbuf_alloc(buf, 128);

	while (1) {
		ret = HAL_I2C_Master_Transmit(&hi2c1, 0x48, send_buf, 1, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			printf("i2s send error\r\n");
		} else {
			ret = HAL_I2C_Master_Receive(&hi2c1, 0x48, i2c_rx_buf, 2,
			HAL_MAX_DELAY);

			if (ret != HAL_OK) {
				printf("i2s receive error\r\n");
			} else {
//				printf("test %d lora_rv_data : %x %x %x %x %x %x\r\n",
//						sizeof(i2c_rx_buf), i2c_rx_buf[0], i2c_rx_buf[1],
//						i2c_rx_buf[2], i2c_rx_buf[3], i2c_rx_buf[4],
//						i2c_rx_buf[5]);
//				uint32_t temp = (float) ((float) 175.0
//						* (float) (i2c_rx_buf[0] * 0x100 + i2c_rx_buf[1])
//						/ (float) 65535.0 - 45.0);
//				uint32_t humi = (float) ((float) 100.0
//						* (float) (i2c_rx_buf[3] * 0x100 + i2c_rx_buf[4])
//						/ (float) 65535.0);
				//printf("temp : %d , humi : %d \r\n", temp, humi);
//				t = temp;
//				h = humi;

				//packet.head = 0xAE; //head
				//packet.type = REQ; //request type

				//netconn_write(conn, send_buf, sizeof(send_buf), NETCONN_NOFLAG);

			}
		}
		osDelay(1000);
	}
}

//bool readChipId() {
//
//	uint8_t send_buf[1] = { 0x00 };
//	uint8_t i2c_rx_buf[1] = { 0 };
//
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf,
//			sizeof(send_buf),
//			HAL_MAX_DELAY);
//
//	if (ret != HAL_OK) {
//		printf("communication : SEND ERROR\r\n");
//		return false;
//	} else {
//		ret = HAL_I2C_Master_Receive(&hi2c1, 0x30, i2c_rx_buf,
//				sizeof(i2c_rx_buf),
//				HAL_MAX_DELAY);
//
//		if (ret != HAL_OK) {
//			printf("communication : RECEIVE ERROR\r\n");
//			return false;
//		} else {
//			if (i2c_rx_buf[0] == 0x16) {
//				printf("communication : OK\r\n");
//				return true;
//			} else {
//				printf("communication : ERROR\r\n");
//				return false;
//			}
//		}
//	}
//}
//
//void bma4_set_advance_power_save(uint8_t enable) {
//	uint8_t send_buf[2] = { 0x7C, enable };
//	if (HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf, sizeof(send_buf),
//	HAL_MAX_DELAY) == HAL_OK) {
//		printf("disable PWR_CONF.adv_power_save \r\n");
//	}
//}
//
//void bma4_write_regs(uint8_t init_ctrl) {
//	uint8_t send_buf[2] = { 0x59, init_ctrl };
//	if (HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf, sizeof(send_buf),
//	HAL_MAX_DELAY) == HAL_OK) {
//		printf("prepare feature engine INIT_CTRL =0x00 \r\n");
//	}
//
//}
//
//void stream_transfer_write() {
//	uint8_t send_buf[1] = { 0x5E };
//	if (HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf, sizeof(send_buf),
//	HAL_MAX_DELAY) == HAL_OK) {
//		printf("burst_write_reg start\r\n");
//	}
//
//	if (HAL_I2C_Master_Transmit(&hi2c1, 0x30, bma456_config_file,
//			sizeof(bma456_config_file), HAL_MAX_DELAY) == HAL_OK) {
//		printf("burst_write_reg finish\r\n");
//	}
//
//}
//
//
//
//bool checkStatus() {
//
//	osDelay(140);
//	uint8_t send_buf[1] = { 0x2A };
//	uint8_t i2c_rx_buf[1] = { 0 };
//
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf,
//			sizeof(send_buf),
//			HAL_MAX_DELAY);
//
//	if (ret != HAL_OK) {
//		printf("checkStatus : SEND ERROR\r\n");
//		return false;
//	} else {
//		ret = HAL_I2C_Master_Receive(&hi2c1, 0x30, i2c_rx_buf,
//				sizeof(i2c_rx_buf),
//				HAL_MAX_DELAY);
//
//		if (ret != HAL_OK) {
//			printf("checkStatus : RECEIVE ERROR\r\n");
//			return false;
//		} else {
//			printf("@@@ %x\r\n", i2c_rx_buf[0]);
//			if (i2c_rx_buf[0] == 0x01) {
//				printf("checkStatus : OK\r\n");
//				return true;
//			} else {
//				printf("checkStatus : ERROR\r\n");
//				return false;
//			}
//		}
//	}
//}
//
//
//void initialization() {
//
//	osDelay(1);
//
//	bma4_set_advance_power_save(0x00);
//
//	osDelay(1); // delay 450us
//
//	bma4_write_regs(0x00);
//
//	osDelay(1);
//
//	stream_transfer_write();
//
//	osDelay(1);
//
//	bma4_write_regs(0x01);
//
//	osDelay(1); // delay 150us
//
//}
//
//void initBMA456() {
//	osDelay(10);
//
//	if (!readChipId()) {
//		return;
//	}
//
//	initialization();
//	checkStatus();
//}

//	uint8_t send_buf[1] = { 0x00 };
//	uint8_t i2c_rx_buf[1] = { 0 };
//
//	ret = HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf, sizeof(send_buf),
//			HAL_MAX_DELAY);
//	if (ret != HAL_OK) {
//		printf("i2s send error\r\n");
//		return;
//	} else {
//		ret = HAL_I2C_Master_Receive(&hi2c1, 0x30, i2c_rx_buf,
//				sizeof(i2c_rx_buf),
//				HAL_MAX_DELAY);
//
//		if (ret != HAL_OK) {
//			printf("i2s receive error\r\n");
//		} else {
//
//		}
//	}

void initLoRa() {
	osDelay(10);
	HAL_UART_Receive_IT(&huart1, &lora_rv_data, 1);

// init lora config (region , tx power , app eui, dev eui, app key )
	Lora_Config(&huart1);

	printf("LORA RAK JOIN CHECK START!\r\n");

	//state = Lora_Status(&huart1);
	while (1) {
		printf("LORA RAK STATUS CHECK!\r\n");
		state = Lora_Status(&huart1);
		if (state) {
			lora_init = true;
			break;
		} else {
			state = Lora_Join(&huart1);
			if (state == true) {
				lora_init = true;
				break;
			}
		}
	}
	printf("LORA RAK JOIN CHECK Finish!\r\n");
}

void StartClientTask(void const *argument) {

	osDelay(1000);
	printf("----- StartClientTask ------\r\n");
	err_t err;
	struct netbuf *buf;
	void *data;

	u16_t len; //buffer length
	u16_t nRead; //read buffer index
	u16_t nWritten; //write buffer index
	LWIP_UNUSED_ARG(argument);

	while (1) {

		if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0
				|| gnetif.gw.addr == 0) {
			printf("no valid ip \r\n");
			osDelay(1000);
			continue;
		}
		print_ip(gnetif.ip_addr.addr, gnetif.netmask.addr, gnetif.gw.addr);
		//printf("%"PRIu32"\r\n", gnetif.ip_addr.addr); //print time information

		nRead = 0;			//clear indexes
		nWritten = 0;

		//conn = netconn_new(NETCONN_TCP); //new tcp netconn
		conn = netconn_new(NETCONN_UDP);			//new udp netconn

		if (conn != NULL) {

			printf("start Socket connect\r\n");

			IP4_ADDR(&server_addr, SERVER_IP1, SERVER_IP2, SERVER_IP3,
					SERVER_IP4);	//server ip
			err = netconn_connect(conn, &server_addr, SERVER_PORT);	//connect to the server

			if (err != ERR_OK) {
				printf("connect ERR %d\r\n", err);
				netconn_delete(conn); //free memory
				osDelay(1000);
				continue;
			}
			printf("connected %d\r\n", conn->state);

			//memset(&packet, 0, sizeof(struct time_packet));
			//packet.head = 0xAE; //head
			//packet.type = REQ; //request type

			//netconn_write(conn, &packet, sizeof(struct time_packet), NETCONN_NOFLAG);

//			do {
//
//				if (netconn_write_partly(conn, //connection
//						(const void*) (&packet + nWritten), //buffer pointer
//						(sizeof(struct time_packet) - nWritten), //buffer length
//						NETCONN_NOFLAG, //no copy
//						(size_t*) &len) != ERR_OK) //written len
//						{
//					netconn_close(conn); //close session
//					netconn_delete(conn); //free memory
//					continue;
//				} else {
//					nWritten += len;
//				}
//			} while (nWritten < sizeof(struct time_packet)); //send request

			while (netconn_recv(conn, &buf) == ERR_OK)	//receive the response
			{
				do {

					netbuf_data(buf, &data, &len); //receive lora_rv_data pointer & length

					//memcpy(&packet + nRead, lora_rv_data, len);
					//printf("receive@@ %d \r\n", len);
					nRead += len;
				} while (netbuf_next(buf) >= 0); //check buffer empty
				netbuf_delete(buf); //clear buffer
			}

			printf("end \r\n");
			netconn_close(conn); //close session
			netconn_delete(conn); //free memory
			conn = NULL;
		}
	}
}

uint8_t send_buf[512];
void startUdpSendTask(void const *argument) {

	printf("----- startUdpSendTask ------\r\n");

	struct netbuf *netbuf;
	netbuf = netbuf_new();
	while (1) {
		if (conn != NULL && conn->state == NETCONN_NONE) {

			memset(send_buf, 0, sizeof(send_buf));
			sprintf((char*) send_buf,
					"{\"device_id\":%d,\"temp\":%f,\"vocs\":%f,\"smoke\":%f,\"vibr\":%f,\"fire\":%x,\"fan\":%x,\"fext_stat\":%x}",
					device_id,
					temp, // 0
					vocs, // 0
					co, vibr, fire, fan_stat,
					fext_stat);
			printf("Send UDP Msg : %s\r\n", send_buf);

			//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13); //toggle running led

			//UDP
			netbuf_ref(netbuf, send_buf, strlen(send_buf));
			netconn_send(conn, netbuf);

			reset();


			//TCP
			//netconn_write(conn, send_buf, sizeof(send_buf), NETCONN_NOFLAG);

		}
		osDelay(2000);

	}
}

void startLoRaSendTask(void const *argument) {

	printf("----- startLoRaSendTask ------\r\n");
	while (1) {
		// send LoRa message
		if (lora_init) { // lora_init
			state = Lora_Status(&huart1);
			if (state) {
				Lora_Send(&huart1);
			} else { // JOIN
				state = Lora_Join(&huart1);
				if (state == true)
					printf("Lora JOIN Success!!\r\n");
			}
		}
		osDelay(1000);
	}
}

void startLedBlinkTask(void const *argument) {

	printf("----- startLedBlinkTask ------\r\n");
	while (1) {
		HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
		osDelay(200);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_ADC3_Init();
	MX_ADC2_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	// power led
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20303E5D;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, FAN_Pin | GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 LED2_Pin LED1_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | LED2_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : SW3_1_Pin SW3_2_Pin SW3_3_Pin SW3_4_Pin */
	GPIO_InitStruct.Pin = SW3_1_Pin | SW3_2_Pin | SW3_3_Pin | SW3_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : FIRE0_Pin FIRE1_Pin */
	GPIO_InitStruct.Pin = FIRE0_Pin | FIRE1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : FAN_Pin PG6 */
	GPIO_InitStruct.Pin = FAN_Pin | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : FEXT_NO_Pin FEXT_NC_Pin PG7 */
	GPIO_InitStruct.Pin = FEXT_NO_Pin | FEXT_NC_Pin | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA10 PA11 PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PD5 PD6 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PG9 PG14 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_13) { // when debug switch press this callback call

		//uint8_t init_sw = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		//HAL_GPIO_TogglePin(GPIOB, RED_LED_Pin);
		//printf("PREESED\r\n");
	}
}

void initTxType(void) {

	uint8_t sw3_1 = HAL_GPIO_ReadPin(GPIOE, SW3_1_Pin);
	uint8_t sw3_2 = HAL_GPIO_ReadPin(GPIOE, SW3_2_Pin);
	uint8_t sw3_3 = HAL_GPIO_ReadPin(GPIOE, SW3_3_Pin);
	uint8_t sw3_4 = HAL_GPIO_ReadPin(GPIOE, SW3_4_Pin);

	UDP = sw3_1;
	LORA = sw3_2;

}

void initDeviceId(void) {

	uint8_t sw3_1 = HAL_GPIO_ReadPin(GPIOE, SW3_1_Pin);
	uint8_t sw3_2 = HAL_GPIO_ReadPin(GPIOE, SW3_2_Pin);
	uint8_t sw3_3 = HAL_GPIO_ReadPin(GPIOE, SW3_3_Pin);
	uint8_t sw3_4 = HAL_GPIO_ReadPin(GPIOE, SW3_4_Pin);

	//device_id = sw3_4 << 3 | sw3_3 << 2 | sw3_2 << 1 | sw3_1;
	device_id = DEVICEID;
	printf("DEVICE_ID : %d\r\n", device_id);
}

void initDeviceEui(void) {

	unsigned char a[] = { 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, (device_id
			& 0xff) };

	char *s = malloc(sizeof a * 2 + 1);
	for (size_t i = 0; i < sizeof a; i++)
		sprintf(s + i * 2, "%02x", a[i]);

	printf("deviceEUI : %s\r\n", s);

	device_eui = s;
	//free(s);

}

uint16_t initBma456(void) {
	/* Variable to store the status of API */
	int8_t rslt;

	/* Sensor initialization configuration */

	/* Variable to store accel data ready interrupt status */
	uint16_t int_status = 0;

	/* Variable that holds the accelerometer sample count */
	uint8_t n_data = 100;

	struct bma4_accel_config accel_conf = { 0 };

	/* Function to select interface between SPI and I2C, according to that the device structure gets updated */
	rslt = bma4_interface_selection(&bma);

	bma4_error_codes_print_result("bma4_interface_selection", rslt);
	//printf("bma4_interface_selection status\r\n");

	/* Sensor initialization */
	rslt = bma456_init(&bma);
	bma4_error_codes_print_result("bma456_init", rslt);
	//printf("bma456_init status\r\n");

	/* Upload the configuration file to enable the features of the sensor. */

	rslt = bma456_write_config_file(&bma);
	bma4_error_codes_print_result("bma456_write_config_file", rslt);
	//printf("bma456_write_config status\r\n");

	/* Enable the accelerometer */
	rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma);
	bma4_error_codes_print_result("bma4_set_accel_enable", rslt);
	//printf("bma4_set_accel_enable status\r\n");

	/* Accelerometer configuration settings */
	/* Output data Rate */
	accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;

	/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
	accel_conf.range = BMA4_ACCEL_RANGE_2G;

	/* The bandwidth parameter is used to configure the number of sensor samples that are averaged
	 * if it is set to 2, then 2^(bandwidth parameter) samples
	 * are averaged, resulting in 4 averaged samples
	 * Note1 : For more information, refer the datasheet.
	 * Note2 : A higher number of averaged samples will result in a less noisier signal, but
	 * this has an adverse effect on the power consumed.
	 */
	accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

	/* Enable the filter performance mode where averaging of samples
	 * will be done based on above set bandwidth and ODR.
	 * There are two modes
	 *  0 -> Averaging samples (Default)
	 *  1 -> No averaging
	 * For more info on No Averaging mode refer datasheet.
	 */
	//accel_conf.perf_mode = BMA4_CIC_AVG_MODE;
	/* Set the accel configurations */
	rslt = bma4_set_accel_config(&accel_conf, &bma);

	//printf("bma4_set_accel_config status\r\n");

	/* Mapping data ready interrupt with interrupt pin 1 to get interrupt status once getting new accel data */
	rslt = bma456_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE,
			&bma);
	//printf("bma456_map_interrupt status\r\n");

	//printf("Ax[m/s2], Ay[m/s2], Az[m/s2]\r\n");

	while (1) {
		/* Read interrupt status */
		rslt = bma456_read_int_status(&int_status, &bma);
		bma4_error_codes_print_result("bma456_read_int_status", rslt);

		/* Filtering only the accel data ready interrupt */
		if ((rslt == BMA4_OK) && (int_status & BMA4_ACCEL_DATA_RDY_INT)) {
			bma_init = true;

			printf("BMA456 INIT Success\r\n");
//
//            /* Read the accel x, y, z data */
//            rslt = bma4_read_accel_xyz(&sens_data, &bma);
//            bma4_error_codes_print_result("bma4_read_accel_xyz status", rslt);
//
//            if (rslt == BMA4_OK)
//            {
//
//                /* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
//                x = lsb_to_ms2(sens_data.x, 2, bma.resolution);
//                y = lsb_to_ms2(sens_data.y, 2, bma.resolution);
//                z = lsb_to_ms2(sens_data.z, 2, bma.resolution);
//
//                /* Print the data in m/s2 */
//                printf("%4.2f, %4.2f, %4.2f\r\n", x, y, z);
//            }
//
//            /* Decrement the count that determines the number of samples to be printed */
//            n_data--;
//
//            /* When the count reaches 0, break and exit the loop */
//            if (n_data == 0)
//            {
//                printf("NData is 0 \r\n");
//            }

			return;
		}
	}

	return rslt;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* init code for LWIP */
	MX_LWIP_Init();
	/* USER CODE BEGIN 5 */

	device_id = DEVICEID;

	printf("### DEVICE_ID : %d ### \r\n",device_id);


	// init status led blink start
	osThreadDef(led_blink_task, startLedBlinkTask, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE);
	led_blink_task = osThreadCreate(osThread(led_blink_task), NULL);

	osDelay(1000);

	initTxType();
	//initDeviceId();
	initBma456();


	// sensor data collection task.
	osThreadDef(collection_task, collectionTask, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE);
	collection_task = osThreadCreate(osThread(collection_task), NULL);

	osDelay(1000);

	if (LORA) {
		printf("#######  start LORA ###### \r\n");
		initDeviceEui();
		initLoRa();
		osThreadDef(lora_send_task, startLoRaSendTask, osPriorityNormal, 0,
				configMINIMAL_STACK_SIZE);
		lora_send_task = osThreadCreate(osThread(lora_send_task), NULL);
	}

	if (UDP) {
		printf("#######  start UDP ###### \r\n");
		osThreadDef(tcp_task, StartClientTask, osPriorityNormal, 0,
				configMINIMAL_STACK_SIZE);
		tcp_task = osThreadCreate(osThread(tcp_task), NULL);

		osThreadDef(udp_send_task, startUdpSendTask, osPriorityNormal, 0,
				configMINIMAL_STACK_SIZE);
		udp_send_task = osThreadCreate(osThread(udp_send_task), NULL);
	}


	//start i2c
//	osThreadDef(i2c_task, StartI2CTask, osPriorityNormal, 0,
//			configMINIMAL_STACK_SIZE);
//	i2c_task = osThreadCreate(osThread(i2c_task), NULL);

	// init finish
	osThreadTerminate(led_blink_task);
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);

	HAL_StatusTypeDef ret;
	/* Infinite loop */
	for (;;) {
		if (bma_init) {
//			uint8_t send_buf[1] = { 0x12 };
//			uint8_t i2c_rx_buf[6] = { 0 };
//
//			//lora_rv_data = netbuf_alloc(buf, 128);
//
//			ret = HAL_I2C_Master_Transmit(&hi2c1, 0x30, send_buf, 1,
//					HAL_MAX_DELAY);
//			if (ret != HAL_OK) {
//				printf("i2s send error\r\n");
//			} else {
//				ret = HAL_I2C_Master_Receive(&hi2c1, 0x30, i2c_rx_buf, 6,
//				HAL_MAX_DELAY);
//
//				printf("RAW## ");
//				for(int i =0 ; i < 6 ;i++){
//					printf("%x ",i2c_rx_buf[i]);
//				}
//				printf("\r\n");
//
//			}

			int8_t rslt = bma4_read_accel_xyz(&sens_data, &bma);
			bma4_error_codes_print_result("bma4_read_accel_xyz status", rslt);

			if (rslt == BMA4_OK) {

				/* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
				vibr_x = lsb_to_ms2(sens_data.x, 2, bma.resolution);
				vibr_y = lsb_to_ms2(sens_data.y, 2, bma.resolution);
				vibr_z = lsb_to_ms2(sens_data.z, 2, bma.resolution);
				vibr_abs_new = sqrt(
						pow(vibr_x, 2.0) + pow(vibr_y, 2.0) + pow(vibr_z, 2.0));

				if (vibr_abs_old != 0) {
					//printf("####### VIBR %f , %f , %f#######\r\n",vibr,DIFF_ABS(vibr_abs_new,vibr_abs_old), fmaxf(vibr,DIFF_ABS(vibr_abs_new,vibr_abs_old)));
					c_vibr = DIFF_ABS(vibr_abs_new, vibr_abs_old);
					vibr = fmaxf(vibr, c_vibr);
				}

//				if (vibr_abs
//						!= 0 && DIFF_ABS(vibr_abs_temp,vibr_abs) > VIBR_LIMIT) {
//					printf("####### VIBR #######\r\n");
//					vibr = 0x01;
//				}

				vibr_abs_old = vibr_abs_new;
				/* Print the data in m/s2 */
				//printf("ANGLE : x:%f, y:%f, z:%f , %f\r\n", vibr_x, vibr_y,vibr_z,vibr_abs);
			}
			osDelay(100);
		}
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
