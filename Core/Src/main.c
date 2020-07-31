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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "lwip/api.h"
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
	uint32_t device_id;
	uint8_t temp;
	uint8_t vocs;
	uint8_t smoke;
	uint8_t vibr;
	uint8_t sound;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER_IP1  192 //server ip address
#define SERVER_IP2  168
#define SERVER_IP3  1
#define SERVER_IP4  137
#define SERVER_PORT	8000 //server listen port

// lora define
#define REGION	"KR920"
#define TX_POWER "0"
#define APP_EUI	"0000000000000001"
#define DEV_EUI	"323833355e387a01"
#define APP_KEY	"FB3D12987DD6969B0BE17121174C6396"

//8000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
extern struct netif gnetif; //extern gnetif
osThreadId tcp_task;  //tcp client task handle
osThreadId i2c_task;
osThreadId adc_task;
osThreadId send_task;
ip_addr_t server_addr; //server address
struct time_packet packet; //256 bytes time_packet structure

int t;
int h;

struct netconn *conn;
bool state; // lora join state
bool lora_init;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */
void StartClientTask(void const *argument); //tcp client task function
void StartI2CTask(void const *argument);
void startSendTask(void const *argument);
void AdcTask(void const *argument);
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

void AdcTask(void const *argument) {
	printf("----- StartAdcTask ------\r\n");
	uint32_t ADC_Data;
	while (1) {
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);
		uint8_t txBuffer[11]; /* ADC: 0000\r\n */

		ADC_Data = HAL_ADC_GetValue(&hadc1);
		sprintf((char*) txBuffer, "ADC: %04d\r\n", ADC_Data);
		HAL_UART_Transmit(&huart3, txBuffer, sizeof(txBuffer), 100);
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
	} else if (uartHandle->Instance == huart2.Instance) {

		lora_rv_buf[lora_rv_buf_pos++] = lora_rv_data;

		if (lora_rv_buf_pos == sizeof(lora_rv_buf)) {
			//printf("%s", lora_rv_buf);
			lora_rv_buf_pos = 0;
		}

		HAL_UART_Receive_IT(&huart2, &lora_rv_data, 1);
	}
}

void Lora_Buf_Clear() {
	memset(lora_rv_buf, 0x0, sizeof(lora_rv_buf));
	lora_rv_buf_pos = 0;
	HAL_Delay(100);
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


bool Lora_Confirm(UART_HandleTypeDef *huart) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, "at+set_config=lora:confirm:1\r\n");
	HAL_Delay(1000);

	//Print_Lora_Buf();

	if (Lora_Str_Find("OK")) {
		printf("Lora_Confirm OK\r\n");
		return true;
	} else {
		printf("Lora_Confirm FAIL\r\n");
		return false;
	}
}

bool Lora_INIT(UART_HandleTypeDef *huart, char *cmd) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, cmd);
	HAL_Delay(1000);

	//check uart response
	//Print_Lora_Buf();

	if (Lora_Str_Find("OK")) {
		printf("Lora_INIT OK : %s", cmd);
		return true;
	} else {
		printf("Lora_INIT FAIL : %s", cmd);
		//Print_Lora_Buf();
	}
	return false;

}

void Lora_Version(UART_HandleTypeDef *huart) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, "at+version\r\n");
	HAL_Delay(1000);
	if (Lora_Str_Find("OK")) {
		printf("Lora_Version OK\r\n");
		//Print_Lora_Buf();
	} else {
		printf("Lora_Version FAIL\r\n");
	}
}

bool Lora_Config(UART_HandleTypeDef *huart) {

	char cmd[128];
	// set KR920
	sprintf(cmd, "at+set_config=lora:region:%s\r\n", REGION);
	if (Lora_INIT(huart, cmd)) {
	}

	// set TX Power
	sprintf(cmd, "at+set_config=lora:tx_power:%s\r\n", TX_POWER);
	if (Lora_INIT(huart, cmd)) {
	}

	// set app_eui
	sprintf(cmd, "at+set_config=lora:app_eui:%s\r\n", APP_EUI);
	if (Lora_INIT(huart, cmd)) {
	}

	// set dev_eui
	sprintf(cmd, "at+set_config=lora:dev_eui:%s\r\n", DEV_EUI);
	if (Lora_INIT(huart, cmd)) {
	}

	// set app_key
	sprintf(cmd, "at+set_config=lora:app_key:%s\r\n", APP_KEY);
	if (Lora_INIT(huart, cmd)) {
	}

}

bool Lora_Status(UART_HandleTypeDef *huart) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, "at+get_config=lora:status\r\n");
	HAL_Delay(1000);
	//Print_Lora_Buf();
	if (Lora_Str_Find("Joined Network:true")) {
		printf("Joined Network:true\r\n");
		return true;
	} else {
		printf("Joined Network:false\r\n");
		return false;
	}
}

bool Lora_Join(UART_HandleTypeDef *huart) {
	Lora_Buf_Clear();
	Lora_Cmd_Send(huart, "at+join\r\n");
	HAL_Delay(5000);
	Print_Lora_Buf();
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
bool Lora_Send(UART_HandleTypeDef *huart) {

	len = sprintf((char*) lora_send_msg, "at+send=lora:10:");

	lora_packet_t.device_id = 1;
	lora_packet_t.temp = 2;
	lora_packet_t.vocs = 3;
	lora_packet_t.smoke = 4;
	lora_packet_t.vibr = 5;
	lora_packet_t.sound = 6;

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
	HAL_Delay(500);

	for (uint8_t retry_cnt = 0; retry_cnt < 10; retry_cnt++) {
		HAL_Delay(1000);
		//Print_Lora_Buf();
		if (Lora_Str_Find("OK")) {
			if (Lora_Str_Find("at+recv")) {
				state = true;
			} else {
				state = false;
			}
			break;
		} else if (Lora_Str_Find("ERROR: RUI_LORA_STATUS_NO_NETWORK_JOINED 86")) { // JOIN 이 안되어있는 경우
			state = false;
			break;
		}
	}
}

void StartI2CTask(void const *argument) {
	printf("----- startI2CTask ------\r\n");
	HAL_StatusTypeDef ret;
	uint8_t send_buf[2] = { 0x24, 0x0b };
	uint8_t i2c_rx_buf[6] = { 0 };

//lora_rv_data = netbuf_alloc(buf, 128);

	while (1) {
		ret = HAL_I2C_Master_Transmit(&hi2c2, 0x88, send_buf, 2, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			printf("i2s send error\r\n");
		} else {
			ret = HAL_I2C_Master_Receive(&hi2c2, 0x88, i2c_rx_buf, 6,
			HAL_MAX_DELAY);

			if (ret != HAL_OK) {
				printf("i2s receive error\r\n");
			} else {
//				printf("test %d lora_rv_data : %x %x %x %x %x %x\r\n",
//						sizeof(i2c_rx_buf), i2c_rx_buf[0], i2c_rx_buf[1],
//						i2c_rx_buf[2], i2c_rx_buf[3], i2c_rx_buf[4],
//						i2c_rx_buf[5]);
				uint32_t temp = (float) ((float) 175.0
						* (float) (i2c_rx_buf[0] * 0x100 + i2c_rx_buf[1])
						/ (float) 65535.0 - 45.0);
				uint32_t humi = (float) ((float) 100.0
						* (float) (i2c_rx_buf[3] * 0x100 + i2c_rx_buf[4])
						/ (float) 65535.0);
				//printf("temp : %d , humi : %d \r\n", temp, humi);
				t = temp;
				h = humi;

				//packet.head = 0xAE; //head
				//packet.type = REQ; //request type

				//netconn_write(conn, send_buf, sizeof(send_buf), NETCONN_NOFLAG);

			}
		}
		HAL_Delay(10000);
	}
}

void initLoRa() {

	if (HAL_UART_Receive_IT(&huart2, &lora_rv_data, 1) == HAL_OK) {
		printf("\r\n----- LORA_UART_RECEIVER HAL_OK ------\r\n");
	}

// set Lora Confirm mode
	Lora_Confirm(&huart2);

// init lora config (region , tx power , app eui, dev eui, app key )
	Lora_Config(&huart2);

// check lora version
	Lora_Version(&huart2);

	printf("LORA RAK JOIN CHECK START!\r\n");

	while (1) {
		state = Lora_Status(&huart2); // JOIN이 되어있음
		if (state) {
			lora_init = true;
			break;
		} else { // JOIN 안되어 있음
			state = Lora_Join(&huart2);
			if (state == true) {
				lora_init = true;
				break;
			}
		}
	}
	printf("LORA RAK JOIN CHECK Finish!\r\n");
}

void StartClientTask(void const *argument) {

	HAL_Delay(1000);
	printf("----- StartClientTask ------\r\n");
	err_t err;
	struct netbuf *buf;
	void *data;

	u16_t len; //buffer length
	u16_t nRead; //read buffer index
	u16_t nWritten; //write buffer index

//LWIP_UNUSED_ARG(argument); // ?��?��?���??????? ?��?�� ?��?��?�� ???�� 컴파?��?�� 경고 ?���??????? 목적

	while (1) {
		if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0
				|| gnetif.gw.addr == 0) {
			printf("no valid ip \r\n");
			HAL_Delay(1000);
			continue;
		}
		//print_ip(gnetif.ip_addr.addr, gnetif.netmask.addr, gnetif.gw.addr);
		//printf("%"PRIu32"\r\n", gnetif.ip_addr.addr); //print time information

		nRead = 0;			//clear indexes
		nWritten = 0;

		//conn = netconn_new(NETCONN_TCP); //new tcp netconn
		conn = netconn_new(NETCONN_UDP);			//new udp netconn

		if (conn != NULL) {

			//printf("start TCP connect\r\n");

			IP4_ADDR(&server_addr, SERVER_IP1, SERVER_IP2, SERVER_IP3,
					SERVER_IP4);	//server ip
			err = netconn_connect(conn, &server_addr, SERVER_PORT);	//connect to the server

			if (err != ERR_OK) {
				printf("connect ERR %d\r\n", err);
				netconn_delete(conn); //free memory
				HAL_Delay(1000);
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

void startSendTask(void const *argument) {

// UDP
	uint8_t send_buf[128];
	struct netbuf *netbuf;
	netbuf = netbuf_new();

	while (1) {
		HAL_Delay(10000);

		// send UCP message
		if (conn != NULL && conn->state == NETCONN_NONE) {
			memset(send_buf, 0, sizeof(send_buf));
			sprintf((char*) send_buf,
					"{\"device_id\":1,\"temp\":%d,\"vocs\":1,\"smoke\":2,\"vibr\":3,\"sound\":4}",
					t);
			printf("Send UDP Msg : %s\r\n", send_buf);

			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13); //toggle running led

			//UDP

			netbuf_ref(netbuf, send_buf, strlen(send_buf));
			netconn_send(conn, netbuf);

			//TCP
			//netconn_write(conn, tcp_buf, sizeof(tcp_buf), NETCONN_NOFLAG);

		}

		// send LoRa message
		if (lora_init) { // lora_init = 최초 보드 실행 후 JOIN 성공 시 true. JOIN 이 한번이라도 성공하지 않으면 계속 JOIN이 붙을 때 까지 요청
			if (state) {
				Lora_Send(&huart2);
			} else {
				state = Lora_Join(&huart2);
				if (state == true)
					printf("Lora JOIN Success!!");
			}
		}
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
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_I2C2_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
//printf("###### Board Start#####\r\n");
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
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
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
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x20303E5D;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PF13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PG9 PG14 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

	// start tcp client
	osThreadDef(tcp_task, StartClientTask, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE);
	tcp_task = osThreadCreate(osThread(tcp_task), NULL); //run tcp client task

	// start i2c
	osThreadDef(i2c_task, StartI2CTask, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE);
	i2c_task = osThreadCreate(osThread(i2c_task), NULL);

	// start send Task
	osThreadDef(send_task, startSendTask, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE);
	send_task = osThreadCreate(osThread(send_task), NULL);

// start adc
//osThreadDef(adc_task, AdcTask, osPriorityNormal, 0,configMINIMAL_STACK_SIZE);
//adc_task = osThreadCreate(osThread(adc_task), NULL);

	initLoRa();
	/* Infinite loop */
	for (;;) {

//		HAL_Delay(1000);
//		printf("\r\n send message\r\n");
//		uint8_t lte_cmd[64];
//		sprintf((char*) lte_cmd, "\r\nhello world@@\r\n");
//		HAL_UART_Transmit_IT(&huart2, lte_cmd, strlen((char*) lte_cmd));
//	  if (gnetif.ip_addr.addr == 0 || gnetif.netmask.addr == 0 || gnetif.gw.addr == 0) //system has no valid ip address
//	  {
//
//		   HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13); //toggle running led
//	       HAL_Delay(1000);
//	       continue;
//	  }

		//printf("test\r\n");
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13); //toggle running led
		//HAL_Delay(100);
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
