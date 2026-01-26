/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD5933.h"
#include "Communication.h"
#include "stdio.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "cJSON.h"
#include "semphr.h"
#include "message_buffer.h"
#include "sht2x_for_stm32_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    char   *buf;
    size_t  len;
} JsonMsg_t;

QueueHandle_t xJsonQueue;   // 创建时给尺寸, 例如 16

double          Zr_raw, Zi_raw, phase, phase_comp, phase_real;
double          magnitude, impedance;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ZIGBEE_RECV_READY_Pin GPIO_PIN_0
#define ZIGBEE_RECV_READY_GPIO_Port GPIOA
#define ZIGBEE_WAKEUP_Pin GPIO_PIN_4
#define ZIGBEE_WAKEUP_GPIO_Port GPIOA
#define SYS_LED_Pin GPIO_PIN_5
#define SYS_LED_GPIO_Port GPIOA

#define	ZIGBEE_TRANS(n)			(n?HAL_GPIO_WritePin(ZIGBEE_WAKEUP_GPIO_Port,ZIGBEE_WAKEUP_Pin,GPIO_PIN_RESET):HAL_GPIO_WritePin(ZIGBEE_WAKEUP_GPIO_Port,ZIGBEE_WAKEUP_Pin,GPIO_PIN_SET))
#define SYS_LED_BLINK()			HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin)
#define SYS_LED_ON()			HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET)
#define SYS_LED_OFF()			HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET)

// Get the PANID of the ZigBee module.
uint8_t ZigBee_Get_PANID_Cmd[7] =
	{0xFC, 0x03, 0x02, 0x00, 0x00, 0x00, 0xFD};
// Get the network ID of the ZigBee module.
uint8_t ZigBee_Get_NETID_Cmd[7] =
	{0xFC, 0x03, 0x04, 0x00, 0x00, 0x00, 0xFB};
// Get the MAC address of the ZigBee module.
uint8_t ZigBee_Get_MACID_Cmd[7] =
	{0xFC, 0x03, 0x05, 0x00, 0x00, 0x00, 0xFA};
// Get the signal channel of the ZigBee module.
uint8_t ZigBee_Get_SIGCHAN_Cmd[7] =
	{0xFC, 0x03, 0x09, 0x00, 0x00, 0x00, 0xF6};
// Get the node type of the ZigBee module.
uint8_t ZigBee_Get_NODETYPE_Cmd[7] =
	{0xFC, 0x03, 0x11, 0x00, 0x00, 0x00, 0xEE};

// Set the PANID of the ZigBee module to 0x0397 (919).
uint8_t ZigBee_Set_PANID_Cmd[7] =
	{0xFC, 0x06, 0x02, 0x00, 0x97, 0x03, 0x6C};
// Set the node type of the ZigBee module to 0x02 (end device with low power).
uint8_t ZigBee_Set_NODETYPE_Cmd[7] =
	{0xFC, 0x06, 0x11, 0x00, 0x02, 0x00, 0xE9};

uint8_t ZigBee_NETID[4] =
	{0}; // High byte is in the front, low byte is in the back.
uint8_t ZigBee_MACID[8] =
	{0}; // High byte is in the front, low byte is in the back.
uint16_t ADCBuff[2] =
	{0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float SHT2x_GetRelativeHumidity(uint8_t hold_master);
static double computePhaseComp(unsigned long freq);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile unsigned long startFreq = 5000;   // Start Frequency for the sweeping process
unsigned long   incFreq;   // Step size
unsigned short  incNum;   // Number of steps
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId AD5933_TaskHandle;
osThreadId ZigBee_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Read_TempHum(float *t, float *h);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void AppTask_AD5933(void const * argument);
void AppTask_ZigBee(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  xJsonQueue = xQueueCreate(16, sizeof(JsonMsg_t));
  configASSERT(xJsonQueue);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of AD5933_Task */
  osThreadDef(AD5933_Task, AppTask_AD5933, osPriorityNormal, 0, 2048);
  AD5933_TaskHandle = osThreadCreate(osThread(AD5933_Task), NULL);

  /* definition and creation of ZigBee_Task */
  osThreadDef(ZigBee_Task, AppTask_ZigBee, osPriorityHigh, 0, 2048);
  ZigBee_TaskHandle = osThreadCreate(osThread(ZigBee_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	  for(;;)
	  {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  vTaskDelay(250);
	  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_AppTask_AD5933 */
/**
* @brief Function implementing the AD5933_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_AD5933 */
void AppTask_AD5933(void const * argument)
{
  /* USER CODE BEGIN AppTask_AD5933 */
	  // AD5933 初始化
	  AD5933_Init();
	  AD5933_Reset();
	  AD5933_SetSystemClk(AD5933_CONTROL_INT_SYSCLK, 0);
	  AD5933_SetRangeAndGain(AD5933_RANGE_1000mVpp, AD5933_GAIN_X1);

	  // SHT20初始化
      SHT2x_Init(&hi2c3);
	  SHT2x_SetResolution(RES_14_12);

	  // ID 与 NT
	  char id[32];
	  const char* NT = "";
	  uint32_t uid = HAL_GetUIDw0();
	  snprintf(id, sizeof(id), "0012", (unsigned long)uid);

	  while (1) {
	    const unsigned long current_startFreq = startFreq;
	    const unsigned long incFreq   = 1000;
	    const unsigned short incNum   = 95;

	    // 配置扫频
	    AD5933_ConfigSweep(current_startFreq, incFreq, incNum);
	    AD5933_StartSweep();

	    for (unsigned short step = 0; step <= incNum; step++)
	    {
	        int current_freq = current_startFreq + step * incFreq;
	        double Z, Zr, Zi, phase, Z_real, Z_imag;
	        double gainFactor = 4.3675e-9;
	        double phase_comp = computePhaseComp(current_freq);

	        // 串行读取 AD5933
	        if (step == 0) {
	            AD5933_GetComplexImpedance(gainFactor, AD5933_FUNCTION_INIT_START_FREQ, &Z, &Zr, &Zi, &phase);
	        } else {
	            AD5933_GetComplexImpedance(gainFactor, AD5933_FUNCTION_INC_FREQ, &Z, &Zr, &Zi, &phase);
	        }

	        double phase_real = phase - phase_comp;
	        double theta_rad  = phase_real * M_PI / 180.0;
	        Z_real = Z * cos(theta_rad);
	        Z_imag = Z * sin(theta_rad);

		    float temp = SHT2x_GetTemperature(1);
		    float hum  = SHT2x_GetRelativeHumidity(1);
		    vTaskDelay(10);

	        // 组 JSON
	        cJSON *root = cJSON_CreateObject();
	        cJSON_AddStringToObject(root, "ID",  id);
	        cJSON_AddStringToObject(root, "NT",  NT);
	        cJSON_AddNumberToObject(root, "Freq", current_freq);
	        cJSON_AddNumberToObject(root, "Zr", Z_real);
	        cJSON_AddNumberToObject(root, "Zi", Z_imag);
	        cJSON_AddNumberToObject(root, "T", temp);
	        cJSON_AddNumberToObject(root, "H", hum);

	        char *json = cJSON_PrintUnformatted(root);
	        cJSON_Delete(root);

	        if (json) {
	            size_t len = strlen(json);
	            char *send_buf = pvPortMalloc(len + 2);
	            configASSERT(send_buf);
	            memcpy(send_buf, json, len);
	            send_buf[len++] = '\r';
	            send_buf[len++] = '\n';

	            JsonMsg_t msg = { .buf = send_buf, .len = len };
	            xQueueSend(xJsonQueue, &msg, portMAX_DELAY);

	            cJSON_free(json);
	        }

	        vTaskDelay(250);
	    }
	  }
  /* USER CODE END AppTask_AD5933 */
}

/* USER CODE BEGIN Header_AppTask_ZigBee */
/**
* @brief Function implementing the ZigBee_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_ZigBee */
void AppTask_ZigBee(void const * argument)
{
  /* USER CODE BEGIN AppTask_ZigBee */
    // 一次性唤醒并等待稳定
    ZIGBEE_TRANS(1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 进入配置
    HAL_UART_Transmit(&huart1, ZigBee_Set_PANID_Cmd, 7, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    HAL_UART_Transmit(&huart1, ZigBee_Set_NODETYPE_Cmd, 7, 100);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 后面不再切 ZIGBEE_TRANS
    for (;;) {
        JsonMsg_t msg;
        if (xQueueReceive(xJsonQueue, &msg, portMAX_DELAY) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t *)msg.buf, msg.len, HAL_MAX_DELAY);
            vPortFree(msg.buf);
        }
    }
  /* USER CODE END AppTask_ZigBee */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static double computePhaseComp(unsigned long freq) {
        return 7.53601e-4 * (double)freq - 90.46298;
}
/* USER CODE END Application */
