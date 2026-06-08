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
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include "message_buffer.h"
#include "sht2x_for_stm32_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    char   *buf;
    size_t  len;
} JsonMsg_t;

typedef enum {
    APP_FREQ_MODE_SWEEP = 0,
    APP_FREQ_MODE_FIXED = 1
} AppFreqMode_t;

typedef struct {
    AppFreqMode_t  mode;
    unsigned long  startFreqHz;
    unsigned long  endFreqHz;
    unsigned long  incFreqHz;
    unsigned long  fixedFreqHz;
    unsigned short incNum;
    uint32_t       version;
} AppAD5933Config_t;

QueueHandle_t xJsonQueue;   // 创建时给尺寸, 例如 16
SemaphoreHandle_t xAD5933ConfigMutex;

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

#define CMD_LINE_MAX_LEN        96U
#define CMD_ARGV_MAX            12
#define CMD_FREQ_SCALE_HZ       1000UL

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
static void QueueUartText(const char *text);
static void PublishMeasurementJson(const char *id, const char *nt,
                                   unsigned long currentFreqHz,
                                   double impedanceValue,
                                   double phaseDeg);
static void GetAD5933Config(AppAD5933Config_t *config);
static uint8_t IsAD5933ConfigChanged(uint32_t version);
static void HandleCommandLine(const char *line);
static void StartCommandRx(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static AppAD5933Config_t gAD5933Config = {
    .mode = APP_FREQ_MODE_SWEEP,
    .startFreqHz = 5000UL,
    .endFreqHz = 100000UL,
    .incFreqHz = 1000UL,
    .fixedFreqHz = 10000UL,
    .incNum = 95U,
    .version = 0U
};

static uint8_t cmdRxByte;
static char cmdRxLine[CMD_LINE_MAX_LEN];
static char cmdPendingLine[CMD_LINE_MAX_LEN];
static volatile uint16_t cmdRxIndex;
static volatile uint8_t cmdPendingReady;
static volatile uint8_t cmdLineOverflow;
static volatile uint8_t cmdLineDropped;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId AD5933_TaskHandle;
osThreadId ZigBee_TaskHandle;
osThreadId Command_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void AppTask_AD5933(void const * argument);
void AppTask_ZigBee(void const * argument);
void AppTask_Command(void const * argument);

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
  xAD5933ConfigMutex = xSemaphoreCreateMutex();
  configASSERT(xAD5933ConfigMutex);

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
  StartCommandRx();
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
  osThreadDef(Command_Task, AppTask_Command, osPriorityAboveNormal, 0, 512);
  Command_TaskHandle = osThreadCreate(osThread(Command_Task), NULL);
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
  (void)argument;
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
	  (void)argument;
	  // AD5933 初始化
	  AD5933_Init();
	  AD5933_Init();
	  AD5933_Reset();
	  AD5933_SetSystemClk(AD5933_CONTROL_INT_SYSCLK, 0);
	  AD5933_SetRangeAndGain(AD5933_RANGE_1000mVpp, AD5933_GAIN_X1);

	  // SHT20初始化
      SHT2x_Init(&hi2c3);
      SHT2x_Init(&hi2c3);
	  SHT2x_SetResolution(RES_14_12);

	  // ID 与 NT
	  char id[32];
	  const char* NT = "";
	  snprintf(id, sizeof(id), "0004");

	  while (1) {
	    AppAD5933Config_t config;
	    const double gainFactor = 4.3675e-9;

	    GetAD5933Config(&config);

	    // 配置扫频
	    if (config.mode == APP_FREQ_MODE_FIXED) {
	        AD5933_ConfigSweep(config.fixedFreqHz, 0UL, 0U);
	    } else {
	        AD5933_ConfigSweep(config.startFreqHz, config.incFreqHz, config.incNum);
	    }
	    AD5933_StartSweep();

	    if (config.mode == APP_FREQ_MODE_FIXED) {
	        while (IsAD5933ConfigChanged(config.version) == 0U)
	        {
	            double Z, Zr, Zi, phase;

	            AD5933_GetComplexImpedance(gainFactor, AD5933_FUNCTION_REPEAT_FREQ, &Z, &Zr, &Zi, &phase);
	            PublishMeasurementJson(id, NT, config.fixedFreqHz, Z, phase);
	            vTaskDelay(250);
	        }
	        continue;
	    }

	    for (unsigned short step = 0; step <= config.incNum; step++)
	    {
	        unsigned long current_freq = config.startFreqHz  + (unsigned long)step * config.incFreqHz;
	        double Z, Zr, Zi, phase, Z_real, Z_imag;
	        double phase_comp = computePhaseComp(current_freq);

	        if (IsAD5933ConfigChanged(config.version)) {
	            break;
	        }

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
    (void)argument;
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

/* USER CODE BEGIN Header_AppTask_Command */
/**
* @brief Function implementing the Command_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_Command */
void AppTask_Command(void const * argument)
{
  /* USER CODE BEGIN AppTask_Command */
    (void)argument;
    QueueUartText("\r\nCMD READY: set -o <0|1> [-q fixed_kHz] [-s start_kHz -e end_kHz -i step_kHz]\r\n");
    QueueUartText("Examples: set -o 0 -s 5 -e 100 -i 1; set -o 1 -q 10\r\n");

    for (;;) {
        char line[CMD_LINE_MAX_LEN];
        uint8_t hasLine = 0U;
        uint8_t overflow = 0U;
        uint8_t dropped = 0U;

        taskENTER_CRITICAL();
        if (cmdPendingReady != 0U) {
            memcpy(line, cmdPendingLine, sizeof(line));
            cmdPendingReady = 0U;
            hasLine = 1U;
        }
        if (cmdLineOverflow != 0U) {
            cmdLineOverflow = 0U;
            overflow = 1U;
        }
        if (cmdLineDropped != 0U) {
            cmdLineDropped = 0U;
            dropped = 1U;
        }
        taskEXIT_CRITICAL();

        if (overflow != 0U) {
            QueueUartText("CMD ERR: line too long\r\n");
        }
        if (dropped != 0U) {
            QueueUartText("CMD ERR: previous command not processed, line dropped\r\n");
        }
        if (hasLine != 0U) {
            HandleCommandLine(line);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
  /* USER CODE END AppTask_Command */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static double computePhaseComp(unsigned long freq) {
        return 7.53601e-4 * (double)freq - 90.46298;
}

static void QueueUartText(const char *text)
{
    size_t len;
    char *send_buf;
    JsonMsg_t msg;

    if ((text == NULL) || (xJsonQueue == NULL)) {
        return;
    }

    len = strlen(text);
    send_buf = pvPortMalloc(len);
    if (send_buf == NULL) {
        return;
    }

    memcpy(send_buf, text, len);
    msg.buf = send_buf;
    msg.len = len;

    if (xQueueSend(xJsonQueue, &msg, portMAX_DELAY) != pdPASS) {
        vPortFree(send_buf);
    }
}

static void PublishMeasurementJson(const char *id, const char *nt,
                                   unsigned long currentFreqHz,
                                   double impedanceValue,
                                   double phaseDeg)
{
    double phase_comp = computePhaseComp(currentFreqHz);
    double phase_real = phaseDeg - phase_comp;
    double theta_rad  = phase_real * M_PI / 180.0;
    double Z_real = impedanceValue * cos(theta_rad);
    double Z_imag = impedanceValue * sin(theta_rad);
    float temp = SHT2x_GetTemperature(1);
    float hum  = SHT2x_GetRelativeHumidity(1);
    cJSON *root;
    char *json;

    vTaskDelay(10);

    root = cJSON_CreateObject();
    if (root == NULL) {
        return;
    }

    cJSON_AddStringToObject(root, "ID", id);
    cJSON_AddStringToObject(root, "NT", nt);
    cJSON_AddNumberToObject(root, "Freq", currentFreqHz);
    cJSON_AddNumberToObject(root, "Zr", Z_real);
    cJSON_AddNumberToObject(root, "Zi", Z_imag);
    cJSON_AddNumberToObject(root, "T", temp);
    cJSON_AddNumberToObject(root, "H", hum);

    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json != NULL) {
        size_t len = strlen(json);
        char *send_buf = pvPortMalloc(len + 2U);

        if (send_buf != NULL) {
            JsonMsg_t msg;

            memcpy(send_buf, json, len);
            send_buf[len++] = '\r';
            send_buf[len++] = '\n';
            msg.buf = send_buf;
            msg.len = len;
            xQueueSend(xJsonQueue, &msg, portMAX_DELAY);
        }

        cJSON_free(json);
    }
}

static void GetAD5933Config(AppAD5933Config_t *config)
{
    if (config == NULL) {
        return;
    }

    if (xAD5933ConfigMutex != NULL) {
        xSemaphoreTake(xAD5933ConfigMutex, portMAX_DELAY);
        *config = gAD5933Config;
        xSemaphoreGive(xAD5933ConfigMutex);
    } else {
        *config = gAD5933Config;
    }
}

static uint8_t IsAD5933ConfigChanged(uint32_t version)
{
    AppAD5933Config_t config;

    GetAD5933Config(&config);
    return (config.version != version) ? 1U : 0U;
}

static void PrintCommandHelp(void)
{
    QueueUartText("CMD HELP: set -o <0|1> [-q fixed_kHz] [-s start_kHz -e end_kHz -i step_kHz]\r\n");
    QueueUartText("  -o 0: sweep mode, -o 1: fixed-frequency mode\r\n");
    QueueUartText("  -q X: fixed frequency in kHz\r\n");
    QueueUartText("  -s X -e Y -i Z: sweep start/end/step in kHz\r\n");
    QueueUartText("Examples: set -o 0 -s 5 -e 100 -i 1; set -o 1 -q 10; set -q 10\r\n");
}

static uint8_t ParsePositiveKHz(const char *text, unsigned long *freqHz)
{
    char *end = NULL;
    long value;

    if ((text == NULL) || (freqHz == NULL)) {
        return 0U;
    }

    value = strtol(text, &end, 10);
    if ((end == text) || (value <= 0)) {
        return 0U;
    }

    while ((*end == ' ') || (*end == '\t')) {
        end++;
    }

    if (*end != '\0') {
        return 0U;
    }

    *freqHz = (unsigned long)value * CMD_FREQ_SCALE_HZ;
    return 1U;
}

static int SplitCommandLine(char *line, char *argv[], int maxArgc)
{
    int argc = 0;
    char *p = line;

    while (*p != '\0') {
        while ((*p == ' ') || (*p == '\t')) {
            p++;
        }

        if (*p == '\0') {
            break;
        }

        if (argc >= maxArgc) {
            return -1;
        }

        argv[argc++] = p;

        while ((*p != '\0') && (*p != ' ') && (*p != '\t')) {
            p++;
        }

        if (*p != '\0') {
            *p = '\0';
            p++;
        }
    }

    return argc;
}

static void HandleCommandLine(const char *line)
{
    char lineCopy[CMD_LINE_MAX_LEN];
    char *argv[CMD_ARGV_MAX];
    AppAD5933Config_t nextConfig;
    unsigned long incNum;
    int argc;
    int opt;
    int selectedMode = -1;
    uint8_t hasMode = 0U;
    uint8_t hasFixedFreq = 0U;
    uint8_t hasSweepParam = 0U;
    char reply[160];

    if (line == NULL) {
        return;
    }

    strncpy(lineCopy, line, sizeof(lineCopy) - 1U);
    lineCopy[sizeof(lineCopy) - 1U] = '\0';

    argc = SplitCommandLine(lineCopy, argv, CMD_ARGV_MAX);
    if (argc == 0) {
        return;
    }
    if (argc < 0) {
        QueueUartText("CMD ERR: too many arguments\r\n");
        return;
    }

    if ((strcmp(argv[0], "help") == 0) || (strcmp(argv[0], "?") == 0)) {
        PrintCommandHelp();
        return;
    }

    if (strcmp(argv[0], "set") != 0) {
        QueueUartText("CMD ERR: command must start with 'set'\r\n");
        PrintCommandHelp();
        return;
    }

    GetAD5933Config(&nextConfig);
    selectedMode = (int)nextConfig.mode;

    optind = 1;
    opterr = 0;

    while ((opt = getopt(argc, argv, "ho:q:s:e:i:")) != -1) {
        switch (opt) {
        case 'h':
            PrintCommandHelp();
            return;

        case 'o':
            if ((strcmp(optarg, "0") != 0) && (strcmp(optarg, "1") != 0)) {
                QueueUartText("CMD ERR: -o must be 0(sweep) or 1(fixed)\r\n");
                return;
            }
            selectedMode = (optarg[0] == '0') ? APP_FREQ_MODE_SWEEP : APP_FREQ_MODE_FIXED;
            hasMode = 1U;
            break;

        case 'q':
            if (ParsePositiveKHz(optarg, &nextConfig.fixedFreqHz) == 0U) {
                QueueUartText("CMD ERR: invalid -q fixed frequency\r\n");
                return;
            }
            hasFixedFreq = 1U;
            break;

        case 's':
            if (ParsePositiveKHz(optarg, &nextConfig.startFreqHz) == 0U) {
                QueueUartText("CMD ERR: invalid -s sweep start frequency\r\n");
                return;
            }
            hasSweepParam = 1U;
            break;

        case 'e':
            if (ParsePositiveKHz(optarg, &nextConfig.endFreqHz) == 0U) {
                QueueUartText("CMD ERR: invalid -e sweep end frequency\r\n");
                return;
            }
            hasSweepParam = 1U;
            break;

        case 'i':
            if (ParsePositiveKHz(optarg, &nextConfig.incFreqHz) == 0U) {
                QueueUartText("CMD ERR: invalid -i sweep step frequency\r\n");
                return;
            }
            hasSweepParam = 1U;
            break;

        default:
            QueueUartText("CMD ERR: unknown option or missing option value\r\n");
            PrintCommandHelp();
            return;
        }
    }

    if (optind < argc) {
        QueueUartText("CMD ERR: unexpected argument after options\r\n");
        return;
    }

    if ((hasMode == 0U) && (hasFixedFreq != 0U) && (hasSweepParam == 0U)) {
        selectedMode = APP_FREQ_MODE_FIXED;
    } else if ((hasMode == 0U) && (hasSweepParam != 0U) && (hasFixedFreq == 0U)) {
        selectedMode = APP_FREQ_MODE_SWEEP;
    } else if ((hasMode == 0U) && (hasSweepParam != 0U) && (hasFixedFreq != 0U)) {
        QueueUartText("CMD ERR: use -o when mixing -q with sweep parameters\r\n");
        return;
    }

    if (nextConfig.endFreqHz < nextConfig.startFreqHz) {
        QueueUartText("CMD ERR: sweep end must be >= sweep start\r\n");
        return;
    }
    if (nextConfig.incFreqHz == 0UL) {
        QueueUartText("CMD ERR: sweep step must be > 0\r\n");
        return;
    }
    if (((nextConfig.endFreqHz - nextConfig.startFreqHz) % nextConfig.incFreqHz) != 0UL) {
        QueueUartText("CMD ERR: (end-start) must be divisible by step\r\n");
        return;
    }

    incNum = (nextConfig.endFreqHz - nextConfig.startFreqHz) / nextConfig.incFreqHz;
    if (incNum > AD5933_MAX_INC_NUM) {
        QueueUartText("CMD ERR: too many sweep steps, max incNum is 511\r\n");
        return;
    }

    nextConfig.mode = (selectedMode == APP_FREQ_MODE_FIXED) ? APP_FREQ_MODE_FIXED : APP_FREQ_MODE_SWEEP;
    nextConfig.incNum = (unsigned short)incNum;
    nextConfig.version++;

    xSemaphoreTake(xAD5933ConfigMutex, portMAX_DELAY);
    gAD5933Config = nextConfig;
    xSemaphoreGive(xAD5933ConfigMutex);

    if (nextConfig.mode == APP_FREQ_MODE_FIXED) {
        snprintf(reply, sizeof(reply), "CMD OK: mode=fixed freq=%lu Hz\r\n", nextConfig.fixedFreqHz);
    } else {
        snprintf(reply, sizeof(reply),
                 "CMD OK: mode=sweep start=%lu Hz end=%lu Hz step=%lu Hz points=%lu\r\n",
                 nextConfig.startFreqHz,
                 nextConfig.endFreqHz,
                 nextConfig.incFreqHz,
                 incNum + 1UL);
    }
    QueueUartText(reply);
}

static void StartCommandRx(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &cmdRxByte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        char ch = (char)cmdRxByte;

        if ((ch == '\r') || (ch == '\n')) {
            if (cmdRxIndex > 0U) {
                if (cmdPendingReady == 0U) {
                    uint16_t i;

                    for (i = 0U; i < cmdRxIndex; i++) {
                        cmdPendingLine[i] = cmdRxLine[i];
                    }
                    cmdPendingLine[cmdRxIndex] = '\0';
                    cmdPendingReady = 1U;
                } else {
                    cmdLineDropped = 1U;
                }
                cmdRxIndex = 0U;
            }
        } else if ((ch == '\b') || (ch == 0x7f)) {
            if (cmdRxIndex > 0U) {
                cmdRxIndex--;
            }
        } else if (cmdRxIndex < (CMD_LINE_MAX_LEN - 1U)) {
            cmdRxLine[cmdRxIndex++] = ch;
        } else {
            cmdRxIndex = 0U;
            cmdLineOverflow = 1U;
        }

        StartCommandRx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        cmdRxIndex = 0U;
        StartCommandRx();
    }
}
/* USER CODE END Application */
