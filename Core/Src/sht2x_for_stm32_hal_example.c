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
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * Minimal setup:
 *								 _________
 *					            |  _____  |
 *                      N/C 4 --| |_____| |-- 3 N/C
 *                      VDD 5 --|         |-- 2 GND
 *                     SDA1 6 --|         |-- 1 SCL1
 *					            |_________|
 *
 *				Refer to datasheets for further information.
 *
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sht2x_for_stm32_hal.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/**
 * @brief  The application entry point.
 * @retval int
 */
int sht2x_main(void)
{
//	SHT2x_Init(&hi2c1);
//	SHT2x_SetResolution(RES_14_12);
//
//	while (1) {
//		unsigned char buffer[100] = { 0 };
//		/* Gets current temperature & relative humidity. */
//		float cel = SHT2x_GetTemperature(1);
//		/* Converts temperature to degrees Fahrenheit and Kelvin */
//		float fah = SHT2x_CelsiusToFahrenheit(cel);
//		float kel = SHT2x_CelsiusToKelvin(cel);
//		float rh = SHT2x_GetRelativeHumidity(1);
//		/* May show warning below. Ignore and proceed. */
//		sprintf(buffer,
//				"%d.%dºC, %d.%dºF, %d.%d K, %d.%d%% RH\n",
//				SHT2x_GetInteger(cel), SHT2x_GetDecimal(cel, 1),
//				SHT2x_GetInteger(fah), SHT2x_GetDecimal(fah, 1),
//				SHT2x_GetInteger(kel), SHT2x_GetDecimal(kel, 1),
//				SHT2x_GetInteger(rh), SHT2x_GetDecimal(rh, 1));
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), 1000);
//		HAL_Delay(250);
//	}
	return 0;
}

