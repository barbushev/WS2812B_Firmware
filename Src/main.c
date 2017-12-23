/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ctype.h"
#include "stdarg.h"

#include "WS2812B.h"

void USB_SEND(const char *string, ...);
void USB_sendOK();
void clear_rcv_data();

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_USB_DEVICE_Init();

while (!usbIsConnected())
	HAL_Delay(1000);

	while (1)
	{
	if ((hUsbDevice_0 != NULL) && (hUsbDevice_0->dev_state) && receive_total)		//if usb is not null, is connected and something was received
		{
		ledcolor_t newColor;
		uint16_t data[5]; //1 command + up to 4 parameters
		uint8_t error = 0;

		char *token;
		token = strtok ((char *)received_data, " ");
		for (uint8_t i = 0; i < 5; i++)
		{
			if (token == NULL) break;
			data[i] = atoi(token);
			token = strtok (NULL, " ");
		}

		switch (data[0])
		{
			case cmdStripInit:
			{
				ws2812b_Init(data[1]);
				break;
			}

			case cmdStripOff:
			{
				ws2812b_SetStripOff();
				break;
			}

			case cmdStripSetColor:
			{
				newColor.green = data[1];
				newColor.red = data[2];
				newColor.blue = data[3];
				ws2812b_SetStripColor(&newColor);
				break;
			}

			case cmdStripRotate:
			{
			    ws2812b_RotateStrip(data[1], data[2]);
			    break;
			}

	     	case cmdLedSwap:
			{
				ws2812b_LedSwap(data[1], data[2]);
				break;
			}

			case cmdLedSetColor:
			{
			    newColor.green = data[2];
			    newColor.red = data[3];
		        newColor.blue = data[4];
			    ws2812b_LedSet(&data[1], &newColor);
			    break;
			}

			case cmdLedCopy:
			{
				ws2812b_LedCopy(data[1], data[2], data[3]);
				break;
			}

			case cmdLedMove:
			{
				ws2812b_LedMove(data[1], data[2], data[3]);
				break;
			}

			case cmdLedIncColor:
			{
				newColor.green = data[2];
				newColor.red = data[3];
				newColor.blue = data[4];
				ws2812b_LedIncrementColor(&data[1], &newColor);
				break;
			}

			case cmdLedDecColor:
			{
				newColor.green = data[2];
				newColor.red = data[3];
				newColor.blue = data[4];
				ws2812b_LedDecrementColor(&data[1], &newColor);
				break;
			}

			case cmdStripIncColor:
			{
				newColor.green = data[1];
				newColor.red = data[2];
				newColor.blue = data[3];
				ws2812b_StripIncrementColor(&newColor);
				break;
			}

			case cmdStripDecColor:
			{
				newColor.green = data[1];
				newColor.red = data[2];
				newColor.blue = data[3];
				ws2812b_StripDecrementColor(&newColor);
				break;
			}

			case cmdLedCheck:
			{
				newColor.green = data[2];
				newColor.red = data[3];
				newColor.blue = data[4];
				error = (ws2812b_LedCheck(&data[1], &newColor) != 1);
				break;
			}

			case cmdStripRotateSegment:
			{
				ws2812b_RotateStripSegment(data[1], data[2], data[3], data[4]);
				break;
			}

			default: error = 1;
		}

		 if (error == 0) USB_sendOK();
		 clear_rcv_data();
		}
	}
}

void USB_sendOK()
{
	while (CDC_Transmit_FS((uint8_t *)"OK", 3));
}

//clear the received_data
void clear_rcv_data()
{
	received_data_size = 0;
	memset(received_data, 0, receive_total);
	receive_total = 0;
}


/*
 * Helper function to send a bunch of data over
 */
void USB_SEND(const char *string, ...)
{
char buf[512];
va_list arglist;
va_start(arglist,string);
uint8_t length = vsprintf(buf,string,arglist);
va_end(arglist);

while (CDC_Transmit_FS((uint8_t *)buf, length));
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

//#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	USB_SEND("error %s, %u", file, line);
}
//#endif
