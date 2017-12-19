/*
 * WS2812B.c
 *
 *  Created on: Dec 3, 2017
 *      Author: Ted
 */

#include "WS2812B.h"
#include "string.h"  //for memcpy
#include <stdlib.h>  //for calloc and free
#include "stm32f1xx_hal.h"

/*
 * PWM at =1.25µs±600ns
 * a 0 is 0.4µs±150ns of ON time  //400±150ns, 250 to 550ns
 * a 1 is 0.8µs±150ns of ON time  //
 * reset > 50µs aka latch or pause between pulse chains
 * Clock source is 72 000 000. Will set the prescaler to 1 to keep it running at 72 Mhz.
 * The Period or Tarr (auto reload register) for 800 Khz (1.25uS) is Period = (72 Mhz / 800 Khz) = 90
 */

//memory per LED = 1 * 24 * uint8_t = 24 bytes
//strip of 300   48 * 300 = 14400 bytes
#define ledControlPin GPIO_PIN_6           //TIM4 Channel1
#define ledControlPort GPIOB			   //TIM4 Channel1
#define T_ZERO (29)  //The clock is running at 72 MHz, which is a period of 13.888..ns.  400ns / 13.88 = 28.8
#define T_ONE (58)   //Same as above, 800ns / 13.88 = 57.6
#define stripLatch (40) //Called RES in the data sheet and has to be >50uS. Number of 1.25us time periods - 50uS/ 1.25us = 40.

static const uint16_t totalCycles =  180;        //Used for configuring PWM. A value of 90 is 800Khz. A value of 180 is 400Khz
static uint16_t numberOfLeds;       //represents the total number of LEDs on the strip.
static uint32_t numberOfColorBits;  // = numberOfLeds * 24
static uint32_t fullFrameSize;      // = (numberOfLeds * 24) + stripLatch

static TIM_HandleTypeDef handleLedPwm;
static DMA_HandleTypeDef handleLedDma;


uint8_t *stripData = NULL;  //the state of the entire strip is stored here. The circular DMA cycles through this array.

static void ws2812b_ColorToBits(const uint8_t *colorByte, uint8_t *arrayOfBits);

uint8_t ws2812b_Init(uint16_t ledCount)
{
	  HAL_StatusTypeDef result;
      __HAL_RCC_TIM4_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();

	  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 2);
	  HAL_NVIC_EnableIRQ(TIM4_IRQn);
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	  GPIO_InitStruct.Pin = ledControlPin;
	  HAL_GPIO_Init(ledControlPort, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(ledControlPort, ledControlPin, GPIO_PIN_RESET);

	  handleLedPwm.Instance = TIM4;
	  handleLedPwm.Init.Prescaler =  0;
	  handleLedPwm.Init.CounterMode = TIM_COUNTERMODE_UP;
	  handleLedPwm.Init.Period = totalCycles;
	  handleLedPwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	 // handleHTRPWM.Init.RepetitionCounter = 99;  //doesn't apply for tim4
	  result = HAL_TIM_Base_Init(&handleLedPwm);

  	  TIM_ClockConfigTypeDef sClockSourceConfig;
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  result |= HAL_TIM_ConfigClockSource(&handleLedPwm, &sClockSourceConfig);

	  result |= HAL_TIM_PWM_Init(&handleLedPwm);

	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	  result |= HAL_TIM_SlaveConfigSynchronization(&handleLedPwm, &sSlaveConfig);

	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  result |= HAL_TIMEx_MasterConfigSynchronization(&handleLedPwm, &sMasterConfig);

	  TIM_OC_InitTypeDef sConfigOC;
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 10;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

	  result |= HAL_TIM_PWM_ConfigChannel(&handleLedPwm, &sConfigOC, TIM_CHANNEL_1);

	  // Configure TIM4 DMA
	  __HAL_RCC_DMA1_CLK_ENABLE();
	  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 15);
	  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	  handleLedDma.Instance = DMA1_Channel1; //DMA1_Channel7;
	  handleLedDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
	  handleLedDma.Init.PeriphInc = DMA_PINC_DISABLE;
	  handleLedDma.Init.MemInc = DMA_MINC_ENABLE;
	  handleLedDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	  handleLedDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	  handleLedDma.Init.Mode = DMA_CIRCULAR;
	  handleLedDma.Init.Priority = DMA_PRIORITY_HIGH;
	  result |= HAL_DMA_Init(&handleLedDma);

	  __HAL_LINKDMA(&handleLedPwm,hdma[TIM_DMA_ID_CC1],handleLedDma);

	  if (stripData != NULL)
		  free(stripData);

	  numberOfLeds = ledCount;
	  numberOfColorBits = numberOfLeds * 24;
	  fullFrameSize = numberOfColorBits + stripLatch;

	  stripData = (uint8_t *)calloc(fullFrameSize, sizeof(uint8_t));  //allocate and set every value to 0
	  if (stripData == NULL)
		  result = HAL_ERROR;
	  else
	  {
		  ws2812b_SetStripOff();
		  result |= HAL_TIM_PWM_Start_DMA(&handleLedPwm, TIM_CHANNEL_1, (uint32_t *)stripData, fullFrameSize);
	  }

	  return result;
}

void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handleLedDma);
}

/*
 * Turns off entire LED strip by setting G = 0, R = 0, B = 0 for each LED
 */
void ws2812b_SetStripOff()
{
	//fill buffer with  T_ZEROs
	for(uint32_t i = 0; i < numberOfColorBits; i++)
		stripData[i] = T_ZERO;
}

/*
 * Sets the entire LED strip to newColor
 */
void ws2812b_SetStripColor(const ledcolor_t *newColor)
{
	for(uint16_t i = 0; i < numberOfLeds; i++)
		ws2812b_LedSet(&i, newColor);
}

void ws2812b_RotateStrip(rotation_t direction, uint16_t rotateBy)
{
	//since each color bit is one element in the array, 24 elements need to be moved in order to rotate by one led.
	uint8_t temp[24];
	if (direction == rotateClockwise)
	{
		for (uint32_t numberRotations = 0; numberRotations < rotateBy; numberRotations++)
		{
		memcpy(temp, stripData, 24);
		for (uint32_t i = 0; i < numberOfColorBits - 24; i++) //using numberOfColorBits and not fullFrameSize as we don't want to move the last 40 bytes used for reset.
			stripData[i] = stripData[i+24];
		memcpy(&stripData[numberOfColorBits - 24], temp, 24);
		}
	}
	else if (direction == rotateCounterClockwise)
	{
		for (uint32_t numberRotations = 0; numberRotations < rotateBy; numberRotations++)
		{
		memcpy(temp, &stripData[numberOfColorBits-24], 24);
		for (uint32_t i = numberOfColorBits - 1; i > 23; i--)
			stripData[i] = stripData[i-24];
		memcpy(stripData, temp, 24);
		}
	}
}

void ws2812b_LedSwap(uint16_t position1, uint16_t position2)
{
	uint8_t temp[24];
	position1 *= 24;
	position2 *= 24;
	memcpy(temp, &stripData[position2], 24);
	memcpy(&stripData[position2], &stripData[position1], 24);
	memcpy(&stripData[position1], temp, 24);
}

void ws2812b_LedCopy(uint16_t numLedsToCopy, uint16_t copyFrom, uint16_t copyTo)
{
	memmove(&stripData[copyTo * 24], &stripData[copyFrom * 24], numLedsToCopy * 24);
}

/*
 * Similar to LedCopy, but after it copies to the new location, it frees (turns off) the Led at the old location.
 */
void ws2812b_LedMove(uint16_t numLedsToCopy, uint16_t copyFrom, uint16_t copyTo)
{
	ws2812b_LedCopy(numLedsToCopy, copyFrom, copyTo);
	memset(&stripData[copyFrom * 24], T_ZERO, numLedsToCopy * 24);
}


/*
 * Sets the LED at positionInStrip to newColor
 */
void ws2812b_LedSet(const uint16_t *positionInStrip, const ledcolor_t *newColor)
{
	ws2812b_ColorToBits(&(newColor->green), &stripData[*positionInStrip*24]);
	ws2812b_ColorToBits(&(newColor->red), &stripData[*positionInStrip*24 + 8]);
	ws2812b_ColorToBits(&(newColor->blue), &stripData[*positionInStrip*24 + 16]);
}

/*
G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0
Note: Follow the order of GRB to sent data and the high bit sent at first.
*/
//takes a byte of color and converts it to an 8 element array of bits with most significant bit first
static void ws2812b_ColorToBits(const uint8_t *colorByte, uint8_t *arrayOfBits)
{
	for(uint8_t bit = 0; bit < 7; bit++)
	{
		uint8_t testBit = 7 - bit; //this is in order to get MSB first
		arrayOfBits[bit] = (*colorByte & (1 << testBit)) ? T_ONE : T_ZERO;
	}
}
