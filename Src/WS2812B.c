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
 * PWM at =1.25탎�600ns
 * a 0 is 0.4탎�150ns of ON time  //400�150ns, 250 to 550ns
 * a 1 is 0.8탎�150ns of ON time  //
 * reset > 50탎 aka latch or pause between pulse chains
 * Clock source is 72 000 000. Will set the prescaler to 1 to keep it running at 72 Mhz.
 * The Period or Tarr (auto reload register) for 800 Khz (1.25uS) is Period = (72 Mhz / 800 Khz) = 90
 */

//memory per LED = 1 * 24 * uint8_t = 24 bytes
//strip of 300   48 * 300 = 14400 bytes
#define LED_CONTROL_PIN GPIO_PIN_6           //TIM4 Channel1
#define LED_CONTROL_PORT GPIOB			   //TIM4 Channel1
#define T_ZERO (29)  //The clock is running at 72 MHz, which is a period of 13.888..ns.  400ns / 13.88 = 28.8
#define T_ONE (58)   //Same as above, 800ns / 13.88 = 57.6
#define DATA_LATCH (40) //Called RES in the data sheet and has to be >50uS. Number of 1.25us time periods - 50uS/ 1.25us = 40.
#define RED_OFFSET (8)     //array offset. Red starts at the 8th element
#define BLUE_OFFSET (16)  //array offset. Blue starts at the 16th element.
#define LED_OFFSET (24)   //array offset. LED has 3 colors, 8 elements each = 24

static const uint16_t totalCycles =  180;        //Used for configuring PWM. A value of 90 is 800Khz. A value of 180 is 400Khz
static uint16_t numberOfLeds;       //represents the total number of LEDs on the strip.

static TIM_HandleTypeDef handleLedPwm;
static DMA_HandleTypeDef handleLedDma;


uint8_t *stripData = NULL;  //the state of the entire strip is stored here. The circular DMA cycles through this array.


//private function prototypes
static void ws2812b_BitsToColor(uint8_t *colorByte, const uint8_t *arrayOfBits);
static void ws2812b_ColorToBits(const uint8_t *colorByte, uint8_t *arrayOfBits);
static void ws2812b_IncrementColorChannel(const uint8_t *incrementBy, uint8_t *channelToIncrement);
static void ws2812b_DecrementColorChannel(const uint8_t *decrementBy, uint8_t *channelToDecrement);

/*
 * Initializes the LED strip to the number of ledCount.
 * NOTE that if there is not enough memory, the device will most likely generate a Hard_Fault rather than gracefully return an error.
 */
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

	  GPIO_InitStruct.Pin = LED_CONTROL_PIN;
	  HAL_GPIO_Init(LED_CONTROL_PORT, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(LED_CONTROL_PORT, LED_CONTROL_PIN, GPIO_PIN_RESET);

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
	  sConfigOC.Pulse = 0;
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
	  uint32_t fullFrameSize = (numberOfLeds * LED_OFFSET) + DATA_LATCH;

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
	uint32_t numberOfColorBits = numberOfLeds * LED_OFFSET;
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

/*
 * Rotate a segment of the strip starting at startIndex (included) and ending at endIndex (excluded) by rotateBy LEDs
 */
void ws2812b_RotateStripSegment(rotation_t direction, uint16_t startIndex, uint16_t endIndex, uint16_t rotateBy)
{
	uint8_t temp[LED_OFFSET];	//24 elements need to be moved in order to rotate by one led.
	startIndex *= LED_OFFSET;	//the start index of the first bit
	endIndex *= LED_OFFSET;     //the start index of the first bit
	if (direction == rotateClockwise)
	{
		for (uint32_t numberRotations = 0; numberRotations < rotateBy; numberRotations++)
		{
		memcpy(temp, &stripData[startIndex], LED_OFFSET);  //copy the first LED to temp
		for (uint32_t i = startIndex; i < (endIndex - LED_OFFSET); i++)
			stripData[i] = stripData[i + LED_OFFSET];
		memcpy(&stripData[endIndex - LED_OFFSET], temp, LED_OFFSET);  //copy temp to the last position
		}
	}
	else if (direction == rotateCounterClockwise)
	{
		for (uint32_t numberRotations = 0; numberRotations < rotateBy; numberRotations++)
		{
		memcpy(temp, &stripData[endIndex - LED_OFFSET], LED_OFFSET);  //copy the last LED to temp
		for (uint32_t i = endIndex - 1; i >= (startIndex + LED_OFFSET); i--)
			stripData[i] = stripData[i - LED_OFFSET];
		memcpy(&stripData[startIndex], temp, LED_OFFSET);  //copy temp to the first position
		}
	}
}

/*
 * Rotates the LEDs in a clockwise or counterclockwise direction by a number of rotateBy
 */
void ws2812b_RotateStrip(rotation_t direction, uint16_t rotateBy)
{
	ws2812b_RotateStripSegment(direction, 0, numberOfLeds, rotateBy);
}

/*
 * Swap the colors of LEDs at position1 and position2
 */
void ws2812b_LedSwap(uint16_t position1, uint16_t position2)
{
	uint8_t temp[LED_OFFSET];
	position1 *= LED_OFFSET;
	position2 *= LED_OFFSET;
	memcpy(temp, &stripData[position2], LED_OFFSET);
	memcpy(&stripData[position2], &stripData[position1], LED_OFFSET);
	memcpy(&stripData[position1], temp, LED_OFFSET);
}

/*
 * Copy the LED colors of numLedsToCopy starting at copyFrom and pasting to copyTo
 */
void ws2812b_LedCopy(uint16_t numLedsToCopy, uint16_t copyFrom, uint16_t copyTo)
{
	memmove(&stripData[copyTo * LED_OFFSET], &stripData[copyFrom * LED_OFFSET], numLedsToCopy * LED_OFFSET);
}

/*
 * Calls ws2812b_LedCopy first, and then it turns off the LED(s) at the old location.
 */
void ws2812b_LedMove(uint16_t numLedsToCopy, uint16_t copyFrom, uint16_t copyTo)
{
	ws2812b_LedCopy(numLedsToCopy, copyFrom, copyTo);
	memset(&stripData[copyFrom * LED_OFFSET], T_ZERO, numLedsToCopy * LED_OFFSET);
}

/*
 * Sets the LED at positionInStrip to newColor
 */
void ws2812b_LedSet(const uint16_t *positionInStrip, const ledcolor_t *newColor)
{
	ws2812b_ColorToBits(&(newColor->green), &stripData[*positionInStrip*LED_OFFSET]);
	ws2812b_ColorToBits(&(newColor->red), &stripData[*positionInStrip*LED_OFFSET + RED_OFFSET]);
	ws2812b_ColorToBits(&(newColor->blue), &stripData[*positionInStrip*LED_OFFSET + BLUE_OFFSET]);
}

/*
 * Decreases the color of all LEDs on the strip by a value of newColor
 * If the existing value - new value < 0, it rolls over.
 */
void ws2812b_StripDecrementColor(const ledcolor_t *newColor)
{
	for(uint16_t i = 0; i < numberOfLeds; i++)
		ws2812b_LedDecrementColor(&i, newColor);
}

/*
 * Decreases the color of LED at positionInStrip by a value of newColor
 * If the existing value - new value < 0, it rolls over.
 */
void ws2812b_LedDecrementColor(const uint16_t *positionInStrip, const ledcolor_t *newColor)
{
	if (newColor->green > 0)
		ws2812b_DecrementColorChannel(&(newColor->green), &stripData[*positionInStrip*LED_OFFSET]);

	if (newColor->red > 0)
		ws2812b_DecrementColorChannel(&(newColor->red), &stripData[*positionInStrip*LED_OFFSET + RED_OFFSET]);

	if (newColor->blue > 0)
		ws2812b_DecrementColorChannel(&(newColor->blue), &stripData[*positionInStrip*LED_OFFSET + BLUE_OFFSET]);
}

/*
 * Increases the color of all LEDs on the strip by a value of newColor
 * If the existing value + new value > 255, it rolls over.
 */
void ws2812b_StripIncrementColor(const ledcolor_t *newColor)
{
	for(uint16_t i = 0; i < numberOfLeds; i++)
		ws2812b_LedIncrementColor(&i, newColor);
}

/*
 * Increases the color of LED at positionInStrip by a value of newColor
 * If the existing value + new value > 255, it rolls over.
 */
void ws2812b_LedIncrementColor(const uint16_t *positionInStrip, const ledcolor_t *newColor)
{
	if (newColor->green > 0)
		ws2812b_IncrementColorChannel(&(newColor->green), &stripData[*positionInStrip*LED_OFFSET]);

	if (newColor->red > 0)
		ws2812b_IncrementColorChannel(&(newColor->red), &stripData[*positionInStrip*LED_OFFSET + RED_OFFSET]);

	if (newColor->blue > 0)
		ws2812b_IncrementColorChannel(&(newColor->blue), &stripData[*positionInStrip*LED_OFFSET + BLUE_OFFSET]);
}

/*
 * Check if the current state of the led at positionInStrip matches with colorToCheck
 * Returns 1 if all 3 color channels match.
 */
uint8_t ws2812b_LedCheck(const uint16_t *positionInStrip, const ledcolor_t *colorToCheck)
{
	uint8_t tempGreen = 0, tempRed = 0, tempBlue = 0;
	ws2812b_BitsToColor(&tempGreen, &stripData[*positionInStrip*LED_OFFSET]);
	ws2812b_BitsToColor(&tempRed, &stripData[*positionInStrip*LED_OFFSET + RED_OFFSET]);
	ws2812b_BitsToColor(&tempBlue, &stripData[*positionInStrip*LED_OFFSET + BLUE_OFFSET]);
	return ((colorToCheck->green == tempGreen) && (colorToCheck->red == tempRed) && (colorToCheck->blue == tempBlue));
}

/*
 * Decrement a single color channel (Red, Green or Blue)
 */
static void ws2812b_DecrementColorChannel(const uint8_t *decrementBy, uint8_t *channelToDecrement)
{
	uint8_t temp = 0;
	ws2812b_BitsToColor(&temp, channelToDecrement);
	temp -= *decrementBy;
	ws2812b_ColorToBits(&temp, channelToDecrement);
}

/*
 * Increment a single color channel (Red, Green or Blue)
 */
static void ws2812b_IncrementColorChannel(const uint8_t *incrementBy, uint8_t *channelToIncrement)
{
	uint8_t temp = 0;
	ws2812b_BitsToColor(&temp, channelToIncrement);
	temp += *incrementBy;
	ws2812b_ColorToBits(&temp, channelToIncrement);
}

/*
 * Converts an array of 8 T_ONE or T_ZEROs back to a byte
 */
static void ws2812b_BitsToColor(uint8_t *colorByte, const uint8_t *arrayOfBits)
{
	*colorByte = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		uint8_t testBit = 7 - i;
		*colorByte |= (arrayOfBits[i] == T_ONE) << testBit;
	}
}

/*
 * Takes a byte of color and converts it to an 8 element array of T_ONE or T_ZERO with most significant bit first
 * G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0
 * Note: Follow the order of GRB to sent data MSB first.
 */
static void ws2812b_ColorToBits(const uint8_t *colorByte, uint8_t *arrayOfBits)
{
	for(uint8_t bit = 0; bit < 8; bit++)
	{
		uint8_t testBit = 7 - bit; //this is in order to get MSB first
		arrayOfBits[bit] = (*colorByte & (1 << testBit)) ? T_ONE : T_ZERO;
	}
}
