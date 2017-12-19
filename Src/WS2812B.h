/*
 * WS2812B.h
 *
 *  Created on: Dec 3, 2017
 *      Author: Ted
 */

#ifndef APPLICATION_USER_WS2812B_H_
#define APPLICATION_USER_WS2812B_H_

#include "stdint.h"

typedef enum
{
	cmdStripInit,         //initialize the strip to the count of LEDs.
	cmdStripOff,          //turn off entire strip
	cmdStripSetColor,     //set all LEDs in the strip to a specific color
	cmdStripFadeIn,       //increments each LED on the strip's G, R and B value the supplied value
	cmdStripFadeOut,      //decrements each LED on the strip's G, R and B value the supplied value
	cmdStripRotateClockwise,      //shift all LEDs in the strip in the positive direction by a specific number of LEDs.
	cmdStripRotateCounterClockwise,
	cmdLedSwap,			  //swap the values of two Leds
	cmdLedSetColor,		  //sets an led at a selected location to a selected value
	cmdLedGetColor,        //gets the Led values at a selected location
	cmdLedFadeIn,
	cmdLedFadeOut,
	cmdLedCopy,			//copy one or more Leds to a new location - number of LEDs to copy, position of 1st LED to copy, position of 1st LED to paste
	cmdLedMove,    //Similar to LedCopy, but after it copies to the new location, it frees (turns off) the Led at the old location.
} commands_t;

typedef enum
{
	rotateClockwise,
	rotateCounterClockwise,
} rotation_t;

typedef struct
{
	uint8_t green;
	uint8_t red;
	uint8_t blue;
} ledcolor_t;

uint8_t ws2812b_Init(uint16_t ledCount);
void ws2812b_SetStripOff();
void ws2812b_SetStripColor(const ledcolor_t *newColor);
void ws2812b_RotateStrip(rotation_t direction, uint16_t rotateBy);
void ws2812b_LedCopy(uint16_t numLedsToCopy, uint16_t copyFrom, uint16_t copyTo);
void ws2812b_LedMove(uint16_t numLedsToCopy, uint16_t copyFrom, uint16_t copyTo);
void ws2812b_LedSet(const uint16_t *positionInStrip, const ledcolor_t *newColor);
void ws2812b_LedSwap(uint16_t position1, uint16_t position2);

#endif /* APPLICATION_USER_WS2812B_H_ */
