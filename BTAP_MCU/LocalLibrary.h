//
// File			LocalLibrary.h
// Brief		Library header
//
// Project	 	BTAP_MCU
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author		Samúel Úlfr Þór Hjaltalín Guðjónsson
// 				Samúel Úlfr Þór Hjaltalín Guðjónsson
// Date			2014.1.19 19:21
// Version		<#version#>
// 
// Copyright	© Samúel Úlfr Þór Hjaltalín Guðjónsson, 2014
// License		<#license#>
//
// See			ReadMe.txt for references
//


// Core library - IDE-based
#if defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(CORE_TEENSY) // Teensy specific
#include "WProgram.h"
#elif defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(ARDUINO) && (ARDUINO >= 100) // Arduino 1.0x and 1.5x specific
#include "Arduino.h"
#elif defined(ARDUINO) && (ARDUINO < 100)  // Arduino 23 specific
#include "WProgram.h"
#endif // end IDE

#ifndef BTAP_MCU_LocalLibrary_h
#define BTAP_MCU_LocalLibrary_h

//
// Brief	Blink a LED
// Details	LED attached to pin is light on then light off
// Total cycle duration = ms
// Parameters:
//      pin pin to which the LED is attached
//      times number of times
//      ms cycle duration in ms
//
void blink(uint8_t pin, uint8_t times, uint16_t ms);

#endif
