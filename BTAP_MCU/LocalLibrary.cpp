//
// LocalLibrary.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		BTAP_MCU
//
// Created by 	Samúel Úlfr Þór Hjaltalín Guðjónsson, 2014.1.19 19:21
// 				Samúel Úlfr Þór Hjaltalín Guðjónsson
//	
// Copyright 	© Samúel Úlfr Þór Hjaltalín Guðjónsson, 2014
// License 		<#license#>
//
// See 			LocalLibrary.cpp.h and ReadMe.txt for references
//


#include "LocalLibrary.h"

void blink(uint8_t pin, uint8_t times, uint16_t ms) {
  for (uint8_t i=0; i<times; i++) {
    digitalWrite(pin, HIGH); 
    delay(ms >> 1);               
    digitalWrite(pin, LOW);  
    delay(ms >> 1);              
  }
}
