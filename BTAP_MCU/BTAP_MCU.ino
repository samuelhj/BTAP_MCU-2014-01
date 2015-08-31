// 
// BTAP_MCU 
//
// Program for the MCU of the 2014-01 balloon launch.
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Authors	 	Samúel Úlfr Þór Hjaltalín Guðjónsson
// 				samuel@ulfr.net
//
//              Michael Hansen
//				michael@bylur.net
//
//
// Date			2014.1.19 19:21
// Version		version
// 
// Copyright	© Samúel Úlfr Þór Hjaltalín Guðjónsson, 2014
// License

/*				Copyright 2014 Bifrost the Aurora Project All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. Modification, or partial use is permitted as long as the original author is NOT mentioned in any way.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY BIFROST THE AURORA PROJECT ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BIFRÖST THE AURORA PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of Bifröst the Aurora Project

*/
// See			ReadMe.txt for references
//

// Core library for code-sense
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"   
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(CORE_TEENSY) // Teensy specific
#include "WProgram.h"
#elif defined(ARDUINO) && (ARDUINO >= 100) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#elif defined(ARDUINO) && (ARDUINO < 100) // Arduino 23 specific
#include "WProgram.h"
#else // error
#error Platform not defined
#endif

// Include application, user and local libraries
#include "LocalLibrary.h"

// BTAP specific
#include <EEPROM.h>


// DEFINES

// CLEAR EEPROM upon start?
// 1 = YES
// 0 = NO

#define CLEAR_EEPROM 0 // define for EEPROM to clear it.

#define EEPROM_OFFSET_MAX 1022 // this _must_ be dividable by two
// The Atmega328P has 1kiB of EEPROM. So to fully use the memory set this at 1022
// memory bank 1023 (1024) stores the value for the counter that is used to determine
// how far into the temperature write list we've gone. ie stores what memory bank to write to next.
// Memory bank 0 (1) is not used to get the thing to be able to be divided by two.
// this reduces overhead calculations to determine in what memory bank we're writing to.
// see read_temp()


#define SERIAL_BAUD 9600	// Baud rate of serial output
#define SERIAL_LOG_HEAD "Start of log."
#define SERIAL_LOG_TAIL "End of log."
#define SERIAL_LOG_SPACE "  " // divider used between in and out values in alternative version

// define for debug, 1 == on, 0 == off
#define DEBUG 1
#define DEBUG_INTERVAL 2000

// define for light beacon

#define BEACON_INTERVAL 1500 // for testing
#define BEACON_INTERVAL_OFF 1000 // Light beacon is off for defined milliseconds
#define BEACON_INTERVAL_ON 250 // Light beacon is on for defined milliseconds
#define BEACON_LEDPIN   13 // pin 13 is used for testing on the dev board, pin 12 is the one that's actually used in the payload
// REMEMBER TO CHANGE THIS TO 12 PRIOR TO FLIGHT!!!!!!
#define STATUS_LEDPIN 13 // pin is used for status display on the dev board.

// define for sensors

// define the pins we use to read the temperature sensor - Should explain it self
#define LM35_INTERNAL_POSITIVE 0
#define LM35_INTERNAL_NEGATIVE 1
#define LM35_EXTERNAL_POSITIVE 2
#define LM35_EXTERNAL_NEGATIVE 3

// define interval of EEPROM write (normally 90,000 ms or 90s)
#define EEPROM_WRITE_INTERVAL 20000



// Functions start here


// Debug section

// variable for last time the timer was "restarted"
unsigned long last_avr_runtime_timestamp = 0;

void avr_runtime()
{
	unsigned long avr_runtime_timestamp = millis();
	
	// Check if we should run the debug output
	if((avr_runtime_timestamp - last_avr_runtime_timestamp) >= DEBUG_INTERVAL)
	{
		last_avr_runtime_timestamp = avr_runtime_timestamp;
		Serial.begin(SERIAL_BAUD);
		Serial.println("AVR has been running for ");
		Serial.println(millis());
		Serial.println(" milliseconds \n");
		Serial.end();
		avr_runtime_timestamp = millis();
		
	}
}


// light beacon

// variables for timers of the light beacon
unsigned long beacon_timestamp_on_old = 0;
unsigned long beacon_timestamp_off_old = 0;
unsigned long beacon_timestamp_old = 0;

void statusled_on()
{
	
	// turn on led, regardless of current state
	
	
	digitalWrite(STATUS_LEDPIN, HIGH);

}

void statusled_off()
{
	
	// turn off led, regardless of current state
	
	digitalWrite(STATUS_LEDPIN, LOW);

}



void beacon_on()
{
	
	// turn on led, regardless of current state
	
	 
	digitalWrite(BEACON_LEDPIN, HIGH);
	beacon_timestamp_on_old = millis(); /* so the first beacon_toggle following this call does not instantanously turn the led off */
}

void beacon_off()
{
	
	// turn off led, regardless of current state
	
	 
	digitalWrite(BEACON_LEDPIN, LOW);
	beacon_timestamp_off_old = millis(); /* so the first beacon_toggle following this call does not instantanously turn the led on */
}


// Here we shift between HIGH and LOW state of light beacon

void beacon_toggle()
{
	// define variables for beacon_toggle()
	unsigned long beacon_timestamp_on = millis();
	unsigned long beacon_timestamp_off = millis();

	// millis() counts the milliseconds since the AVR started to run
	
    // We check how long since the light beacon was on
    if ((beacon_timestamp_on - beacon_timestamp_on_old) >= BEACON_INTERVAL_OFF)
    {
        beacon_timestamp_on_old = beacon_timestamp_on;
        digitalWrite(BEACON_LEDPIN, HIGH);
		
		// debug
		if(DEBUG == 1)
		{
			Serial.begin(SERIAL_BAUD);
			Serial.println("beacon ON: ");
			Serial.println(beacon_timestamp_on);
			Serial.end();
    
		}
	}
	
    else if ((beacon_timestamp_off - beacon_timestamp_off_old) >= BEACON_INTERVAL_ON)
    {
        beacon_timestamp_off_old = beacon_timestamp_off;
        digitalWrite(BEACON_LEDPIN, LOW);
		
		// debug
		if(DEBUG == 1)
		{
			Serial.begin(SERIAL_BAUD);
			Serial.println("beacon off: ");
			Serial.println(beacon_timestamp_off);
			Serial.end();
		}
    }

}


// EEPROM


// To easily log the output from EEPROM_transfer()
// use screen with these commands when you fire up the arduino board with the BTA MCU in it.
// 'screen -L /dev/tty.usbmodem1431 9600' and log file will be written to screenlog.N

void EEPROM_transfer()
{
	unsigned int offset = 0; /* current offset to write to */
//	const uint8_t*	offset = 0;
	float value;
	
	beacon_on();
	
	/* initialise serial transfer, assumingly this is part of the AVR ecosystem too? */
	Serial.begin(SERIAL_BAUD);
	Serial.println(SERIAL_LOG_HEAD);
	
	while (offset < EEPROM_OFFSET_MAX)
	{
	//	value = EEPROM.read(offset); /* this can probably fail, so some kind of errorcheck would be nice :) */
		value = EEPROM.read(offset);
		Serial.println(value);
		
		offset++;
	}
	
	/* Alternatively you can write the log with the pairs of inside/outside meassures written like "<in>  <out>"
	 * instead of listing all the inside values first and then the outside (in case you want to compare the two side-by-side)
	 *
     
     while (offset < (EEPROM_OFFSET_MAX / 2))
     {
     value = EEPROM.read(offset);
     Serial.print(value);
     
     Serial.print(SERIAL_LOG_SPACE);
     
     value = EEPROM.read(offset + (EEPROM_OFFSET_MAX / 2);
     Serial.println(value);
     
     offset++;
     }
     */
	
	Serial.println(SERIAL_LOG_TAIL);
	Serial.end();
	beacon_off();
}


// LM35 Temperature sensors


// define variables

unsigned int sensor_eeprom_offset = 1; /* how far into the eeprom memory we are (aka. next offset to write at) */


// variable for timestamp, the last time we wrote to EEPROM.
unsigned long last_eeprom_write = 0;


int sensor_read()
{
	float temp_internal, temp_external, read_positive, read_negative;
	
	
	// We read memory bank 1024, where we store sensor_eeprom_offset
//	EEPROM.read(1023, sensor_eeprom_offset);
	sensor_eeprom_offset = EEPROM.read(1023);
	sensor_eeprom_offset = sensor_eeprom_offset * 2;
	
	/*
	 * bail out if we have already written the maximum number of pairs we have room for
	 *
	 */
	
	
	if (sensor_eeprom_offset >= ((EEPROM_OFFSET_MAX)/2))
	{
		// Ticket #7
	
		if(DEBUG == 1)
		{
			Serial.begin(SERIAL_BAUD);
			Serial.println("EEPROM at max: ");
			Serial.println(sensor_eeprom_offset);
			Serial.end();
		}
		
		return(0);

		

	}
	
		unsigned long eeprom_write_timestamp = millis();
		
	if((eeprom_write_timestamp - last_eeprom_write) >= EEPROM_WRITE_INTERVAL)
	{
		last_eeprom_write = eeprom_write_timestamp;
		
		// We read the LM35 (temperature) here.
		
		
		// Ticket #8
		// possibly solved?
		/*
		 * read internal temp sensor
		 *
		 */
		
		read_positive = analogRead(LM35_INTERNAL_POSITIVE);
		read_negative = analogRead(LM35_INTERNAL_NEGATIVE);
		
		/* magic */
		read_positive = ((read_positive / 1024.0) * 5000) / 10.0;
		read_negative = ((read_negative / 1024.0) * 5000) / 10.0;
		
		temp_internal = read_positive - read_negative;
		
		/*
		 * read external temp sensor
		 *
		 */
		
		read_positive = analogRead(LM35_EXTERNAL_POSITIVE);
		read_negative = analogRead(LM35_EXTERNAL_NEGATIVE);
		
		// magic for the ADC. The reference voltage level is 5V
		// more later.. I'm feeling zzz...
		
		read_positive = ((read_positive / 1024.0) * 5000) / 10.0;
		read_negative = ((read_negative / 1024.0) * 5000) / 10.0;
		
		temp_external = read_positive - read_negative;
		
		/*
		 * write sensor readings to EEPROM
		 *
		 */
		
		/* these writes most certainly need some checks/delays as we found out in a previous discussion,
		 * i have added delay() but checks would be required for anything beyond a basic prototype
		 */
		
		// We need interval timer here for the writing of the temp_external & temp_internal
		// fixed
		// also need to implement some error checking on the write of the eeprom
		// see http://playground.arduino.cc/Code/EEPROMex
		// ticket #5
		
		// interval timer

		// We need to check if the EEPROM is ready/written etc.. Ticket #3
		
		// write internal temperature to
		EEPROM.write(sensor_eeprom_offset, temp_internal);
		delay(100);
		
		// see if it measures and writes internal temperature
		if(DEBUG == 1)
		{
			Serial.begin(SERIAL_BAUD);
			Serial.println("We write to EEPROM: ");
			Serial.println(temp_internal);
			Serial.println("\n");
			// need to read from eeprom here
			Serial.println(EEPROM.read(sensor_eeprom_offset)); //new, did this work?
			Serial.end();
		}
		
		

		EEPROM.write((sensor_eeprom_offset + (EEPROM_OFFSET_MAX/2)), temp_external);
		delay(100);
		
		// see if it measures and writes external temperature
		if(DEBUG == 1)
		{
			Serial.begin(SERIAL_BAUD);
			Serial.println("We write to EEPROM: ");
			Serial.println(temp_external);
			// need to read from eeprom here
			Serial.println("EEPROM location: ");
			Serial.println('sensor_eeprom_offset');
			Serial.println("\n");
			Serial.end();
		}
		
		
		sensor_eeprom_offset++; // update offset to use on next call to this function
		
		// We write the sensor_eeprom_offset to EEPROM memory bank 1024.

		EEPROM.write(1023, ((sensor_eeprom_offset)/2));
		
		if(DEBUG == 1)
		{
			Serial.begin(SERIAL_BAUD);
			Serial.println("we are at memory bank: ");
			int a=0;
			a = EEPROM.read(1023);
			Serial.println('a');
			Serial.println("\n");
			Serial.end();
			delay(1000);
		}
			
	}
	
	return(1);
}


// Clear EEPROM

void EEPROM_clear()
{
	// write a 1 to all 1024 bytes of the EEPROM
	// to clear the EEPROM
	// 1 is used for so memory bank 1023 always starts 'fresh' at 1 for the counter
	// This takes 100ms + 10ms * 1023 = 112.530 ms (1m52s)
	if(CLEAR_EEPROM == 1)
	{
		for (int i = 0; i < 1024; i++)
		{
			EEPROM.write(i, 1);
			delay(100);
			
		}
		statusled_on();		// turn on status led, to notify that the erase has been complete
		delay(15000);		// for 15s so that the user has time to reburn with EEPROM_CLEAR = 0
		statusled_off();	// this gives enough time for user to notice that the eeprom clear is done.
	}
}

// setup routine, run once every restart.

void setup()
{
	pinMode(BEACON_LEDPIN, OUTPUT); // iniate BEACON_LEDPIN as output!
	pinMode(STATUS_LEDPIN, OUTPUT); // iniate STATUS_LEDPIN as output!
	
    // Transfer from the memory of the EEPROM to save temperature readings.
	EEPROM_transfer();
	
	// Then we CLEAR the EEPROM
	// If EEPROM_CLEAR == 1

	EEPROM_clear();
	
}

// This thing runs into eternity // main function

void loop()
{
	
	// To check how long the AVR has been running
	if(DEBUG == 1)
	{
		avr_runtime();

	}

	// sensor_read returns 1 as long as the EEPROM memory is not full and 0 if EEPROM memory is full
	// this causes AVR to hang up after finishing writing to EEPROM, we don't want that really...
	// Ticket #2 - Solved
	
	// we want the beacon to run all the time, checking every time it runs through the loop if time is nigh
	
	beacon_toggle();
	
	// Then we'd like to read the temperature sensor and check whether it's time to write to EEPROM
	// In the sensor_read() function we check if the EEPROM is full or not before sensors are read.
			
	sensor_read();
	
}