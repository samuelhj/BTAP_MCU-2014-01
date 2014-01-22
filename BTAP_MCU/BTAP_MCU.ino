// 
// BTAP_MCU 
//
// Program for the MCU of the 2014-01 balloon launch.
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Authors	 	Samúel Úlfr Þór Hjaltalín Guðjónsson
// 				samuel@ulfr.net
//
//              Michael
//
//
// Date			2014.1.19 19:21
// Version		<#version#>
// 
// Copyright	© Samúel Úlfr Þór Hjaltalín Guðjónsson, 2014
// License		<#license#>
//
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


// Prototypes


/* i have put the #defines for each section inside the sections they "belong" to
 * for easier top-to-bottom reading, but you may want to put them all up here for
 * easier quick-overview -- TBD Ticket #5
 */


// DEFINES

// define for EEPROM

#define EEPROM_OFFSET_MAX 1024 /* this _must_ be dividable by two */
#define SERIAL_BAUD 9600
#define SERIAL_LOG_HEAD "Start of log."
#define SERIAL_LOG_TAIL "End of log."
#define SERIAL_LOG_SPACE "  " /* divider used between in and out values in alternative version */

// define for debug, 1 == on, 0 == off
#define DEBUG 1
#define DEBUG_INTERVAL 2000

// define for light beacon

#define BEACON_INTERVAL 1500 // for testing
#define BEACON_INTERVAL_OFF 750 // Light beacon is off for 750ms
#define BEACON_INTERVAL_ON 150 // Light beacon is on for 150ms
#define BEACON_LEDPIN   13 // pin 13 is used for testing on the dev board, pin 12 is the one that's actually used in the payload

// define for sensors

/* define the pins we use to read the temperature sensor */
#define LM35_INTERNAL_POSITIVE 0
#define LM35_INTERNAL_NEGATIVE 1
#define LM35_EXTERNAL_POSITIVE 2
#define LM35_EXTERNAL_NEGATIVE 3

// define interval of EEPROM write
#define EEPROM_WRITE_INTERVAL 90



// Functions start here


// Debug section

// unsigned long debug_timer = DEBUG_INTERVAL;
unsigned long last_debug = 0;


void debug_tester()
{
	unsigned long debug_timestamp = millis();
	
	// Check if we should run the debug output
	if((debug_timestamp - last_debug) >= DEBUG_INTERVAL)
	{
		last_debug = debug_timestamp;
		Serial.begin(SERIAL_BAUD);
		Serial.println("AVR has been running for ");
		Serial.println(millis());
		Serial.println(" milliseconds \n");
		Serial.end();
		debug_timestamp = millis();
		
	}
}




/*
 * beacon
 *
 */



unsigned long beacon_timestamp_on_old = 0;
unsigned long beacon_timestamp_off_old = 0;
unsigned long beacon_timestamp_old = 0;

int beacon_led_state = LOW; /* i assume the led is initially off */


void beacon_init()
{
	pinMode(BEACON_LEDPIN, OUTPUT); /* pinMode() and OUTPUT assumed defined elsewhere */
    
}

void beacon_on()
{
	/*
	 * turn on led, regardless of current state
	 *
	 */
	digitalWrite(BEACON_LEDPIN, HIGH);
	beacon_timestamp_on_old = millis(); /* so the first beacon_toggle following this call does not instantanously turn the led off */
}

void beacon_off()
{
	/*
	 * turn off led, regardless of current state
	 *
	 */
	digitalWrite(BEACON_LEDPIN, LOW);
	beacon_timestamp_off_old = millis(); /* so the first beacon_toggle following this call does not instantanously turn the led on */
}



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

  
/*
	if ((timestamp - beacon_timestamp_old) >= BEACON_INTERVAL)
	{
		beacon_timestamp_old = timestamp;
		
		// I assume ledState and HIGH/LOW are also already defined somewhere else?
		if (beacon_led_state == HIGH)
		{
			beacon_led_state = LOW;
			
		} else {
			
			beacon_led_state = HIGH;
		}
		
		digitalWrite(BEACON_LEDPIN, beacon_led_state);
	}
*/
}

/*
 * EEPROM
 *
 */



// To easily log the output from EEPROM_transfer()
// use screen with these commands when you fire up the arduino board with the BTA MCU in it.
// 'screen -L /dev/tty.usbmodem1431 9600' and log file will be written to screenlog.N

void EEPROM_transfer()
{
	unsigned int offset = 0; /* current offset to write to */
	float value;
	
	beacon_on();
	
	/* initialise serial transfer, assumingly this is part of the AVR ecosystem too? */
	Serial.begin(SERIAL_BAUD);
	Serial.println(SERIAL_LOG_HEAD);
	
	while (offset < EEPROM_OFFSET_MAX)
	{
		value = EEPROM.read(offset); /* this can probably fail, so some kind of errorcheck would be nice :) */
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

/*
 * sensor(s)
 *
 *
 */



// define variables


unsigned int sensor_eeprom_offset = 0; /* how far into the eeprom memory we are (aka. next offset to write at) */

// unsigned long eeprom_write_timer = EEPROM_WRITE_INTERVAL;
unsigned long last_eeprom_write = 0;


int sensor_read()
{
	float temp_internal, temp_external, read_positive, read_negative;
	/*
	 * bail out if we have already written the maximum number of pairs we have room for
	 *
	 */
	if (sensor_eeprom_offset >= (EEPROM_OFFSET_MAX/2))
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
	
	/* magic */
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
	// also need to implement some error checking on the write of the eeprom
	// see http://playground.arduino.cc/Code/EEPROMex
	
	// interval timer
	
	
	
	// Debug section
	

	
	
	

		unsigned long eeprom_write_timestamp = millis();
		
	if((eeprom_write_timestamp - last_eeprom_write) >= EEPROM_WRITE_INTERVAL)
	{
		last_eeprom_write = eeprom_write_timestamp;
		


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
			Serial.end();
			eeprom_write_timestamp = millis();
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
			eeprom_write_timestamp = millis();
		}
		
		
		sensor_eeprom_offset++; // update offset to use on next call to this function
			
	}

	
	
	

	
	return(1);
}

/*
 * main function/loop
 *
 */




void setup()
{
    // Transfer from the memory of the EEPROM to save temperature readings.
	EEPROM_transfer();
	// we initalize the beacon (Isn't this too much of complication?)
    beacon_init();
}

void loop()
{
	

	if(DEBUG == 1)
	{
		debug_tester();

	}

//	beacon_on();
//	beacon shouldn't really be turned on, the beacon_toggle() function should do that entirely.
	
	
	// sensor_read returns 1 as long as the EEPROM memory is not full and 0 if EEPROM memory is full
	// this causes AVR to hang up after finishing writing to EEPROM, we don't want that really...
	// Ticket #2 - Solved
	
	// we want the beacon to run all the time, checking every time it runs through the loop if time is nigh
	beacon_toggle();
	
	// Then we'd like to read the temperature sensor and check whether it's time to write to EEPROM
	
	sensor_read();
	
	/*
	while (sensor_read())
	{
		beacon_toggle();
	}
*/
//	beacon_off(); /* in case our loop ended with the beacon on. */
	
	
}