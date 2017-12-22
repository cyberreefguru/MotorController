/*
 * MotorController.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: tsasala
 */

#include "MotorController.h"

uint8_t leds[NUM_LEDS] = {0,1,2,3}; // Analog port (Port C)
uint8_t inputs[NUM_INPUTS] = {8,11,12}; // Input Pins

/**
 * Constructor
 */
MotorController::MotorController()
{
	motors[0].initialize(2,3,9);
	motors[1].initialize(4,5,10);
}


/**
 * Initializes the hardware
 *
 */
void MotorController::initialize()
{
	uint8_t i;

	// Set internal LED
	digitalWrite(LED_PIN, LOW);

	// Setup inputs
	for(i=0; i<NUM_INPUTS; i++)
	{
		pinMode( inputs[i], INPUT ); // set to input
		digitalWrite(inputs[i], HIGH); // set internal pull up
	}

	// Configure LED port
	DDRC = DDRC | 0x3F; // Set PC0..5 to output
	PORTC = PORTC & 0xC0; // Set port to low

} // end initialize

/**
 * Returns an instance of a motor to control
 *
 */
Motor MotorController::getMotor(MotorEnum m)
{
	return motors[m];

}

/**
 * Sets specified LED to the specified value.
 *
 * false = off; true = on.
 *
 */
void MotorController::toggleLed(LedEnum led)
{
	setLed(led, !getLed(led) );
}

/**
 * Sets specified LED to the specified value.
 *
 * false = off; true = on.
 *
 */
void MotorController::setLed(LedEnum led, boolean v)
{
	if( v == false )
	{
		bitClear(PORTC, led);
	}
	else
	{
		bitSet(PORTC, led);
	}
}

/**
 * Gets specified LED value.
 *
 * false = off; true = on.
 *
 */
boolean MotorController::getLed(LedEnum led)
{
	return bitRead(PORTC, led);
}

