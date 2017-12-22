/*
 * MotorController.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: tsasala
 */

#include "MotorController.h"

uint8_t leds[NUM_LEDS] = {0,1,2,3}; // Analog port (Port C)
uint8_t motor[NUM_MOTORS][3] = { {2, 3, 9}, {4, 5, 10} }; // Out 1, Out 2, Enable
uint8_t inputs[NUM_INPUTS] = {8,11,12}; // Input Pins

/**
 * Constructor
 */
MotorController::MotorController()
{
	// nothing to do
}


/**
 * Initializes the hardware
 *
 */
void MotorController::initialize()
{

	uint8_t i, j;

	// Set LED
	digitalWrite(LED_PIN, LOW);


	// Setup inputs
	for(i=0; i<NUM_INPUTS; i++)
	{
		pinMode( inputs[i], INPUT ); // set to input
		digitalWrite(inputs[i], HIGH); // set internal pull up
	}

	// Setup motors
	for(i = 0; i < NUM_MOTORS; i++)
	{
		for(j = 0; j< 3; j++ )
		{
			pinMode(motor[i][j], OUTPUT);
			digitalWrite(motor[i][j], LOW);
		}
	}

	// Configure LED port
	DDRC = DDRC | 0x3F; // Set PC0..5 to output
	PORTC = PORTC & 0xC0; // Set port to low


	// I don't know if this is needed, but since we
	// initialized the pins as digital above, I
	// figure it won't hurt
	analogWrite(motor[0][2], 0);
	analogWrite(motor[1][2], 0);

	// Initialize variables
	led = LOW;

#ifdef __DEBUG
	Serial.println("INITIALIZED");
#endif

} // end initialize

/**
 * Steps the output from initial setting n times with specified delay
 * between, where n = repeatValue.  Value set to endValue upon completion
 */
void MotorController::step(Motor motorIndex, Direction direction, uint8_t initialSpeed, uint8_t endSpeed, int8_t speedStep, uint16_t delayValue, uint16_t repeatValue)
{

	uint16_t i = 0;

	int8_t speed = initialSpeed;

	for(i=0; i<repeatValue; i++)
	{
		if(direction == Forward)
		{
			forward( motorIndex, speed );

		} else if( direction == Backward)
		{
			backward( motorIndex, speed);

		}
		delay( delayValue );
		speed += speedStep;

	}

	if(direction == Forward)
	{
		forward( motorIndex, endSpeed );

	} else if( direction == Backward)
	{
		backward( motorIndex, endSpeed );
	}


}

/**
 * Moves specified motor in specified direction at specified speed for specified duration.
 */
void MotorController::moveTimed(Motor motor, Direction direction, uint8_t speed, uint16_t duration)
{
	move( motor, direction, speed );
	delay( duration);
	stop( motor );
}

/**
 * Sets output in specific direction and speed.
 *
 */
void MotorController::move( Motor motorIndex, Direction direction, uint8_t speed)
{
	if( direction == Forward )
	{
		digitalWrite(motor[motorIndex][0], HIGH);
		digitalWrite(motor[motorIndex][1], LOW);
		analogWrite( motor[motorIndex][2], speed );
	}
	else if( direction == Backward )
	{
		digitalWrite(motor[motorIndex][0], LOW);
		digitalWrite(motor[motorIndex][1], HIGH);
		analogWrite( motor[motorIndex][2], speed );
	}
}

/**
 * Sets motor speed
 */
void MotorController::setSpeed(Motor motorIndex, uint8_t speed)
{
	analogWrite( motor[motorIndex][2], speed );
}

/**
 * Moves motor forward
 */
void MotorController::forward(Motor motorIndex, uint8_t speed)
{
	digitalWrite(motor[motorIndex][0], HIGH);
	digitalWrite(motor[motorIndex][1], LOW);
	analogWrite( motor[motorIndex][2], speed );
}

/**
 * Moves motor backwards
 */
void MotorController::backward(Motor motorIndex, uint8_t speed)
{
	digitalWrite(motor[motorIndex][0], LOW);
	digitalWrite(motor[motorIndex][1], HIGH);
	analogWrite( motor[motorIndex][2], speed );
}

/**
 * Stops motor
 */
void MotorController::stop(Motor motorIndex)
{
	digitalWrite(motor[motorIndex][0], LOW);
	digitalWrite(motor[motorIndex][1], LOW);
	analogWrite( motor[motorIndex][2], 0 );
}

/**
 * Sets specified LED to the specified value.
 *
 * false = off; true = on.
 *
 */
void MotorController::setLed(Led led, boolean v)
{
	uint8_t value = PORTC;

	switch(led)
	{
	case Led1:
		if(v == false )
		{
			value = value & 0xFE;
		}
		else
		{
			value = value | 0x01;
		}
		break;
	case Led2:
		if(v == false )
		{
			value = value & 0xFD;
		}
		else
		{
			value = value | 0x02;
		}
		break;
	case Led3:

		if(v == OFF )
		{
			value = value & 0xFB;
		}
		else
		{
			value = value | 0x04;
		}
		break;
	case Led4:
		if(v == OFF )
		{
			value = value & 0xF7;
		}
		else
		{
			value = value | 0x08;
		}
		break;
	default:
		break;
	}

	PORTC = value;

}
