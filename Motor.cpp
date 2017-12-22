/*
 * MotorWire.cpp
 *
 *  Created on: Dec 22, 2017
 *      Author: tsasala
 */

#include "Motor.h"

/**
 * Constructor
 */
Motor::Motor()
{
	this->dirPinA = 0;
	this->dirPinB = 0;
	this->speedPin = 0;
}

/**
 * Initialize motor
 */
void Motor::initialize(uint8_t a, uint8_t b, uint8_t s)
{
	this->dirPinA = a;
	this->dirPinB = b;
	this->speedPin = s;

	pinMode(a, OUTPUT); // set to output
	pinMode(b, OUTPUT); // set to output
	pinMode(s, OUTPUT); // set to output

	digitalWrite(a, LOW);
	digitalWrite(b, LOW);
	analogWrite(s, 0);

}

void Motor::forward(uint8_t speed)
{
	move(Backward, speed);

}

void Motor::backward(uint8_t speed)
{
	move(Backward, speed);

}

void Motor::stop()
{
	setSpeed(0);
}

void Motor::step(Direction direction, uint8_t initialSpeed, uint8_t endSpeed, int8_t speedStep, uint16_t delayValue, uint16_t repeatValue)
{

	uint16_t i = 0;

	int8_t speed = initialSpeed;

	for(i=0; i<repeatValue; i++)
	{
		if(direction == Forward)
		{
			move( Forward, speed );

		} else if( direction == Backward)
		{
			move( Backward, speed);

		}
		delay( delayValue );
		speed += speedStep;

	}

	if(direction == Forward)
	{
		move( Forward, endSpeed );

	} else if( direction == Backward)
	{
		move( Backward, endSpeed );
	}


}


void Motor::moveTimed(Direction direction, uint8_t speed, uint16_t duration)
{

	move( direction, speed );
	delay( duration);
	stop( );

}

void Motor::setSpeed(uint8_t speed)
{
	analogWrite( speedPin, speed );
}


void Motor::move(Direction direction, uint8_t speed)
{

	if( direction == Forward )
	{
		digitalWrite(dirPinA, HIGH);
		digitalWrite(dirPinB, LOW);
	}
	else if( direction == Backward )
	{
		digitalWrite(dirPinA, LOW);
		digitalWrite(dirPinB, HIGH);
	}
	else if( direction == Stop)
	{
		digitalWrite(dirPinA, LOW);
		digitalWrite(dirPinB, LOW);
		speed = 0;
	}

	analogWrite( speedPin, speed );
}

