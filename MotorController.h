/*
 * TopSpinShield.h
 *
 *  Created on: Aug 20, 2013
 *      Author: tsasala
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "Arduino.h"

// This is just general Arduino description; not program specific
//PORTD maps to Arduino digital pins 0 to 7
//    DDRD - The Port D Data Direction Register - read/write
//    PORTD - The Port D Data Register - read/write
//    PIND - The Port D Input Pins Register - read only
//
//PORTB maps to Arduino digital pins 8 to 13 The two high bits (6 & 7) map to the crystal pins and are not usable
//    DDRB - The Port B Data Direction Register - read/write
//    PORTB - The Port B Data Register - read/write
//    PINB - The Port B Input Pins Register - read only
//
//PORTC maps to Arduino analog pins 0 to 5. Pins 6 & 7 are only accessible on the Arduino Mini
//    DDRC - The Port C Data Direction Register - read/write
//    PORTC - The Port C Data Register - read/write
//    PINC - The Port C Input Pins Register - read only

#define LED_PIN		13

#define NUM_MOTORS	2
#define NUM_INPUTS	3
#define NUM_LEDS	4

extern uint8_t leds[NUM_LEDS];
extern uint8_t motor[NUM_MOTORS][3];
extern uint8_t inputs[NUM_INPUTS];

#define NUM_LEDS	4
#define ON 			true
#define OFF			false
#define DONT_CARE	2


enum Motor { Motor1=0, Motor2=1 };

enum Direction {Forward=0, Backward=1, Stop=2};

enum Led {Led1=0, Led2, Led3, Led4 };



class MotorController {

public:
	MotorController();

	/**
	 * Function stereotypes
	 */
	void initialize();

	void sample();

	void forward(Motor motorIndex, uint8_t speed);
	void backward(Motor motorIndex, uint8_t speed);
	void stop(Motor motorIndex);
	void setSpeed(Motor motorIndex, uint8_t speed);

	void step(Motor index, Direction direction, uint8_t initialSpeed, uint8_t endSpeed, int8_t speedStep, uint16_t delayValue, uint16_t repeatValue);
	void move( Motor motorIndex, Direction direction, uint8_t speed);
	void moveTimed(Motor motorIndex, Direction direction, uint8_t speed, uint16_t duration);
	void setLed(Led led, boolean value);


private:
	volatile uint8_t led;

};

#endif /* MOTORCONTROLLER_H_ */
