/*
 * MotorController.h
 *
 *  Created on: Aug 20, 2013
 *      Author: tsasala
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "Arduino.h"

#include "Motor.h"

/**
 * Hardware Description - Version 1.0
 *
 * Motor 1A - D2
 * Motor 1B - D3
 * Motor 2A - D4
 * Motor 2B - D5
 * Motor 1 Enable - D9
 * Motor 2 Enable - D10
 *
 * Digital Input 1 - D8
 * Digital Input 2 - D11
 * Digital Input 3 - D12
 * Digital Input 4 - D13 (does not work with internal LED)
 *
 * LEDs Active High
 * LED1 = A0
 * LED2 = A1
 * LED3 = A2
 * LED4 = A3
 *
 */

#define LED_PIN		13

#define NUM_MOTORS	2
#define NUM_INPUTS	3
#define NUM_LEDS	4

extern uint8_t leds[NUM_LEDS];
extern uint8_t inputs[NUM_INPUTS];

#define NUM_LEDS	4
#define ON 			true
#define OFF			false
#define DONT_CARE	2

enum MotorEnum { Motor1=0, Motor2=1 };
enum LedEnum {Led1=0, Led2=1, Led3=2, Led4=3 };

class MotorController {

public:
	MotorController();

	/**
	 * Function stereotypes
	 */
	void initialize();

	Motor getMotor(MotorEnum index);

	void setLed(LedEnum led, boolean value);
	boolean getLed(LedEnum led);
	void toggleLed(LedEnum led);


private:
	Motor motors[NUM_MOTORS];

};


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

#endif /* MOTORCONTROLLER_H_ */
