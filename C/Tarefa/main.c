/**
 * Processo Seletivo 2017 - main.c
 *
 * Equipe ThundeRatz de Robótica
 * 03/2017
 */

/*
 * Vitor Sternlicht 
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "sensors.h"
#include "motors.h"

// Tweak constants for maximum performance
#define thresF 500
#define thresL 400
#define attack_speed 255
#define turning_speed 200


int main() {
	sensors_init();
	motors_init();
	sei();
	wdt_reset();
	wdt_disable();

	uint16_t FE = 0, FD = 0, LE = 0, LD = 0;	// these variables will store the values that the sensors read
	int8_t dir = 1; 							// this is the turning direction

	while (true) {
		
		// read sensor values
		FE = get_sensor(SENSOR_FE), FD = get_sensor(SENSOR_FD), LE = get_sensor(SENSOR_LE), LD = get_sensor(SENSOR_LD);
		
		if (FE < thresF && FD < thresF) {					// if both front sensors see something
			motors(attack_speed, attack_speed);				// move forward with attack speed
		} else {											// if they don't, then turn
			if (FE < thresL || LE < thresL) dir = 1;		// right, if the last sensor to see something is in the left 
			else if (FD < thresL || LD < thresL) dir = -1;	// left, if the last sensor to see something is in the right
			motors(dir*turning_speed, -dir*turning_speed);	// (here's the actual turning)
		}

		_delay_ms(10);
	}

	return 0;
}
