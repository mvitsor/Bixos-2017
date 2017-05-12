/**
 * Processo Seletivo 2017 - main.c
 *
 * Equipe ThundeRatz de Robótica
 * 05/2017
 */

/*
 * César Takose
 * Rafael Gehrke
 * Vitor Sternlicht 
 * Walter Ventura
 */

/* 
 * Estratégia ("O conceito de estratégia, em grego strateegia, em latim strategi, em francês stratégie...")
 * 
 * 1. se estiver vendo o adversário (adv) com um dos três sensores de dist frontais,
 *    vai para frente com controle proporcional
 * 2. se não estiver vendo o adv com os 3 sensores frontais, gira para o último lado (sent) onde viu o adv
 * 3. atualiza a variável sent sempre q vir o adv com algum sensor
 * 4. se encontrar linha com os sensores frontais, dar um pouco de r�
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "sensors.h"
#include "motors.h"

// Sensors
#define dLE 0 //sensor de distancia lateral esquerdo 
#define dFE 1 //sensor de distancia frontal esquerdo
#define dFC 2 //sensor de distancia frontal central
#define dFD 3 //sensor de distancia frontal direito
#define dLD 4 //sensor de distancia lateral direito
#define lFE 0 //sensor de linha frontal esquerdo 
#define lFD 1 //sensor de linha frontal direito
#define lTD 2 //sensor de linha traseiro direito
#define lTE 3 //sensor de linha traseiro esquerdo
#define eixoX 0
#define eixoY 1
#define eixoZ 2

// Tweak constants for maximum performance
#define thresF 500
#define thresL 400
#define attack_speed 255
#define turning_speed 200
#define kp 0.2

void filter_motors(int left, int right){
	if (left > 255){
		left = 255;
	}
	else if (left < -255){
		left = -255;
	}
	if (right > 255){
		right = 255;
	}
	else if (right < -255){
		right = -255;
	}
	motors(left, right);
}

int main() {
	sensors_init();
	motors_init();
	sei();
	wdt_reset();
	wdt_disable();

	uint16_t FE = 0, FD = 0, LE = 0, LD = 0;	// these variables will store the values that the sensors read
	int8_t sent = 1; 							// this is the turning direction
    uint16_t VE = 0, VD = 0;                    // these are the motor speeds
    int error;

	while (1) {
		// read sensor values
		update_distance();
		update_line();

		if (distance[dFE] > thresF || distance[dFC] > thresF || distance[dFD] > thresF) {  // if any front sensor sees something
			while(line[lFD] < thresL && line[lFE] < thresL){         //Execute proportional until the border of the dojo
				// proportional control
				update_distance();
				update_line();
				error = distance[dFE] - distance[dFD];
				filterMotors(attack_speed - kp*error, attack_speed + kp*error);
				_delay_ms(10);
			}
			motors(-255, -255);  //reached border of the dojo. Stay away from it
			_delay_ms(400);
		} else {										// if they don't, then set turning direction as
			if (distance[dLE] < thresL) sent = -1;		// left, if the last sensor to see something is in the left 
			else if (distance[dLD] < thresL) sent = 1;	// right, if the last sensor to see something is in the right

			VE = sent*turning_speed; VD = -sent*turning_speed; //overwrite motor speeds
		}
		motors(VE, VD); //actually move the motors
		_delay_ms(10);
	}
	return 0;
}
