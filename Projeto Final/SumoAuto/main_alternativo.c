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
#define lTE 3 //sensor de linha traseiro esquerdo]
#define eixoX 0
#define eixoY 1
#define eixoZ 2

// Tweak constants for maximum performance
#define thresFron 256 // threshold frontal
#define thresLat 400 // threshold lateral
#define thresLine 800 // threshold dos sensores de linha
#define attack_speed 255
#define turning_speed 200
#define kp 0.2

void filter_motors(int left, int right);
int mod( int x); /*'mod' de modulo*/


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
	int iteracoes = 5; 
	int giroscopioZ;
	int anguloCurva;

	while (1) {
		// read sensor values
		update_distance();
		update_line();

		while ( line[lFD] < thresLine && line[lFE] < thresLine &&  /*enquanto nao estiver pisando em linhas*/
		        line[lTD] < thresLine && line[lTE] < thresLine ){
			
			if ( accel[2] < 0 &&  iteracoes <5 ) { /*caso esteja tentando ir pra frente e indo pra tras*/

				
				/* da ré girando no sentido sorteado  */
				filterMotors(-attack_speed -sent*128, -attack_speed + sent*128);/*esse valor 128 eh arbitario*/

				/*a ideia eh dar ré por 50 ms mas se colocarmos aqui um delay muito grande, o robô poderá pisar 
				  na linha e a isso nao estarah sendo verificado, portanto usaremo 10ms durante 5 iteracoes*/
				_delay_ms(10);
				iteracoes += 1;
				
				update_distance();
				update_line();
				update_acell();
			} 
			else { 
				iteracoes = 0;
				sent = get_tick() % 2;  /*sorteia um sentido: 1 direita, 0 esquerda */
				if ( sent != 1) 		
					sent = -1;         

				erro = distance[dFE] - distance [dFD];
				filterfilterMotors(attack_speed - kp*error, attack_speed + kp*error); /* acelera pra frente 
																					de forma proporcional */																
				update_distance();
				update_line();
				_delay_ms(10);
				update_acell();	
			}
		}
		while ( line[lFD] > thresLine || line[lFE] > thresLine ||  /*enquanto estiver pisando em alguma linha*/
		        line[lTD] > thresLine || line[lTE] < thresLine ) {
			
			giroscopioZ = gyro[eixoZ]; /*supondo que gyro retorna a medida em graus, de 0 a 359 ou algo assim*/
			anguloCurva = 135;

			if ( line[lFE] > thresLine && line[lFD] > thresLine){ /*se os dois frontais estiverem na linha*/
				sent = get_tick() % 2;   /*sorteia um sentido: 1 direita(Horário), 0 esquerda (Anti-Horário)*/
				if ( sent != 1) 
					sent = -1;
				anguloCurva = 180;
			}
			else if ( line[lFE] > thresLine ) /*se apenas o frontal esquerdo estiver na linha*/
				sent = 1;
			else if ( line[lFD] > thresLine ) /*se apenas o frontal direito estiver na linha*/
				sent = -1;
				
			while ( mod (gyro[eixoZ] - giroscopioZ) < anguloCurva && /*enquanto nao tiver feito a curva*/
			distance[dFE] < thresF || distance[dFC] < thresF || distance[dFD] < thresF){  /*e nao avistar o adversario*/
					
				VE = sent*turning_speed; VD = -sent*turning_speed; 
				motors(VE, VD); 
				update_distance();
				update_line();
				update_acell();
				_delay_ms(10);
			} 
			erro = distance[dFE] - distance [dFD];
			filterfilterMotors(attack_speed - kp*error, attack_speed + kp*error);	
			update_distance();
			update_line();
			update_acell();
			_delay_ms(10);
			iteracoes = 5;
		}

	return 0;
}

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

void mod( int x){
	if ( x < 0)
		x = -x;
}
