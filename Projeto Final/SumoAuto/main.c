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
 * 1. Enquanto não estiver sobre linhas
 *		1.1. Se não estiver no meio de uma esquiva (perdeu colisão)
 *		 	1.1.1. Se estiver vendo o adversário com os sensores da frente, avança com controle proporcional
 *			1.1.2. Se não estiver vendo o adv, vai para a frente com a msm velocidade nos dois motores
 *			1.1.3. Verifica se houve colisão e atualiza perdeu_col
 *		1.2. Se perdeu colisão, faz as iterações da esquiva
 *			1.2.1. Se está na primeira iteração, define o sentido
 *			1.2.2. Se está na última iteração, zera iteracoes e muda perdeu_col pra falso
 *			1.2.3. Faz a curva de ré para esquivar
 * 2. Se pisou na linha (oq deve ser verdadeiro, pq saiu do while anterior)
 *		2.1. Define o sentido de rotação
 *		2.2. Se estiver com os dois sensores frontais sobre a linha, gira 180 graus ou até encontrar o adversário (sentido aleatório)
 *		2.3. Se estiver com apenas um sensor frontal sobre a linha, gira 135 graus ou até encontrar o adversário (pro lado q não está sobre a linha)
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


#define TRUE  1
#define FALSE 0

#define mod(x) ((x) < 0 ? -(x) : (x))

// Tweak constants for maximum performance
#define thresFron 256 // threshold frontal
#define thresLat  256 // threshold lateral
#define thresLine 800 // threshold dos sensores de linha
#define thresAccel 5;
#define attack_speed 255
#define turning_speed 200
#define kp 0.2
#define tempo_para_girar_180o 180
#define tempo_para_girar_135o 135

void filter_motors(int left, int right);
int perdeu_colisao();
void update_all();

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
	int iteracoes = 0; 
	int perdeu_col = FALSE ;
	int deltaT;
	int limiteT;

	while (1) {
		update_all(); // read all sensor values
		
		//não está pisando na linha
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		while ( line[lFD] < thresLine && line[lFE] < thresLine && line[lTD] < thresLine && line[lTE] < thresLine ){
			update_all(); // read all sensor values
			if ( perdeu_col == FALSE){ /*caso nao tenha perdido uma colisao*/
				if(distance[dFE] > thresF || distance[dFD] > thresF ){
					erro = distance[dFE] - distance [dFD];								/* acelera pra frente */
					filter_motors(attack_speed - kp*error, attack_speed + kp*error);   /*de forma proporcional */
				} else
					filter_motors(attack_speed, attack_speed);   /*de forma proporcional */
				_delay_ms(10);
				perdeu_col = perdeu_colisao(); /*verifica se perdeu uma colisao*/
			}
			if ( perdeu_col == TRUE )   /* caso tenha perdido*/
			{
				if (iteracoes == 0){ //define o sentido
					if ( distance[dLE] > thresLat || distance[dLD] > thresLat ){ 
						if ( distance[dLE] > distance[dLD]) sent = -1;
						else sent = 1;
					}
					else{
						sent = get_tick() % 2;  /*sorteia um sentido para dar ré: 1 direita, 0 esquerda */
						if ( sent != 1) sent = -1;
					}
				}
				/* da ré girando no sentido sorteado (o sorteio de sentido está no 'if' deste 'else') */
				filter_Motors(-attack_speed - sent*128, -attack_speed + sent*128); /*esse valor 128 eh arbitrario*/
				/*a ideia eh dar ré por 50 ms mas se colocarmos aqui um delay muito grande, o robô poderá pisar 
				  na linha e a isso nao estarah sendo verificado, portanto usaremo 10ms durante 5 iteracoes*/
				_delay_ms(10);
				iteracoes += 1;
				if (iteracoes == 5){
					perdeu_col = FALSE;
					iteracoes = 0;
				}
			}  
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		perdeu_col = FALSE; //ele só vai sair do while anterior se pisar numa linha; 
		iteracoes = 0; //nesse caso, msm q esteja num mov de esquiva de colisao, priorizamos o desvio da linha, setando perdeu_col = falso
		
		//pisou na linha
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		limiteT = tempo_para_girar_135o; /*tempo necessario para girar 135 graus( ainda nao sabemos quanto eh) */

		if ( line[lFE] > thresLine && line[lFD] > thresLine){ /*se os dois frontais estiverem na linha*/
			if ( distance[dLE] > thresLat || distance[dLD] > thresLat ){   
					if ( distance[dLE] > distance[dLD]) sent = -1;
					else sent = 1;
				}
				else{
					sent = get_tick() % 2;/*sorteia um sentido: 1 direita(Horário), 0 esquerda (Anti-Horário)*/
					if (sent != 1) sent = -1;
					limiteT = tempo_para_girar_180o; /*muda o limite de tempo para o tempo necessario para girar 180 graus*/
				}
		}
		else if ( line[lFE] > thresLine ) /*se apenas o frontal esquerdo estiver na linha*/
			sent = 1;
		else if ( line[lFD] > thresLine ) /*se apenas o frontal direito estiver na linha*/
			sent = -1;
			
		deltaT = 0;
		while ( (distance[dFE] < thresF && distance[dFC] < thresF && distance[dFD] < thresF) && deltaT < limiteT ){  
			//enquanto nao avistar o adversario e ainda estiver no movimento de giro
			VE = sent*turning_speed; VD = -sent*turning_speed; /*gira no sentido definido*/
			motors(VE, VD); 
			update_all();
			_delay_ms(10);
			deltaT += 10; /*contabiliza a quanto tempo está girando*/
		} 
		/*quando acabar de girar ou avistar o adversario, acelera proporcionalmente*/
		erro = distance[dFE] - distance [dFD]; 
		filter_motors(attack_speed - kp*error, attack_speed + kp*error);	
		update_distance();
		update_line();
		update_accel();
		_delay_ms(10);
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

int perdeu_colisao(){
	int accel;
	int new_accel;
	int perdeu = FALSE;

	atualiza_accel();
	
	if ( accel[eixoX] <  -thresAccel){ //levou impacto frontal
		accel = accel[eixoX];
		_delay_ms(50);
		atualiza_accel();
		new_accel = accel[eixoX] //acel um pouco dps do impacto (negativa se está sendo empurrado pra trás)
	}
	if (new_accel*accel > 0) perdeu = TRUE; //neg*neg=pos

	return perdeu;
}

void update_all(){
	update_distance();
	update_line();
	update_accel();
}