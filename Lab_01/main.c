/* Unuversidaddel Valle de Guatemala
 * Electronica Digital 2
 * Jose Javier Viñals Veliz Carnet 22619
 *Laboratorio_01.c
 *
 * Created: 23/01/2025 16:58:12
 * Author : Javier
 */
//****************************************************************
//Librerias
//**************************************************************** 
#define F_CPU 16000000
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "display7/display7.h"
//****************************************************************
//Prototipo de funcion
//****************************************************************
void initPorts(void);
void display7(uint8_t numero);

//****************************************************************
//Variables Globales
//****************************************************************
int J1 = 0;
int J2 = 0;
int control = 0;

//****************************************************************
//Codigo Principal
//****************************************************************
int main(void)
{
    /* Replace with your application code */
	UCSR0B = 0;
	initPorts();
	displayInit();
	sei();
	displayoff();
	
    while (1) {
		while (control == 1){
		if (J1==0){
			PORTC &= ~(1<<PORTC2);
			PORTC &= ~(1<<PORTC3);
			PORTC &= ~(1<<PORTC4);
			PORTC &= ~(1<<PORTC5);
		}else if (J1==1){
			PORTC |= (1<<PORTC2);
			PORTC &= ~(1<<PORTC3);
			PORTC &= ~(1<<PORTC4);
			PORTC &= ~(1<<PORTC5);
		}else if (J1==2){
			PORTC |= (1<<PORTC2);
			PORTC |= (1<<PORTC3);
			PORTC &= ~(1<<PORTC4);
			PORTC &= ~(1<<PORTC5);
		}else if (J1==3){	
			PORTC |= (1<<PORTC2);
			PORTC |= (1<<PORTC3);
			PORTC |= (1<<PORTC4);
			PORTC &= ~(1<<PORTC5);
		}else if (J1==4){
			PORTC |= (1<<PORTC2);
			PORTC |= (1<<PORTC3);
			PORTC |= (1<<PORTC4);
			PORTC |= (1<<PORTC5);}
		
		if (J2==0){
			PORTD &= ~(1<<PORTD2);
			PORTD &= ~(1<<PORTD3);
			PORTD &= ~(1<<PORTD4);
			PORTD &= ~(1<<PORTD5);
		}else if (J2==1){
			PORTD &= ~(1<<PORTD2);
			PORTD &= ~(1<<PORTD3);
			PORTD &= ~(1<<PORTD4);
			PORTD |= (1<<PORTD5);
		}else if (J2==2){
			PORTD &= ~(1<<PORTD2);
			PORTD &= ~(1<<PORTD3);
			PORTD |= (1<<PORTD4);
			PORTD |= (1<<PORTD5);
		}else if (J2==3){
			PORTD &= ~(1<<PORTD2);
			PORTD |= (1<<PORTD3);
			PORTD |= (1<<PORTD4);
			PORTD |= (1<<PORTD5);
		}else if (J2==4){
			PORTD |= (1<<PORTD2);
			PORTD |= (1<<PORTD3);
			PORTD |= (1<<PORTD4);
			PORTD |= (1<<PORTD5);}
		}
		}
		}


//****************************************************************
//Sub-Rutinas
//****************************************************************
void initPorts(void){
	//Salidas leds
	//Definir como PC5 salida
	DDRC |= (1<<PORTC5);
	PORTC &= ~(1<<PORTC5); //Colocarlo en 0
	//Definir como PC4 salida
	DDRC |= (1<<PORTC4);
	PORTC &= ~(1<<PORTC4);//Colocarlo en 0
	//Definir como PC3 salida
	DDRC |= (1<<PORTC3);
	PORTC &= ~(1<<PORTC3);//Colocarlo en 0
	//Definir como PC2 salida
	DDRC |= (1<<PORTC2);
	PORTC &= ~(1<<PORTC2);//Colocarlo en 0
	
	//Definir como PD2 salida
	DDRD |= (1<<PORTD2);
	PORTD &= ~(1<<PORTD2);//Colocarlo en 0
	//Definir como PD3 salida
	DDRD |= (1<<PORTD3);
	PORTD &= ~(1<<PORTD3);//Colocarlo en 0
	//Definir como PD4 salida
	DDRD |= (1<<PORTD4);
	PORTD &= ~(1<<PORTD4);//Colocarlo en 0
	//Definir como PD5 salida
	DDRD |= (1<<PORTD5);
	PORTD &= ~(1<<PORTD5);//Colocarlo en 0
	
	//Entradas pulsadores
	//Entrada pulsador J1
	DDRC &= ~(1<<PORTC1);
	PORTC |=(1<<PORTC1); //pullup
	//Entrada pulsador J2
	DDRC &= ~(1<<PORTC0);
	PORTC |=(1<<PORTC0); //pullup
	//Entrada iniciar carrera
	DDRD &= ~(1<<PORTD0);
	PORTD |=(1<<PORTD0); //pullup
	
	//habilitar mask
	PCICR |= (1<<PCIE2); //boton carrera
	PCMSK2 |= (1<<PCINT16);
	
	PCICR |= (1<<PCIE1); //boton J1 C0
	PCMSK1 |= (1<<PCINT9);
	
	PCICR |= (1<<PCIE1); //boton J2 C1
	PCMSK1 |= (1<<PCINT8);
	
	
}


//****************************************************************
//ISR
//****************************************************************
ISR(PCINT2_vect){
	if (PIND & (1<<DDD0)){
		PORTC &= ~(1<<PORTC2);
		PORTC &= ~(1<<PORTC3);
		PORTC &= ~(1<<PORTC4);
		PORTC &= ~(1<<PORTC5);
		J1 = 0;
		PORTD &= ~(1<<PORTD2);
		PORTD &= ~(1<<PORTD3);
		PORTD &= ~(1<<PORTD4);
		PORTD &= ~(1<<PORTD5);
		J2 = 0;	
				displayNum(5);
				_delay_ms(1000);
				displayNum(4);
				_delay_ms(1000);
				displayNum(3);
				_delay_ms(1000);
				displayNum(2);
				_delay_ms(1000);
				displayNum(1);
				_delay_ms(1000);
				displayNum(0);
				_delay_ms(1000);
				displayoff();
				
				control = 1;
				
				}
	}


ISR(PCINT1_vect){
	if (control == 1){
		_delay_ms(50);
	if (J1<=3 && J2<=3){
		if (PINC & (1<<PINC0)){
			J1++;
			if(J1==5){
				J1=4;
			}
		}
		if(PINC & (1<<DDC1)){
			J2++;
			if(J2==5){
				J2=4;
			}
		}
		_delay_ms(50);
	}
	}
	}
	