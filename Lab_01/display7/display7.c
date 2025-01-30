/*
 * display7.c
 *
 * Created: 23/01/2025 19:05:10
 *  Author: Javier
 */ 
#include "display7.h"
#include <avr/io.h>
#include <stdint.h>

void displayInit(void){
	//Salida Display 7Seg
	//Definir como PD6 salida
	DDRD |= (1<<PORTD6); //c
	PORTD &= ~(1<<PORTD6);//Colocarlo en 0
	//Definir como PD7 salida
	DDRD |= (1<<PORTD7); //d
	PORTD &= ~(1<<PORTD7);//Colocarlo en 0
	//Definir como PB0 salida
	DDRB |= (1<<PORTB0); //e
	PORTB &= ~(1<<PORTB0);//Colocarlo en 0
	//Definir como PB1 salida
	DDRB |= (1<<PORTB1); //b
	PORTB &= ~(1<<PORTB1);//Colocarlo en 0
	//Definir como PB2 salida
	DDRB |= (1<<PORTB2); //a
	PORTB &= ~(1<<PORTB2);//Colocarlo en 0
	//Definir como PB3 salida
	DDRB |= (1<<PORTB3); //f
	PORTB &= ~(1<<PORTB3);//Colocarlo en 0
	//Definir como PB4 salida
	DDRB |= (1<<PORTB4); //g
	PORTB &= ~(1<<PORTB4);//Colocarlo en 0
}

void displayNum(uint8_t numero){
	switch(numero){
		case 0: 
		PORTB |= (1<<PORTB2);
		PORTB |= (1<<PORTB1);
		PORTD |= (1<<PORTD6);
		PORTD |= (1<<PORTD7);
		PORTB |= (1<<PORTB0);
		PORTB |= (1<<PORTB3);
		PORTB &= ~(1<<PORTB4);
		break;
		case 1:
		PORTB &= ~(1<<PORTB2);
		PORTB |= (1<<PORTB1);
		PORTD |= (1<<PORTD6);
		PORTD &= ~(1<<PORTD7); 
		PORTB &= ~(1<<PORTB0);
		PORTB &= ~(1<<PORTB3);
		PORTB &= ~(1<<PORTB4);
		break;
		case 2:
		PORTB |= (1<<PORTB2); //a
		PORTB |= (1<<PORTB1); //b
		PORTD &= ~(1<<PORTD6); //c
		PORTD |= (1<<PORTD7); //d
		PORTB |= (1<<PORTB0); //e
		PORTB &= ~(1<<PORTB3); //f
		PORTB |= (1<<PORTB4); //g
		break;
		case 3:
		PORTB |= (1<<PORTB2); //a
		PORTB |= (1<<PORTB1); //b
		PORTD |= (1<<PORTD6); //c
		PORTD |= (1<<PORTD7); //d
		PORTB &= ~(1<<PORTB0); //e
		PORTB &= ~(1<<PORTB3); //f
		PORTB |= (1<<PORTB4); //g
		break;
		case 4:
		PORTB &= ~(1<<PORTB2); //a
		PORTB |= (1<<PORTB1); //b
		PORTD |= (1<<PORTD6); //c
		PORTD &= ~(1<<PORTD7); //d
		PORTB &= ~(1<<PORTB0); //e
		PORTB |= (1<<PORTB3); //f
		PORTB |= (1<<PORTB4); //g
		break;
		case 5:
		PORTB |= (1<<PORTB2); //a
		PORTB &= ~(1<<PORTB1); //b
		PORTD |= (1<<PORTD6); //c
		PORTD |= (1<<PORTD7); //d
		PORTB &= ~(1<<PORTB0); //e
		PORTB |= (1<<PORTB3); //f
		PORTB |= (1<<PORTB4); //g
		break;
	}
}

void displayoff(void){
	PORTB &= ~(1<<PORTB2);
	PORTB &= ~(1<<PORTB1);
	PORTD &= ~(1<<PORTD6);
	PORTD &= ~(1<<PORTD7);
	PORTB &= ~(1<<PORTB0);
	PORTB &= ~(1<<PORTB3);
	PORTB &= ~(1<<PORTB4);
}