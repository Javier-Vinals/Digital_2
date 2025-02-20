/*
 * Slave.c
 *
 * Created: 13/02/2025 06:26:52
 * Author : Javier
 */ 

//Slave Counter
#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "I2C/I2C.h"

#define SlaveAddress 0x30

uint8_t buffer = 0;
volatile uint8_t valorADC = 0;

void initPorts(void);
void refreshPORT(uint8_t valor);
void initADC(void);

int main(void)
{
    DDRB |= (1<<DDB5);
	PORTB &= ~(1<<PORTB5);
	initPorts();
	initADC();
	
	I2C_Slave_Init(SlaveAddress);
	
	sei();
    while (1) 
    {
		//iniciando secuancia adc
		ADCSRA |= (1<<ADSC);
		_delay_ms(100);
		refreshPORT(valorADC);
		if (buffer == 'R'){
			PINB |= (1<<PINB5);
			buffer = 0;
		}
    }
}

ISR(TWI_vect){
	uint8_t estado;
	estado = TWSR & 0xFC;
	switch(estado){
		case 0x60:
		case 0x70:
			TWCR |= (1<<TWINT);
			break;
		case 0x80:
		case 0x90:
			buffer = TWDR;
			TWCR |= (1<<TWINT); //limpia la bandera
			break;
		case 0xA8:
		case 0xB8:
			TWDR = valorADC; //cargar el dato
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA); //inicia el envio
			break;
		default: // se libera el bus de cualqlier error
			TWCR |= (1<<TWINT)|(1<<TWSTO);
			break;
			
			
	}
}


