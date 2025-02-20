/*
 * MASTER.c
 *
 * Created: 13/02/2025 06:13:38
 * Author : Javier
 */ 
#define  F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include "I2C/I2C.h"

#define slave1 0x30
#define  slave2 0x40

uint8_t direccion;
uint8_t temp;
uint8_t bufferI2C;
uint8_t valorI2C = 0;

void initPorts(void);
void refreshPORT(uint8_t valor);


int main(void)
{
	I2C_Master_Init(100000, 1);
    initPorts();
    while (1) 
    {
		PORTB |=(1<<PORTB5);
		
		I2C_Master_Start();
		//ESCRITURA
		bufferI2C = slave1 << 1 & 0b11111110;
		
		temp = I2C_Master_Write(bufferI2C);
		if (temp !=1){
			I2C_Master_Stop();
		}
		TWCR = (1<<TWINT)|(1<<TWEN);
		while (!(TWCR&(1<<TWINT)));
		
		valorI2C = TWDR;
		
		I2C_Master_Stop();
		
	//	refreshPORT(valorI2C);
		
		_delay_ms(500);
		
		
    }
}

//void initPorts(void){}
