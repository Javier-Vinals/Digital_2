/*
 * I2C.c
 *
 * Created: 12/02/2025 17:21:39
 *  Author: Javier
 */ 
#include "I2C.h"

//********************************************************
//Funcion para inicializar I2C Maestro
//********************************************************
void I2C_Master_Init(unsigned long SCL_Clock,uint8_t Prescaler){
	DDRC &= ~((1<<DDC4)|(1<<DDC5));	//Pines de i2c como entradas
	
	switch(Prescaler){
		case 1:
			TWSR &= ~((1<<TWPS1)|(1<<TWPS0));
		break;
		case 4:
			TWSR &= ~(1<<TWPS1);
			TWSR |= (1<<TWPS0);
		break;
		case 16:
			TWSR &= ~(1<<TWPS0);
			TWSR |= (1<<TWPS1);
		break;
		case 64:
			TWSR &= ~((1<<TWPS1)|(1<<TWPS0));
			Prescaler = 1;
		break;
	}
	TWBR = ((F_CPU/SCL_Clock)-16)/(2*Prescaler); // must be > 10 for estable operation
	TWCR |= (1<<TWEN);
}

//********************************************************
//Funcion de inicio de la comunicacion I2C
//********************************************************
void I2C_Master_Start(void){
	uint8_t estado;
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //Iniciar condicion de start
	while(!(TWCR &(1<<TWINT))); //espera a que termine la flag TWINT
}

//********************************************************
//Funcion de parada de la comunicacion I2C
//********************************************************
void I2C_Master_Stop(void){
	TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO); //inicia el envio secuencia parada stop
}
//********************************************************
//Funcion de transmision de datos del maestro al esclavo
//********************************************************
uint8_t I2C_Master_Write(uint8_t dato){
	uint8_t estado;
	TWDR = dato; //cargar el dato
	TWCR = (1<<TWEN)|(1<<TWINT); //Inicia el envio
	while(!(TWCR&(1<<TWINT))); //Espera al flag TWINT
	estado = TWSR & 0xF8; //verificar estado
	//verificar si se transmitio una SLA + W con AFK, SLA + R con ACK, o un Dato con ACK
	if(estado == 0x18 || estado == 0x28 || estado == 0x40){
		return 1;
	}else{
		return estado;
	}
}

//********************************************************
//Funcion de recepcion de datos enviados´por el esclavo al maestro
//********************************************************
uint8_t I2C_Master_Read(uint8_t *buffer, uint8_t ack){
	uint8_t estado;
	if(ack){
		TWCR |= (1<<TWEA); //Lectura con ACK
	}else{
		TWCR &= ~(1<<TWEA); //Lectura sin ACK
	}
	TWCR |= (1<<TWINT); //Iniciamos la lectura
	while(!(TWCR&(1<<TWINT))); //ESPERA AL FLAG TWINT
	estado = TWSR & 0xF8; //verificar estado
	//Verificar dato leido con ACK o sin ACK
	if(estado == 0x58 || estado == 0x50){
		*buffer = TWDR;
		return 1;
	}else{
		return estado;
	}	
}
//********************************************************
//Funcion para inicializar I2C Esclavo
//********************************************************
void I2C_Slave_Init(uint8_t adress){
	DDRC &= ~((1<<DDC4)|(1<<DDC5)); //PINES DE I2C COMO ENTRADAS
	TWAR = adress <<1; //se asigna la direccion que tendra
	TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE);
}