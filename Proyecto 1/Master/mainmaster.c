// Master Code
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include "lcd/lcd.h"

#define SCL_CLOCK 100000L
#define SLAVE_ADDRESS_HX711 0x10

void I2C_Init(void) {
	TWSR = 0x00;
	TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
	TWCR = (1 << TWEN);
}

void I2C_Start(void) {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop(void) {
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void I2C_Write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_Read_ACK(void) {
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t I2C_Read_NACK(void) {
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

void I2C_Request_Data(long *weight, uint8_t *status, uint8_t *distance) {
	I2C_Start();
	I2C_Write((SLAVE_ADDRESS_HX711 << 1) | 1);

	uint8_t bytes[6];
	for (uint8_t i = 0; i < 5; i++) {
		bytes[i] = I2C_Read_ACK();
	}
	bytes[5] = I2C_Read_NACK();
	I2C_Stop();

	*weight = ((long)bytes[0] << 24) | ((long)bytes[1] << 16) | ((long)bytes[2] << 8) | (long)bytes[3];
	*status = bytes[4];
	*distance = bytes[5];
}

void uart_init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void uart_print(const char *str) {
	while (*str) {
		uart_transmit(*str++);
	}
}

int main(void) {
	char buffer[16];

	initLCD8bits();
	I2C_Init();

	LCD_Set_Cursor(1, 1);
	LCD_Write_String("Peso:");

	while (1) {
		long weight;
		uint8_t status, distance;
		I2C_Request_Data(&weight, &status, &distance);

		// Mostrar peso en pantalla
		LCD_Set_Cursor(1, 2);
		if (weight > 0) {
			snprintf(buffer, sizeof(buffer), "%ld", weight);
			LCD_Write_String(buffer);

			snprintf(buffer, sizeof(buffer), "Peso: %ld\r\n", weight);
			uart_print(buffer);
			} else {
			LCD_Write_String("Error");
			uart_print("Error de peso\n");
		}

		// Mostrar distancia en la línea superior
		LCD_Set_Cursor(9, 1);
		if (distance == 255) {
			LCD_Write_String("Midiendo...");
			} else {
			snprintf(buffer, sizeof(buffer), "%dcm         ", distance);
			LCD_Write_String(buffer);
		}

		// Mostrar estado en la línea inferior
		LCD_Set_Cursor(9, 2);
		if (status == 1) {
			LCD_Write_String("ROJO    ");
			} else if (status == 2) {
			LCD_Write_String("AMARILLO");
			} else {
			LCD_Write_String("N/A      ");
		}

		_delay_ms(500);
	}
}
