/*
 * LCD.c
 *
 * Created: 29/01/2025 21:06:16
 *  Author: Javier
 */ 
#include "LCD.h"
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void initLCD8bits(void) {
	// Configurar pines de datos y control como salida
	DDRD |= 0xFF;  // Pines de datos D0-D7 como salida
	DDRB |= (1 << DDB0) | (1 << DDB1);  // RS y E como salida
	PORTD = 0x00;
	PORTB &= ~((1 << DDB0) | (1 << DDB1));

	_delay_ms(20);  // Espera inicial mínima (20 ms)

	// Secuencia de inicialización en modo 8 bits
	LCD_CMD(0x30);  // Función inicial en modo 8 bits
	_delay_ms(5);
	LCD_CMD(0x30);  // Repetir comando
	_delay_us(150);
	LCD_CMD(0x30);
	_delay_ms(1);
	LCD_CMD(0x38);  // Función: 8 bits, 2 líneas, 5x8
	_delay_ms(1);
	LCD_CMD(0x0C);  // Display ON, sin cursor ni parpadeo
	_delay_ms(1);
	LCD_CMD(0x01);  // Limpiar pantalla
	_delay_ms(2);
	LCD_CMD(0x06);  // Incremento automático sin desplazamiento
	_delay_ms(1);
}

void LCD_CMD(char cmd) {
	PORTB &= ~(1 << PORTB0);  // RS = 0 (modo comando)
	PORTD = cmd;              // Coloca el comando en PORTD
	PORTB |= (1 << PORTB1);   // EN = 1
	_delay_us(200);           // Asegurar tiempo de estabilidad
	PORTB &= ~(1 << PORTB1);  // EN = 0
	_delay_ms(2);             // Esperar el procesamiento del comando
}

void LCD_Write_Char(char a) {
	PORTB |= (1 << PORTB0);   // RS = 1 (modo datos)
	PORTD = a;             // Coloca el dato en PORTD
	PORTB |= (1 << PORTB1);   // EN = 1
	_delay_us(200);           // Retardo para asegurar estabilidad
	PORTB &= ~(1 << PORTB1);  // EN = 0
	_delay_ms(2);
}

void LCD_Write_String(const char *str) {
	while (*str) {
		LCD_Write_Char(*str++);
	}
}

void LCD_Set_Cursor(char c, char f) {
	char temp;
	if (f == 1) {
		temp = 0x80 + (c - 1);  // Primera línea
		} else if (f == 2) {
		temp = 0xC0 + (c - 1);  // Segunda línea
		} else {
		return;  // Evitar posiciones inválidas
	}
	LCD_CMD(temp);
}
