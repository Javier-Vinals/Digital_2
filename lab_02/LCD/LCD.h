/*
 * LCD.h
 *
 * Created: 29/01/2025 20:43:51
 *  Author: Javier
 */ 


#ifndef LCDLIB_H_
#define LCDLIB_H_
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>



#define E (1<<PORTB1)  // Definir el pin E

// Funci�n para inicializar el LCD
void initLCD8bits(void);
// Funci�n para colocar en el puerto un valor
void LCD_Port(char a);
// Funci�n para enviar un comando
void LCD_CMD(char cmd);
// Funci�n para enviar un car�cter
void LCD_Wtrite_Char(char a);
// Funci�n para enviar una cadena
void LCD_Write_String(const char *str);

// Desplazamiento hacia la derecha
void LCD_Shift_Right(void);
// Desplazamiento hacia la izquierda
void LCD_Shift_Left(void);
// Establecer el cursor
void LCD_Set_Cursor(char c, char f);

#endif /* LCD_H_ */
