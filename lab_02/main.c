/* Unuversidaddel Valle de Guatemala
 * Electronica Digital 2
 * Jose Javier Viñals Veliz Carnet 22619
 *Laboratorio_2.c
 *
 * Created: 29/01/2025 20:43:02
 * Author : Javier
 */
//****************************************************************
//Librerias
//**************************************************************** 
#define F_CPU 16000000
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include "LCD/LCD.h"
//#include "adc/adc.h"
//****************************************************************
//Prototipo de funcion
//****************************************************************

//****************************************************************
//Variables Globales
//****************************************************************
uint16_t adc_value1 = 0; // Almacena el valor del primer potenciómetro
uint16_t adc_value2 = 0; // Almacena el valor del segundo potenciómetro
//****************************************************************
//Codigo Principal
//****************************************************************
int main(void) {
	char buffer[16]; 

	// Inicializar ADC y LCD
	ADC_Init();
	initLCD8bits();

	LCD_Set_Cursor(1, 1);
	LCD_Write_String("V 1");
	LCD_Set_Cursor(6, 1);
	LCD_Write_String("V 2");

	while (1) {
		adc_value1 = ADC_Read(6);
		adc_value2 = ADC_Read(7);

		// Mostrar valores en pantalla
		LCD_Set_Cursor(1, 2);
		snprintf(buffer, sizeof(buffer), "%d", adc_value1);
		LCD_Write_String(buffer);

		LCD_Set_Cursor(6, 2);
		snprintf(buffer, sizeof(buffer), "%d", adc_value2);
		LCD_Write_String(buffer);

		_delay_ms(100);
	
					}
					}

	









