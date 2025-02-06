/*
 * ADC.c
 *
 * Created: 5/02/2025 17:30:52
 *  Author: Javier
 */ 

#include "ADC.h"
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void ADC_Init(void) {
	// Referencia AVCC y entrada izquierda en el ADC
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
	
	// Habilitar ADC y preescaler de 128 para frecuencia de reloj de 125 kHz
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel) {
	if (channel > 7) return 0xFFFF;  // Verificar canal válido

	// Seleccionar el canal (limpiar bits anteriores)
	ADMUX = (ADMUX & 0xF8) | channel;

	// Iniciar conversión
	ADCSRA |= (1 << ADSC);

	// Esperar hasta que termine la conversión
	while (ADCSRA & (1 << ADSC));

	return ADC;
}

