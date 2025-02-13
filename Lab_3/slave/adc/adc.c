
#include "adc.h"

void ADC_Init(void) {
	// Referencia AVCC y entrada izquierda en el ADC
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
	
	// Habilitar ADC y preescaler de 128 para frecuencia de reloj de 125 kHz
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel) {
	if (channel > 7) return 0xFFFF;  // Verificar canal v�lido

	// Seleccionar el canal (limpiar bits anteriores)
	ADMUX = (ADMUX & 0xF8) | channel;

	// Iniciar conversi�n
	ADCSRA |= (1 << ADSC);

	// Esperar hasta que termine la conversi�n
	while (ADCSRA & (1 << ADSC));

	// Retornar el valor le�do
	return ADC;
}
