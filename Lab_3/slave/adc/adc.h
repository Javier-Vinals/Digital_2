
#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <stdint.h>

// Función para inicializar el ADC
void ADC_Init(void);

// Función para leer el ADC de un canal específico
uint16_t ADC_Read(uint8_t channel);

#endif /* ADC_H_ */
