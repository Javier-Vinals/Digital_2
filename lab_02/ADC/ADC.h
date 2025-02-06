/*
 * ADC.h
 *
 * Created: 5/02/2025 17:31:03
 *  Author: Javier
 */ 


#ifndef ADC_H_
#define ADC_H_

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Función para inicializar el ADC
void ADC_Init(void);

// Función para leer el ADC de un canal específico
uint16_t ADC_Read(uint8_t channel);

#endif /* ADC_H_ */
