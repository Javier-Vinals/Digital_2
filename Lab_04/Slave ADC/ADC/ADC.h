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

// Funci�n para inicializar el ADC
void ADC_Init(void);

// Funci�n para leer el ADC de un canal espec�fico
uint16_t ADC_Read(uint8_t channel);

#endif /* ADC_H_ */
