#define F_CPU 16000000

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "SPI/SPI.h"
#include "adc/adc.h"
#include <util/delay.h>
#include <stdlib.h>

#define BAUD 9600
#define MYUBRR ((F_CPU / (8UL * BAUD)) - 1UL)
 // Configuración con divisor en modo doble velocidad

void initUART9600(void);
void sendUARTChar(char data);
void sendUARTString(const char* str);
uint16_t adc_value1 = 0;

int main(void)
{
	initUART9600(); // Inicializar UART

	_delay_ms(100); // Espera inicial para estabilización
	char buffer[16];  // Buffer para almacenar el valor de los ADCs
	// Inicializar ADC
	ADC_Init();
	DDRD |= (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7);
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5);
	PORTD &= ~((1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7));
	PORTB &= ~((1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5));
	spiInit(SPI_SlAVE_SS,SPI_DATA_ORDER_MSB,SPI_CLOCK_IDLE_LOW,SPI_CLOCK_FIRST_EDGE);
	SPCR |= (1<<SPIE);
	sei();
	while (1)
	{
		uint16_t adc_value1 = ADC_Read(6);
		uint16_t adc_value2 = ADC_Read(7);
		sprintf(buffer, "%d", adc_value1);
		uint8_t text = 0b01001011;	
		sendUARTString("Hola, mundo!\r\n"); // Enviar cadena de texto
		_delay_ms(1000); // Esperar un segundo
		refreshPORT(text);
	}
}

void initUART9600(void) {
	// Configurar el baud rate
	UBRR0H = (unsigned char)(MYUBRR >> 8);
	UBRR0L = (unsigned char)MYUBRR;

	// Habilitar doble velocidad
	UCSR0A |= (1 << U2X0);

	// Habilitar transmisor y receptor
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);

	// Configurar formato: 8 bits de datos, sin paridad, 1 bit de parada
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void sendUARTChar(char data) {
	// Esperar a que el buffer de transmisión esté vacío
	while (!(UCSR0A & (1 << UDRE0)));
	// Colocar el dato en el registro de datos
	UDR0 = data;
}

void sendUARTString(const char* str) {
	while (*str) {
		sendUARTChar(*str++); // Enviar cada carácter de la cadena
	}
}

ISR(SPI_STC_vect){
	uint8_t spiValor = SPDR;
	if (spiValor== 'c')
	{
		spiWrite(adc_value1);
	}
}
void refreshPORT(uint8_t valor){
	if (valor & 0b10000000){
		PORTD |= (1<<PORTD2);
		}else{
		PORTD &= ~(1<<PORTD2);
	}
	
	if (valor & 0b01000000){
		PORTD |= (1<<PORTD3);
		}else{
		PORTD &= ~(1<<PORTD3);
	}
	
	if (valor & 0b00100000){
		PORTD |= (1<<PORTD4);
		}else{
		PORTD &= ~(1<<PORTD4);
	}
	
	if (valor & 0b00010000){
		PORTD |= (1<<PORTD5);
		}else{
		PORTD &= ~(1<<PORTD5);
	}
	
	if (valor & 0b00001000){
		PORTD |= (1<<PORTD6);
		}else{
		PORTD &= ~(1<<PORTD6);
	}
	
	if (valor & 0b00000100){
		PORTD |= (1<<PORTD7);
		}else{
		PORTD &= ~(1<<PORTD7);
	}
	
	if (valor & 0b00000010){
		PORTB |= (1<<PORTB0);
		}else{
		PORTB &= ~(1<<PORTB0);
	}
	
	if (valor & 0b00000001){
		PORTB |= (1<<PORTB1);
		}else{
		PORTB &= ~(1<<PORTB1);
	}
}



