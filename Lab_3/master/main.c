/*
 * Lab_03.c
 *
 * Created: 6/02/2025 10:07:47
 * Author : Javier
 */ 
#define F_CPU 16000000UL
#define UART_BUFFER_SIZE 64  // Tamaño del buffer de recepción

#include <avr/io.h> 
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "SPI/SPI.h"

volatile char UART_Buffer[UART_BUFFER_SIZE];  // Buffer circular
volatile uint8_t UART_Head = 0;  // Índice de escritura
volatile uint8_t UART_Tail = 0;  // Índice de lectura
uint8_t valorSPI = 0;

void refreshPORT(uint8_t valor);
void initUART9600(void);
void WriteMessage(char* mensaje);
void WriteUART(char caracter);
char ReadUART(void);
void WriteNumber(uint8_t num);

int main(void) { 
	cli();
	initUART9600();
	sei();
	
	// Configuración de pines de salida
	DDRD |= (1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7);
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5);
	PORTD &= ~((1<<DDD2)|(1<<DDD3)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7));
	PORTB &= ~((1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5));

	// Inicializar SPI
	spiInit(SPI_MASTER_OSC_DIV16, SPI_DATA_ORDER_MSB, SPI_CLOCK_IDLE_LOW, SPI_CLOCK_FIRST_EDGE);

	// Enviar mensaje inicial
	WriteMessage("Mensaje de prueba\n");


	while (1) {
		// Comunicación SPI
		PORTB &= ~(1<<PORTB2);
		spiWrite('c');
		valorSPI = spiRead();
		refreshPORT(valorSPI);
		PORTB |= (1<<PORTB2);
		WriteNumber(valorSPI);
		WriteMessage("\n\n");
/*
		// Leer UART y reenviar lo recibido
		char dato = ReadUART();
		if (dato) {
			WriteUART(dato);  // Enviar de vuelta al monitor serie
		}
*/
		_delay_ms(100);
	}
}

void refreshPORT(uint8_t valor) {
	// Actualizar los pines de salida en función del valor recibido
	PORTD = (PORTD & 0x03) | ((valor & 0b11111100) >> 2);
	PORTB = (PORTB & 0xFC) | (valor & 0b00000011);
}

void WriteNumber(uint8_t num) {
	char buffer[4];  // Espacio para hasta 3 dígitos y '\0'
	sprintf(buffer, "%d", num);
	WriteMessage(buffer);  // Usa WriteMessage para enviar la cadena completa
}

void WriteMessage(char* mensaje) {
	while (*mensaje) {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *mensaje++;
	}
}

void WriteUART(char caracter) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = caracter;
}

char ReadUART(void) {
	while (UART_Head == UART_Tail);  // Espera hasta que haya datos
	char c = UART_Buffer[UART_Tail];
	UART_Tail = (UART_Tail + 1) % UART_BUFFER_SIZE;
	return c;
}


void initUART9600(void) {
	// Configurar TX (salida) y RX (entrada)
	DDRD &= ~(1<<DDD0);  // RX como entrada
	DDRD |= (1<<DDD1);   // TX como salida
	
	// Habilitar doble velocidad y configurar el baud rate
	UCSR0A = 0;
	UCSR0A |= (1 << U2X0);  // Activar doble velocidad

	// Habilitar RX, TX e interrupción de recepción
	UCSR0B = 0;
	UCSR0B |= (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);

	// Configurar el formato de trama (8 bits, sin paridad, 1 bit de stop)
	UCSR0C = 0;
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);

	// Configurar UBRR0 para 9600 bps con doble velocidad (U2X0 = 1)
	UBRR0 = 207;
}

// Interrupción de recepción UART
ISR(USART_RX_vect) {
	char received = UDR0;  // Leer el dato recibido
	uint8_t nextHead = (UART_Head + 1) % UART_BUFFER_SIZE;

	// Si el buffer no está lleno, guardar el dato
	if (nextHead != UART_Tail) {
		UART_Buffer[UART_Head] = received;
		UART_Head = nextHead;
	}
}

