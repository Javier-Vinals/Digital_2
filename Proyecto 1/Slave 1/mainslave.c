// Slave Code
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define HX711_DT PD4
#define HX711_SCK PD5
#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3
#define STATUS_PIN1 PD2
#define STATUS_PIN2 PD3
#define TRIG_PIN PD6
#define ECHO_PIN PD7

const uint8_t step_sequence[8] = {
	0b0001, 0b0011, 0b0010, 0b0110,
	0b0100, 0b1100, 0b1000, 0b1001
};

void I2C_Slave_Init(uint8_t address) {
	TWAR = (address << 1);
	TWCR = (1 << TWEA) | (1 << TWEN);
}

void I2C_Slave_Respond(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
}

long hx711_read(void) {
	uint16_t timeout = 10000;
	while ((PIND & (1 << HX711_DT)) && timeout--) {
		_delay_us(1);
	}
	if (timeout == 0) return -1;

	uint32_t count = 0;
	for (uint8_t i = 0; i < 24; i++) {
		PORTD |= (1 << HX711_SCK);
		_delay_us(0.5);
		count = count << 1;
		PORTD &= ~(1 << HX711_SCK);
		_delay_us(0.5);
		if (PIND & (1 << HX711_DT)) {
			count++;
		}
	}

	PORTD |= (1 << HX711_SCK);
	_delay_us(0.5);
	PORTD &= ~(1 << HX711_SCK);
	_delay_us(0.5);

	if (count & 0x800000) {
		count |= 0xFF000000;
	}

	return (long)count;
}

void stepper_step(uint16_t steps, uint8_t direction) {
	for (uint16_t i = 0; i < steps; i++) {
		for (uint8_t j = 0; j < 8; j++) {
			uint8_t step_index = direction ? j : (7 - j);
			PORTB = step_sequence[step_index];
			_delay_ms(2);
		}
	}
}

// Función mejorada para medir distancia con el sensor ultrasónico
uint8_t measure_distance() {
	uint32_t timeout = 30000; // Evitar bucles infinitos
	uint16_t duration;
	uint16_t distance;

	// Enviar pulso de 10 us
	PORTD |= (1 << TRIG_PIN);
	_delay_us(10);
	PORTD &= ~(1 << TRIG_PIN);

	// Esperar el inicio del pulso de ECHO (con timeout)
	while (!(PIND & (1 << ECHO_PIN)) && timeout--) {
		_delay_us(1);
	}
	if (timeout == 0) return 255; // Error: Sin respuesta

	// Medir duración del pulso HIGH en ECHO
	TCNT1 = 0;
	while ((PIND & (1 << ECHO_PIN)) && timeout--) {
		_delay_us(1);
	}
	if (timeout == 0) return 255; // Error: Tiempo de espera agotado

	duration = TCNT1;

	// Convertir a distancia en cm
	distance = ((duration * 10) / 95)-2;

	// Si la distancia es muy pequeña, corregir o descartar
	if (distance < 3) return 0;  // Forzar a 0 cm si es muy bajo
	if (distance > 8) return 255; // Fuera de rango

	return (uint8_t)distance;
}



int main(void) {
	I2C_Slave_Init(0x10);

	DDRD &= ~((1 << HX711_DT) | (1 << STATUS_PIN1) | (1 << STATUS_PIN2) | (1 << ECHO_PIN));
	DDRD |= (1 << HX711_SCK) | (1 << TRIG_PIN);
	PORTD &= ~(1 << HX711_SCK);

	DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

	// Configurar Timer1 con prescaler 64 para mayor precisión en el ultrasonido
	TCCR1B |= (1 << CS11) | (1 << CS10);

	long prev_weight = hx711_read();
	uint8_t motor_moved = 0;

	while (1) {
		TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
		while (!(TWCR & (1 << TWINT)));

		long weight = hx711_read();
		if (weight == -1) continue;

		if ((weight < 73000 || weight > 78000) && !motor_moved) {
			stepper_step(140, 0);
			_delay_ms(200);
			stepper_step(140, 1);
			motor_moved = 1;
			} else if (weight >= 73000 && weight <= 78000) {
			motor_moved = 0;
		}

		prev_weight = weight;

		uint8_t status = ((PIND & (1 << STATUS_PIN1)) >> STATUS_PIN1) |
		(((PIND & (1 << STATUS_PIN2)) >> STATUS_PIN2) << 1);
		uint8_t distance = measure_distance();

		uint8_t bytes[6];
		bytes[0] = (weight >> 24) & 0xFF;
		bytes[1] = (weight >> 16) & 0xFF;
		bytes[2] = (weight >> 8) & 0xFF;
		bytes[3] = weight & 0xFF;
		bytes[4] = status;
		bytes[5] = distance;

		for (uint8_t i = 0; i < 6; i++) {
			I2C_Slave_Respond(bytes[i]);
		}
	}
}
