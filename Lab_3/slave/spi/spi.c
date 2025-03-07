#include "spi.h"

void spiInit(Spi_Type sType, Spi_Data_Order sDataOrder, Spi_Clock_Polarity sClockPolarity, Spi_Clock_Phase sClockPhase) {
	// PB2 -> SS, PB3 -> MOSI, PB4 -> MISO, PB5 -> SCK

	if (sType & (1 << MSTR)) { // If Master Mode
		DDRB |= (1 << DDB3) | (1 << DDB5) | (1 << DDB2); // MOSI, SCK, SS as output
		DDRB &= ~(1 << DDB4); // MISO as input
		SPCR |= (1 << MSTR); // Master mode enabled

		uint8_t clockDivider = sType & 0b00000111;
		// Configure SPI clock speed
		switch (clockDivider) {
			case 0: // DIV2
			SPCR &= ~((1 << SPR1) | (1 << SPR0));
			SPSR |= (1 << SPI2X);
			break;
			case 1: // DIV4
			SPCR &= ~((1 << SPR1) | (1 << SPR0));
			SPSR &= ~(1 << SPI2X);
			break;
			case 2: // DIV8
			SPCR |= (1 << SPR0);
			SPCR &= ~(1 << SPR1);
			SPSR |= (1 << SPI2X);
			break;
			case 3: // DIV16
			SPCR |= (1 << SPR0);
			SPCR &= ~(1 << SPR1);
			SPSR &= ~(1 << SPI2X);
			break;
			case 4: // DIV32
			SPCR &= ~(1 << SPR0);
			SPCR |= (1 << SPR1);
			SPSR |= (1 << SPI2X);
			break;
			case 5: // DIV64
			SPCR &= ~(1 << SPR0);
			SPCR |= (1 << SPR1);
			SPSR &= ~(1 << SPI2X);
			break;
			case 6: // DIV128
			SPCR |= (1 << SPR0) | (1 << SPR1);
			SPSR &= ~(1 << SPI2X);
			break;
		}
		} else { // If Slave Mode
		DDRB |= (1 << DDB4); // MISO as output
		DDRB &= ~((1 << DDB3) | (1 << DDB5) | (1 << DDB2)); // MOSI, SCK, SS as input
		SPCR &= ~(1 << MSTR); // Slave mode enabled
	}

	// Enable SPI, configure data order, clock polarity, and clock phase
	SPCR |= (1 << SPE) | sDataOrder | sClockPolarity | sClockPhase;
}

static void spiReceiveWait() {
	while (!(SPSR & (1 << SPIF))); // Wait for Data Receive complete
}

void spiWrite(uint8_t dat) { // Write data to SPI bus
	SPDR = dat;
	spiReceiveWait(); // Ensure transmission completes
}

unsigned spiDataReady() { // Check whether the data is ready to read
	return (SPSR & (1 << SPIF));
}

uint8_t spiRead(void) { // Read the received data
	spiReceiveWait(); // Wait for Data Receive complete
	return SPDR; // Read the received data from the buffer
}

void spiEnableSS() {
	PORTB &= ~(1 << PB2); // Pull SS low to enable slave
}

void spiDisableSS() {
	PORTB |= (1 << PB2); // Pull SS high to disable slave
}
