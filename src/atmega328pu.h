#ifndef __ATMEGA328PU_H
#define __ATMEGA328PU_H

#include <avr/io.h>
#include <stdint.h>
#include <stddef.h>

/**
 *  Atmega 328 pu hardware interface
 *
 *        +---u---+
 *   PC6 -| 1   28|- PC5
 *   PD0 -| 2   27|- PC4
 *   PD1 -| 3   26|- PC3
 *   PD2 -| 4   25|- PC2
 *   PD3 -| 5   24|- PC1
 *   PD4 -| 6   23|- PC0
 *   VCC -| 7   22|- GND
 *   GND -| 8   21|- AREF
 *   PB6 -| 9   20|- AVCC
 *   PB7 -|10   19|- PB5
 *   PD5 -|11   18|- PB4
 *   PD6 -|12   17|- PB3
 *   PD7 -|13   16|- PB2
 *   PB0 -|14   15|- PB1
 *        +-------+
 *
 *
 *  +-----+------+--------------------+-----------+
 *  | Pin | Name | Function           | Ext       |
 *  +-----+------+--------------------+-----------+
 *  |   1 |  PC6 | Reset              |           |
 *  |   2 |  PD0 | Digital            | RX        |
 *  |   3 |  PD1 | Digital            | TX        |
 *  |   4 |  PD2 | Digital            |           |
 *  |   5 |  PD3 | Digital            | PWM       |
 *  |   6 |  PD4 | Digital            |           |
 *  |   7 |  VCC | Power              |           |
 *  |   8 |  GND | Ground             |           |
 *  |   9 |  PB6 | Crystal Oscillator |           |
 *  |  10 |  PB7 | Crystal Oscillator |           |
 *  |  11 |  PD5 | Digital            | PWM       |
 *  |  12 |  PD6 | Digital            | PWM       |
 *  |  13 |  PD7 | Digital            |           |
 *  |  14 |  PB0 | Digital            |           |
 *  |  15 |  PB1 | Digital            | PWM       |
 *  |  16 |  PB2 | Digital            | PWM, SS   |
 *  |  17 |  PB3 | Digital            | PWM, MOSI |
 *  |  18 |  PB4 | Digital            | MISO      |
 *  |  19 |  PB5 | Digital            | SCK       |
 *  |  20 | AVCC | V+ for ADC         |           |
 *  |  21 | AREF | Reference Voltage  |           |
 *  |  22 |  GND | Ground             |           |
 *  |  23 |  PC0 | Analog Input       |           |
 *  |  24 |  PC1 | Analog Input       |           |
 *  |  25 |  PC2 | Analog Input       |           |
 *  |  26 |  PC3 | Analog Input       |           |
 *  |  27 |  PC4 | Analog Input       |           |
 *  |  28 |  PC5 | Analog Input       |           |
 *  +-----+------+--------------------+-----------+
 */

//#define F_CPU 16000000UL

#define setBit(byte, n)   byte |= (1 << n)
#define unsetBit(byte, n) byte &= ~(1 << n)
#define getBit(byte, n)   (byte >> n) & 1



void adc_init() {
  // Select Vref=AVcc
  ADMUX |= (1 << REFS0);
  //set prescaller to 128 and enable ADC 
  ADCSRA |=
    (1 << ADPS2) |
    (1 << ADPS1) |
    (1 << ADPS0) |
    (1 << ADEN);
}

uint16_t adc_read (uint8_t channel) {
  //select ADC channel with safety mask
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  
  //single conversion mode
  ADCSRA |= (1 << ADSC);
  
  // wait until ADC conversion is complete
  while (ADCSRA & (1 << ADSC) );
  
  return ADC;
}



#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU/(USART_BAUDRATE*16UL)))-1)

void usart_init() {
  UCSR0B |= (1<<RXEN0)  | (1<<TXEN0);
  UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01);
  UBRR0H  = (BAUD_PRESCALE >> 8);
  UBRR0L  = BAUD_PRESCALE;
}

uint8_t usart_read() {
  // wait until a byte is ready to read
  while( ( UCSR0A & ( 1 << RXC0 ) ) == 0 ){}

  // grab the byte from the serial port
  return UDR0;
}

void usart_write(uint8_t byte) {
  // wait until the port is ready to be written to
  while( ( UCSR0A & ( 1 << UDRE0 ) ) == 0 ){}

  // write the byte to the serial port
  UDR0 = byte;
}

void usart_write_line(char* buffer) {
  while (*buffer != 0) {
    usart_write(*buffer);
    buffer++;
  }
}
void usart_write_buffer(char* buffer, size_t len) {
  while (len > 0) {
    usart_write(*buffer);
    buffer++;
    len--;
  }
}
size_t usart_read_line(char* buffer, size_t maxlen) {
  size_t len = 0;
  while (len < maxlen) {
    *buffer = usart_read();
    len++;
    buffer++;
    
    if (*buffer == '\n')
      break;
  }
  
  return len;
}

// Byte order is least significant bit first.
#define SPI_OPTIONS_LSB                       (1 << DORD)
// Clock signal is high when idle.
#define SPI_OPTIONS_CLOCK_POLARITY_HIGH_IDLE  (1 << CPOL)
// Sample data on trailing edge.
#define SPI_OPTIONS_CLOCK_PHASE_TRAILING_EDGE (1 << CPHA)

// SCK frequency (default is fOsc/4)
// fOsc is the speed of the internal clock, which is divided by the
// option given. FOSC4 is four times slower than the internal clock,
// FOSC16 is sixteen times slower.
#define SPI_OPTIONS_SPEED_FOSC4               0
#define SPI_OPTIONS_SPEED_FOSC16              (1 << SPR0)
#define SPI_OPTIONS_SPEED_FOSC64              (1 << SPR1)
#define SPI_OPTIONS_SPEED_FOSC128             (1 << SPR0 | 1 << SPR1)

#define SPI_OPTIONS_SPEED_FOSC2               (1 << SPI2X)
#define SPI_OPTIONS_SPEED_FOSC8               (1 << SPI2X | 1 << SPR0)
#define SPI_OPTIONS_SPEED_FOSC32              (1 << SPI2X | 1 << SPR1)
//#define SPI_OPTIONS_SPEED_FOSC64              (1 << SPI2X | 1 << SPR0 | 1 << SPR1)


void spi_init_master(uint8_t options) {
  // MOSI output, SCK output
  setBit(DDRB, PB3);
  setBit(DDRB, PB5);

  // MISO input
  unsetBit(DDRB, PB4);

  SPCR =
    // spi enable
    (1 << SPE)  |
    // master mode
    (1 << MSTR) |
    options;
}

void spi_init_slave(uint8_t options) {
  // MISO output
  setBit(DDRB, PB4);
  // MOSI, SCK input
  unsetBit(DDRB, PB3);
  unsetBit(DDRB, PB5);
  
  SPCR =
    (1 << SPE)  |
    options;
}

uint8_t spi_transfer(uint8_t in) {
  // set the Data Register
  SPDR = in;

  // Wait for the Interupt Flag in the Status Register
  while (!(SPSR & (1 << SPIF)));

  // Return the Data Register 
  return (SPDR);
}

#define spi_read()      spi_transfer(0x00)
#define spi_write(data) spi_transfer(data)

void spi_read_buffer(uint8_t* buffer, size_t len) {
  while(len--) {
    *buffer = spi_read();
    buffer++;
  }
}

void spi_write_buffer(uint8_t* buffer, size_t len) {
  while(len--) {
    spi_write(*buffer);
    buffer++;
  }
}


#endif
