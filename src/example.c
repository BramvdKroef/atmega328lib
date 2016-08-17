
#include "atmega328pu.h"

uint8_t example_digital() {
  // port B, bit 1 (PB1, pin 15)
  
  // output mode
  setBit(DDRB, PB1);

  // set high
  setBit(PORTB, PB1);

  // set low
  unsetBit(PORTB, PB1);
  
  // input mode
  unsetBit(DDRB, PB1);

  // pullup mode (default high, connect to ground for low)
  setBit(PORTB, PB1);

  return getBit(PINB, PB1);
}

void example_analog() {
  adc_init();
  adc_read(PC0); // port c, bit 0 (analog input 0, pin 23)
}

void example_usart () {
  char buffer[100];
  size_t len;
  
  usart_init();
  usart_write_line("Hello World");

  // echo
  len = usart_read_line(buffer, 100);
  usart_write_buffer(buffer, len);
}

void example_spi() {
  uint8_t buffer[20];
  
  spi_init_master(0);

  // source select
  unsetBit(PORTB, PB2);

  spi_write(0xFF);
  
  spi_read_buffer(buffer, 20);
  spi_write_buffer(buffer, 20);

  // unselect source.
  setBit(PORTB, PB2);
}
int main() {
  example_digital();

  example_analog();

  example_usart();

  example_spi();
}
