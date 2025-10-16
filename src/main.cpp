/*
 * generate a 440Hz tone, i.e. interrupts at 880Hz
 * Connect speaker to pin 3
 */
#include <avr/interrupt.h>
#include <avr/io.h>

/*
 * prototypes
 */
void setup_timer0(void);

/*
 * COMPA timer0
 */
ISR(TIMER0_COMPA_vect) {
  PORTD ^= (1<<PD3);      // toggle
}

/*
 * main
 */
int main(void){
  // setup
  DDRD |= (1<<PD3); // pin 3 tone
  setup_timer0();

  // endless loop
  while(1) {
                    // empty
  }

  return(0);        // never reached
}

/*
 * timer0 stuff
 * 880Hz => 0,001136s period
 * COM0x[1:0] = 0b00 (pin OC0x disconnected)
 * WGM0[2:0] = 0b010 (mode 2: CTC)
 * Fint = Fcpu / (N * (OCR0A+1)) <=>
 * OCR0A = Fcpu / (N * Fint) - 1
 * OCR0A = 16000000 / (N * 880) - 1
 *    = 18180 (with N=1)
 *    = 2271 (with N=8)
 *    = 263 (with N=64)
 *    = 70 (with N=256)!
 * CS0[2:0] = 0b100 (1/256 prescale)
 * OCIE0A = 1 (enable interrupt on OCR0A match)
 */
void abcRegisterSetup() {
  ADMUX = 0;
  ADMUX |= (1 << ADLAR);
  ADMUX |= (1 << REFS0);
  ADMUX |= (0 & 0x0F);

  ADCSRA = 0;
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADATE);
  ADCSRA |= (1 << ADIE); 
  ADCSRA |= (1 << ADPS2); 
  ADCSRA |= (1 << ADPS1);  
  ADCSRA |= (1 << ADPS0);   
  
  ADCSRB = 0;
  ADCSRA |= (1 << ADSC); 
}