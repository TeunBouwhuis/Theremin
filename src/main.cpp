#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h> // VOOR SERIAL EN DEBUG WEGHALEN IN TOEKOMST (DENK IK)

volatile uint8_t DRAAI_DING_WAARDE = 0;
volatile uint8_t LEES_DATA = 0;
// test bro
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
  ADCSRA |= (1 << ADPS0);   // 128
  
  ADCSRB = 0;
  ADCSRA |= (1 << ADSC);  // BEGIN LEZEN
}

ISR(ADC_vect) {
    DRAAI_DING_WAARDE = ADCH;          // LEES DING
    LEES_DATA = 1;   // IS GELEZEN
}

void sensorSetup() {
    DDRB &= ~(1 << DDB0); 
    DDRB |= (1 << DDB1); 
}

int main(void) {
    Serial.begin(9600);
    abcRegisterSetup();
    sei();  // ZET INTERRUPTS AAN
    
    while(1) {
        if(LEES_DATA) {
            LEES_DATA = 0; // LEES OPNIEUW
            Serial.println(DRAAI_DING_WAARDE);

        }
    }
}