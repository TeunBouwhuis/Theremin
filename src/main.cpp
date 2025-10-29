#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h> // VOOR SERIAL EN DEBUG WEGHALEN IN TOEKOMST (DENK IK)
#define F_CPU 16000000UL
#define TRIG_PIN  PB1
#define TRIG_DDR  DDRB
#define TRIG_PORT PORTB


volatile uint8_t DRAAI_DING_WAARDE = 0;
volatile uint8_t LEES_DATA = 0;
volatile uint16_t TIME = 0;

volatile uint16_t ICR_VALUE = 0;
volatile bool captureFlag = false;
volatile bool risingEdge = true;


typedef enum {
    START_PULSE,
    END_PULSE,
    WAIT
} SENSOR_STATE;

SENSOR_STATE sensor = START_PULSE;


void sensorRegisterSetup() {
    TCCR1A = 0x0;
    TCCR1B = (1 << ICES1);
    TCCR1B = (1 << CS11);  // 8
    TIMSK1 = (1 << ICIE1);
    TCNT1 = 0;
    TRIG_DDR |= (1 << TRIG_PIN); // output
}

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

void registerSetup() {
    sensorRegisterSetup();
    abcRegisterSetup();
}



void setTriggerHigh(void) {
    TRIG_PORT |= (1 << TRIG_PIN);
}

void setTriggerLow(void) {
    TRIG_PORT &= ~(1 << TRIG_PIN);
}

void triggerPulse(void) {
    setTriggerHigh();
    _delay_us(10); // 10 
    setTriggerLow();
}

float calculateDistance(uint16_t sensorTick) {
    float time_us = (float)sensorTick * 0.5;
    float distance_cm = time_us / 58.0;
    return distance_cm;
}

void sensorStateMachine() {
    switch (sensor)
    {
    case START_PULSE:
        setTriggerHigh();
        TIME = ICR1;
        sensor = END_PULSE;
        _delay_us(10); 
        break;
    
    case END_PULSE:
        setTriggerLow();
        sensor = WAIT;
        break;
    
    case WAIT:
        if (captureFlag) {
            Serial.println(calculateDistance(ICR1));
            captureFlag = false;
            sensor = START_PULSE;
        }
        break;
    
    default:
        break;
    }
}



ISR(ADC_vect) {
    DRAAI_DING_WAARDE = ADCH;          // LEES DING
    LEES_DATA = 1;   // IS GELEZEN
}

ISR(TIMER1_CAPT_vect) {
    if (risingEdge) {
        TCNT1 = 0;
        TCCR1B &= ~(1 << ICES1);
        risingEdge = false;
    } else {
        TCCR1B |= (1 << ICES1);
        risingEdge = true;
        captureFlag = true;
    }
}



int main(void) {
    SENSOR_STATE sensor = START_PULSE;
    registerSetup();
    Serial.begin(9600);
    sei();  // ZET INTERRUPTS AAN
    
    while(1) {
        sensorStateMachine();
        if(LEES_DATA) {
            LEES_DATA = 0; // LEES OPNIEUW
            Serial.println(DRAAI_DING_WAARDE);

        }
    }
}