#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h> // VOOR SERIAL EN DEBUG WEGHALEN IN TOEKOMST (DENK IK)
#define F_CPU 16000000UL
#define TRIG_PIN  PB1
#define TRIG_DDR  DDRB
#define TRIG_PORT PORTB
#define FILTER_MAX 15
#define FILTER_MIN 1
#define MAX_F 1400
#define MIN_F 230
#define MAX_D 60

volatile float DISTANCE;
volatile float FREQUENCY = 300;
volatile uint8_t FILTER_SOFTMAX = 5;

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

typedef struct {
    uint8_t age;
    float value;
} FILTER_ITEM;

typedef struct {
    FILTER_ITEM* FILTER_ITEMS;   // dynamically allocated array
    uint8_t size;        // number of valid elements
} FILTER_LIST;

FILTER_LIST filter;

void newFilter(FILTER_LIST* filter) {
    filter->FILTER_ITEMS = (FILTER_ITEM*)malloc(FILTER_MAX * sizeof(FILTER_ITEM));
    filter->size = 0;
}

void filterAdd(FILTER_LIST* filter, float newValue) {
    uint8_t ageMaxxerIndex = 0;

    for (uint8_t i = 0; i < filter->size; i++) { // VERHOOG AGE BIJ 1 VOOR ELKE ITEM
        filter->FILTER_ITEMS[i].age++;
        if (filter->FILTER_ITEMS[i].age > filter->FILTER_ITEMS[ageMaxxerIndex].age) {
                ageMaxxerIndex = i;
        }
    }
    if (filter->size >= FILTER_MAX || filter->size >= FILTER_SOFTMAX) {
        filter->FILTER_ITEMS[ageMaxxerIndex].value = newValue;
        filter->FILTER_ITEMS[ageMaxxerIndex].age = 0;
    } else {
        filter->FILTER_ITEMS[ filter->size].value = newValue;
        filter->FILTER_ITEMS[ filter->size].age = 0;
        filter->size++;
    }
}
void filterRemove(FILTER_LIST* filter) {
    uint8_t ageMaxxerIndex = 0;
    for (uint8_t i = 0; i < filter->size; i++) { // VERHOOG AGE BIJ 1 VOOR ELKE ITEM
        if (filter->FILTER_ITEMS[i].age > filter->FILTER_ITEMS[ageMaxxerIndex].age) {
            ageMaxxerIndex = i;
        }
    }
    filter->FILTER_ITEMS[ageMaxxerIndex].value = filter->FILTER_ITEMS[filter->size].value;
    filter->FILTER_ITEMS[ageMaxxerIndex].age = filter->FILTER_ITEMS[filter->size].age;
    filter->size--;
}


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

void calculateDistance(uint16_t sensorTick) {
    float time_us = (float)sensorTick * 0.5;
    float distance_cm = time_us / 58.0;
    DISTANCE = distance_cm;
}

void calculateFrequency(){
    FREQUENCY = MAX_F  - ((MAX_F - MIN_F) * DISTANCE / 60 );
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
            calculateDistance(ICR1);
            calculateFrequency();
            filterAdd(&filter, FREQUENCY);
                        Serial.println(DISTANCE);
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
                Serial.println(DISTANCE);
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
    setTriggerLow();
    registerSetup();
    newFilter(&filter);

    Serial.begin(9600);
    sei();  // ZET INTERRUPTS AAN
    
    while(1) {
        sensorStateMachine();
        if(LEES_DATA) {
            LEES_DATA = 0; // LEES OPNIEUW

        }
    }
}