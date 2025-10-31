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

<<<<<<< Updated upstream
volatile float DISTANCE;
volatile float FREQUENCY = 300;
=======
#define BUZZER_PIN PD3

volatile float DISTANCE = 0;
volatile float FREQUENCY = 400;
volatile uint8_t volume = 128;
>>>>>>> Stashed changes
volatile uint8_t FILTER_SOFTMAX = 5;

volatile uint8_t DRAAI_DING_WAARDE = 0;
volatile uint8_t LEES_DATA = 0;
volatile uint16_t TIME = 0;

volatile uint16_t ICR_VALUE = 0;
volatile bool captureFlag = false;
volatile bool risingEdge = true;

<<<<<<< Updated upstream
=======
typedef struct {
  uint8_t age;
  float value;
} FILTER_ITEM;

typedef struct {
  FILTER_ITEM *FILTER_ITEMS;
  uint8_t size;
} FILTER_LIST;

FILTER_LIST filter;




void newFilter(FILTER_LIST *filter) {
  filter->FILTER_ITEMS = (FILTER_ITEM *)malloc(FILTER_MAX * sizeof(FILTER_ITEM));
  filter->size = 0;
}

void filterAdd(FILTER_LIST *filter, float newValue) {
  uint8_t ageMax = 0;
  for (uint8_t i = 0; i < filter->size; i++) {
    filter->FILTER_ITEMS[i].age++;
    if (filter->FILTER_ITEMS[i].age > filter->FILTER_ITEMS[ageMax].age)
      ageMax = i;
  }
  if (filter->size >= FILTER_MAX || filter->size >= FILTER_SOFTMAX) {
    filter->FILTER_ITEMS[ageMax].value = newValue;
    filter->FILTER_ITEMS[ageMax].age = 0;
  } else {
    filter->FILTER_ITEMS[filter->size].value = newValue;
    filter->FILTER_ITEMS[filter->size].age = 0;
    filter->size++;
  }
}

int compareFilterItems(const void *a, const void *b) {
  float valA = ((FILTER_ITEM *)a)->value;
  float valB = ((FILTER_ITEM *)b)->value;
  return (valA > valB) - (valA < valB);
}

float filterGetMedian(FILTER_LIST *filter) {
  if (filter->size == 0) return 0.0;
  FILTER_ITEM temp[filter->size];
  for (uint8_t i = 0; i < filter->size; i++) temp[i] = filter->FILTER_ITEMS[i];
  qsort(temp, filter->size, sizeof(FILTER_ITEM), compareFilterItems);
  return temp[filter->size / 2].value;
}

void calculateDistance(uint16_t sensorTick) {
  float time_us = (float)sensorTick * 0.5;
  DISTANCE = time_us / 58.0;
}

float calculateFrequency(float distance_cm) {
  if (distance_cm < 0) distance_cm = 0;
  if (distance_cm > MAX_D) distance_cm = MAX_D;
  return MAX_F - ((MAX_F - MIN_F) * distance_cm / MAX_D);
}


void pwmRegisterSetup(void) {
  DDRD |= (1 << PD3); // OC2B output
  TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2B1); // Fast PWM, non-inverting
  TCCR2B = (1 << CS20); // no prescaler
  OCR2B = volume;
  TIMSK2 = 0;
}

void buzzerRegisterSetup(void) {
  TCCR0A = (1 << WGM01); // CTC mode
  TCCR0B = (1 << CS02);  // prescaler 256 for 230–1400Hz range
  TIMSK0 = (1 << OCIE0A);
}

void abcRegisterSetup(void) {
  ADMUX = (1 << ADLAR) | (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE)
         | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRB = 0;
  ADCSRA |= (1 << ADSC);
}

void sensorRegisterSetup(void) {
  TCCR1A = 0;
  TCCR1B = (1 << ICES1) | (1 << CS11);
  TIMSK1 = (1 << ICIE1);
  DDRB |= (1 << TRIG_PIN);
}

void buttonRegisterSetup(void) {
  DDRD &= ~((1 << PD4) | (1 << PD5));
  PORTD |= (1 << PD4) | (1 << PD5);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);
}

void registerSetup() {
  buttonRegisterSetup();
  sensorRegisterSetup();
  abcRegisterSetup();
  pwmRegisterSetup();
  buzzerRegisterSetup();
}

void setTriggerHigh(void) { TRIG_PORT |= (1 << TRIG_PIN); }
void setTriggerLow(void)  { TRIG_PORT &= ~(1 << TRIG_PIN); }

void set_frequency(uint16_t freq) {
  if (freq < MIN_F) freq = MIN_F;
  if (freq > MAX_F) freq = MAX_F;
  FREQUENCY = freq;

  uint32_t ocr_value = (16000000UL / (256UL * (2UL * freq))) - 1;
  if (ocr_value > 255) ocr_value = 255;
  OCR0A = (uint8_t)ocr_value;
  TCNT0 = 0; // reset counter to avoid lag/stuck
}



>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
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

int compareFilterItems(void* a, void* b) {
    float valA = ((FILTER_ITEM*)a)->value;
    float valB = ((FILTER_ITEM*)b)->value;
    if (valA < valB) return -1;
    if (valA > valB) return 1;
    return 0;
}

float filterGetMedian(FILTER_LIST* filter) {
    if (filter->size == 0) return 0.0;
    FILTER_ITEM temp[filter->size];
    for (uint8_t i = 0; i < filter->size; i++) {
        temp[i] = filter->FILTER_ITEMS[i];
    }
    qsort(temp, filter->size, sizeof(FILTER_ITEM), compareFilterItems);
    uint8_t mid = filter->size / 2;

    float median = temp[mid].value;
    return median;
=======

ISR(TIMER0_COMPA_vect) {
  static bool enabled = true;
  if (enabled) {
    TCCR2A &= ~(1 << COM2B1); // disable PWM output
    enabled = false;
  } else {
    TCCR2A |= (1 << COM2B1); // enable PWM output
    enabled = true;
  }
}

ISR(ADC_vect) {
  uint8_t val = ADCH;
  if (val < 10) val = 10;
  if (val > 245) val = 245;
  volume = val;
  OCR2B = volume;
}

ISR(TIMER1_CAPT_vect) {
  static bool rising = true;
  if (rising) {
    TCNT1 = 0;
    TCCR1B &= ~(1 << ICES1);
    rising = false;
  } else {
    TCCR1B |= (1 << ICES1);
    rising = true;
    captureFlag = true;
  }
>>>>>>> Stashed changes
}

void buttonsInit(void) {
    // Set PD4 and PD5 as inputs
    DDRD &= ~((1 << PD4) | (1 << PD5));

    // Enable internal pull-ups
    PORTD |= (1 << PD4) | (1 << PD5);

    // Enable Pin Change Interrupt for PCINT[23:16] (Port D)
    PCICR |= (1 << PCIE2);

    // Enable PCINT20 (PD4) and PCINT21 (PD5)
    PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);
}

// ISR for PD4 / PD5 change
ISR(PCINT2_vect) {
    // Read pin states (inverted because of pull-ups)
    uint8_t buttonState = PIND;

    // If PD4 pressed → increase size
    if (!(buttonState & (1 << PD4))) {
        if (FILTER_SOFTMAX < FILTER_MAX) FILTER_SOFTMAX++;
    }

    // If PD5 pressed → decrease size
    if (!(buttonState & (1 << PD5))) {
        if (FILTER_SOFTMAX > FILTER_MIN) FILTER_SOFTMAX--;
    }
}

<<<<<<< Updated upstream
void sensorRegisterSetup() {
    TCCR1A = 0x0;
    TCCR1B = (1 << ICES1) | (1 << CS11);
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
    buttonsInit();
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

void Filter_Print(FILTER_LIST* filter) {
    if (filter == NULL || filter->FILTER_ITEMS == NULL) {
        Serial.println("Filter not initialized.");
        return;
    }

    Serial.println("---- FILTER CONTENTS ----");
    Serial.print("Size: ");
    Serial.println(filter->size);

    for (uint8_t i = 0; i < filter->size; i++) {
        Serial.print("[");
        Serial.print(i);
        Serial.print("] Age: ");
        Serial.print(filter->FILTER_ITEMS[i].age);
        Serial.print("  |  Value: ");
        Serial.println(filter->FILTER_ITEMS[i].value, 2); // 2 decimal places
    }
    Serial.println(filterGetMedian(filter));
    Serial.println("--------------------------");

}

float calculateFrequency(float distance_cm) {
    if (distance_cm < 0) distance_cm = 0;
    if (distance_cm > MAX_D) distance_cm = MAX_D;

    float FREQUENCY = MAX_F - ((MAX_F - MIN_F) * distance_cm / MAX_D);
    return FREQUENCY;
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
            FREQUENCY = calculateFrequency(DISTANCE);
            if (filter.size > FILTER_SOFTMAX) {
                filterRemove(&filter);
            } else {
                filterAdd(&filter, FREQUENCY);

            }
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



=======
>>>>>>> Stashed changes
int main(void) {
    setTriggerLow();
    registerSetup();
    newFilter(&filter);
<<<<<<< Updated upstream

    Serial.begin(9600);
    sei();  // ZET INTERRUPTS AAN
    
    while(1) {
        sensorStateMachine();
        if(LEES_DATA) {
            Filter_Print(&filter);
            LEES_DATA = 0; // LEES OPNIEUW

        }
    }
}
=======
    Serial.begin(9600);
    sei();
  while (1) {
    sensorStateMachine();
    Serial.println(FREQUENCY);
   //  lcd.print("test");
  }
}
>>>>>>> Stashed changes
