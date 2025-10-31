#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include "hd44780pcf8574.h"

#define F_CPU 16000000UL

#define TRIG_PIN  PB1
#define TRIG_DDR  DDRB
#define TRIG_PORT PORTB
#define FILTER_MAX 15
#define FILTER_MIN 1
#define MAX_F 1400
#define MIN_F 230
#define MAX_D 60

#define BUZZER_PIN PD3 // OC2B pin

volatile float DISTANCE = 0;
volatile float FREQUENCY = 400;
volatile uint8_t volume = 128;
volatile uint8_t FILTER_SOFTMAX = 5;

volatile bool captureFlag = false;
volatile bool risingEdge = true;

typedef struct {
  uint8_t age;
  float value;
} FILTER_ITEM;

typedef struct {
  FILTER_ITEM *FILTER_ITEMS;
  uint8_t size;
} FILTER_LIST;

FILTER_LIST filter;

#define LCD_ADDR 0x27 



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


typedef enum {
  START_PULSE,
  END_PULSE,
  WAIT
} SENSOR_STATE;

SENSOR_STATE sensor = START_PULSE;

void sensorStateMachine() {
  switch (sensor) {
    case START_PULSE:
      setTriggerHigh();
      _delay_us(10);
      setTriggerLow();
      sensor = WAIT;
      break;

    case WAIT:
      if (captureFlag) {
        calculateDistance(ICR1);
        filterAdd(&filter, calculateFrequency(DISTANCE));
        set_frequency(filterGetMedian(&filter));
        captureFlag = false;
        sensor = START_PULSE;
      }
      break;
  }
}


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

// Ultrasonic echo
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
}

// Buttons
ISR(PCINT2_vect) {
  uint8_t s = PIND;
  if (!(s & (1 << PD4)) && FILTER_SOFTMAX < FILTER_MAX) FILTER_SOFTMAX++;
  if (!(s & (1 << PD5)) && FILTER_SOFTMAX > FILTER_MIN) FILTER_SOFTMAX--;
}

// =============== MAIN =================
int main(void) {
    PORTC |= (1 << PC4) | (1 << PC5);
    registerSetup();
    newFilter(&filter);
    Serial.begin(9600);
    sei();
    //  HD44780_PCF8574_Init(LCD_ADDR);
    // HD44780_PCF8574_DisplayOn(LCD_ADDR);
    // HD44780_PCF8574_PositionXY(LCD_ADDR, 0, 0);
    // HD44780_PCF8574_DrawString(LCD_ADDR, "Hello, world!");
    // HD44780_PCF8574_PositionXY(LCD_ADDR, 0, 1);
    // HD44780_PCF8574_DrawString(LCD_ADDR, "I2C LCD Ready");
    // lcd.init();
    // lcd.backlight();
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("test");

  while (1) {
    sensorStateMachine();
    Serial.println(FREQUENCY);
   //  lcd.print("test");
  }
}
