#include <Arduino.h>
#include <avr/sleep.h>

#define PIN_LED 1
#define PIN_BUTTON 0
#define PCIE_BUTTON PCINT0
#define STOP_DELAY_MS 480000 // 8 min
#define LED_MIN_VALUE 140
#define LED_MAX_VALUE 255
#define INSPIRE_DIVIDER_START 4360 // 2.18s
#define INSPIRE_DIVIDER_END 8000 // 4s
#define EXPIRE_DIVIDER_START 6540 // 3.27s
#define EXPIRE_DIVIDER_END 12000 // 6s

enum emode {
  OFF, INSPIRE, EXPIRE
};

volatile emode g_mode = OFF;

ISR(PCINT0_vect) {
  if ( (PINB & _BV(PCIE_BUTTON)) == 0) {
    if(g_mode != OFF) {
      g_mode = OFF;
    }
  }
}

void sleep() {
  GIMSK |= _BV(PCIE); // - Turns on pin change interrupts
  PCMSK |= _BV(PIN_BUTTON); // - Turns on interrupts on pins
  ADCSRA &= ~_BV(ADEN); // - Disable ADC, saves ~230uA
  sleep_mode(); // - Go to sleep

  //ADCSRA |= _BV(ADEN); // - Enable ADC
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  digitalWrite(PIN_BUTTON, HIGH); // Enable pullup resistor

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop()
{
  static unsigned long g_startTime = 0;
  static unsigned long g_changeTime = 0;
  static unsigned int g_previousLedValue = 0;

  unsigned int newLedValue;
  unsigned long timeValueSinceChange;
  unsigned long timeValueSinceStart;
  unsigned long pulseDivider;

  timeValueSinceChange = millis() - g_changeTime;
  timeValueSinceStart = millis() - g_startTime;

  switch(g_mode) {
    case OFF:
      g_startTime = 0;
      g_previousLedValue = 0;
      g_changeTime = 0;
      analogWrite(PIN_LED, LOW);
      sleep();

      g_mode = INSPIRE;
      g_previousLedValue = 0;
      g_startTime = millis();
      g_changeTime = g_startTime;
    break;
    case INSPIRE:
      pulseDivider = timeValueSinceStart * (INSPIRE_DIVIDER_END - INSPIRE_DIVIDER_START) / STOP_DELAY_MS + INSPIRE_DIVIDER_START;
      newLedValue = sin( 2 * PI / pulseDivider * timeValueSinceChange - PI / 2 ) * (LED_MAX_VALUE - LED_MIN_VALUE) + LED_MIN_VALUE;
      analogWrite(PIN_LED, newLedValue);

      if(newLedValue < g_previousLedValue) {
        g_mode = EXPIRE;
        g_changeTime = millis();
      }
      g_previousLedValue = (g_mode == EXPIRE) ? LED_MAX_VALUE : newLedValue;
    break;
    case EXPIRE:
      pulseDivider = timeValueSinceStart * (EXPIRE_DIVIDER_END - EXPIRE_DIVIDER_START) / STOP_DELAY_MS + EXPIRE_DIVIDER_START;
      newLedValue = sin( 2 * PI / pulseDivider * timeValueSinceChange + PI / 2 ) * (LED_MAX_VALUE - LED_MIN_VALUE) + LED_MIN_VALUE;
      analogWrite(PIN_LED, newLedValue);

      if(newLedValue > g_previousLedValue) {
        g_mode = INSPIRE;
        g_changeTime = millis();
      }
      g_previousLedValue = (g_mode == INSPIRE) ? 0 : newLedValue;
    break;
  }

  if(timeValueSinceStart > STOP_DELAY_MS) {
    g_mode = OFF;
  }
}
