/*
build for Arduino Nano, max 30720 bytes code, 2048 bytes RAM

           code     RAM
1.0 12/11/2020 4834 15% 290 14% initial deployment
1.1                             reduce threshold from 35C to 30
1.2                             slight tweak to temp on LEDs timing
                                reduce threshold from 30C to 25 until calibration improved

todo:
* piezo not used except powerup @ present, maybe error if temp > threshold

after power-on test of piezo, fan & LEDs,
the temperature is continually measured & displayed on the red & green LEDs
  red LED on
  red LED off
  red LED pulse for each count of temperature "tens" (quick pulse if zero)
  green LED pulse for each count of temperature "tens" (quick pulse if zero)
if temperature >= threshold fan is turned on
if temperature < threshold let fan continue to run for a while then fan is turned off

serial output:
---------------
Inverter fan controller
12 ADC:154 V:0.69 C:19.18
21 ADC:155 V:0.70 C:19.63
31 ADC:170 V:0.76 C:26.37 fan turned on
39 ADC:174 V:0.78 C:28.16
49 ADC:164 V:0.74 C:23.67 fan turned off
56 ADC:148 V:0.66 C:16.48
64 ADC:145 V:0.65 C:15.14
71 ADC:147 V:0.66 C:16.04
79 ADC:149 V:0.67 C:16.93
87 ADC:150 V:0.67 C:17.38
96 ADC:161 V:0.72 C:22.32
102 ADC:158 V:0.71 C:20.98
108 ADC:157 V:0.71 C:20.53
114 ADC:164 V:0.74 C:23.67
121 ADC:172 V:0.77 C:27.27 fan turned on
130 ADC:175 V:0.79 C:28.61
140 ADC:169 V:0.76 C:25.92
148 ADC:165 V:0.74 C:24.12 fan turned off
155 ADC:161 V:0.72 C:22.32
*/
#include <AjhUtils.h>

#define FAN_ON_TEMPERATURE 25
#define FAN_RUN_ON_SECS 10 * 60 // minimum on time once temperature reduces below the threshold
#define TEMPERATURE_CALIBRATION_OFFSET 2 // add this to measurement, although the TMP36 appears to exhibit +/-2C of hysteresis, so of marginal benefit

#define GPIO_PIN_TMP36 0
#define GPIO_PIN_GREEN_LED 2
#define GPIO_PIN_FAN 3
#define GPIO_PIN_PIEZO 4
#define GPIO_PIN_RED_LED 13 // AKA LED_BUILTIN

#define DELAY_BEEP_MS 20
#define DELAY_DSP_TEMP_PULSE_MS 250
#define DELAY_DSP_TEMP_ZERO_MS 50
#define DELAY_DSP_TEMP_INTER_DIGIT_MS 1000
#define DELAY_DSP_TEMP_START_MS 3000
#define DELAY_LOOP_MS 1000

unsigned long lastFanOnSecs = 0;

void setup()
{
  stdSerialInit("Inverter fan controller");
  analogReference(DEFAULT); // ref. https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
  pinMode(GPIO_PIN_FAN, OUTPUT);
  pinMode(GPIO_PIN_GREEN_LED, OUTPUT);
  pinMode(GPIO_PIN_RED_LED, OUTPUT);
  pinMode(GPIO_PIN_PIEZO, OUTPUT);
  powerOnTest();

#if 0
  shortBeep();
  displayTemperatureOnLeds(1);
  displayTemperatureOnLeds(10);
  displayTemperatureOnLeds(25);
  displayTemperatureOnLeds(37);
  displayTemperatureOnLeds(53);
  shortBeep();
#endif
}

void loop()
{
  logTime();
  const uint8_t temperature = getAndLogTemperature();
  displayTemperatureOnLeds(temperature);
  if (temperature >= FAN_ON_TEMPERATURE)
  {
    if (!isFanOn())
    {
      Serial.print(" fan turned on");
      digitalWrite(GPIO_PIN_FAN, HIGH);
    }
    lastFanOnSecs = millis() / 1000;
  }
  else if (millis() / 1000 - lastFanOnSecs >= FAN_RUN_ON_SECS
      && isFanOn())
  {
    Serial.print(" fan turned off");
    digitalWrite(GPIO_PIN_FAN, LOW);
  }
  delay(DELAY_LOOP_MS);
  Serial.println();
}

void displayTemperatureOnLeds(uint8_t temperature)
/*
long red flash then,
red LED for 10s,
green LED for units.
*/
{
  pulseGpio(GPIO_PIN_RED_LED, DELAY_DSP_TEMP_START_MS, DELAY_DSP_TEMP_PULSE_MS);
  pulseLed(GPIO_PIN_RED_LED, temperature / 10);
  delay(DELAY_DSP_TEMP_INTER_DIGIT_MS);
  pulseLed(GPIO_PIN_GREEN_LED, temperature % 10);
}

uint8_t getAndLogTemperature()
/*
If sensor disconnected the voltage will wander up/ around, generally > 80C, but no specific 0 or 1023 value.
*/
{
  const int adc = analogRead(GPIO_PIN_TMP36); // 10-bit --> 0..1023, 100uS per reading
  Serial.print("ADC:");
  Serial.print(adc);

  const float ADC_AREF = 4.6;
  const float voltage = (adc * ADC_AREF) / 1024.0;
  Serial.print(" V:");
  Serial.print(voltage);

  const float temperature = (voltage - 0.5) * 100 + TEMPERATURE_CALIBRATION_OFFSET; // convert from 10 mv per degree with 500 mV offset
  Serial.print(" C:");
  Serial.print(temperature);

  return temperature; // uint8 is good enough resolution
}

bool isFanOn()
{
  return digitalRead(GPIO_PIN_FAN) == 1;
}

void logTime()
{
  Serial.print(millis() / 1000);
  Serial.print(' ');
}

void logPin(uint8_t pin, const char* txt)
{
  Serial.print("pin ");
  Serial.print(pin);
  Serial.print(' ');
  Serial.print(txt);
  Serial.print(" read ");
  Serial.print(digitalRead(pin));
  Serial.print(' ');
}

void powerOnTest()
{
  shortBeep();
  pulseGpio(GPIO_PIN_FAN, 2000, 0);
  const uint16_t millis = 500;
  for (uint8_t x = 0; x < 3; ++x)
  {
    pulseGpio(GPIO_PIN_GREEN_LED, millis, 0);
    pulseGpio(GPIO_PIN_RED_LED,   millis, 0);
  }
  delay(1000);
}

void pulseGpio(uint8_t pin, uint16_t millisHigh, uint16_t millisLow)
{
  digitalWrite(pin, HIGH);
  //logPin(pin, "HIGH");
  delay(millisHigh);
  digitalWrite(pin, LOW);
  //logPin(pin, "LOW");
  delay(millisLow);
}

void pulseLed(uint8_t pin, uint8_t count)
{
  if (count == 0)
  {
    pulseGpio(pin, DELAY_DSP_TEMP_ZERO_MS, DELAY_DSP_TEMP_PULSE_MS); // quick flash represents '0'
    delay(DELAY_DSP_TEMP_PULSE_MS);
  }
  else
  {
    for (uint8_t x = 0; x < count; ++x)
    {
      pulseGpio(pin, DELAY_DSP_TEMP_PULSE_MS, DELAY_DSP_TEMP_PULSE_MS);
    }
  }
}

void shortBeep()
{
  pulseGpio(GPIO_PIN_PIEZO, DELAY_BEEP_MS, 0);
}

#if 0 // changed UI design
uint8_t getLedPulses(uint8_t temperature)
{
  uint8_t x = 0;
  while (temperature < table[x].minTemperature)
  {
    x++;
  }
  return table[x].ledPulses;
}

const struct Table
{
  uint8_t minTemperature;
  uint8_t ledPulses;
} table[] =
{
  20, 1,

  23, 2,
  24, 3,
  25, 4,
  26, 5,
  27, 6,

#if 0
  30, 2,
  35, 3,
  40, 4,
  45, 5,
  50, 6,
  55, 7,
  60, 8
#endif

  255, 10
};
#endif