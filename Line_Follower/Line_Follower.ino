#define F_CPU 16000000UL

#include <avr/io.h>
#include "stdint.h"
#include "util/delay.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#define BATTERY_PIN 0x06
#define Fast_PWM_TOP 15999

#define Calibration_Switch (1 << PD2)
#define U1_RightSensor PC0
#define U2_LeftSensor PC1
#define OB_LED (1 << PB5)
int U1_black_avg = 0;
int U1_white_avg = 0;
int U2_black_avg = 0;
int U2_white_avg = 0;

const int base_speed = 126;
const int8_t max_speed = 127;
const int8_t min_speed = -127;

#define REVERSE_TIME_MS 3000
volatile unsigned long timer_miliseconds = 0;
volatile unsigned long start_time = 0;
volatile bool isTiming = false;

#define EEPROM_ADDR_THRESHOLD1 0x00  // Address for the first sensor threshold
#define EEPROM_ADDR_THRESHOLD2 0x02  // Address for the second sensor threshold


#define Speed_M1_Pin (1 << PB1)
#define Direction_of_Travel_M1_Pin (1 << PB0)

#define Speed_M2_Pin (1 << PB2)
#define Direction_of_Travel_M2_Pin (1 << PB3)


volatile uint8_t ReceivedByte;
volatile bool newByteReceived = false;

volatile uint16_t adcResult = 0;
volatile bool adcResultReady = false;


void InitADC(void);
void warmUpADC(void);
void StartADCSingleConversion(uint8_t channel);
void readBatteryLevel(void);
void InitTimer1(void);
void InitTimer0(void);
int Calibration(uint8_t channel);
void saveThresholdsToEEPROM(int threshold1, int threshold2);
int readThresholdFromEEPROM(uint16_t address);
void clearEEPROM(void);
void setMotorSpeed(int8_t speed1, int8_t speed2);
void setM1Speed(int8_t SpeedAndDirection);
void setM2Speed(int8_t SpeedAndDirection);

int main(void) {
  DDRB |= Speed_M1_Pin | Direction_of_Travel_M1_Pin | Speed_M2_Pin | Direction_of_Travel_M2_Pin | OB_LED;
  DDRD &= ~(Calibration_Switch);
  PORTB &= ~OB_LED;
  PORTD |= Calibration_Switch;
  InitTimer0();
  InitTimer1();
  sei();
  InitADC();
  warmUpADC();
  readBatteryLevel();
  while (!adcResultReady)
    ;
  float battery_level = map(adcResult, 0, 1023, 0, 5);
  adcResultReady = false;
  if (battery_level < 0) {
    for (int i = 0; i < 5; i++) {
      DDRB |= OB_LED;
      PORTB |= OB_LED;
      _delay_ms(250);
      PORTB &= ~OB_LED;
      _delay_ms(200);
    }
    return 0;
  }
  //InitUSART(9600);
  int threshold_1 = readThresholdFromEEPROM(EEPROM_ADDR_THRESHOLD1);
  int threshold_2 = readThresholdFromEEPROM(EEPROM_ADDR_THRESHOLD2);

  if ((threshold_1 == 0xFFFF) || (threshold_2 == 0xFFFF)) {
    while (1) {

      if (!(PIND & Calibration_Switch)) {
        _delay_ms(100);
        while (!(PIND & Calibration_Switch)) {
          PORTB |= OB_LED;  //signifies that calibration process has begun, first we do for black on both sensors!
          U1_black_avg = Calibration(U1_RightSensor);
          U2_black_avg = Calibration(U2_LeftSensor);
          _delay_ms(500);
          PORTB &= ~OB_LED;  //signifies that the calibration process has finished!
          _delay_ms(2000);
        }

        while (PIND & Calibration_Switch)
          ;
        while (!(PIND & Calibration_Switch)) {
          PORTB |= OB_LED;  //signifies that calibration process has begun, first we do for black on both sensors!
          U1_white_avg = Calibration(U1_RightSensor);
          U2_white_avg = Calibration(U2_LeftSensor);
          _delay_ms(500);
          PORTB &= ~OB_LED;  //signifies that the calibration process has finished!
          _delay_ms(2000);
        }

        threshold_1 = (U1_black_avg + U1_white_avg) / 2;

        threshold_2 = (U2_black_avg + U2_white_avg) / 2;

        saveThresholdsToEEPROM(threshold_1, threshold_2);
      }
    }
  } else {
  }
  clearEEPROM();

  while (true) {

    StartADCSingleConversion(U1_RightSensor);
    while (!adcResultReady)
      ;
    int sensor1_right = adcResult;
    adcResultReady = false;
    StartADCSingleConversion(U2_LeftSensor);
    while (!adcResultReady)
      ;
    int sensor2_left = adcResult;
    adcResultReady = false;

    if (sensor1_right < threshold_1 && sensor2_left < threshold_2) {
      if (!isTiming) {
        start_time = timer_miliseconds;  // Start timing
        isTiming = true;
      } else if ((timer_miliseconds - start_time) >= REVERSE_TIME_MS) {
        while (!((sensor1_right > threshold_1) || (sensor2_left > threshold_2))) {
          setMotorSpeed(-base_speed, -base_speed);  // Reverse
          StartADCSingleConversion(U1_RightSensor);
          while (!adcResultReady)
            ;
          sensor1_right = adcResult;
          adcResultReady = false;
          StartADCSingleConversion(U2_LeftSensor);
          while (!adcResultReady)
            ;
          sensor2_left = adcResult;
          adcResultReady = false;
          isTiming = false;
        }
        
      }
    } else if ((sensor1_right > threshold_1) && (sensor2_left > threshold_2)) {
      setMotorSpeed(base_speed, base_speed);
    } else if ((sensor1_right > threshold_1) && (sensor2_left < threshold_2)) {
      setMotorSpeed(0, base_speed);
    } else if ((sensor1_right < threshold_1) && (sensor2_left > threshold_2)) {
      setMotorSpeed(base_speed, 0);
    }
  }
}

void InitADC(void) {
  ADMUX = (1 << REFS0);  //reference voltage internal VCC of 5V

  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);  //prescale division factor 128, ADC clock 125kHz sampling rate 9.6kHz
  ADCSRA |= (1 << ADEN);                                               //enable ADC
}

void warmUpADC(void) {
  ADCSRA |= (1 << ADSC);  // Start an ADC conversion
  while (ADCSRA & (1 << ADSC))
    ;                    // Wait until the conversion is complete
  uint16_t dummy = ADC;  // Read and discard the ADC value to clear the ADC interrupt flag
}

void StartADCSingleConversion(uint8_t channel) {
  const uint8_t kMuxMask = ((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
  ADMUX &= ~kMuxMask;  //clear mux
  //DIDR0 |= (1 << channel);
  ADMUX |= channel;  //set input channel

  ADCSRA |= (1 << ADSC);  //start first single conversion
}

ISR(ADC_vect) {
  adcResult = ADC;
  adcResultReady = true;
}

void readBatteryLevel(void) {
  const uint8_t kMuxMask = ((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
  ADMUX &= ~kMuxMask;    //clear mux
  ADMUX |= BATTERY_PIN;  // select the correct channel
  ADCSRA |= (1 << ADSC);
}

void InitTimer1(void) {
  ICR1 = Fast_PWM_TOP;

  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  //Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode); Fast PWM Mode; and a prescaler of 8
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
}

void InitTimer0(void) {
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;

  TCCR0B |= (1 << CS01) | (1 << CS00);  //prescaler of 64 to get an overflow at 1ms
  TIMSK0 |= (1 << TOIE0);
}

ISR(TIMER0_OVF_vect) {
  timer_miliseconds++;
}

void setM1Speed(int8_t SpeedAndDirection) {
  if (!(PORTB & Speed_M1_Pin)) {
    PORTB |= Speed_M1_Pin;
  }

  if (SpeedAndDirection > 0) {
    PORTB |= Direction_of_Travel_M1_Pin;
  }

  if (SpeedAndDirection < 0) {
    PORTB &= ~Direction_of_Travel_M1_Pin;
  }

  int speed_and_direction = abs(SpeedAndDirection);
  OCR1A = map(speed_and_direction, 0, 127, 0, Fast_PWM_TOP);
}

void setM2Speed(int8_t SpeedAndDirection) {
  if (!(PORTB & Speed_M2_Pin)) {
    PORTB |= Speed_M2_Pin;
  }

  if (SpeedAndDirection > 0) {
    PORTB |= Direction_of_Travel_M2_Pin;
  }

  if (SpeedAndDirection < 0) {
    PORTB &= ~Direction_of_Travel_M2_Pin;
  }

  int speed_and_direction = abs(SpeedAndDirection);
  OCR1B = map(speed_and_direction, 0, 127, 0, Fast_PWM_TOP);
}

void setMotorSpeed(int8_t speed1, int8_t speed2) {
  setM1Speed(speed1);
  setM2Speed(speed2);
}

int Calibration(uint8_t channel) {
  int sum_readings = 0;

  for (int i = 0; i < 10; i++) {
    StartADCSingleConversion(channel);
    while (!adcResultReady)
      ;
    sum_readings += ADC;
    adcResultReady = false;
    _delay_ms(100);
  }
  return sum_readings / 10;
}

void saveThresholdsToEEPROM(int threshold1, int threshold2) {
  eeprom_update_word((uint16_t*)EEPROM_ADDR_THRESHOLD1, (uint16_t)threshold1);
  eeprom_update_word((uint16_t*)EEPROM_ADDR_THRESHOLD2, (uint16_t)threshold2);
}

int readThresholdFromEEPROM(uint16_t address) {
  return eeprom_read_word((const uint16_t*)address);
}

void clearEEPROM(void) {
  uint16_t eeprom_size = 1024;
  for (uint16_t i = 0; i < eeprom_size; i++) {
    eeprom_update_byte((uint8_t*)i, 0xFF);  //update each byte to 0xFF
  }
}
