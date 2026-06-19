#define F_CPU 16000000UL

#include<SoftwareSerial.h>
#include<avr/io.h>
#include"stdint.h"
#include"util/delay.h"

SoftwareSerial mySerial(7,6);// 7-Ardino Rx, 6-Arduino Tx

#define OnBoard_LED (1<<PB5)
#define S1_RightSensor PC0
#define S2_LeftSensor PC1
#define Switch (1<<PD2)

#define Speed_Motor1_Pin (1 << PB1)
#define Direction_M1_Pin (1 << PB0)

#define Speed_Motor2_Pin (1 << PB2)
#define Direction_M2_Pin (1 << PB3)

char Incoming_value = 0;
void InitT1(void);
void Speed_M1(int8_t Speed_Direction);
void Speed_M2(int8_t Speed_Direction);
void Motor_Speed(int8_t S1, int8_t S2);

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(5, OUTPUT);
  DDRB |= Speed_Motor1_Pin | Speed_Motor2_Pin | Direction_M1_Pin | Direction_M2_Pin;
}

void loop() {
    digitalWrite(5,   HIGH);
   if(mySerial.available() > 0){  // with D9 as Rx and D10 as Tx
     Incoming_value = mySerial.read();
      if(Incoming_value == '4'){
         Motor_Speed(50,50);
      }
      else if(Incoming_value == '3'){
        Motor_Speed(-50,-50);
      }
      else if(Incoming_value == '1'){
        Motor_Speed(-50,50); 
      }
      else if(Incoming_value == '2'){
        Motor_Speed(50,-50);
      }
   }
}

void InitT1(void){
   ICR1 = 15999;  //Fast PWM Top  (125Hz) 16000000/8/125 -1
   //Setting Fast PWM mode
   TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(WGM11);
   //Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM
   TCCR1B |= (1<<WGM12)|(1<<WGM13)|(1<<CS11);//Prescalar of 8
}
void Speed_M1(int8_t Speed_Direction){
   float Speed = abs(Speed_Direction);
   OCR1A = map(Speed,0,100,0,15999);
   if(Speed_Direction > 0){
    PORTB |= Speed_Motor1_Pin;
    PORTB |= Direction_M1_Pin; //Forward
   } 
   if(Speed_Direction < 0){
    PORTB |= Speed_Motor1_Pin;
    PORTB &= ~Direction_M1_Pin; //Backward
   }
}
void Speed_M2(int8_t Speed_Direction){
   float Speed = abs(Speed_Direction);
   OCR1B = map(Speed,0,100,0,15999);
   if(Speed_Direction > 0){
    PORTB |= Speed_Motor2_Pin;
    PORTB |= Direction_M2_Pin; //Forward
   } 
   if(Speed_Direction < 0){
    PORTB |= Speed_Motor2_Pin;
    PORTB &= ~Direction_M2_Pin; //Backward
   }
}
void Motor_Speed(int8_t S1, int8_t S2){
  Speed_M1(S1);
  Speed_M2(S2);
}