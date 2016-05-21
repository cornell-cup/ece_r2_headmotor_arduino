
/*
 * File:   main.c
 * Author: Syed Tahmid Mahbub and Laura Ng
 *
 * Created on April 28, 2016
 */


// PIC 32MX250F128B

#include <stdint.h>
#include <Servo.h>
#include <Encoder.h>
#include "math.h"

#define PIN_HEAD 10
#define PIN_DRAWER1 5
#define PIN_DRAWER2 6

Servo head;
Servo drawer1;
Servo drawer2;

int oldAngle;
int correction;
int corrNum;
Encoder myEnc(2, 3);

void setup() {
  //initialize PWM
  head.attach(PIN_HEAD);
  //drawer1.attach(PIN_DRAWER1);
  //drawer2.attach(PIN_DRAWER2);

  //initialize Serial/UART
  Serial.begin(9600);
  while(!Serial);

  //initialize each motor in their resting position
  head.writeMicroseconds(1500);        //1000-2000, 4.17rev; midpt = 1500us
  delay(10000); //10s
  oldAngle = 180;
  correction = 0;
  corrNum = 0;
  myEnc.write(4096);
  
  //drawer1.writeMicroseconds(0);
  //drawer2.writeMicroseconds(0);
  
  Serial.println("\n\rServo test program for R2\n\r");
}

void loop() {
    uint16_t rxdat;
    float head_dc;
    int encAngle = myEnc.read() * (float)360 / (float)8192 + correction;
    Serial.print("encAngle: ");
    Serial.println(encAngle);

    //blocks any new angle command if we still haven't gotten to old angle
    if ((encAngle-oldAngle > 45 || oldAngle-encAngle > 45) && corrNum < 5) {             //tolerance = 45deg
      correction = oldAngle - encAngle;
      rxdat = oldAngle + correction;

      Serial.print("rxdat correct = ");
      Serial.println(rxdat);
      move(rxdat);
      corrNum++;
    }
    else {
      corrNum = 0;
      rxdat = serial_get_angle(); //blocks until receive command
      rxdat = rxdat + correction;   //keep corrections consistent for future commands
      oldAngle = rxdat;
      
      Serial.print("rxdat = ");
      Serial.println(rxdat);      
      move(rxdat);
    }
}

uint8_t serial_get_byte(void){
  while(Serial.available() <= 0);
  uint8_t i = (uint8_t)Serial.read();
  return i;
}

int serial_get_angle(){
    #define BUFMAX  2           //limit input to 2 characters
    uint8_t rxchar;
    uint8_t rxbuf[BUFMAX+1];
    uint8_t idx;
    
    uint16_t channel;    //current code does not make use of channel or negative yet
    int negative;       //neg = 1, pos = 0
    uint16_t angle;
    
    rxchar = serial_get_byte();
    
    //divide by 16 to get MSB 4 bits of first byte = channel[4]
    channel = rxchar / 16;
    rxchar = rxchar % 16;   //trim off 4 bits
    if (rxchar / 8)
        negative = 1;
    else
        negative = 0;
    rxchar = rxchar % 8;    //trim to 3 bits
    uint16_t temp = rxchar;      //temporarily store 3 MSB of angle data

    //retrieve second byte, containing rest of message
    rxchar = serial_get_byte();
    
    angle = (temp << 8) | rxchar;
        
    //apply negative/positive sign
    //if (negative) angle = angle * -1;
        
    return angle;
}

void move(int rxdat) {
    if (rxdat > 360) rxdat = 360;             
    float x = 1050 + (rxdat * (float)900) / (float)360;
    Serial.print("raw = ");
    Serial.println(x);
    head.writeMicroseconds((int)(x));        //rxdat*total_range/360
    delay(10000);   //allow 8s for head to try to reach rxdat angle
}

