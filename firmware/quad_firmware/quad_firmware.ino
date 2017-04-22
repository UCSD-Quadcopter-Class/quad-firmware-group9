#include <radio.h>

#define MOTOR_PIN 3

char buff[5];
int number = 0;
int count = 0;
int y=0;
unsigned long t;

void setup() {
  // put your setup code here, to run once:  
  const int RADIO_CHANNEL = 11;        // Channel for radio communications (can be 11-26)
  const int SERIAL_BAUD = 9600;        // Baud rate for serial port 
  const int SERIAL1_BAUD = 9600;     // Baud rate for serial1 port

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL1_BAUD);

  rfBegin(RADIO_CHANNEL);
  
  pinMode(MOTOR_PIN, OUTPUT);
  t = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (rfAvailable()){
    uint8_t c = rfRead();
    if(count == 0){
      number = c;
      number = number<<8;
      count++;
    }
    else {
      count = 0;
      number |= c;
    } 
    t = millis(); 
  }
  if ((millis()-t) > 300){
    number = 0;
  }
//  if (count == 0){
//    Serial.println(number); 
//  }
  motor(number);
//  delay(20);
}

void motor(int speed){
  if (speed < 160 || speed > 820){
    analogWrite(MOTOR_PIN, 0);
    Serial.println("0");
  }
  else {
    y = map(speed,150,825,0,200);
    analogWrite(MOTOR_PIN, y);
    Serial.println(y);
  }
}

