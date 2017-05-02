#include <radio.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_Simple_AHRS.h>

#define MOTOR1_PIN 4
#define MOTOR2_PIN 5
#define MOTOR3_PIN 8
#define MOTOR4_PIN 9

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
char buff[5];
char index = 0;
int number = 0;
char count = 0;
unsigned long t;
int numbers[8] = {0,0,0,0,0,0,0,0};

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void setup() {
  // put your setup code here, to run once:  
  const int RADIO_CHANNEL = 26;        // Channel for radio communications (can be 11-26)
  const int SERIAL_BAUD = 9600;        // Baud rate for serial port 
  const int SERIAL1_BAUD = 9600;     // Baud rate for serial1 port

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL1_BAUD);

  rfBegin(RADIO_CHANNEL);
  
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  setupSensor();
  t = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_vec_t orientation;
  if(ahrs.getOrientation(&orientation)){
    pid();
  }
  control();
  motor_speed();
  
  if ((millis()-t) > 1000){
    for (char i = 0; i < 8; i++){
      numbers[i] = 0;
    }
  }
}

//extra methods
void control(){
  if (rfAvailable()){
    uint8_t c = rfRead();
    if (!isin(c)) {
      if(count == 0){
        number = c;
        number = number<<8;
        count++;
      }
      else {
        count = 0;
        number |= c;
        if (index < 8){
          numbers[index] = number;
        }
      }
    }
    else {
      index = indexof(c);
      count = 0;
    }
//    if(index ==0){
//      for (char i=0; i < 8; i++) {
//        Serial.print(numbers[i]);
//        Serial.print(" ");
//      }
//      Serial.println("");
//    }
    t = millis();
  } 
}
bool isin(uint8_t c){
  if (c=='A'||c=='B'||c=='C'||c=='D'||c=='E'||c=='F'||c=='G'||c=='H'){
    return true;
  }
  return false;
}
char indexof(uint8_t c){
  if(c=='A'){return 0;}
  else if(c=='B'){return 1;}
  else if(c=='C'){return 2;}
  else if(c=='D'){return 3;}
  else if(c=='E'){return 4;}
  else if(c=='F'){return 5;}
  else if(c=='G'){return 6;}
  else if(c=='H'){return 7;}
  else {return 8;}
}

void motor(int speed, int channel){
  if (speed < 140 || speed > 820){
    analogWrite(channel, 0);
    if(channel == 4){
      Serial.println("0");
    }
  }
  else {
    int y = map(speed,140,825,40,255);
    analogWrite(channel, y);
    if (channel==4){
      Serial.println(y);
    }
  }
}
void motor_speed(){
  //change speed of all 4 motors
  motor(numbers[0],MOTOR1_PIN);
  motor(numbers[0],MOTOR2_PIN);
  motor(numbers[0],MOTOR3_PIN);
  motor(numbers[0],MOTOR4_PIN);
}
void pitch(){
  //raise or lower 2 motors
}
void roll(){
  //same as pitch
}
void yaw() {
  //raise speed of 2 diagonal motors
}

//void pid(){
//  perror = ptarget - psensor;
//  pintegral = (pintegral/2) + perror;
//  if(perror == 0){
//    pintegral = 0; 
//  }
//  if(abs(perror) > 40){
//    pintegral = 0;
//  }
//  pderivative = perror - pprev_error;
//  pprev_error = perror;
//  pspeed = (kp*perror) + (ki*pintegral) + (kd+pderivative);

//  rerror = rtarget - rsensor;
//  rintegral = (integral/2) + rerror;
//  if(rerror == 0){
//    rintegral = 0; 
//  }
//  if(abs(rerror) > 40){
//    rintegral = 0;
//  }
//  rderivative = rerror - rprev_error;
//  rprev_error = rerror;
//  rspeed = (kp*rerror) + (ki*rintegral) + (kd+rderivative);

//  yerror = ytarget - ysensor;
//  yintegral = (yintegral/2) + yerror;
//  if(yerror == 0){
//    yintegral = 0; 
//  }
//  if(abs(yerror) > 40){
//    yintegral = 0;
//  }
//  yderivative = yerror - yprev_error;
//  yprev_error = yerror;
//  yspeed = (kp*yerror) + (ki*yintegral) + (kd+yderivative);
//}

