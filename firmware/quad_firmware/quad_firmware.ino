#include <radio.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // Backend for Adafruit LSM9DS1 and Simple_AHRS
#include <Adafruit_Simple_AHRS.h>

#define MOTOR1_PIN 4
#define MOTOR2_PIN 5
#define MOTOR3_PIN 8
#define MOTOR4_PIN 9
#define P_OFFSET 0

/**------- SENSOR DECLARATIONS --------**/
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getGyro(), true); // true used to disambiguate between original and personal versions

/**----- USED FOR CONTROLLER RAW RECEIVING -------**/
char index = 0;
char count = 0;
int num_received = 0;
int raw_input[8] = {0,0,0,0,0,0,0,0}; // raw controller input

/**
 * NOTE: ANGULAR ACCELERATION IDEALLY ABOUT 50deg/sec
 * ROLL/PITCH RECEIVE ANGLE INPUTS OF MAX 45 deg
 */

/** PID CONSTANTS **/
//int filteredp;
//float phistory[5];
//int filteredr;
//float rhistory[5];
//int BOX_SIZE = 5;
float pkp = 1.4;
float pki = 0.25;
float pkd = 0.2;  // 1.1 - 0.2 - 0.3
float compVal = 0;
 
/**----- FINAL CALCULATION PLACING --------**/
uint8_t motors[4] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};
unsigned int motor_speeds[4] = {0,0,0,0};
unsigned int adjusted_speeds[4] = {0,0,0,0};  // values to put onto motors
unsigned int MIN_THROTTLE = 50;

/**------ MISC. TIMINGS ------**/
unsigned long remote_time;
unsigned long debug_time;
uint16_t dtMeasure;

/** FUNCTION DECLARATIONS **/
void setupSensor();
void export_debug(sensors_vec_t orientation);
void check_remote();
void control();
bool isin(uint8_t c);
char indexof(uint8_t c);
void pid(sensors_vec_t orientation);
void switchModes();
void complimentaryFilter(sensors_vec_t* orientation);

//??????
void motor_speed();
void roll(float val);
void pitch(float val);
void yaw(float val);
int smoothp(float val);
int smoothr(float val);

/************************ SETUP AND LOOP ************************/
void setup() {
  // Set up communication interfaces
  const int RADIO_CHANNEL = 21;        // Channel for radio communications (can be 11-26)
  const int SERIAL_BAUD = 9600;        // Baud rate for serial port 
  const int SERIAL1_BAUD = 9600;     // Baud rate for serial1 port

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL1_BAUD);
  rfBegin(RADIO_CHANNEL);

  // Initialize pins
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  // Initialize and setup sensor gain and measuring frequency
  if(!lsm.begin()) {
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  setupSensor();

  // Initialize start times
  remote_time = millis();
  debug_time = millis();
  dtMeasure = millis();
}

void loop() {
  sensors_vec_t orientation;  // reinitialize on every loop

  // MAIN CONTROLLER
  control();  // Update any raw controller values
  switchModes();
  ahrs.getRollPitchYaw(&orientation);  // Get all necessary sensor values
  complimentaryFilter(&orientation);  //  filter the input data

  // smoothing
  filteredp = smoothp(orientation.pitch);
  filteredr = smoothr(orientation.roll);
  
//  pid(orientation);  // Calculate PID values using received values
  if(raw_input[6] == 0 && raw_input[7] == 1)  // button determines if motors will update
    motor_speed();  // Update motor values as necessary
  check_remote(); // shuts of all inputs if no inputs remote inputs are given

  // DEBUG OUTPUT ONLY
  if(debug_time+20 <= millis()) {
    export_debug(orientation);
    debug_time = millis();
  }
}

/*********************************************** HELPER METHODS *****************************************/ 
void complimentaryFilter(sensors_vec_t* orientation) {
  // compliment pitch data with gyro data
  orientation->pitch = compVal*(orientation->pitch+orientation->gyro.y*dtMeasure) + (1-compVal)*orientation->pitch;

  // compliment roll data with gyro data
  orientation->roll = compVal*(orientation->roll+orientation->gyro.x*dtMeasure) + (1-compVal)*orientation->roll;
}

void switchModes() {
  return;
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void export_debug(sensors_vec_t orientation) {
    Serial.print((int)orientation.roll);
    Serial.print(" ");
//    Serial.print(filteredr);
//    Serial.print(" ");
//    Serial.print(filteredp);
//    Serial.print(" ");
    Serial.print((int)orientation.pitch);
//    Serial.print(" ");
//    Serial.print((int)orientation.yaw.xlx);
//    Serial.print(" ");
//    Serial.print((int)orientation.yaw.gyrox);
    Serial.print("\n");
}

void check_remote() {
    if ((millis()-remote_time) > 1000){
    for (char i = 0; i < 8; i++){
      raw_input[i] = 0;
    }
  }  
}

void control(){
  if (rfAvailable()){
    uint8_t c = rfRead();
    if (!isin(c)) {
      if(index != 0 && index != 4 && index != 5) {  //others sent as uint8 type
        num_received = (int8_t)c;
        raw_input[index] = num_received;
      }
      if(count == 0){
        num_received = c;
        num_received = num_received<<8;
        count++;
      }
      else {
        count = 0;
        num_received |= c;
        if (index < 8){
          raw_input[index] = num_received;
        }
      }
    }
    else {
      index = indexof(c);
      count = 0;
    }

//    VISUALIZATION OF INPUT
//    if(index ==0){
//      for (char i=0; i < 8; i++) {
//        Serial.print(raw_input[i]);
//        Serial.print(" ");
//      }
//      Serial.println("");
//    }

    remote_time = millis();
  }

   // Supposing that throttle changes, this is a good place to change the motor values
   for(char i = 0; i<4; i++) {
    motor_speeds[i] = raw_input[0]; // sets default value (changes due to PID)
    adjusted_speeds[i] = raw_input[0];
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

/**
 * 0 - BL
 * 1 - BR
 * 2 - TL
 * 3 - TR
 */
void motor_speed(){
  for(char i=0; i<4; i++) {
    analogWrite(motors[i], adjusted_speeds[i]);
  }
}

// Modifies the current pitch to suit desired parameters.
void pitch(float pspeed){
  int toChange = abs((int)pspeed);

  if(raw_input[0] < MIN_THROTTLE) {
    return;
  } else {
    if(pspeed > 2) {
      adjusted_speeds[0] = motor_speeds[0] - toChange;
//      adjusted_speeds[3] = motor_speeds[3] + toChange;
      adjusted_speeds[1] = motor_speeds[1] + toChange;
    } else if (pspeed < -2) {
      adjusted_speeds[0] = motor_speeds[0] + toChange;
//      adjusted_speeds[3] = motor_speeds[3] - toChange; 
      adjusted_speeds[1] = motor_speeds[1] - toChange;
    }
  }
}

void roll(float rspeed){
  //same as pitch
}

void yaw(float yspeed) {
  //raise speed of 2 diagonal motors
}

//int smoothr(float rIn){
//  static int ind;
//  static float toLeave;
//  static float sum;
//
//  //reset counter
//  if(ind == BOX_SIZE)
//    ind = 0;
//
//  //otherwise add new value
//  rhistory[ind] = rIn;
//
//  // take weighted average of the items
//  sum -= toLeave-rIn;
//  ind++;
//  toLeave = rhistory[(ind+1)%BOX_SIZE];
//  
//  return sum/BOX_SIZE;
//}
//
//int smoothp(float pIn){
//  static int ind;
//  static float toLeave;
//  static float sum;
//
//  //reset counter
//  if(ind == BOX_SIZE)
//    ind = 0;
//
//  //otherwise add new value
//  phistory[ind] = pIn;
//
//  // take weighted average of the items
//  sum -= toLeave-pIn;
//  ind++;
//  toLeave = phistory[(ind+1)%BOX_SIZE];
//  
//  return sum/BOX_SIZE;
//}

void pid(sensors_vec_t orientation){
  static float prev_error;
  static float pintegral;
  
  int ptarget = raw_input[2];
  float psensor = orientation.pitch - P_OFFSET;
  float pierror = (float)ptarget - psensor;  // proportional
  
  if(abs(pierror) > 20)
    pintegral = 0;
  if(pintegral > 70) // Set hard limits on integral
    pintegral = 70;
  else if(pintegral < -70)
    pintegral = -70;

  float pderivative = (orientation.pitch - prev_error)/(dtMeasure);  // derivative

  float pspeed = (pkp*pierror) + (pintegral) + (pkd*pderivative); // final value of sum
  if(pspeed > 70)  // hard limit on pspeed
    pspeed = 70;
  else if(pspeed < -70)
    pspeed = -70;
  pitch(pspeed);
  Serial.println((int)(pierror));
//  if(debug_time+100 <= millis()) {
//    Serial.println((int)psensor);
//    debug_time = millis();
//  }
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
}

