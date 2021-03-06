// DEFAULT:
// 0.03 = CF VAL
// 6 = Kpp
// 1.7 = Kpi
// 2.1 = Kpd

#include <radio.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // Backend for Adafruit LSM9DS1 and Simple_AHRS
#include <Adafruit_Simple_AHRS.h>

#define MOTOR1_PIN 4  //TL
#define MOTOR2_PIN 5  //TR
#define MOTOR3_PIN 8  //BL
#define MOTOR4_PIN 9  //BR
#define BOX_SIZE 5  // gyro box filter size
#define Z_BOX_SIZE 16 // z axis gyro box size

/**------- SENSOR DECLARATIONS --------**/
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getGyro(), true); // true used to disambiguate between original and personal versions

/**----- USED FOR CONTROLLER RAW RECEIVING -------**/
char index = 0;
char count = 0;
int num_received = 0;
int raw_input[8] = {0,0,0,0,0,0,0,0}; // raw controller input
bool POTflag = false;
bool POTproc = false;
uint8_t POTsig = 0;
float POTval = 0;
float compPAngle;
float compRAngle;

/**
 * NOTE: ANGULAR ACCELERATION IDEALLY ABOUT 50deg/sec
 * ROLL/PITCH RECEIVE ANGLE INPUTS OF MAX 45 deg
 */

/** FILTER CONSTANTS **/
float compVal = 0.05;
float boxYSumArray[BOX_SIZE];
float boxXSumArray[BOX_SIZE];
float boxZSumArray[Z_BOX_SIZE];

/** PID CONSTANTS **/
float kpp = 0;
float kpi = 0;
float kpd = 0;
float krp = 0;
float kri = 0;
float krd = 0;
float kyp = 0;
float kyi = 0;
float kyd = 0;
float P_OFFSET;
float R_OFFSET;
float Y_OFFSET;
 
/**----- FINAL CALCULATION PLACING --------**/
uint8_t motors[4] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};
int motor_speeds[4] = {0,0,0,0};
int adjusted_speeds[4] = {0,0,0,0};  // values to put onto motors
unsigned int MIN_THROTTLE = 50;

/**------ MISC. TIMINGS ------**/
unsigned long remote_time;
unsigned long debug_time;
unsigned long PIDtime;
unsigned long compdt;
float dt = .01;

/** FUNCTION DECLARATIONS **/
void setupSensor();
void export_debug(sensors_vec_t orientation);
void check_remote();
void control();
bool isin(uint8_t c);
char indexof(uint8_t c);
void pid(sensors_vec_t orientation);
void complimentaryFilter(sensors_vec_t* orientation);
void gyroYBoxFilter(sensors_vec_t* orientation);
void gyroXBoxFilter(sensors_vec_t* orientation);
void gyroZBoxFilter(sensors_vec_t* orientation);
void processPot();
void motor_speed();
void roll(float val);
void pitch(float val);
void yaw(float val);

/************************ SETUP AND LOOP ************************/
void setup() {
  // Set up communication interfaces
  const int RADIO_CHANNEL = 13;        // Channel for radio communications (can be 11-26)
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
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  setupSensor();

  // Initialize start times 
  remote_time = millis();
  debug_time = millis();
  PIDtime = millis();
  compdt = millis();

  // Calibrate on startup
  calibrateInit();
}

void loop() {
  sensors_vec_t orientation;  // reinitialize on every loop

  // MAIN CONTROLLER
  control();  // Update any raw controller values
  ahrs.getRollPitchYaw(&orientation);  // Get all necessary sensor values

  // First filter the data
  complimentaryFilter(&orientation);
  gyroYBoxFilter(&orientation);
  gyroXBoxFilter(&orientation);
  gyroZBoxFilter(&orientation);

  if(PIDtime+10<=millis()) {
    pid(orientation);  // Calculate PID values using received values every 10 ms
    PIDtime = millis();
    motor_speed();  // Update motor values as necessary
  }
  
  check_remote(); // shuts of all inputs if no inputs remote inputs are given
}

/*********************************************** HELPER METHODS *****************************************/ 
void complimentaryFilter(sensors_vec_t* orientation) {
  float tempdt = (millis()-compdt)/1000.0;
  if(compVal >= 0 && compVal <=1) {
    // compliment pitch data with gyro data
    compPAngle = (1-compVal)*(compPAngle+orientation->gyro.y*tempdt) + compVal*orientation->pitch;
    // compliment roll data with gyro data
    compRAngle = (1-compVal)*(compRAngle+orientation->gyro.x*tempdt) + compVal*orientation->roll;
  } else {
    // compensated angles will not change with garbage inputs
  }
  compdt = millis();
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

void check_remote() {
    if ((millis()-remote_time) > 1000){
    for (char i = 0; i < 8; i++){
      raw_input[i] = 0;
    }
  }  
}

void calibrateInit() {
  // Signal acknowledged
  for(int i=0; i<10; i++) {
    rfWrite('A');
    delay(10);
  }

  sensors_vec_t orientation;
  // Perform self-calibration
  for(int i=0; i<250; i++) {
    ahrs.getRollPitchYaw(&orientation);  // Get all necessary sensor values
    
    complimentaryFilter(&orientation);  // And filter them
    gyroYBoxFilter(&orientation);
    gyroXBoxFilter(&orientation);
    gyroZBoxFilter(&orientation);
  }
  P_OFFSET = compPAngle;
  R_OFFSET = compRAngle;
  Y_OFFSET = orientation.gyro.z;

  // Signal done
  for(int i=0; i<10; i++) {
    rfWrite((char)'D');
    delay(10);
  }
}

void control(){
  if (rfAvailable()){
    uint8_t c = rfRead();
    if (!isin(c)) {
      if(index != 4) {  //others sent as uint8 type
        num_received = (int8_t)c;
        if(index == 0)
          raw_input[index] = (uint8_t)c;
        else
          raw_input[index] = num_received;
      } else if(index == 4 && POTflag == false) {
        // pot values sent as 8bit uint + 16bit int for the floating part
        POTsig = c;  // Passes integer portion into this var
        POTflag = true;
      } else if(count == 0){
        num_received = c;
        num_received = num_received<<8;
        count++;
      } else {
        count = 0;
        num_received |= c;
        POTproc = true;
        if (index < 8)
          raw_input[index] = num_received;
      }
    } else {
      if(c == 'Z') {
        if(rfRead() == 'C')
          calibrateInit();
      }
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
    if(POTproc && count == 0)
      processPot();
  }

   // Supposing that throttle changes, this is a good place to change the default motor values
   for(char i = 0; i<4; i++) {
    motor_speeds[i] = raw_input[0]; // sets default value
    adjusted_speeds[i] = raw_input[0];  // default init PID value
   }
}

void processPot() {
  POTflag = false;
  POTproc = false;
  POTval = (float)POTsig + (float)raw_input[4]/10000.0;
  switch(raw_input[7]) {  // Process the desired input
    case 0:
      kpp=POTval; break;
    case 1:
      kpi=POTval; break;
    case 2:
      kpd=POTval; break;
    case 3:
      krp=POTval; break;
    case 4:
      kri=POTval; break;
    case 5:
      krd=POTval; break;
    case 6:
      kyp=POTval; break;
    case 7:
      kyi=POTval; break;
    case 8:
      kyd=POTval; break;
    case 9:
      if(POTval < 1)
        compVal=POTval;
      break;
    default:
      break;
  }
}

bool isin(uint8_t c){
  if (c=='A'||c=='B'||c=='C'||c=='D'||c=='E'||c=='F'||c=='G'||c=='H'||c=='Z'){
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

void motor_speed(){
  for(char i=0; i<4; i++) {
    if(adjusted_speeds[i]<=20)
      analogWrite(motors[i], 0);
    else {
      if(i == 0 || i == 1) 
        analogWrite(motors[i], limitThrottleVals((adjusted_speeds[i])));
      else
        analogWrite(motors[i], limitThrottleVals(adjusted_speeds[i]));
    }
  }
}

void pitch(float pspeed){
  int toChange = abs((int)pspeed/2);

  if(raw_input[0] < MIN_THROTTLE) {
    return;
  } else {
    if(pspeed > 3) {
      // Error is positive -> Need to pitch forward
      adjusted_speeds[0] -= toChange;
      adjusted_speeds[1] -= toChange;
      adjusted_speeds[2] += toChange;
      adjusted_speeds[3] += toChange;
    } else if (pspeed < -3) {
      // Error is negative -> Need to pitch backwards
      adjusted_speeds[0] += toChange;
      adjusted_speeds[1] += toChange;
      adjusted_speeds[2] -= toChange;
      adjusted_speeds[3] -= toChange;
    }
  }
}

void roll(float rspeed){
  int toChange = abs((int)rspeed/2);

  if(raw_input[0] < MIN_THROTTLE) {
    return;
  } else {
    if(rspeed > 3) {
      // Error is positive -> Need to roll to the right
      adjusted_speeds[0] += toChange;
      adjusted_speeds[1] -= toChange;
      adjusted_speeds[2] += toChange;
      adjusted_speeds[3] -= toChange;
    } else if(rspeed < -3) {
      // Error is negative -> Need to roll to the left
      adjusted_speeds[0] -= toChange;
      adjusted_speeds[1] += toChange;
      adjusted_speeds[2] -= toChange;
      adjusted_speeds[3] += toChange;
    }
  }
}

void yaw(float yspeed) {
  int toChange = abs((int)yspeed/2);

  if(raw_input[0] < MIN_THROTTLE) {
    return;
  } else {
    if(yspeed > 3) {
      // Error is positive -> Need to turn clockwise
      adjusted_speeds[0] -= toChange;
      adjusted_speeds[1] += toChange;
      adjusted_speeds[2] += toChange;
      adjusted_speeds[3] -= toChange;
    } else if(yspeed < -3) {
      // Error is negative -> Need to turn cclockwise
      adjusted_speeds[0] += toChange;
      adjusted_speeds[1] -= toChange;
      adjusted_speeds[2] -= toChange;
      adjusted_speeds[3] += toChange;
    }
  }
}

int limitThrottleVals(int currThrottle) {
  if(currThrottle <= 0)
    return 0;
  else if(currThrottle >= 255)
    return 255;
  else
    return currThrottle;
}

void gyroYBoxFilter(sensors_vec_t* orientation) {
  float currVal = orientation->gyro.y;
  static float boxSum;
  static int i;

  boxYSumArray[i] = currVal;
  boxSum += (currVal-boxYSumArray[((i+1)%BOX_SIZE)])/BOX_SIZE;
  i = (i+1)%BOX_SIZE;

  // Finally adjust the actual value
  orientation->gyro.y = boxSum;
}

void gyroXBoxFilter(sensors_vec_t* orientation) {
  float currVal = orientation->gyro.x;
  static float boxSum;
  static int i;

  boxXSumArray[i] = currVal;
  boxSum += (currVal-boxXSumArray[((i+1)%BOX_SIZE)])/BOX_SIZE;
  i = (i+1)%BOX_SIZE;

  // Finally adjust actual value
  orientation->gyro.x = boxSum;
}

void gyroZBoxFilter(sensors_vec_t* orientation) {
  float currVal = orientation->gyro.z;
  static float boxSum;
  static int i;

  boxZSumArray[i] = currVal;
  boxSum += (currVal-boxZSumArray[((i+1)%Z_BOX_SIZE)])/Z_BOX_SIZE;
  i = (i+1)%Z_BOX_SIZE;

  // Finally adjust actual value
  orientation->gyro.z = boxSum;
}

void pid(sensors_vec_t orientation){
  // PITCH VALUES
  static float pprev_input;
  static float pintegral;
  int ptarget = raw_input[2];
  float psensor = compPAngle - P_OFFSET;
  float pierror = (float)ptarget - psensor;  // proportional

  pintegral += kpi*pierror*dt;  // integral
  if(pintegral > 100) {
    pintegral = 100;
  } else if(pintegral < -100) {
    pintegral = -100;
  }

  float pderivative = orientation.gyro.y;   //derivative
  pprev_input = psensor;
  
  float pspeed = (kpp*pierror) + pintegral + (kpd*pderivative); // final value of sum
  if(pspeed > 200)  // throttle max change
    pspeed = 200;
  else if(pspeed < -200)
    pspeed = -200;
  pitch(pspeed);

  // ROLL VALUES
  static float rprev_error;
  static float rintegral;
  int rtarget = raw_input[3];
  float rsensor = compRAngle - R_OFFSET;
  float rierror = (float)rtarget - rsensor; // proportional

  rintegral += kri*rierror*dt;  // integral
  if(rintegral > 100) {
    rintegral = 100;
  } else if(rintegral < -100) {
    rintegral = -100;
  }
  
  float rderivative = orientation.gyro.x;   //derivative
  rprev_error = rierror;
  
  float rspeed = (krp*rierror) + rintegral + (krd*rderivative); // final value of sum
  if(rspeed > 200)  // throttle max change
    rspeed = 200;
  else if(rspeed < -200)
    rspeed = -200;
  roll(pspeed);

  // YAW VALUES
  static float yprev_error;
  static float yintegral;
  int ytarget = raw_input[1];
  float ysensor = orientation.gyro.z - Y_OFFSET;
  float yierror = (float)rtarget - ysensor; // proportional

  yintegral += kyi*yierror*dt;
  if(yintegral > 100) {
    yintegral = 100;
  } else if(rintegral < -100) {
    yintegral = -100;
  }
  
  float yderivative = yierror-yprev_error;   //derivative
  yprev_error = yierror;
  
  float yspeed = (kyp*yierror) + (10*kyi*yintegral) + (10*kyd*yderivative); // final value of sum
  if(yspeed > 200)  // throttle max change
    yspeed = 200;
  else if(yspeed < -200)
    yspeed = -200;
  yaw(yspeed);

//  // VISUALIZATION OF PID ERRORS
//  Serial.print(pierror);
//  Serial.print(" ");
//  Serial.print(rierror);
//  Serial.print(" ");
//  Serial.println(yierror);
}

