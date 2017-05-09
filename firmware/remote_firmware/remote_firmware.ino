#include <radio.h>
#include <serLCD.h>
#include <Arduino.h>
#include <stdlib.h>

#define PIN_YAW      0 //A0
#define PIN_THROTTLE  1// A1 
#define PIN_ROLL    2// A2 
#define PIN_PITCH   3//A3  
#define PIN_POT1    7//A7
#define PIN_POT2    6//A6
#define PIN_BTN1    16    // PG0 (schematic) G0 (red board)
#define PIN_BTN2    17    // PG1 (schematic) G1 (red board)
#define PIN_LED_BLUE  22    // PD6 (schematic) D4 (red board)
#define PIN_LED_GRN   23    // PD5 (schematic) D5 (red board)
#define PIN_LED_RED   24    // PD4 (schematic) D6 (red board)   

// Initialize global variables for storing incoming data from input pins
int readYaw = 0;
int readThrottle = 0;
int readRoll = 0;
int readPitch = 0; 
int readPot1 = 0;
int readPot2 = 0;
bool btn1Hi = 0;
bool btn2Hi = 0;
uint8_t PIDflag;
uint8_t PIDopt;
bool LEDVal = 0;
bool latch = false;

// time variables
unsigned long last = 0;
unsigned long screen_last = 0;

// misc. variables
uint8_t scale[8] = 
                 {B00000000,
                  B00000000,
                  B00000000,
                  B00000000,
                  B00000000,
                  B00000000,
                  B00000000,
                  B00000000};
int numbers[8] = {0,1,2,3,4,5,6,7};
int thrust;
short tensPowerPot;
short valsPowerPot;
int8_t YPRVals[3] = {0, 0, 0};
int min_val[8] = {124,116,116,117,115,115,0,0};   // LAST RECORDED VALS
int max_val[8] = {816,816,834,815,790,790,1024,1024}; // LAST RECORDED VALS
int stdy_val[4] = {459, 505, 468}; // STDY FOR Y/P/R
char magic[8] = {'A','B','C','D','E','F','G','H'};
char *labels[8] = {"T ", "Y ", "P ", "R ", "P1", "P2", "B1", "B2"}; // used for serial display
char pins[8] = {PIN_THROTTLE, PIN_YAW, PIN_PITCH, PIN_ROLL, PIN_POT1, PIN_POT2, PIN_POT1, PIN_POT2};
float calibrateArr[10] = {0,0,0,0,0,0,0,0,0};
bool locked[10] = {true, true, true, true, true, true, true, true, true, true};
serLCD lcd;   // Holder for LCD Object
bool mtr_switch = LOW;
unsigned int maxYawVal = 50;  // 50 was a good compromise without knowing the system specs
unsigned int DEAD_THRESH = 5; // dead threshold (distance for which nothing happens)

void updatePIDVals() {
  static short sequenced[5] = {0,0,0,0,0};
  
  if(!locked[PIDopt]) {
    sequenced[tensPowerPot-1] = valsPowerPot;
  } else {
    unsigned long temp = calibrateArr[PIDopt]*10000;
    for(char i=4; i>=0; i--) {
      sequenced[i] = temp%10;
      temp/=10;
    }
  }

  float valPlace = 1.0;
  float placeHold;
  for(char i=0; i<5; i++) {
    placeHold += sequenced[i]/valPlace;
    valPlace*=10;
  }
    calibrateArr[PIDopt] = placeHold;
}

/**
 * Maps certain values to a particular scheme. In particular, thrust and yaw are passed as their original counterparts, while
 * pitch and roll are passed as the wanted euler angles.
 * INVERTED: YAW, PITCH
 * NORMAL: THROTTLE, ROLL
 */
void mapping_scheme() {
  // Map throttle
  if(mtr_switch) {
    if(numbers[0] < min_val[0]+DEAD_THRESH)
      thrust = 0;
    else
      thrust = (int)map(numbers[0], min_val[0]+DEAD_THRESH, max_val[0], 40, 210);
  }
  
  // Map yaw (yaw is inverted)
  if(numbers[1] > stdy_val[0]-DEAD_THRESH && numbers[1] < stdy_val[0]+DEAD_THRESH)
    YPRVals[0] = 0;
  else if(numbers[1] > stdy_val[0]+DEAD_THRESH) {
    int negMax = 0-maxYawVal;
    YPRVals[0] = -1*(int)map(numbers[1], stdy_val[0], max_val[1], 0, negMax);
  } else
    YPRVals[0] = -1*(int)map(numbers[1], min_val[1], stdy_val[0], maxYawVal, 0);
  
  // Map pitch (pitch is inverted)
  if(numbers[2] > stdy_val[1]-DEAD_THRESH && numbers[2] < stdy_val[1]+DEAD_THRESH)
    YPRVals[1] = 0;
  else if(numbers[2] > stdy_val[1]+DEAD_THRESH)
    YPRVals[1] = (int)map(numbers[2], stdy_val[1], max_val[2], 0, 45);
  else
    YPRVals[1] = (int)map(numbers[2], min_val[2], stdy_val[1], -45, 0);
  
  // Map roll
  if(numbers[3] > stdy_val[2]-DEAD_THRESH && numbers[3] < stdy_val[2]+DEAD_THRESH)
    YPRVals[2] = 0;
  else if(numbers[3] > stdy_val[2]+DEAD_THRESH)
    YPRVals[2] = (int)map(numbers[3], stdy_val[2], max_val[3], 0, 45);
  else
    YPRVals[2] = (int)map(numbers[3], min_val[3], stdy_val[2], -45, 0);

  // Map Potentiometers when necessary
  if(PIDflag == 1) {
    if(PIDopt == 9) {
      tensPowerPot = map(numbers[4], min_val[4], max_val[4], 1, 3);
    } else {
      tensPowerPot = map(numbers[4], min_val[4], max_val[4], 1, 5);
    }
    valsPowerPot = map(numbers[5], min_val[5], max_val[5], 0, 9);
  }
}

void lcdprint_float(float num, int dec_places) {
  char buff[dec_places+5];
  
  dtostrf(num, dec_places+2, dec_places, buff);
  lcd.print(buff);
}

void update_display() {
  lcd.home();

  if(PIDflag==0) {
    for(char h = 0; h < 8; h++) {
        char buf[2];
        buf[0] = labels[h][0];
        buf[1] = 0;
        lcd.print(buf);
        int n = numbers[h] >> 6;
        if (n > 8) {
           lcd.printCustomChar(n - 8);
        } else {
           lcd.print(" ");
        }
    }
  
    for(char h = 0; h < 8; h++) {
        char buf[2];
        buf[0] = labels[h][1];
        buf[1] = 0;
        lcd.print(buf);
        int n = numbers[h] >> 6;
        if (n >= 8) {
           lcd.printCustomChar(8);
        } else if (n == 0) {
          lcd.print(" ");
        } else {
           lcd.printCustomChar(n);
        }
    }
  } else {
      switch(PIDopt) {
        case 0:
          lcd.print("Pitch: Kp ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[0], 4);
          break;
        case 1:
          lcd.print("Pitch: Ki ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[1], 4);
          break;
        case 2:
          lcd.print("Pitch: Kd ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[2], 4);
          break;
        case 3:
          lcd.print("Roll:  Kp ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[3], 4);
          break;
        case 4:
          lcd.print("Roll:  Ki ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[4], 4);
          break;
        case 5:
          lcd.print("Roll:  Kd ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[5], 4);
          break;
        case 6:
          lcd.print("Yaw:   Kp ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[6], 4);
          break;
        case 7:
          lcd.print("Yaw:   Ki ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[7], 4);
          break;
        case 8:
          lcd.print("Yaw:   Kd ");
          lcd_printplacement(4);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("         ");
          lcdprint_float(calibrateArr[8], 4);
          break;
        case 9:
          lcd.print("CF Value:   ");
          lcd_printplacement(2);
          lcd.print(locked[PIDopt]?"L":" ");
          lcd.print("           ");
          lcdprint_float(calibrateArr[9], 2);
          break;
        default:
          char bufd[8];
          lcd.print(itoa(PIDopt, bufd, 10)); 
      }
  }
}

void lcd_printplacement(int precision) {
  if(precision == 4) {
    switch(tensPowerPot) {
      case 1:
        lcd.print("_.xxxx"); break;
      case 2:
        lcd.print("x._xxx"); break;
      case 3:
        lcd.print("x.x_xx"); break;
      case 4:
        lcd.print("x.xx_x"); break;
      case 5:
        lcd.print("x.xxx_"); break;
      deafault:
        break;
    }
  } else if(precision == 2) {
    switch(tensPowerPot) {
      case 1:
        lcd.print("_.xx"); break;
      case 2:
        lcd.print("x._x"); break;
      case 3:
        lcd.print("x.x_"); break;
      default:
        break;
    }
  }
}

void button_opts() {
  // Read incoming presses from buttons:
  numbers[6] = (digitalRead(PIN_BTN1)==0)?1024:0; 
  numbers[7] = (digitalRead(PIN_BTN2)==0)?1024:0;

  // Button options
  if(numbers[6] == 1024 && numbers[7] == 1024 && !btn1Hi && !btn2Hi) { // buttons pressed first moment
    btn1Hi = true;
    btn2Hi = true;
    
    PIDflag = PIDflag==0?1:0;
    if(PIDflag == 1) {
      PIDopt = 0;
      if(mtr_switch) {
        mtr_switch = !mtr_switch;
        digitalWrite(PIN_LED_RED, mtr_switch);
      }
      // ensure motor is stopped
      thrust = 0;
    } else
      PIDopt = 0;
    lcd.clear();
  } else if(numbers[6] == 1024 && PIDflag == 0 && !btn1Hi) {  // Left button pressed in free mode
    btn1Hi = true;
    
    //switch on/off motor
    mtr_switch = !mtr_switch;
    if(!mtr_switch) {
      thrust = 0;
    }
    digitalWrite(PIN_LED_RED, mtr_switch);
  } else if(PIDflag == 1 && numbers[7] ==1024 && !btn1Hi){ // right button pressed in PID mode
    btn1Hi = true;
    locked[PIDopt] = true;
    PIDopt = (1+PIDopt)%10;  // 10 options to choose from
    lcd.clear();
  } else if(PIDflag == 1 && numbers[6] == 1024 && !btn2Hi){ // left button pressed in PID mode (toggle lock)
    btn2Hi = true;
    locked[PIDopt] = !locked[PIDopt];
  } else if(PIDflag == 1 && numbers[0] < min_val[0]+DEAD_THRESH && YPRVals[2] > 40 && YPRVals[1] > 40 && !latch) {
    latch = true;
    mtr_switch = !mtr_switch;
    if(!mtr_switch) {
      thrust = 0;
    }
    digitalWrite(PIN_LED_RED, mtr_switch);
  } else if(latch && YPRVals[2] < 10 && YPRVals[1] < 10) {
    latch = false;
  }else if(numbers[6] == 0 && numbers[7] == 0) { // buttons released
    btn1Hi = false;
    btn2Hi = false;
  }

  // indicators for modes
  if (PIDflag == 1) {
    digitalWrite(PIN_LED_BLUE, HIGH);
    digitalWrite(PIN_LED_GRN, LOW);
  } else {
    digitalWrite(PIN_LED_BLUE, LOW);
    digitalWrite(PIN_LED_GRN, HIGH);
  }
}

void setup() {
  const int RADIO_CHANNEL = 21;        // Channel for radio communications (can be 11-26)
  const int SERIAL_BAUD = 9600;        // Baud rate for serial port 
  const int SERIAL1_BAUD = 9600;     // Baud rate for serial1 port

  Serial.begin(SERIAL_BAUD);           // Start up serial
  Serial1.begin(SERIAL_BAUD);  

  delay(100);
  lcd.display();
  lcd.clear();  // Clear lcd only on init
  for(char i = 0; i < 8; i++) {
    scale[7-i] = B11111111;
    lcd.createChar(i+1, scale);
    delay(10);
  }
  
  rfBegin(RADIO_CHANNEL);              // Initialize ATmega128RFA1 radio on given channel
  
  // Set pin modes for all input pins
  pinMode(PIN_YAW, INPUT);             // Gimbal: Yaw
  pinMode(PIN_THROTTLE, INPUT);        // Gimbal: throttle
  pinMode(PIN_ROLL, INPUT);            // Gimbal: roll
  pinMode(PIN_PITCH, INPUT);           // Gimbal: pitch
  pinMode(PIN_POT1, INPUT);            // Potentiometer 1
  pinMode(PIN_POT2, INPUT);            // Potentiometer 2
  
  pinMode(PIN_BTN1, INPUT_PULLUP);            // Button 1
  pinMode(PIN_BTN2, INPUT_PULLUP);            // Button 2
  
  pinMode(PIN_LED_BLUE, OUTPUT);       // LED Indicator: Blue
  pinMode(PIN_LED_GRN, OUTPUT);        // LED Indicator: Green
  pinMode(PIN_LED_RED, OUTPUT);        // LED Indicator: Red

  // Reset indicators on quad
  rfWrite(magic[6]);
  rfWrite(PIDflag);
  rfWrite(magic[7]);
  rfWrite(PIDopt);
}

void loop() {
  button_opts();

  /* Refresh screen every 100 ms */
  if (screen_last + 100 <= millis()) {
      update_display();
      screen_last = millis();
  }
  
  // Read analog values
  for(char i = 0; i < 6; i++) {
    numbers[i] = analogRead(pins[i]);
  }

  // Map all the numbers to reasonable values
  mapping_scheme();

  // if in PID mode, create an integer using given inputs
  if(PIDflag == 1) {
    updatePIDVals();
  }

  // Send thrust values
  rfWrite(magic[0]);
  rfWrite(highByte(thrust));
  rfWrite(lowByte(thrust));
  delay(20);

  // Send angular values
  for(char i=0; i<3; i++) {
    rfWrite(magic[i+1]);
    rfWrite(YPRVals[i]);
  }
  delay(20);

  // Send option modes
  if(last+1000 < millis()) {
    if(millis()-last<0) { // timer overflowed
      last = millis();
      screen_last = millis();
    }
    rfWrite(magic[5]);
    rfWrite((uint8_t)(mtr_switch?1:0));
    rfWrite(magic[6]);
    rfWrite(PIDflag);
    delay(20);

    // Send potentiometer entries if needed
    if(PIDflag == 1) {
      rfWrite(magic[7]);
      rfWrite(PIDopt);
    
      rfWrite(magic[4]);
      uint8_t toSend1 = (int)calibrateArr[PIDopt];
      rfWrite(toSend1);
      int toSend = (calibrateArr[PIDopt]-(int)calibrateArr[PIDopt])*10000;
      rfWrite(highByte(toSend));
      rfWrite(lowByte(toSend));
      delay(20);
    }
    
    last = millis();
  }
}

