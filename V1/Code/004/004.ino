#include <IBusBM.h>
IBusBM IBus; // IBus object

// ramp lib
#include <Ramp.h>

#include <Dynamixel2Arduino.h>
#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 21; // DYNAMIXEL Shield DIR PIN 
const float DXL_PROTOCOL_VERSION = 2.0;

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;
#define BNO08X_INT  17
#define BNO08X_RST  16
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

float pitch;
float roll;
float yaw;

float x;
float z;
float y;

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;
int ch9;
int ch10;

float RFB;
float RFBa;
float RFBFiltered;
float RLR;
float RLRa;
float RLRFiltered;
float RT;
float RTa;
float RTFiltered;

float LFB;
float LFBa;
float LFBFiltered;
float LLR;
float LLRa;
float LLRFiltered;
float LT;
float LTa;
float LTFiltered;

int legUp;
int legDown;
int leg1x;
int leg1y;
int leg1z;
int leg2x;
int leg2y;
int leg2z;

unsigned long currentMillis;
long previousMillis = 0;        // set up timers
long interval = 10;             // time constant for timer

int stepFlag = 0;
int leftStepFlag = 0;
int rightStepFlag = 0;
int timer1;
int timer2;
int timer3;
long previousStepMillis = 0;    // timers for walking states 
long previousRightStepMillis = 0;    // timers for walking states 
long previousLeftStepMillis = 0;    // timers for walking states 

class Interpolation {
  public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value

      if (interpolationFlag == 0) {                                   // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }

      int output = myRamp.update();
      return output;
    }
};    // end of class

Interpolation interpFRX;        // interpolation objects
Interpolation interpFRY;
Interpolation interpFRZ;

Interpolation interpFLX;        // interpolation objects
Interpolation interpFLY;
Interpolation interpFLZ;


void setup() {

  //DEBUG_SERIAL.begin(115200);

  IBus.begin(Serial1, IBUSBM_NOTIMER);    // change to Serial1 or Serial2 port when required

  // Set Port baudrate
  dxl.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(0);
  dxl.ping(1);
  dxl.ping(2);
  dxl.ping(3);
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(0);
  dxl.torqueOff(1);
  dxl.torqueOff(2);
  dxl.torqueOff(3);
  dxl.setOperatingMode(0, OP_POSITION);
  dxl.setOperatingMode(1, OP_POSITION);
  dxl.setOperatingMode(2, OP_POSITION);
  dxl.setOperatingMode(3, OP_POSITION);
  dxl.torqueOn(0);
  dxl.torqueOn(1);
  dxl.torqueOn(2);
  dxl.torqueOn(3);

  Wire.begin();
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");
  // Wire.setClock(400000); //Increase I2C data rate to 400kHz
  setReports();
  }

  

  // Here is where you define the sensor outputs you want to receive
  void setReports(void) {
    Serial.println("Setting desired reports");
    if (myIMU.enableRotationVector() == true) {
      Serial.println(F("Rotation vector enabled"));
      Serial.println(F("Output in form roll, pitch, yaw"));
    } else {
      Serial.println("Could not enable rotation vector");
    }



  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, 0, 200);

}

void loop() {

      currentMillis = millis();
      if (currentMillis - previousMillis >= 10) {  // start timed event    
          previousMillis = currentMillis;

          IBus.loop();
    
          ch1 = IBus.readChannel(0); // get latest value for servo channel 0
          ch2 = IBus.readChannel(1); // get latest value for servo channel 1
          ch3 = IBus.readChannel(2); // get latest value for servo channel 3
          ch4 = IBus.readChannel(3); // get latest value for servo channel 4
          ch5 = IBus.readChannel(4); // get latest value for servo channel 5
          ch6 = IBus.readChannel(5); // get latest value for servo channel 6
          ch7 = IBus.readChannel(6); // get latest value for servo channel 7
          ch8 = IBus.readChannel(7); // get latest value for servo channel 8
          ch9 = IBus.readChannel(8); // get latest value for servo channel 9
          ch10 = IBus.readChannel(9); // get latest value for servo channel 10
    
          LFB = ch1;
          RFB = ch4;
          RLR = ch2;
          RT = ch6;
          LLR = ch3;
          LT = ch5;
          
          // *** threshold sticks ***
          RFBa = thresholdStick(RFB);
          RLRa = thresholdStick(RLR);
          RTa = thresholdStick(RT);
          LFBa = thresholdStick(LFB);
          LLRa = thresholdStick(LLR);
          LTa = thresholdStick(LT);
    
          // *** filter sticks ***
          RFBFiltered = filter(RFBa, RFBFiltered,20);
          RLRFiltered = filter(RLRa, RLRFiltered,20);
          RTFiltered = filter(RTa, RTFiltered,20);
          LFBFiltered = filter(LFBa, LFBFiltered,20);
          LLRFiltered = filter(LLRa, LLRFiltered,20);
          LTFiltered = filter(LTa, LTFiltered,20);    
    
          //Serial.print(RFBFiltered);
          //Serial.print(" , ");
          //Serial.print(LTFiltered);
          //Serial.print(" , ");    

          if (myIMU.wasReset()) {
              Serial.print("sensor was reset ");
              setReports();
          }
    
          // Has a new event come in on the Sensor Hub Bus?
          if (myIMU.getSensorEvent() == true) {
      
          // is it the correct sensor data we want?
          if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      
              roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
              pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
              yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
              }
          }    
   
          // *** kinematics test mode ***

          if (ch7 < 1300) {               
                z =  map(LTFiltered,-255,255,170,290);
                x =  map(RFBFiltered,-255,255,60,-60);
      
                kinematics(1, x, y ,z ,0, 0);
                kinematics(2, x, y ,z ,0, 0);
          } 

          

          // *** walking mode ***

          else if (ch7 > 1400 && ch7 < 1700) {  

              timer1 = 300;   // step trigger time
              timer2 = (abs(roll*10)) + 50;   // leg trigger timer
              timer3 = 250 - (abs(roll*3));   // leg step timer  ** inverted **
              timer3 = constrain(timer3,200,300);
              legUp = 240;
              legDown = 260;  

              x =  map(RFBFiltered,-255,255,-60,60);

              // *** step triggers ***
              if (stepFlag == 0 && currentMillis - previousStepMillis > timer1) {
                previousStepMillis = currentMillis;
                stepFlag = 1; 
                rightStepFlag = 1;   
              } 
              else if (stepFlag == 1 && currentMillis - previousStepMillis > timer2) {
                previousStepMillis = currentMillis;
                stepFlag = 2;   
              }

              else if (stepFlag == 2 && currentMillis - previousStepMillis > timer1) {
                previousStepMillis = currentMillis;
                stepFlag = 3; 
                leftStepFlag = 1;         
              }

              else if (stepFlag == 3 && currentMillis - previousStepMillis > timer2) {
                previousStepMillis = currentMillis;
                stepFlag = 0;          
              }
              // *** end of step triggers ***


              // *** right leg steps ***

              if (rightStepFlag == 1 && currentMillis - previousRightStepMillis > (timer3/2)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 2;
                leg2z = legUp;
              } 
              else if (rightStepFlag == 2 && currentMillis - previousRightStepMillis > (timer3/2)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 3;
                leg2x = x;
              } 
              else if (rightStepFlag == 3 && currentMillis - previousRightStepMillis > (timer3/2)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 4;
                leg2z = legDown;     
              }
              else if (rightStepFlag == 4 && currentMillis - previousRightStepMillis > (timer3/2)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 5;
                leg2x = x*-1;   
              }

              // *** left leg steps ***

              if (leftStepFlag == 1 && currentMillis - previousLeftStepMillis > (timer3/2)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 2;
                leg1z = legUp;    
              } 
              else if (leftStepFlag == 2 && currentMillis - previousLeftStepMillis > (timer3/2)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 3;
                leg1x = x;    
              } 
              else if (leftStepFlag == 3 && currentMillis - previousLeftStepMillis > (timer3/2)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 4;
                leg1z = legDown;     
              }
              else if (leftStepFlag == 4 && currentMillis - previousLeftStepMillis > (timer3/2)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 5;
                leg1x = x*-1;    
              }

              leg2y = 0;
              leg1y = 0;

              kinematics(1, leg1x+30, leg1y ,leg1z ,1, timer3);              
              kinematics(2, leg2x+30, leg2y ,leg2z ,1, timer3);

          }
  

  }     // end of timed loop

   

}
