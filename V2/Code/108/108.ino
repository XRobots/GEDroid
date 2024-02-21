#include <IBusBM.h>
IBusBM IBus; // IBus object

//PID
#include <PID_v1.h>

// ramp lib
#include <Ramp.h>

#include <Dynamixel2Arduino.h>
#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 21; // DYNAMIXEL Shield DIR PIN 
const float DXL_PROTOCOL_VERSION = 2.0;

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 8;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {0, 1, 2, 3, 4, 5, 6 ,7};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int32_t goal_position[2] = {1024, 2048};
uint8_t goal_position_index = 0;

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;
#define BNO08X_INT  17
#define BNO08X_RST  16
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

float roll;
float pitch;
float yaw;

float x;
float z;
float y;
float r;

int headOutput;

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

float pos1;
float pos2;
float pos3;
float pos4;
float pos5;
float pos6;

int pos1a;
int pos2a;
int pos3a;
int pos4a;
int pos5a;
int pos6a;

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

float rInterp;
float rollError;
float rollErrorFiltered;

int legRot = 0;
int legRotTracked = 0;
int legRotInterp = 0;
int legRotOut = 2060;

int walk = 0;

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

// PID1
double Pk1 = 1.5; 
double Ik1 = 0;
double Dk1 = 00;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// PID2
double Pk2 = 10; 
double Ik2 = 0;
double Dk2 = .01;

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

double Output2Filtered;


class Interpolation {
  public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;

    int go(float input, int duration) {

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
Interpolation interpFRT;

Interpolation interpFLX;        // interpolation objects
Interpolation interpFLY;
Interpolation interpFLZ;
Interpolation interpFLT;

Interpolation interpROT;


void setup() {

  //DEBUG_SERIAL.begin(115200);

  IBus.begin(Serial1, IBUSBM_NOTIMER);    // change to Serial1 or Serial2 port when required

  uint8_t i;
  DEBUG_SERIAL.begin(115200);
  dxl.begin(115200);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

// Prepare the SyncRead structure
  for(i = 0; i < DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for(i = 0; i < DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;

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

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-8, 8);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-50, 50);
  PID2.SetSampleTime(10);


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

          if (myIMU.wasReset()) {
              Serial.print("sensor was reset ");
              setReports();
          }
    
          // Has a new event come in on the Sensor Hub Bus?
          if (myIMU.getSensorEvent() == true) {
      
          // is it the correct sensor data we want?
          if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      
              roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
              roll = (roll+.5); // roll trim
              pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
              pitch = pitch + 0.7;
              yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
              }
          }

          Serial.print(roll);
          Serial.print(" , ");
          Serial.println(pitch);

          // front/back stability control PID

          Input2 = pitch;
          Setpoint2 = 0;
          PID2.Compute();


          // *** kinematics test mode ***

          if (ch7 < 1300) {               
                z = map(LTFiltered,-255,255,170,340);
                x = map(RFBFiltered,-255,255,-60,60);
                y = map(RLRFiltered,-255,255,-40,40);
                r = map(LLRFiltered,-255,255,30,-30);
                r = constrain(r,-20,20);
                z = constrain(z,170,290);
                
                // head control

                headOutput = map(LFBFiltered,-255,90,1024,3070);
                headOutput = headOutput + (Output2 * -10); // dynamic control
                headOutput = constrain(headOutput,1800,2500);

                x = x + Output2;  // dynamic control
                
                kinematics(1, x, y-15 ,z, r ,0, 0);
                kinematics(2, x, y+15 ,z ,r ,0, 0);        
          }          

          // *** walking mode ***

          else if (ch7 > 1400 && ch7 < 1700) {

              // scale timers if the physical roll (error) is too high
              // make the time between steps longer, but the step itself shorter

              timer1 = 460;   // step trigger time
              timer3 = 120;   // leg step timer  ** inverted **
                            
              legUp = 240;          // leg lengths for each step
              legDown = 260;
                      
              x =  map(RFBFiltered,-255,255,-60,60);    // walking step distance
              legRot = map(LTFiltered,-255,255,90,-90);    // approx 8 degrees around the centre point of the servo

              // *** step triggers timing using dynamic timers above ***
              
              if (stepFlag == 0 && currentMillis - previousStepMillis > timer1) {
                previousStepMillis = currentMillis;
                stepFlag = 1; 
                rightStepFlag = 1;
              }            
              else if (stepFlag == 1 && currentMillis - previousStepMillis > timer1) {
                previousStepMillis = currentMillis;
                stepFlag = 0; 
                leftStepFlag = 1;  
              }


              // *** end of step triggers ***                       

              // *** right leg steps ***

              if (rightStepFlag == 1 && currentMillis - previousRightStepMillis > (timer3)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 2;
                leg2z = legUp;
                                
              } 
              else if (rightStepFlag == 2 && currentMillis - previousRightStepMillis > (timer3)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 3;
                leg2x = x;    // take steps as the controller is pushed
                r = -5;   
                
              } 
              else if (rightStepFlag == 3 && currentMillis - previousRightStepMillis > (timer3)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 4;
                leg2z = legDown;
                                
              }
              else if (rightStepFlag == 4 && currentMillis - previousRightStepMillis > (timer3)) {
                previousRightStepMillis = currentMillis;
                rightStepFlag = 5;
                leg2x = x*-1;    // take steps as the controller is pushed              
                legRotTracked = legRot;   // rotate legs
              }

              // *** left leg steps ***

              if (leftStepFlag == 1 && currentMillis - previousLeftStepMillis > (timer3)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 2;
                leg1z = legUp;
                 
              } 
              else if (leftStepFlag == 2 && currentMillis - previousLeftStepMillis > (timer3)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 3;
                leg1x = x;    // take steps as the controller is pushed
                r = 5;
                
              } 
              else if (leftStepFlag == 3 && currentMillis - previousLeftStepMillis > (timer3)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 4;
                leg1z = legDown;
                        
              }
              else if (leftStepFlag == 4 && currentMillis - previousLeftStepMillis > (timer3)) {
                previousLeftStepMillis = currentMillis;
                leftStepFlag = 5;
                leg1x = x*-1;    // take steps as the controller is pushed
                legRotTracked = legRot *- 1;    // rotate legs the other way
              }

              leg1y = -20;     // move legs slightly outwards - this is fed intot he kinematics
              leg2y = 20;

              // *** roll interpolation before feeding inverse kinematics ***
              // makes target roll linear from -r to +r over dynamic timer
              rInterp = interpFRT.go((r*100),timer1);   // interpolate the roll value, it is multipled by 100
              rInterp = rInterp / 100;                  // divide by 100 to get a float again
              rInterp = rInterp - 0;  // roll trim

              // PID to track roll target

              Input1 = rInterp;
              Setpoint1 = roll;
              PID1.Compute(); 

              // front/back stablity control

              headOutput = 2500;
              headOutput = headOutput + (Output2*-8);
              headOutput = constrain(headOutput,1800,2500);

              Output2Filtered = (filter(Output2, Output2Filtered, 50)/2);     // additonally filter this PID and add it to the front/back translation axis below

              Output1 = Output1 + 0.6;   // roll trim
              
              kinematics(1, leg1x, leg1y ,leg1z, Output1 ,1, timer3);         // run kinematics          
              kinematics(2, leg2x, leg2y ,leg2z, Output1 ,1, timer3);         // interpolate everthing of dynamic timers

              legRotInterp = interpROT.go(legRotTracked,timer3);      // interpolate leg rotation, there's no inverse kinematics
              legRotOut = 2060 + legRotInterp;                        // add centre pouint              

          }

          // **** write to dynamixels ****

          if (ch8 > 1300) {     // motor enable
          
          // Insert a new Goal Position to the SyncWrite Packet
              
              sw_data[0].goal_position = pos3a;     // DXL ID 0 - RIGHT SHOULDER
              sw_data[1].goal_position = pos1a;     // DXL ID 1 - LEFT SHOULDER
              sw_data[2].goal_position = pos2a;     // DXL ID 2 - LEFT KNEE
              sw_data[3].goal_position = pos4a;     // DXL ID 3 - RIGHT KNEE

              sw_data[4].goal_position = pos5a;      // DXL ID 4 - LEFT HIP
              sw_data[5].goal_position = pos6a;      // DXL ID 5 - RIGHT HIP
              sw_data[6].goal_position = legRotOut;      // DXL ID 6 - LEG ROT
              sw_data[7].goal_position = headOutput;      // DXL ID 7 - HEAD ARM
           
              // Update the SyncWrite packet status
              sw_infos.is_info_changed = true;
              
              // Build a SyncWrite Packet and transmit to DYNAMIXEL  
              dxl.syncWrite(&sw_infos);          
          }  

  }     // end of timed loop

   

}
