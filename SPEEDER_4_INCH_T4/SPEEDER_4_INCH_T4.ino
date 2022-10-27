/*  EjoWerks Speeder Teensy 4.0 
 *  v0.1 /// last tested 10/26/2022
 *  
 *  This version has only been implemented with the Teensy 4.0 but may work with 4.1, etc
 *  
 *  Please note this is experimental code that requires arduino-based tinkering experience
 *
 *  Heed the warnings in readme file
 *  
 *  You should have 5 tabs if you load this correctly in Arduino IDE
 *  
 *  Random snippets of code and layout methodology borrowed from https://github.com/nickrehm/dRehmFlight 
 *  
 *  This project has at least 6 months of random experimentation buried in it so enjoy the wacky arithmetic,
 *  redundant code, unnecessary vars, and general inconsistency. I'm sure a good hacker could do this in a tenth of the space
 *  but hey, it FLIES :-)
 *  
 *  To Do: Cleanup, remove unused vars, standardize things, remove redundant code. What a mess. Lolz.
 */
 
#include <Servo.h>    // rear thruster ESC 
#include <PID_v1.h>   
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include "src/SBUS/SBUS.h"   //sBus interface  
#include <SimpleKalmanFilter.h> 
#include <VL53L1X.h>  // rear safety sensor
#include <TFMPlus.h>  // TFMini Plus Library v1.5.0

TFMPlus tfmP;
VL53L1X sensorBack;
#include "PMW3901.h"
static PMW3901 flow(10);
Servo ESC;
SBUS sbus(Serial2);

////////////// CHANGE STUFF HERE /////////////
int cells = 4;
String batterytype = "lipo"; /// lipo|lion
int taranis = 0;  // change this to a 1 if flying with aircraft radio -- don't forget hover PIDs won't be set if you do this!
int hoverHeight = 400; // mm
/////////////////////////////////////////////

/// TO DO: Clean out unused vars
const uint32_t currentMillis = millis();
const int numReadings = 30; // anti-skid num readings for averaging filter
int landing_Time = 0,thrMaxTmp = 0,readings[numReadings],readIndex = 0,total = 0, average = 0, thrMaxClimb = 0, turnheight = 0, autolanding = 0, pitch_des_flow = 0, roll_des_flow = 0, pitch_des_flow_raw = 0, roll_sum, roll_des_flow_raw = 0, escIDLE = 0, escWrite = 0, isFlying = 0, stmix_pos, maxThruster = 250, ltm_pitch = 0, ltm_roll = 0, killAll = 0, led = 13, rangeBackFiltered, thro_channel_rx, m1_command_PWM, hoverHeightRaw, thrMin, thrMax, Engage = 0, prevEngage = 0, thruster_loop_Time, print_loop_Time, flow_loop_Time, radio_rw_loop_Time, toF_loop_Time, thr_Wind_time, rampTime, hoverThrottle, rangeRaw, rangeFrontRaw, rangeBackRaw, rangeFiltered, ranger_loop_Time, ledInterval;
float criticalVolt, lowVolt, ltm_voltagecell, ltm_voltage, m1_command_scaled, KpTune, KiTune, KdTune, KpHover, KiHover, KdHover, dt, hoverRatio, hoverRatio2;
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm, channel_10_pwm, channel_11_pwm, channel_12_pwm, channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev, channel_5_pwm_prev, channel_6_pwm_prev, idleFlow, blink_counter, blink_delay, voltLowTimer, voltLowTimerMillis, valueChangeTime, current_time, prev_time, print_counter, serial_counter, engageSwitch, armSwitch, engageDelay;
bool blinkAlternate, sbusFailSafe, sbusLostFrame;
double deltaYinput, deltaYroll, deltaXinput, deltaXpitch, deltaXsetpoint = 0.00, deltaYsetpoint = 0.00, moveX, moveY, PIDSetPointmm, PIDInputHeightmm, PIDOutThrottlePWM, newThrottlePWM;
int16_t sbusChannels[16], deltaX = 0, deltaY = 0, deltaXfiltered, deltaYfiltered, tfDist = 0, tfFlux = 0, tfTemp = 0;

const int PPM_Pin = 16;  /// from RC car RX 
const int m1Pin = 0; // Thruster ESC

// PID hoverPID
PID hoverPID(&PIDInputHeightmm, &PIDOutThrottlePWM, &PIDSetPointmm, KpHover, KiHover, KdHover, P_ON_M, DIRECT); /// note "P_ON_M" 
// hover flow pid stuff
PID xPid(&moveX, &deltaXpitch, &deltaXsetpoint, KpTune, KiTune, KdTune, P_ON_E, DIRECT);
PID yPid(&moveY, &deltaYroll, &deltaYsetpoint, KpTune, KiTune, KdTune, P_ON_E, DIRECT);

SimpleKalmanFilter hoverFilter(3, 1, 0.01);
SimpleKalmanFilter throFilter(15, 1, 0.01);
SimpleKalmanFilter sensorBackFilter(1, 1, 0.01);
SimpleKalmanFilter pitchFilter(1, 1, 0.01);
SimpleKalmanFilter rollFilter(1, 1, 0.01);


void setup() {

  ESC.attach(m1Pin, 1000, 2000);
  Serial.begin(115200);
  Serial3.begin(9600);  // LTM input from FC
  Serial5.begin(115200);  // Initialize TFMPLus device serial port.
  Wire.begin();
  Wire.setClock(1000000); // use 1000 kHz I2C

  /// Hover PID STUFF
  hoverPID.SetOutputLimits(0, 255);
  hoverPID.SetMode(AUTOMATIC);
  hoverPID.SetSampleTime(1);
  
  /// flow sensor pids
  xPid.SetMode(AUTOMATIC);
  xPid.SetSampleTime(1);
  xPid.SetOutputLimits(-255, 255);
  yPid.SetMode(AUTOMATIC);
  yPid.SetSampleTime(1);
  yPid.SetOutputLimits(-255, 255);


  // fire this thing up
  tfInit();
  rearToFinit();
  radioSetup();
  sbus.begin();
  flowInit();


  // set everything to zero for safety? I dunno
  channel_1_pwm = 1500;
  channel_2_pwm = 1500;
  KpHover = 0;
  KiHover = 0;
  KdHover = 0;
  thrMin = 0;
  thrMax = 0;
  PIDSetPointmm = 0;
  engageDelay = 0;
  engageSwitch = 1000;

}


void loop() {

  uint32_t currentMillis = millis();
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;

  controlSafe(); 
  hoverMain();
  flowIt();
  radioComms();
  readLTM();
  
  // Serial print stuff for debug -- leave off unless troubleshooting

  if (currentMillis > print_loop_Time) {
    //printRadioData();
    //printBig();
    //printRangers();
    //printFlow();
    //printFlowControl();
    print_loop_Time = currentMillis + 10;
  }


  //printLoopRate(); 
  loopRate(2000); // thanks Nick Rhem 
}
