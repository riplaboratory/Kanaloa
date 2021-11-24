// UPSV arduino code for remote control operations
// Last update: 2021.10.05
// Based on MRUH main arduino code, editted by tabata viso-naffah

// Library inclusions
#include <Arduino.h>                  // Arduino inclusions

// Pin definitions
const byte ch1Pin = 2;                // input PWM pin from manual controller receiver channel 1 (left stick up-down)
// const byte ch2Pin = 3;             // input PWM pin from manual controller receiver channel 2 (left stick left-right)
const byte ch3Pin = 4;                // input PWM pin from manual controller receiver channel 3 (right stick left-right)
// const byte ch4Pin = 5;             // input PWM pin from manual controller receiver channel 4 (momentary SH switch at top right of receiver)
const byte ch7Pin = 6;                // input PWM pin from manual controller gear channel 7 (remote kill switch)
const byte physKillPin1 = 52;         // input analog pin from physical kill switch (this should really be a digital pin, but whatever for now)
const byte physKillPin2 = 53;
const byte killPortRelayPin = 24;     // output digital pin to kill PORT SIDE switch relay 
const byte killStarRelayPin = 26;     // output digital pin to kill STARBOARD SIDE switch relay
const byte portPin = 12;              // output PWM pin to PORT SIDE motor controller
const byte starPin = 11;              // output PWM pin to STARBOARD SIDE motor controller
// const byte q3Pin = 9;              // output PWM pin to Q3 motor controller (as of MCM demo, we do not have a third motor controller)
// const byte q4Pin = 10;             // output PWM pin to Q4 motor controller (as of MCM demo, we do not have a fourth motor controller)
const byte redPin = 28;               // output digital pin to red light relay
// const byte bluePin = 16;           // output digital pin to blue light relay (as of MCM demo, we do not have a battery check in place)
const byte greenPin = 30;             // output digital pin to green light relay
const byte batteryPin = A0;           // input analog pin from battery voltage divider (as of MCM demo, we do not have a battery check in place)

// Global system (constant) variables
const int ch1PulseMin = 993;      // minimum expected pulse width of ch1 [us]
const int ch1PulseMax = 1988;     // maximum expected pulse width of ch1 [us]
// const int ch2PulseMin = 993;      // minimum expected pulse width of ch2 [us]
// const int ch2PulseMax = 1988;     // maximum expected pulse width of ch2 [us]
const int ch3PulseMin = 993;      // minimum expected pulse width of ch3 [us]
const int ch3PulseMax = 1988;     // maximum expected pulse width of ch3 [us]
// const int ch4PulseMin = 993;      // minimum expected pulse width of ch4 [us]
// const int ch4PulseMax = 1988;     // maximum expected pulse width of ch4 [us]
const int ch7PulseMin = 993;      // minimum expected pulse width of ch7 [us]
const int ch7PulseMax = 1988;     // maximum expected pulse width of ch7 [us]
const int pulseTolerance = 200;   // tolerance range on minimum and maximum expected pulse widths [us]
const int pulseDeadzone = 125;    // deadzone range at the center of each joystick [us]
const int accLimit = 200;         // acceleration limit (maximum change in thrust percentage per second) [%/s]

// Global code variables
int ch1Pulse = 0;
// int ch2Pulse = 0;
int ch3Pulse = 0;
// int ch4Pulse = 0;
int ch7Pulse = 0;
int physKillRaw = 0;
int remKillStatus = 1;
int physKillStatus = 1;
int killStatus = 1;
int lightColor = 0;             // light color variable (0 = off, 1 = white, 2 = red, 3 = yellow, 4 = blue)
int portSetpoint = 0;             // PORT SIDE setpoint from -1000 to 1000 (no acceleration limit)
int starSetpoint = 0;             // STARBOARD SIDE setpoint from -1000 to 1000 (no acceleration limit)
// int q3Setpoint = 0;          // Q3 setpoint from -1000 to 1000 (no acceleration limit) (as of MCM demo, we do not have a third motor controller)
// int q4Setpoint = 0;          // Q4 setpoint from -1000 to 1000 (no acceleration limit) (as of MCM demo, we do not have a fourth motor controller)
float timeNow = 0;              // time of current iteration (for acceleration limit calculations)
float timeLast = 0;             // time of last iteration (for acceleration limit calculations)
int portOut = 0;                  // PORT SIDE output from -1000 to 1000 (with acceleration limits)
int starOut = 0;                  // STARBOARD SIDE output from -1000 to 1000 (with acceleration limits)
// int q3Out = 0;               // Q3 output from -1000 to 1000 (with acceleration limits) (as of MCM demo, we do not have a third motor controller)
// int q4Out = 0;               // Q4 output from -1000 to 1000 (with acceleration limits) (as of MCM demo, we do not have a fourth motor controller)
int portLast = 0;                 // PORT SIDE output for last iteration (for acceleration limit calculations)
int starLast = 0;                 // STARBOARD SIDE output for last iteration (for acceleration limit calculations)
// int q3Last = 0;              // Q3 output for last iteration (for acceleration limit calculations) (as of MCM demo, we do not have a third motor controller)
// int q4Last = 0;              // Q4 output for last iteration (for acceleration limit calculations) (as of MCM demo, we do not have a fourth motor controller)
float battVoltBit = 0;          // battery voltage [bits]
float battVoltArd = 0;          // battery voltage expressed at arduino (through voltage divider) [volt]
float battVoltTrue = 0;         // battery voltage at battery [volt]
float voltMult = 0.73;          // scale output to thrusters by this multiplier [ ]

void setup() {

  // Set pin modes
  pinMode(ch1Pin,INPUT_PULLUP);
  // pinMode(ch2Pin,INPUT_PULLUP);
  pinMode(ch3Pin,INPUT_PULLUP);
  // pinMode(ch4Pin,INPUT_PULLUP);
  pinMode(ch7Pin,INPUT_PULLUP);
  pinMode(physKillPin1,INPUT);     // this pin is physically pulled down to ground in box
  pinMode(physKillPin2,INPUT);
  pinMode(killPortRelayPin,OUTPUT);
  pinMode(killStarRelayPin,OUTPUT);
  pinMode(portPin,OUTPUT);
  pinMode(starPin,OUTPUT);
  // pinMode(q3Pin,OUTPUT);       // (as of MCM demo, we do not have a third motor controller)
  // pinMode(q4Pin,OUTPUT);       // (as of MCM demo, we do not have a fourth motor controller)
  pinMode(redPin,OUTPUT);
  // pinMode(bluePin,OUTPUT);
  pinMode(greenPin,OUTPUT);
  // inMode(batteryPin,INPUT);      // this pin in on a voltage divider in box
  
  // Set kill relay HIGH to KILL
  digitalWrite(killPortRelayPin,HIGH);
  digitalWrite(killStarRelayPin,HIGH);
    
  // Serial setup
  Serial.begin(57600);
  
  // Handles Arduino timers for PWM frequency (for more info: https://playground.arduino.cc/Main/TimerPWMCheatsheet and http://forum.arduino.cc/index.php/topic,72092.0.html)
  int myEraser = 7;         // this is 111 in binary, and used as an eraser
  TCCR2B &= ~myEraser;      // sets three bits in TCCR2B (timer 2) to zero
  TCCR4B &= ~myEraser;      // sets three bits in TCCR4B (timer 4) to zero
  TCCR4B |= 4;              // Set timer 4 (pins 6 and 8) to prescaler 4 (120 Hz).
  TCCR2B |= 6;              // Set timer 2 (pins 10 and 9) to prescaler 6 (120 Hz).

}

void loop() {

  // Read kill state inputs 
  readKill();

  // This code executes when system is unkilled
  if (killStatus == 0) {
    unkillCode(); 
  }

  // This code executes when system is killed
  else {
    killCode();
  }

}
