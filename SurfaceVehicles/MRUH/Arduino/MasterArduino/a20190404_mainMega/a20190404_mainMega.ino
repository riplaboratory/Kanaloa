// MRUH main arduino code
// Last update: 2019.03.03

// Library inclusions
#include <Arduino.h>                    // Arduino inclusions

// Pin definitions
const byte ch1Pin = 2;        // input PWM pin from manual controller receiver channel 1 (left stick up-down)
const byte ch2Pin = 3;        // input PWM pin from manual controller receiver channel 2 (left stick left-right)
const byte ch3Pin = 4;        // input PWM pin from manual controller receiver channel 4 (right stick left-right)
const byte ch7Pin = 12;       // input PWM pin from manual controller gear channel 7 (remote kill switch)
const byte physKillPin = A10; // input analog pin from physical kill switch (this should really be a digital pin, but whatever for now)
const byte killRelayPin = 30; // output digital pin to kill switch relay
const byte q1Pin = 6;         // output PWM pin to Q1 motor controller
const byte q2Pin = 8;         // output PWM pin to Q2 motor controller
const byte q3Pin = 9;         // output PWM pin to Q3 motor controller
const byte q4Pin = 10;        // output PWM pin to Q4 motor controller
const byte redPin = 15;       // output digital pin to red light relay
const byte bluePin = 16;      // output digital pin to blue light relay
const byte greenPin = 17;     // output digital pin to green light relay

// Global system (constant) variables
const int ch1PulseMin = 993;      // minimum expected pulse width of ch1 [us]
const int ch1PulseMax = 1988;     // maximum expected pulse width of ch1 [us]
const int ch2PulseMin = 993;      // minimum expected pulse width of ch2 [us]
const int ch2PulseMax = 1988;     // maximum expected pulse width of ch2 [us]
const int ch3PulseMin = 993;      // minimum expected pulse width of ch3 [us]
const int ch3PulseMax = 1988;     // maximum expected pulse width of ch3 [us]
const int ch7PulseMin = 993;      // minimum expected pulse width of ch7 [us]
const int ch7PulseMax = 1988;     // maximum expected pulse width of ch7 [us]
const int pulseTolerance = 200;   // tolerance range on minimum and maximum expected pulse widths [us]
const int pulseDeadzone = 125;    // deadzone range at the center of each joystick [us]
const int accLimit = 200;         // acceleration limit (maximum change in thrust percentage per second) [%/s]
const float voltMult = 0.73;      // scale output to thrusters by this multiplier [ ]

// Global code variables
int ch1Pulse = 0;
int ch2Pulse = 0;
int ch3Pulse = 0;
int ch7Pulse = 0;
int physKillRaw = 0;
int remKillStatus = 1;
int physKillStatus = 1;
int killStatus = 1;
int lightColor = 0;         // light color variable (0 = off, 1 = white, 2 = red, 3 = yellow, 4 = blue)
int q1Setpoint = 0;         // Q1 setpoint from -1000 to 1000 (no acceleration limit)
int q2Setpoint = 0;         // Q2 setpoint from -1000 to 1000 (no acceleration limit)
int q3Setpoint = 0;         // Q3 setpoint from -1000 to 1000 (no acceleration limit)
int q4Setpoint = 0;         // Q4 setpoint from -1000 to 1000 (no acceleration limit)
float timeNow = 0;          // time of current iteration (for acceleration limit calculations)
float timeLast = 0;         // time of last iteration (for acceleration limit calculations)
int q1Out = 0;              // Q1 output from -1000 to 1000 (with acceleration limits)
int q2Out = 0;              // Q2 output from -1000 to 1000 (with acceleration limits)
int q3Out = 0;              // Q3 output from -1000 to 1000 (with acceleration limits)
int q4Out = 0;              // Q4 output from -1000 to 1000 (with acceleration limits)
int q1Last = 0;             // Q1 output for last iteration (for acceleration limit calculations)
int q2Last = 0;             // Q2 output for last iteration (for acceleration limit calculations)
int q3Last = 0;             // Q3 output for last iteration (for acceleration limit calculations)
int q4Last = 0;             // Q4 output for last iteration (for acceleration limit calculations)

void setup() {

  // Set pin modes
  pinMode(ch1Pin,INPUT_PULLUP);
  pinMode(ch2Pin,INPUT_PULLUP);
  pinMode(ch3Pin,INPUT_PULLUP);
  pinMode(ch7Pin,INPUT_PULLUP);
  pinMode(physKillPin,INPUT);     // this pin is physically pulled down to ground in box
  pinMode(killRelayPin,OUTPUT);
  pinMode(q1Pin,OUTPUT);
  pinMode(q2Pin,OUTPUT);
  pinMode(q3Pin,OUTPUT);
  pinMode(q4Pin,OUTPUT);
  pinMode(redPin,OUTPUT);
  pinMode(bluePin,OUTPUT);
  pinMode(greenPin,OUTPUT);
  
  // Set kill relay HIGH to KILL
  digitalWrite(killRelayPin,HIGH);
  
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
