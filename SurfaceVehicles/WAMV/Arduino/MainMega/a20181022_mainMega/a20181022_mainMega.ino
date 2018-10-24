// Arduino library inclusions
#include <Arduino.h>                  // Arduino inclusions
#include <ros.h>                      // ROS inclusions
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <Wire.h>                     // Adafruit servo driver inclusions
#include <Adafruit_PWMServoDriver.h>

// Assign default I2C address to adafruit servo driver (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//// Pin definitions (global variables)
//const int voltMainPin = A1;     // analog in from voltage divider pin for reading main battery voltage
//const int tempPin = A3;         // analog in from temperature probe pin for reading temperature (optional)
//const byte ch1Pin = 11;         // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
//const byte ch4Pin = 14;         // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
//const byte ch5Pin = 15;         // PWM in from remote control receiver channel 5 (switches between manual and autonomous mode)
//const byte ch6Pin = 16;         // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward)
//const byte ch7Pin = 17;         // PWM in from remote control receiver channel 7 (switches reverse contactors on and off)
//const byte ch8Pin = 18;         // PWM in from remote control receiver channel 8 (kill signal)
//const byte killOutPin = 4;      // digital out to kill-switch Arduino in high current box (HIGH for kill, LOW for unkill)
//const byte statePin = 5;        // digital out to kill-switch Arduino in high current box (HIGH for autonomous, LOW for manual)
//const byte pwmQ1Pin = 6;        // PWM out to Q1 low pass filter
//const byte pwmQ2Pin = 8;        // PWM out to Q2 low pass filter
//const byte pwmQ3Pin = 9;        // PWM out to Q3 low pass filter
//const byte pwmQ4Pin = 10;       // PWM out to Q4 low pass filter
//const byte revConQ1Pin = 22;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
//const byte revConQ2Pin = 24;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
//const byte revConQ3Pin = 26;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
//const byte revConQ4Pin = 28;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
//const byte revConKillPin = 30;  // digital out to kill power to reversing contactors (HIGH to allow reversing contactors, LOW to put motors into BRAKE mode)

// Pin definitions (global variables)
const int voltMainPin = A1;     // analog in from voltage divider pin for reading main battery voltage
const int tempPin = A2;         // analog in from temperature probe pin for reading temperature (optional)
const byte ch1Pin = 18;         // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
const byte ch4Pin = 19;         // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
const byte ch5Pin = 20;         // PWM in from remote control receiver channel 5 (switches between manual and autonomous mode)
const byte ch6Pin = 21;         // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward)
const byte ch7Pin = 3;          // PWM in from remote control receiver channel 7 (switches reverse contactors on and off)
const byte ch8Pin = 2;          // PWM in from remote control receiver channel 8 (kill signal)
const byte remKillPin = 4;      // digital out to kill-switch Arduino in high current box (HIGH for kill, LOW for unkill)
const byte statePin = 5;        // digital out to kill-switch Arduino in high current box (HIGH for autonomous, LOW for manual)
const byte revConKillPin = 30;  // digital out to reversing contactor (HIGH for unkill, LOW for kill)
const byte pwmQ1Pin = 6;        // PWM out to Q1 low pass filter (only if PWM servo sheild is not being used)
const byte pwmQ2Pin = 8;        // PWM out to Q2 low pass filter (only if PWM servo sheild is not being used)
const byte pwmQ3Pin = 9;        // PWM out to Q3 low pass filter (only if PWM servo sheild is not being used)
const byte pwmQ4Pin = 10;       // PWM out to Q4 low pass filter (only if PWM servo sheild is not being used)
const byte revConQ1Pin = 22;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ2Pin = 24;    // digital out to Q2 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ3Pin = 26;    // digital out to Q3 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ4Pin = 28;    // digital out to Q4 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)

// Global variables (general)
int remKill = 1;                          // stores remote kill state: kill = 1, unkill = 2, invalid = 0
int mode = 2;                             // stores manual/auto state: autonomous = 1, manual = 2, invalid = 0
int revConKill = 1;                       // stores reversing contactor state: kill = 1, unkill = 2, invalid = 0
float voltMain = 0;                       // stores main battery voltage
unsigned long timeNow = 0;                // time tracking variable
unsigned long timeLast = 0;               // time tracking variable
const unsigned int voltOutMax = 36;       // maximum allowable output voltage to thrusters [V]
//const unsigned int pulseMax = 2000;   // maximum pulse length from receiver
//const unsigned int pulseMin = 980;    // minimum pulse length from receiver
const unsigned int pulseMax = 1475;       // maximum pulse length from receiver
const unsigned int pulseMin = 385;        // minimum pulse length from receiver
float autoQ1;                             // autonomous command signal from Matlab for Q1 thruster (via ROS)
float autoQ2;                             // autonomous command signal from Matlab for Q2 thruster (via ROS)
float autoQ3;                             // autonomous command signal from Matlab for Q3 thruster (via ROS)
float autoQ4;                             // autonomous command signal from Matlab for Q4 thruster (via ROS)
float Q1Setpoint = 0;                     // Setpoint for Q1 thruster output
float Q2Setpoint = 0;                     // Setpoint for Q2 thruster output
float Q3Setpoint = 0;                     // Setpoint for Q3 thruster output
float Q4Setpoint = 0;                     // Setpoint for Q4 thruster output
float Q1Out = 0;                          // Q1 thruster output (acceleration limited)
float Q2Out = 0;                          // Q2 thruster output (acceleration limited)
float Q3Out = 0;                          // Q3 thruster output (acceleration limited)
float Q4Out = 0;                          // Q4 thruster output (acceleration limited)
float Q1Last = 0;                         // last Q1 thruster output
float Q2Last = 0;                         // last Q2 thruster output
float Q3Last = 0;                         // last Q3 thruster output
float Q4Last = 0;                         // last Q4 thruster output
float Q1DacOut = 0;                       // bit output to Q1 DAC
float Q2DacOut = 0;                       // bit output to Q2 DAC
float Q3DacOut = 0;                       // bit output to Q3 DAC
float Q4DacOut = 0;                       // bit output to Q4 DAC
volatile unsigned int ch1PulseWidth = 0;  // channel 1 pulse width (for interrerupt service routine)
volatile unsigned int ch1Timer = 0;       // channel 1 pulse width timer (for interrerupt service routine)
volatile unsigned int ch4PulseWidth = 0;  // channel 4 pulse width (for interrerupt service routine)
volatile unsigned int ch4Timer = 0;       // channel 4 pulse width timer (for interrerupt service routine)
volatile unsigned int ch5PulseWidth = 0;  // channel 5 pulse width (for interrerupt service routine)
volatile unsigned int ch5Timer = 0;       // channel 5 pulse width timer (for interrerupt service routine)
volatile unsigned int ch6PulseWidth = 0;  // channel 6 pulse width (for interrerupt service routine)
volatile unsigned int ch6Timer = 0;       // channel 6 pulse width timer (for interrerupt service routine)
volatile unsigned int ch7PulseWidth = 0;  // channel 7 pulse width (for interrerupt service routine)
volatile unsigned int ch7Timer = 0;       // channel 7 pulse width timer (for interrerupt service routine)
volatile unsigned int ch8PulseWidth = 0;  // channel 8 pulse width (for interrerupt service routine)
volatile unsigned int ch8Timer = 0;       // channel 8 pulse width timer (for interrerupt service routine)

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;                                   // ROS node handle (allows program to create publishers and subscribers)

// ROS messages
std_msgs::UInt16 Q1Msg;                               // create Q1 thruster message
std_msgs::UInt16 Q2Msg;                               // create Q2 thruster message
std_msgs::UInt16 Q3Msg;                               // create Q3 thruster message
std_msgs::UInt16 Q4Msg;                               // create Q4 thruster message
std_msgs::Float64 voltMainMsg;                        // create main battery voltage message

// ROS publishers
ros::Publisher Q1Pub("Q1", &Q1Msg);                   // Q1 thruster publisher
ros::Publisher Q2Pub("Q2", &Q2Msg);                   // Q2 thruster publisher
ros::Publisher Q3Pub("Q3", &Q3Msg);                   // Q3 thruster publsiher
ros::Publisher Q4Pub("Q4", &Q4Msg);                   // Q4 thruster publisher
ros::Publisher voltMainPub("voltMain", &voltMainMsg); // main battery voltage publisher

// ROS Subscribers
void autoQ1Cb(const std_msgs::Float64& autoQ1CbMsg) {
  autoQ1 = autoQ1CbMsg.data;
}
ros::Subscriber<std_msgs::Float64> autoQ1sub("autoQ1", &autoQ1Cb);   // autonomous Q1 thruster output
void autoQ2Cb(const std_msgs::Float64& autoQ2CbMsg) {
  autoQ2 = autoQ2CbMsg.data;
}
ros::Subscriber<std_msgs::Float64> autoQ2sub("autoQ2", &autoQ2Cb);   // autonomous Q2 thruster output
void autoQ3Cb(const std_msgs::Float64& autoQ3CbMsg) {
  autoQ3 = autoQ3CbMsg.data;
}
ros::Subscriber<std_msgs::Float64> autoQ3sub("autoQ3", &autoQ3Cb);   // autonomous Q3 thruster output
void autoQ4Cb(const std_msgs::Float64& autoQ4CbMsg) {
  autoQ4 = autoQ4CbMsg.data;
}
ros::Subscriber<std_msgs::Float64> autoQ4sub("autoQ4", &autoQ4Cb);   // autonomous Q4 thruster output

void setup() {

  // Initialize ROS node handle
  nh.initNode();

  // Initialize ROS publishers
  nh.advertise(voltMainPub);    // main battery voltage
  nh.advertise(Q1Pub);          // Q1 thruster output
  nh.advertise(Q2Pub);          // Q2 thruster output
  nh.advertise(Q3Pub);          // Q3 thruster output
  nh.advertise(Q4Pub);          // Q4 thruster output

  // Initialize ROS subscibers
  nh.subscribe(autoQ1sub);      // autonomous Q1 thruster output
  nh.subscribe(autoQ2sub);      // autonomous Q2 thruster output
  nh.subscribe(autoQ3sub);      // autonomous Q3 thruster output
  nh.subscribe(autoQ4sub);      // autonomous Q4 thruster output

  // Handles Arduino timers for PWM frequency (for more info: https://playground.arduino.cc/Main/TimerPWMCheatsheet and http://forum.arduino.cc/index.php/topic,72092.0.html)
  int myEraser = 7;         // this is 111 in binary, and used as an eraser
  TCCR2B &= ~myEraser;      // sets three bits in TCCR2B (timer 2) to zero
  TCCR4B &= ~myEraser;      // sets three bits in TCCR4B (timer 4) to zero
  TCCR4B |= 4;              // Set timer 4 (pins 6 and 8) to prescaler 4 (120 Hz).
  TCCR2B |= 6;              // Set timer 2 (pins 10 and 9) to prescaler 6 (120 Hz).

  // Set pin mode
  pinMode(voltMainPin,INPUT);     // analog in from voltage divider pin for reading main battery voltage
  pinMode(tempPin,INPUT);         // analog in from temperature probe pin for reading temperature (optional)
  pinMode(ch1Pin,INPUT);          // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
  pinMode(ch4Pin,INPUT);          // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
  pinMode(ch5Pin,INPUT);          // PWM in from remote control receiver channel 5 (switches between manual and autonomous mode)
  pinMode(ch6Pin,INPUT);          // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward)
  pinMode(ch7Pin,INPUT);          // PWM in from remote control receiver channel 7 (switches reverse contactors on and off)
  pinMode(ch8Pin,INPUT);          // PWM in from remote control receiver channel 8 (kill signal)
  pinMode(remKillPin,OUTPUT);     // digital out to kill-switch Arduino (HIGH for kill, LOW for unkill)
  pinMode(statePin,OUTPUT);       // digital out to kill-switch Arduino (HIGH for autonomous, LOW for manual)
  pinMode(revConKillPin,OUTPUT);  // digital out to reversing contactor (HIGH for unkill, LOW for kill)
  pinMode(pwmQ1Pin,OUTPUT);       // PWM out to Q1 low pass filter
  pinMode(pwmQ2Pin,OUTPUT);       // PWM out to Q2 low pass filter
  pinMode(pwmQ3Pin,OUTPUT);       // PWM out to Q3 low pass filter
  pinMode(pwmQ4Pin,OUTPUT);       // PWM out to Q4 low pass filter
  pinMode(revConQ1Pin,OUTPUT);    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
  pinMode(revConQ2Pin,OUTPUT);    // digital out to Q2 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
  pinMode(revConQ3Pin,OUTPUT);    // digital out to Q3 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
  pinMode(revConQ4Pin,OUTPUT);    // digital out to Q4 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)

  // Initialize PWM shield
  pwm.begin();
  pwm.setPWMFreq(500);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Fall,FALLING);

  // Set serial baud rate
  Serial.begin(57600);
}

void loop() {

  // Set the kill switch pins high
  digitalWrite(remKillPin,HIGH);
  digitalWrite(revConKillPin,HIGH);

  // Write PWM to motor pins
  analogWrite(pwmQ1Pin,10);
  analogWrite(pwmQ2Pin,10);
  analogWrite(pwmQ3Pin,10);
  analogWrite(pwmQ4Pin,10);

  // Write PWM to servo shield
  float dc = 80;
  unsigned int pwmBit = round((dc/100)*4095);
  pwm.setPWM(0,0,pwmBit);

  // Print
//  Serial.print(ch1PulseWidth);
//  Serial.print(" ");
//  Serial.print(ch4PulseWidth);
//  Serial.print(" ");
//  Serial.print(ch5PulseWidth);
//  Serial.print(" ");
//  Serial.print(ch6PulseWidth);
//  Serial.print(" ");
//  Serial.print(ch7PulseWidth);
//  Serial.print(" ");
  Serial.print(ch8PulseWidth);
  Serial.println(" ");  

  delay(10);

}

//
//  // Read mode, revCon, and battery voltage pins
//  unsigned int ch8Raw = 0;
//  unsigned int ch5Raw = 0;
//  unsigned int ch7Raw = 0;
//
//  // Read kill switch pin
//  ch8Raw = pulseIn(ch8Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
//  Serial.print("ch8Raw:" );
//  Serial.println(ch8Raw);
//
//  // Read reversing contactor pin
//  ch7Raw = pulseIn(ch7Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
//  Serial.print("ch7Raw: ");
//  Serial.println(ch7Raw);
//
//
////  int i = 1;
////  while (i == 1) {
////    ch8Raw = pulseIn(ch8Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
////    Serial.print("ch8Raw:" );
////    Serial.println(ch8Raw);
////    unsigned long t0 = micros();
////    while (digitalRead(ch8Pin) == LOW) {}
////    unsigned long tl = micros() - t0;
////    if (tl > 2000) {
////      i = 0;
////    }
////  }
//////  unsigned int ch5Raw = pulseIn(ch5Pin,HIGH);     // PWM in from remote control receiver channel 5 (SF switch) for selecting manual (down) or autonomous (up) mode.
////  i = 1;
////  while (i == 1) {
////    ch5Raw = pulseIn(ch5Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
////    unsigned long t0 = micros();
////    while (digitalRead(ch5Pin) == LOW) {}
////    unsigned long tl = micros() - t0;
////    if (tl > 2000) {
////      i = 0;
////    }
////  }
////  unsigned int ch7Raw = pulseIn(ch7Pin,HIGH);     // PWM in from remote control receiver channel 7 (SB switch) for switching reverse contactor on and off
////  i = 1;
////  while (i == 1) {
////    ch7Raw = pulseIn(ch7Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
////    Serial.print("ch7Raw: ");
////    Serial.println(ch7Raw);
////    unsigned long t0 = micros();
////    while (digitalRead(ch7Pin) == LOW) {}
////    unsigned long tl = micros() - t0;
////    if (tl > 2000) {
////      i = 0;
////    }
////  }
//  readMainBatteryVoltage();                       // reads and smooths battery voltage by taking multiple analogReads and taking the median.
//
////  Serial.println(" ");
////  Serial.print("ch8Raw (kill signal): ");
////  Serial.println(ch8Raw);
////  Serial.print("ch5Raw (auto/man signal): ");
////  Serial.println(ch5Raw);
////  Serial.print("ch7Raw (revCon kill signal): ");
////  Serial.println(ch7Raw);
//
//  int tolerance = 50;
//
//  // Filter out noisy readings from remote kill signal
//  if (ch8Raw > (pulseMin - tolerance) && ch8Raw < (pulseMin + tolerance)) { remKill = 1; }        // switch is in the DOWN position (KILL)
////  else if (ch8Raw > (1500 - tolerance) && ch8Raw < (1500 + tolerance)) { remKill = 1; }           // switch is in the MIDDLE position (KILL)
//  else if (ch8Raw > (pulseMax - tolerance) && ch8Raw < (pulseMax + tolerance)) { remKill = 2; }   // switch is in the UP position (UNKILL)
//  else {
//    remKill = 0;
//    Serial.println("INVALID REMOTE KILL SELECTION ENGAGED. TRANSMITTER CONNECTION NOISY, RE-DOING READING!");
//    while(remKill == 0) {
////      int i = 1;
////      while (i == 1) {
////        ch8Raw = pulseIn(ch8Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
////        unsigned long t0 = micros();
////        while (digitalRead(ch8Pin) == LOW) {}
////        unsigned long tl = micros() - t0;
////        if (tl > 2000) {
////          i = 0;
////        }
////      }
//      ch8Raw = pulseIn(ch8Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
//      if (ch8Raw > (pulseMin - tolerance) && ch8Raw < (pulseMin + tolerance)) { remKill = 1; }      // switch is in the DOWN position (KILL)
//      else if (ch8Raw > (pulseMax - tolerance) && ch8Raw < (pulseMax + tolerance)) { remKill = 2; }  // switch is in the UP position (UNKILL)
//    Serial.print("ch8Raw (re-done):" );
//    Serial.println(ch8Raw);
//    }
//  }
//
////  // Filter out noisy readings from auto/manual mode select signal
////  if (ch5Raw > (pulseMin - tolerance) && ch5Raw < (pulseMin + tolerance)) { mode = 1; }       // switch is in the DOWN position (MANUAL)
////  else if (ch5Raw > (pulseMax - tolerance) && ch5Raw < (pulseMax + tolerance)) { mode = 2; }   // switch is in the UP position (AUTONOMOUS)
////  else {
////    mode = 0;
////    Serial.println("INVALID AUTO/MANUAL MODE SELECTION ENGAGED. TRANSMITTER CONNECTION NOISY, RE-DOING READING!");
////    while(mode == 0) {
////      i = 1;
////      while (i == 1) {
////        ch5Raw = pulseIn(ch5Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
////        unsigned long t0 = micros();
////        while (digitalRead(ch5Pin) == LOW) {}
////        unsigned long tl = micros() - t0;
////        if (tl > 2000) {
////          i = 0;
////        }
////      }
////      if (ch5Raw > (pulseMin - tolerance) && ch5Raw < (pulseMin + tolerance)) { mode = 1; }       // switch is in the DOWN position (MANUAL)
////      else if (ch5Raw > (pulseMax - tolerance) && ch5Raw < (pulseMax + tolerance)) { mode = 2; }  // switch is in the UP position (AUTONOMOUS)
//////      Serial.print(ch5Raw);
//////      Serial.print(" ");
//////      Serial.println(mode);
////    }
//////    Serial.print(ch5Raw);
//////    Serial.print(" ");
//////    Serial.println(mode);
////  }
//
//
//  // Filter out noisy readings from reverse contactor kill signal
//  if (ch7Raw > (900 - tolerance) && ch7Raw < (900 + tolerance)) { revConKill = 2; }         // switch is in the UP position (UNKILL)
////  else if (ch7Raw > (1500 - tolerance) && ch7Raw < (1500 + tolerance)) { revConKill = 1; }            // switch is in the MIDDLE position (KILL)
//  else if (ch7Raw > (2000 - tolerance) && ch7Raw < (2000 + tolerance)) { revConKill = 1; }    // switch is in the DOWN position (KILL)
//  else {
//    remKill = 0;
//    Serial.println("INVALID REVERSE CONTACTOR KILL SELECTION ENGAGED. TRANSMITTER CONNECTION NOISY, RE-DOING READING!");
//    while(revConKill == 0) {
////      i = 1;
////      while (i == 1) {
////        ch7Raw = pulseIn(ch7Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
////        unsigned long t0 = micros();
////        while (digitalRead(ch7Pin) == LOW) {}
////        unsigned long tl = micros() - t0;
////        if (tl > 2000) {
////          i = 0;
////        }
////      }
//      ch7Raw = pulseIn(ch7Pin,HIGH);     // PWM in from remote control receiver channel 8 (SD switch) for selecting remote kill (down) or unkill (up) mode.
//      if (ch7Raw > (900 - tolerance) && ch7Raw < (900 + tolerance)) { revConKill = 2; }         // switch is in the UP position (UNKILL)
////      else if (ch7Raw > (1500 - tolerance) && ch7Raw < (1500 + tolerance)) { revConKill = 1; }            // switch is in the MIDDLE position (KILL)
//      else if (ch7Raw > (2000 - tolerance) && ch7Raw < (2000 + tolerance)) { revConKill = 1; }    // switch is in the DOWN position (KILL)
//      Serial.print("ch7Raw: ");
//      Serial.println(ch7Raw);
//    }
//  }
//
//  // Apply kill switch state
//  if (remKill == 2) {                 // transmitter wants unkill
//    digitalWrite(remKillPin, HIGH);
//  }
//  else {
//    digitalWrite(remKillPin, LOW);   // transmitter wants kill
//  }
//
//  // Apply reverse contactor state
//  if (revConKill == 2) {              // transmitter wants reverse contactor unkill
//    digitalWrite(revConKillPin,LOW);  // unkill the reverse contactor
//    // Check if all output signals are zero (deadzone), if so, kill the reverse contactor
//    if (Q1Out == 0 && Q2Out == 0 && Q3Out == 0 && Q4Out == 0) {
//      digitalWrite(revConKillPin,HIGH);   // kill the reverse contactor
//      Serial.println("killing reverse contactor (in controller deadzone)........");
//    }
//  }
//  else {                              // transmitter wants reverse contactor kill
//    digitalWrite(revConKillPin,HIGH); // kill the reverse contactor
//  }

//  // Apply kill switch state
//    digitalWrite(killOutPin, HIGH); // unkill remote kill switch
//    digitalWrite(revConKillPin,LOW);  // unkill the reverse contactor
//
//  // Manual Code
//  if (mode == 1) {
//
//    // Send WAM-V state to kill-switch Arduino
//    digitalWrite(statePin, LOW);    // LOW means WAM-V is in manual-control state
//
//    // Read signals from manual controller and encode to thruster setpoint components
//    joy2setpoint();
//
//    // Calculate thruster output components                                                                                                                                                                                                                                                                                                                                                                                                                                                                based thruster setpoint components and acceleration limit
//    setpoint2output();
//
//    // Write output thrust components
//    output2dacRevCon();

//    // Print
//    Serial.print("Thruster setpoints:   ");
//    Serial.print(Q1Setpoint);
//    Serial.print(" ");
//    Serial.print(Q2Setpoint);
//    Serial.print(" ");
//    Serial.print(Q3Setpoint);
//    Serial.print(" ");
//    Serial.println(Q4Setpoint);
//    Serial.print("Thruster outputs:     ");
//    Serial.print(Q1Out);
//    Serial.print(" ");
//    Serial.print(Q2Out);
//    Serial.print(" ");
//    Serial.print(Q3Out);
//    Serial.print(" ");
//    Serial.println(Q4Out);
//    Serial.print("Thruster bit output:  ");
//    Serial.print(Q1DacOut);
//    Serial.print(" ");
//    Serial.print(Q2DacOut);
//    Serial.print(" ");
//    Serial.print(Q3DacOut);
//    Serial.print(" ");
//    Serial.println(Q4DacOut);
//    Serial.println(" ");
//  }
//
//  // Autonomous Code
//  else if (mode == 2) {
//
//    // Send WAM-V state to kill-switch Arduino
//    digitalWrite(statePin,HIGH);         // HIGH means WAM-V is in autonomous state
//
//    Serial.println("AUTONOMOUS MODE ENGAGED (WHICH DOES NOTHING OM NOM NOM NOM)");
//    Serial.println(" ");
//    //I2c.scan();
//  }

//  // Invalid input code
//  else {
//
//
//
//    // Send WAM-V state to kill-switch Arduino
////    digitalWrite(statePin,LOW);    // LOW means WAM-V is in manual-control state
//
//    // Re-do reading
//    Serial.println("INVALID MODE SELECTION ENGAGED. TRANSMITTER CONNECTION NOISY, RE-DOING READING!");
//    Serial.println(" ");
//
////  }
//
//  // Small delay to prevent errors
//  delay(5);
//
//  // Spin once to refresh ROS callbacks
//  nh.spinOnce();

//}
//
//void readMainBatteryVoltage() {
//  // Reads and smooths battery voltage by taking multiple analogReads and taking the median.
//  // Note that reading is still very rough (+/-5V) even when doing this.
//
//  float calib = 0.0505;
//  unsigned long total = 0;
//  int voltMainRawMean = 0;
//  int voltMainRawMedian = 0;
//  const unsigned int nReadings = 50;
//  int readings[nReadings];
//
//  // Instantiate readings array with zeros
//  for (unsigned int thisReading = 0; thisReading < nReadings; thisReading++) {
//    readings[thisReading] = 0;
//  }
//
//  // Take readings
//  for (unsigned int thisReading = 0; thisReading < nReadings; thisReading++) {
//    total = total - readings[thisReading];
//    readings[thisReading] = analogRead(voltMainPin);
//    total = total + readings[thisReading];
//    delay(1);
//  }
//
//  // Sort readings
//  sortArray(readings,nReadings);                // sort array to take median
//
//  // Calculate mean and median
//  voltMainRawMean = total / nReadings;          // mean (doesn't get used)
//  voltMainRawMedian = readings[nReadings / 2];  // median
//
//  // Convert to calibrated voltage
//  voltMain = voltMainRawMedian * calib;         // convert voltMainRaw to voltMainRaw
//
//  // Catching code for zero read voltage
//  if (voltMain < 5 && voltMain > -5) {
//    Serial.println("CAUTION: Main battery voltage not detected!  Main battery is probably not plugged in!");
//    Serial.println("Setting voltMain = 50 for debugging purposes...");
//    voltMain = 50;
//  }

// Publish to ROS
//  voltMainMsg.data = voltMain;          // set voltMain message
//  voltMainPub.publish(&voltMainMsg);    // publish voltMain topic

//}/
