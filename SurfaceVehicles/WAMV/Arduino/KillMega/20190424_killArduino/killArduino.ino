/* 
 *  All libraries available from Library Manager built into Arduino IDE.
 *  To access go to Sketch > Include Library > Manage Libraries
 *    QuickMedian: Search for "QuickMedianLib by Luis Llamas" and install
 *    
 *  Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
 *  Last update: 2019.03.05
 *     
 */

// Library inclusions
#include <Arduino.h>                    // Arduino inclusions
#include <QuickMedianLib.h>

// Pin definitions
const byte ch5Pin = 2;              // PWM in from receiver channel 5 (auto/manual mode switch)
const byte ch7Pin = 18;             // PWM in from receiver channel 7 (reverse contactor kill switch)
const byte ch8Pin = 19;             // PWM in from receiver channel 8 (remote kill switch)
const byte physicalKillPin = 3;     // digital in from physical kill switch
const byte killRelayPin = 6;        // digital out to kill relay
const byte revConKillPin = 4;       // digital out to reverse contactor
const byte modeCommPin = 7;         // digital out to main arduino communicating auto/manual mode
const byte revConKillCommPin = 8;   // digital out to main arduino communicating reverse contactor kill stats
const byte killCommPin = 9;         // digital out to main arduino communicating kill state
const byte blueLightPin = 11;       // digital out to blue light relay
const byte greenLightPin = 12;      // digital out to green light relay
const byte redLightPin = 13;        // digital out to red light relay

// Global system (constant) variables
const int nMedian = 8;            // number of readings to take to calculate median from controller inputs
const float receiverRR = 50;      // refresh rate of the receiver [Hz] (MUST BE A FLOAT!)
const int ch5PulseMin = 1160;     // minimum expected pulse width of ch5 [us]
const int ch5PulseMax = 2144;     // maximum expected pulse width of ch5 [us]
const int ch7PulseMin = 1128;     // minimum expected pulse width of ch7 [us]
const int ch7PulseMax = 2128;     // maximum expected pulse width of ch7 [us]
const int ch8PulseMin = 1540;     // minimum expected pulse width of ch8 [us]
const int ch8PulseMax = 2240;     // maximum expected pulse width of ch8 [us]
const int pulseTolerance = 150;   // tolerance range on minimum and maximum expected pulse widths [us]

// Global volatile variables
volatile int ch5PulseRaw = 0;   // channel 5 pulse width (for interrerupt service routine)
volatile int ch5Timer = 0;      // channel 5 pulse width timer (for interrerupt service routine)
volatile int ch7PulseRaw = 0;   // channel 5 pulse width (for interrerupt service routine)
volatile int ch7Timer = 0;      // channel 5 pulse width timer (for interrerupt service routine)
volatile int ch8PulseRaw = 0;   // channel 5 pulse width (for interrerupt service routine)
volatile int ch8Timer = 0;      // channel 5 pulse width timer (for interrerupt service routine)

// Global code variables
int ch5Filtered = 0;          // filtered output from ch5
int ch7Filtered = 0;          // filtered output from ch7
int ch8Filtered = 0;          // filtered output from ch8
int remoteKillStatus = 1;     // remote kill status variable
int modeStatus = 1;           // auto/manual mode status variable
int revConKillStatus = 1;     // reverse contactor kill status variable
int physicalKillStatus = 1;   // physical kill status variable
int killStatus = 1;           // overall kill status based on kill switches (HIGH for kill, LOW for UNKILL)





void setup() {

  // Set pin mode
  pinMode(ch5Pin,INPUT);
  pinMode(ch7Pin,INPUT);
  pinMode(ch8Pin,INPUT);
  pinMode(physicalKillPin,INPUT);
  pinMode(killRelayPin,OUTPUT);
  pinMode(modeCommPin,OUTPUT);
  pinMode(revConKillCommPin,OUTPUT);
  pinMode(revConKillPin, OUTPUT);
  pinMode(killCommPin,OUTPUT);
  pinMode(blueLightPin,OUTPUT);
  pinMode(greenLightPin,OUTPUT);
  pinMode(redLightPin,OUTPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch5Pin),ch5Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch7Pin),ch7Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Fall,FALLING);

  // Set buad rate
  Serial.begin(57600);

}

void loop() {

  // Filter noise from remote control switch inputs using meadian of multiple measurements
  filterSwitch();

  // Calculate status variables based on filtered switch inputs  
  switch2Status();

  // Communicate status variables to main Arduino
  status2mainArduino();

  // Command LED light based on status variables
  status2Led();

  // Short delay to prevent errors
  delay(5);

}
