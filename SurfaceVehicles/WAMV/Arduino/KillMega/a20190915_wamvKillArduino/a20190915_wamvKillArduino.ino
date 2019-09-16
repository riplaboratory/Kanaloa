/*
    All libraries available from Library Manager built into Arduino IDE.
    To access go to Sketch > Include Library > Manage Libraries
      PinChangeInterrupt: Search for "PinChangeInterrupt by NicoHood" and install
      QuickMedian: Search for "QuickMedianLib by Luis Llamas" and install

    Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
    Last update: 2019.09.15

    Note that the Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69).
*/

// Library inclusions
#include <Arduino.h>                    // Arduino inclusions
#include <ros.h>                        // ROS inclusions
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <QuickMedianLib.h>             // Quickmedianlib inclusions
#include <PinChangeInterrupt.h>         // Pin change interrupt inclusions
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

// Pin definitions
const byte ch4Pin = 13;             // PWM/PPM in from handheld RC receiver channel 4 (kill)
const byte physicalKillPin = 5;     // digital in from physical kill switch
const byte killCommPin = 7;         // digital in from kill arduino for communicating kill status
const byte q1KillPin = 22;          // digital out to Q1 kill relay
const byte q2KillPin = 23;          // digital out to Q2 kill relay
const byte q3KillPin = 24;          // digital out to Q3 kill relay
const byte q4KillPin = 25;          // digital out to Q4 kill relay

// Constant global variables
const int nMedian = 8;              // number of readings to take to calculate median from controller inputs
const int pulseTolerance = 150;     // tolerance range on minimum and maximum expected pulse widths [us]

// Volatile global variables
volatile bool ch4State = 0;               // channel 4 digital state
volatile int ch4PulseRaw = 0;             // channel 4 pulse width direct reading
volatile int ch4PulseArray[nMedian] = {}; // channel 4 pulse width circular buffer array
volatile float ch4Timer = 0;              // channel 4 pulse width timer

// Global code variables
int remoteKillStatus = 1;       // status of the remote kill switch
int physicalKillStatus = 1;     // status of the physical kill switch
int killStatus = 1;             // status of the kill switch from kill Arduino (kill = 1, unkill = 0)
int ch4PulseMedian;             // median of channel 4 pulse width measurements from handheld receiver

//// FrSky R9m transmitter-receiver pulse widths
//const int ch4PulseMin = 1052;       // minimum expected pulse width of ch4 [us]
//const int ch4PulseMax = 2052;       // maximum expected pulse width of ch4 [us]

// ImmersionRC EzUHF transmitter-receiver pulse
const int ch4PulseMin = 1012;       // minimum expected pulse width of ch4 [us]
const int ch4PulseMax = 2016;       // maximum expected pulse width of ch4 [us]

void setup() {

  const byte ch4Pin = 13;             // PWM/PPM in from handheld RC receiver channel 4 (kill)
  const byte physicalKillPin = 5;     // digital in from physical kill switch
  const byte killCommPin = 7;         // digital in from kill arduino for communicating kill status
  const byte q1KillPin = 22;          // digital out to Q1 kill relay
  const byte q2KillPin = 23;          // digital out to Q2 kill relay
  const byte q3KillPin = 24;          // digital out to Q3 kill relay
  const byte q4KillPin = 25;          // digital out to Q4 kill relay

  // Set pin mode
  pinMode(ch4Pin, INPUT_PULLUP);
  pinMode(physicalKillPin, INPUT);
  pinMode(killCommPin, OUTPUT);
  pinMode(q1KillPin, OUTPUT);
  pinMode(q2KillPin, OUTPUT);
  pinMode(q3KillPin, OUTPUT);
  pinMode(q4KillPin, OUTPUT);

  // Attach interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch4Pin), ch4Change, CHANGE);

  // Set buad rate
  Serial.begin(57600);

}

void loop() {

  // Read inputs
  readRemoteKill();         // read input from handheld receiver
  readPhysicalKill();       // read input from physical kill switch

  // Check interrupt timer
  checkInterruptTimeout();

  // Determine kill state
  setKillStatus();          // set kill status, unkill thrusters, and send unkill command to main arduino

  // Short delay to prevent errors
  delay(5);

}
