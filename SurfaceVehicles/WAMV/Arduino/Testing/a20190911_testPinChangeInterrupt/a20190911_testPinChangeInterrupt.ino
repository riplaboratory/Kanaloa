/*
    All libraries available from Library Manager built into Arduino IDE.
    To access go to Sketch > Include Library > Manage Libraries
      PinChangeInterrupt: Search for "PinChangeInterrupt by NicoHood" and install
      QuickMedian: Search for "QuickMedianLib by Luis Llamas" and install

    Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
    Last update: 2019.09.11

    The Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,
    51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69).

*/

// Library inclusions
#include <QuickMedianLib.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

// Pin definitions
// Note: the Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69).
const byte ch1Pin = 10;   // PWM/PPM in from handheld RC receiver channel 1 (surge)
const byte ch2Pin = 11;   // PWM/PPM in from handheld RC receiver channel 2 (sway)
const byte ch3Pin = 12;   // PWM/PPM in from handheld RC receiver channel 3 (mode)
const byte ch4Pin = 13;   // PWM/PPM in from handheld RC receiver channel 4 (kill)
const byte ch5Pin = 50;   // PWM/PPM in from handheld RC receiver channel 5 (LC batt)
const byte ch6Pin = 52;   // PWM/PPM in from handheld RC receiver channel 7 (HC batt)

// Constant global variables
const int nMedian = 8;          // number of readings to take to calculate median from controller inputs (don't make this too large, or else it will slow down the interrupt service routines)
const int ch1PulseMin = 1032;     // minimum expected pulse width of ch1 [us] (this should be individually tested for each channel)
const int ch1PulseMax = 2032;     // maximum expected pulse width of ch1 [us] (this should be individually tested for each channel)
const int ch2PulseMin = 1072;     // minimum expected pulse width of ch2 [us] (this should be individually tested for each channel)
const int ch2PulseMax = 2072;     // maximum expected pulse width of ch2 [us] (this should be individually tested for each channel)
const int ch3PulseMin = 1160;     // minimum expected pulse width of ch3 [us] (this should be individually tested for each channel)
const int ch3PulseMax = 2160;     // maximum expected pulse width of ch3 [us] (this should be individually tested for each channel)
const int ch4PulseMin = 1200;     // minimum expected pulse width of ch4 [us] (this should be individually tested for each channel)
const int ch4PulseMax = 2192;     // maximum expected pulse width of ch4 [us] (this should be individually tested for each channel)
const int ch5PulseMin = 1248;     // minimum expected pulse width of ch5 [us] (this should be individually tested for each channel)
const int ch5PulseMax = 2208;     // maximum expected pulse width of ch5 [us] (this should be individually tested for each channel)
const int ch6PulseMin = 1120;     // minimum expected pulse width of ch7 [us] (this should be individually tested for each channel)
const int ch6PulseMax = 2128;     // maximum expected pulse width of ch7 [us] (this should be individually tested for each channel)
const int pulseTolerance = 200;   // tolerance range on minimum and maximum expected pulse widths [us]

// Volatile global variables
volatile bool ch1State = 0;               // channel 1 digital state
volatile int ch1PulseRaw = 0;             // channel 1 pulse width direct reading
volatile int ch1PulseArray[nMedian] = {}; // channel 1 pulse width circular buffer array
volatile float ch1Timer = 0;              // channel 1 pulse width timer
volatile bool ch2State = 0;               // channel 2 digital state
volatile int ch2PulseRaw = 0;             // channel 2 pulse width direct reading
volatile int ch2PulseArray[nMedian] = {}; // channel 2 pulse width circular buffer array
volatile float ch2Timer = 0;              // channel 2 pulse width timer
volatile bool ch3State = 0;               // channel 3 digital state
volatile int ch3PulseRaw = 0;             // channel 3 pulse width direct reading
volatile int ch3PulseArray[nMedian] = {}; // channel 3 pulse width circular buffer array
volatile float ch3Timer = 0;              // channel 3 pulse width timer
volatile bool ch4State = 0;               // channel 4 digital state
volatile int ch4PulseRaw = 0;             // channel 4 pulse width direct reading
volatile int ch4PulseArray[nMedian] = {}; // channel 4 pulse width circular buffer array
volatile float ch4Timer = 0;              // channel 4 pulse width timer
volatile bool ch5State = 0;               // channel 5 digital state
volatile int ch5PulseRaw = 0;             // channel 5 pulse width direct reading
volatile int ch5PulseArray[nMedian] = {}; // channel 5 pulse width circular buffer array
volatile float ch5Timer = 0;              // channel 5 pulse width timer
volatile bool ch6State = 0;               // channel 6 digital state
volatile int ch6PulseRaw = 0;             // channel 6 pulse width direct reading
volatile int ch6PulseArray[nMedian] = {}; // channel 6 pulse width circular buffer array
volatile float ch6Timer = 0;              // channel 6 pulse width timer

void setup() {

  // Set pin mode
  pinMode(ch1Pin, INPUT_PULLUP);
  pinMode(ch2Pin, INPUT_PULLUP);
  pinMode(ch3Pin, INPUT_PULLUP);
  pinMode(ch4Pin, INPUT_PULLUP);
  pinMode(ch5Pin, INPUT_PULLUP);
  pinMode(ch6Pin, INPUT_PULLUP);

  // Attach pin change interrupt
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch1Pin), ch1Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch2Pin), ch2Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch3Pin), ch3Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch4Pin), ch4Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch5Pin), ch5Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch6Pin), ch6Change, CHANGE);

  // Set serial baud rate
  Serial.begin(57600);

}

void loop() {

  // Create local, non-volatile copy of pulse widths
  int ch1PulseFilt[nMedian] = {};
  memcpy(ch1PulseFilt, ch1PulseArray, nMedian * 2);
  int ch2PulseFilt[nMedian] = {};
  memcpy(ch2PulseFilt, ch2PulseArray, nMedian * 2);
  int ch3PulseFilt[nMedian] = {};
  memcpy(ch3PulseFilt, ch3PulseArray, nMedian * 2);
  int ch4PulseFilt[nMedian] = {};
  memcpy(ch4PulseFilt, ch4PulseArray, nMedian * 2);
  int ch5PulseFilt[nMedian] = {};
  memcpy(ch5PulseFilt, ch5PulseArray, nMedian * 2);
  int ch6PulseFilt[nMedian] = {};
  memcpy(ch6PulseFilt, ch6PulseArray, nMedian * 2);

  // Take median of pulse width arrays
  int ch1PulseMed = QuickMedian<int>::GetMedian(ch1PulseFilt, nMedian);
  int ch2PulseMed = QuickMedian<int>::GetMedian(ch2PulseFilt, nMedian);
  int ch3PulseMed = QuickMedian<int>::GetMedian(ch3PulseFilt, nMedian);
  int ch4PulseMed = QuickMedian<int>::GetMedian(ch4PulseFilt, nMedian);
  int ch5PulseMed = QuickMedian<int>::GetMedian(ch5PulseFilt, nMedian);
  int ch6PulseMed = QuickMedian<int>::GetMedian(ch6PulseFilt, nMedian);

  // Print pulse widths
  Serial.print("Ch1 (");
  Serial.print(ch1PulseMed);
  Serial.print("); Ch2 (");
  Serial.print(ch2PulseMed);
  Serial.print("); Ch3 (");
  Serial.print(ch3PulseMed);
  Serial.print("); Ch4 (");
  Serial.print(ch4PulseMed);
  Serial.print("); Ch5 (");
  Serial.print(ch5PulseMed);
  Serial.print("); Ch6 (");
  Serial.print(ch6PulseMed);
  Serial.println(");");

  // Short delay
  delay(10);

}








void ch1Change() {

  // Check the current state of the pin
  ch1State = digitalRead(ch1Pin);

  if (ch1State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch1Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch1PulseRaw = micros() - ch1Timer;

    // Only commit reading to memory if it's within expected range
    if (ch1PulseRaw >= ch1PulseMin - pulseTolerance && ch1PulseRaw <= ch1PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch1PulseArray, &ch1PulseArray[1], sizeof(ch1PulseArray) - sizeof(int));
      ch1PulseArray[nMedian - 1] = ch1PulseRaw;

    }

  }

}

void ch2Change() {

  // Check the current state of the pin
  ch2State = digitalRead(ch2Pin);

  if (ch2State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch2Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch2PulseRaw = micros() - ch2Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch2PulseRaw >= ch2PulseMin - pulseTolerance && ch2PulseRaw <= ch2PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch2PulseArray, &ch2PulseArray[1], sizeof(ch2PulseArray) - sizeof(int));
      ch2PulseArray[nMedian - 1] = ch2PulseRaw;

    }

  }

}

void ch3Change() {

  // Check the current state of the pin
  ch3State = digitalRead(ch3Pin);

  if (ch3State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch3Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch3PulseRaw = micros() - ch3Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch3PulseRaw >= ch3PulseMin - pulseTolerance && ch3PulseRaw <= ch3PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch3PulseArray, &ch3PulseArray[1], sizeof(ch3PulseArray) - sizeof(int));
      ch3PulseArray[nMedian - 1] = ch3PulseRaw;

    }

  }

}

void ch4Change() {

  // Check the current state of the pin
  ch4State = digitalRead(ch4Pin);

  if (ch4State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch4Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch4PulseRaw = micros() - ch4Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch4PulseRaw >= ch4PulseMin - pulseTolerance && ch4PulseRaw <= ch4PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch4PulseArray, &ch4PulseArray[1], sizeof(ch4PulseArray) - sizeof(int));
      ch4PulseArray[nMedian - 1] = ch4PulseRaw;

    }

  }

}

void ch5Change() {

  // Check the current state of the pin
  ch5State = digitalRead(ch5Pin);

  if (ch5State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch5Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch5PulseRaw = micros() - ch5Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch5PulseRaw >= ch5PulseMin - pulseTolerance && ch5PulseRaw <= ch5PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch5PulseArray, &ch5PulseArray[1], sizeof(ch5PulseArray) - sizeof(int));
      ch5PulseArray[nMedian - 1] = ch5PulseRaw;

    }

  }

}

void ch6Change() {

  // Check the current state of the pin
  ch6State = digitalRead(ch6Pin);

  if (ch6State == 1) {
    // Pin is high, this was probably a rise event

    // Save current time to timer
    ch6Timer = micros();

  }

  else {
    // Pin is low, this was probably a fall event

    // Subtract ch1Time from current time
    ch6PulseRaw = micros() - ch6Timer;

    // Only commit reading to global variable if it's within expected range
    if (ch6PulseRaw >= ch6PulseMin - pulseTolerance && ch6PulseRaw <= ch6PulseMax + pulseTolerance) {

      // Move new measurement into circular buffer array
      memcpy(ch6PulseArray, &ch6PulseArray[1], sizeof(ch6PulseArray) - sizeof(int));
      ch6PulseArray[nMedian - 1] = ch6PulseRaw;

    }

  }

}
