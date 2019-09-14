/*
    All libraries available from Library Manager built into Arduino IDE.
    To access go to Sketch > Include Library > Manage Libraries
      ROS: Search for "Rosserial Arduino Library by Michael Ferguson" and install.
      PinChangeInterrupt: Search for "PinChangeInterrupt by NicoHood" and install
      QuickMedian: Search for "QuickMedianLib by Luis Llamas" and install

    Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
    Last update: 2019.09.13

    Note that the Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69).
*/

// Library inclusions
#include <Arduino.h>                    // Arduino inclusions
#include <QuickMedianLib.h>             // Quickmedianlib inclusions
#include <PinChangeInterrupt.h>         // Pin change interrupt inclusions
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

// Pin definitions
const byte killCommPin = 7;         // digital in from kill arduino for communicating kill status
const byte ch1Pin = 10;             // PWM/PPM in from handheld RC receiver channel 1 (surge)
const byte ch2Pin = 11;             // PWM/PPM in from handheld RC receiver channel 2 (yaw)
const byte ch3Pin = 12;             // PWM/PPM in from handheld RC receiver channel 3 (mode)
//const byte ch4Pin = 13;             // (this is on the kill arduino) PWM/PPM in from handheld RC receiver channel 4 (kill)
const byte ch5Pin = 50;             // PWM/PPM in from handheld RC receiver channel 5 (main batt)
const byte ch6Pin = 52;             // PWM/PPM in from handheld RC receiver channel 7 (LC batt)
const int voltLowCurrentPin = A1;   // analog in from voltage divider pin for reading low current battery voltage
const byte redPin = 35;             // digital out to red light relay
const byte greenPin = 37;           // digital out to green light relay
const byte bluePin = 39;            // digital out to blue light relay

// Constant global variables
const int nMedian = 8;          // number of readings to take to calculate median from controller inputs (don't make this too large, or else it will slow down the interrupt service routines)
const int ch1PulseMin = 1032;     // minimum expected pulse width of ch1 [us] (this should be individually tested for each channel)
const int ch1PulseMax = 2032;     // maximum expected pulse width of ch1 [us] (this should be individually tested for each channel)
const int ch2PulseMin = 1072;     // minimum expected pulse width of ch2 [us] (this should be individually tested for each channel)
const int ch2PulseMax = 2072;     // maximum expected pulse width of ch2 [us] (this should be individually tested for each channel)
const int ch3PulseMin = 1160;     // minimum expected pulse width of ch3 [us] (this should be individually tested for each channel)
const int ch3PulseMax = 2160;     // maximum expected pulse width of ch3 [us] (this should be individually tested for each channel)
const int ch5PulseMin = 1248;     // minimum expected pulse width of ch5 [us] (this should be individually tested for each channel)
const int ch5PulseMax = 2208;     // maximum expected pulse width of ch5 [us] (this should be individually tested for each channel)
const int ch6PulseMin = 1120;     // minimum expected pulse width of ch7 [us] (this should be individually tested for each channel)
const int ch6PulseMax = 2128;     // maximum expected pulse width of ch7 [us] (this should be individually tested for each channel)
const int pulseTolerance = 200;   // tolerance range on minimum and maximum expected pulse widths [us]
const int pulseDeadzone = 125;    // deadzone range at the center of each joystick [us]

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
volatile bool ch5State = 0;               // channel 5 digital state
volatile int ch5PulseRaw = 0;             // channel 5 pulse width direct reading
volatile int ch5PulseArray[nMedian] = {}; // channel 5 pulse width circular buffer array
volatile float ch5Timer = 0;              // channel 5 pulse width timer
volatile bool ch6State = 0;               // channel 6 digital state
volatile int ch6PulseRaw = 0;             // channel 6 pulse width direct reading
volatile int ch6PulseArray[nMedian] = {}; // channel 6 pulse width circular buffer array
volatile float ch6Timer = 0;              // channel 6 pulse width timer

// Global code variables
int killStatus = 1;             // status of the kill switch from kill Arduino (kill = 1, unkill = 0)
int ch1PulseMedian;             // median of channel 1 pulse width measurements from handheld receiver
int ch2PulseMedian;             // median of channel 2 pulse width measurements from handheld receiver
int ch3PulseMedian;             // median of channel 3 pulse width measurements from handheld receiver
int ch5PulseMedian;             // median of channel 5 pulse width measurements from handheld receiver
int ch6PulseMedian;             // median of channel 6 pulse width measurements from handheld receiver
float voltLowCurrentBatt;       // low current battery voltage
float voltMainBatt;             // main battery voltage
int mode = 1;                   // reflects auto/manual mode (manual = 1, autonomous = 2, invalid = 0)
int mainBattCheck = 0;          // reflects main battery check mode (yes = 1, no = 2, invalid = 0)
int lowCurrentBattCheck = 0;    // reflects low current battery check mode (yes = 1, no = 2, invalid = 0)
int proceedPastColor = 0;       // reflects whether code can proceed past light color assignment
int leftThrusterSetpoint = 0;   // setpoint command for left thruster
int rightThrusterSetpoint = 0;  // setpoint command for right thruster

//int revConKillStatus = 1;   // status of reverse contactor kill switch from kill Arduino
//int modeStatus = 1;         // auto/manual status from kill Arduino
//int ch1Filtered = 0;        // filtered output from ch1
//int ch4Filtered = 0;        // filtered output from ch4
//int ch6Filtered = 0;        // filtered output from ch6
//int q1Setpoint = 0;         // Q1 setpoint from -1000 to 1000 (no acceleration limit)
//int q2Setpoint = 0;         // Q2 setpoint from -1000 to 1000 (no acceleration limit)
//int q3Setpoint = 0;         // Q3 setpoint from -1000 to 1000 (no acceleration limit)
//int q4Setpoint = 0;         // Q4 setpoint from -1000 to 1000 (no acceleration limit)
//float timeNow = 0;          // time of current iteration (for acceleration limit calculations)
//float timeLast = 0;         // time of last iteration (for acceleration limit calculations)
//int q1Out = 0;              // Q1 output from -1000 to 1000 (with acceleration limits)
//int q2Out = 0;              // Q2 output from -1000 to 1000 (with acceleration limits)
//int q3Out = 0;              // Q3 output from -1000 to 1000 (with acceleration limits)
//int q4Out = 0;              // Q4 output from -1000 to 1000 (with acceleration limits)
//int q1Last = 0;             // Q1 output for last iteration (for acceleration limit calculations)
//int q2Last = 0;             // Q2 output for last iteration (for acceleration limit calculations)
//int q3Last = 0;             // Q3 output for last iteration (for acceleration limit calculations)
//int q4Last = 0;             // Q4 output for last iteration (for acceleration limit calculations)
//int q1Dir = 0;              // Q1 direction (0 = forward, 1 = reverse)
//int q2Dir = 0;              // Q2 direction (0 = forward, 1 = reverse)
//int q3Dir = 0;              // Q3 direction (0 = forward, 1 = reverse)
//int q4Dir = 0;              // Q4 direction (0 = forward, 1 = reverse)
//float voltMult = 1;      // voltage multiplier to scale output to thrusters (based on battery voltage)
//float autoQ1 = 0;           // autonomous command signal from ROS for Q1 thruster (via ROS)
//float autoQ2 = 0;           // autonomous command signal from ROS for Q2 thruster (via ROS)
//float autoQ3 = 0;           // autonomous command signal from ROS for Q3 thruster (via ROS)
//float autoQ4 = 0;           // autonomous command signal from ROS for Q4 thruster (via ROS)

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;         // ROS node handle (allows program to create publishers and subscribers)
std_msgs::Int32 int_msg;
std_msgs::String str_msg;

// ROS variables
int q1_thrust = 0;
int q2_thrust = 0;
int q3_thrust = 0;
int q4_thrust = 0;
//int q1_thrust_pwm;
//int q2_thrust_pwm;
//int q3_thrust_pwm;
//int q4_thrust_pwm;

// ROS subscribers
void thrust_input_q1( const std_msgs::Int32& input_msg) {
  q1_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}

void thrust_input_q2(const std_msgs::Int32& input_msg) {
  q2_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}

void thrust_input_q3( const std_msgs::Int32& input_msg) {
  q3_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}

void thrust_input_q4( const std_msgs::Int32& input_msg) {
  q4_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}

// ROS publishers
ros::Subscriber<std_msgs::Int32> q1_sub("q1_thruster_input", &thrust_input_q1 );      //sub to number_topic
ros::Subscriber<std_msgs::Int32> q2_sub("q2_thruster_input", &thrust_input_q2 );      //sub to number_topic
ros::Subscriber<std_msgs::Int32> q3_sub("q3_thruster_input", &thrust_input_q3 );      //sub to number_topic
ros::Subscriber<std_msgs::Int32> q4_sub("q4_thruster_input", &thrust_input_q4 );      //sub to number_topic

void setup() {

  // Set pin mode
  pinMode(killCommPin, INPUT_PULLUP);
  pinMode(ch1Pin, INPUT_PULLUP);
  pinMode(ch2Pin, INPUT_PULLUP);
  pinMode(ch3Pin, INPUT_PULLUP);
  pinMode(ch5Pin, INPUT_PULLUP);
  pinMode(ch6Pin, INPUT_PULLUP);
  pinMode(voltLowCurrentPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  // Attach pin change interrupt
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch1Pin), ch1Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch2Pin), ch2Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch3Pin), ch3Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch5Pin), ch5Change, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ch6Pin), ch6Change, CHANGE);

  // Set serial baud rate
  Serial.begin(57600);

  // Initialize ROS node handle
  nh.initNode();

  // Initialize ROS subscibers
  nh.subscribe(q1_sub);      //initialize ros subscriber
  nh.subscribe(q2_sub);      //initialize ros subscriber
  nh.subscribe(q3_sub);      //initialize ros subscriber
  nh.subscribe(q4_sub);      //initialize ros subscriber

}

void loop() {

  // Read the various inputs
  readKillArduino();                // read kill status from kill arduino
  readHandheldReceiver();           // read inputs from handheld receiver
  readLowCurrentBatteryVoltage();   // read low current battery voltage
  readMainBatteryVoltage();         // read main battery voltage

  // FOR DEBUG PURPOSES ONLY:
  killStatus = 0;

  // Determine mode
  determineMode();

  // Control light
  controlLight();

  if (killStatus == 0) {

    // Manual code goes here
    if (mode == 1) {

      // Take joystick readings, and convert to setpoint thrust values
      joy2Setpoint();

//      Serial.print("LEFT (");
//      Serial.print(leftThrusterSetpoint);
//      Serial.print("); RIGHT (");
//      Serial.print(rightThrusterSetpoint);
//      Serial.println(");");
  
    }

    // Autonomous code goes here
    else if (mode == 2) {
  
      Serial.println("AUTONOMOUS MODE: One day, I shall achieve autonomy! OM NOM NOM NOM!");
      
    }

    else {
      
      Serial.println("Error in mode selection. Check transmitter connection.");
      
    }
  } 
  
  else {
    
    Serial.println("System is killed, waiting for the unkill command");
    
  }

  // Small delay to prevent errors
  delay(5);

  // Spin ROS
  nh.spinOnce();

}
