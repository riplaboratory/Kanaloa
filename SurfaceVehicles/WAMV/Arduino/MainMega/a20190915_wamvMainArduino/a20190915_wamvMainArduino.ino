/*
    All libraries available from Library Manager built into Arduino IDE.
    To access go to Sketch > Include Library > Manage Libraries
      ROS: Search for "Rosserial Arduino Library by Michael Ferguson" and install.
      PinChangeInterrupt: Search for "PinChangeInterrupt by NicoHood" and install
      QuickMedian: Search for "QuickMedianLib by Luis Llamas" and install

    Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
    Last update: 2019.09.15

    Note that the Arduino Mega 2560 can use pin change interrupts on pins 10,11,12,13,50,51,52,53,A8(62),A9(63),A10(64),A11(65),A12(66),A13(67),A14(68),A15(69).
*/

// Library inclusions
#include <Arduino.h>                    // Arduino inclusions
#include <QuickMedianLib.h>             // Quickmedianlib inclusions
#include <Wire.h>                       // I2C library inclusions
#include <PinChangeInterrupt.h>         // Pin change interrupt inclusions
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <ros.h>                        // ROS inclusions
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Definitions
#define SLAVE_ADDR  9    // arbitrary address number to identify slave Arduino
#define ANSWERSIZE  14   // size of message to send to master Arduino
#define MESSAGESIZE 5    // size of message received from master Arduino

// Pin definitions
const byte killCommPin = 7;         // digital in from kill arduino for communicating kill status
const byte ch1Pin = 10;             // PWM/PPM in from handheld RC receiver channel 1 (surge)
const byte ch2Pin = 11;             // PWM/PPM in from handheld RC receiver channel 2 (yaw)
const byte ch3Pin = 12;             // PWM/PPM in from handheld RC receiver channel 3 (mode)
const byte ch5Pin = 50;             // PWM/PPM in from handheld RC receiver channel 5 (main batt)
const byte ch6Pin = 52;             // PWM/PPM in from handheld RC receiver channel 7 (LC batt)
const int voltLowCurrentPin = A1;   // analog in from voltage divider pin for reading low current battery voltage
const byte redPin = 35;             // digital out to red light relay
const byte greenPin = 37;           // digital out to green light relay
const byte bluePin = 39;            // digital out to blue light relay

// Constant global variables
const int nMedian = 8;          // number of readings to take to calculate median from controller inputs (don't make this too large, or else it will slow down the interrupt service routines)
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

// FrSky R9m transmitter-receiver communication global variables
//const int ch1PulseMin = 1032;     // minimum expected pulse width of ch1 [us] (this should be individually tested for each channel)
//const int ch1PulseMax = 2032;     // maximum expected pulse width of ch1 [us] (this should be individually tested for each channel)
//const int ch2PulseMin = 1072;     // minimum expected pulse width of ch2 [us] (this should be individually tested for each channel)
//const int ch2PulseMax = 2072;     // maximum expected pulse width of ch2 [us] (this should be individually tested for each channel)
//const int ch3PulseMin = 1160;     // minimum expected pulse width of ch3 [us] (this should be individually tested for each channel)
//const int ch3PulseMax = 2160;     // maximum expected pulse width of ch3 [us] (this should be individually tested for each channel)
//const int ch5PulseMin = 1248;     // minimum expected pulse width of ch5 [us] (this should be individually tested for each channel)
//const int ch5PulseMax = 2208;     // maximum expected pulse width of ch5 [us] (this should be individually tested for each channel)
//const int ch6PulseMin = 1120;     // minimum expected pulse width of ch7 [us] (this should be individually tested for each channel)
//const int ch6PulseMax = 2128;     // maximum expected pulse width of ch7 [us] (this should be individually tested for each channel)

// ImmersionRC EzUHF transmitter-receiver communication global variables
const int ch1PulseMin = 1008;     // minimum expected pulse width of ch1 [us] (this should be individually tested for each channel)
const int ch1PulseMax = 2016;     // maximum expected pulse width of ch1 [us] (this should be individually tested for each channel)
const int ch2PulseMin = 960;      // minimum expected pulse width of ch2 [us] (this should be individually tested for each channel)
const int ch2PulseMax = 1968;     // maximum expected pulse width of ch2 [us] (this should be individually tested for each channel)
const int ch3PulseMin = 960;      // minimum expected pulse width of ch3 [us] (this should be individually tested for each channel)
const int ch3PulseMax = 1968;     // maximum expected pulse width of ch3 [us] (this should be individually tested for each channel)
const int ch5PulseMin = 1004;     // minimum expected pulse width of ch5 [us] (this should be individually tested for each channel)
const int ch5PulseMax = 2012;     // maximum expected pulse width of ch5 [us] (this should be individually tested for each channel)
const int ch6PulseMin = 960;      // minimum expected pulse width of ch7 [us] (this should be individually tested for each channel)
const int ch6PulseMax = 1968;     // maximum expected pulse width of ch7 [us] (this should be individually tested for each channel)

// I2C communication variables
char dir = 'N';                         // variable to hold direction
String q1Msg = "Q1";                    // String to hold Serial message for Q1
String q2Msg = "Q2";                    // String to hold Serial message for Q2
String q3Msg = "Q3";                    // String to hold Serial message for Q3
String q4Msg = "Q4";                    // String to hold Serial message for Q4
String tempMsg = "";                    // temporary message variable for sending motor commands
volatile String motorCmds = "";         // String message to send to master Arduino
volatile char incoming[MESSAGESIZE];    // character array to store incoming message from master Arduino
int counter = 0;                        // counter used for reading message coming in from master Arduino
float previousVoltage = 0.0;            // temporary variable to store voltage measured

//ROS Verification Publishers
std_msgs::Int32 q1_ver_pub;
std_msgs::Int32 q2_ver_pub;
std_msgs::Int32 q3_ver_pub;
std_msgs::Int32 q4_ver_pub;

//ros::Publisher chatter("chatter", &str_msg);
ros::Publisher q1_pub("q1_ver_pub", &q1_ver_pub);
ros::Publisher q2_pub("q2_ver_pub", &q2_ver_pub);
ros::Publisher q3_pub("q3_ver_pub", &q3_ver_pub);
ros::Publisher q4_pub("q4_ver_pub", &q4_ver_pub);


// ROS communication variables
int q1_thrust = 0;
int q2_thrust = 0;
int q3_thrust = 0;
int q4_thrust = 0;
int q1_thrust_pwm;
int q2_thrust_pwm;
int q3_thrust_pwm;
int q4_thrust_pwm;

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;         // ROS node handle (allows program to create publishers and subscribers)

// ROS subscribers
void thrust_input_q1( const std_msgs::Int32& input_msg) {
  q1_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}
void thrust_input_q2(const std_msgs::Int32& input_msg) {
  q2_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}
void thrust_input_q3( const std_msgs::Int32& input_msg) {
  q3_thrust = input_msg.data; 
  q3_ver_pub.data = q3_thrust;
  q3_pub.publish(&q3_ver_pub);
}
void thrust_input_q4( const std_msgs::Int32& input_msg) {
  q4_thrust = input_msg.data;  
  q4_ver_pub.data = q4_thrust;
  q4_pub.publish(&q4_ver_pub);
}

// ROS publishers
ros::Subscriber<std_msgs::Int32> q1_sub("q1_thruster_input", &thrust_input_q1 );
ros::Subscriber<std_msgs::Int32> q2_sub("q2_thruster_input", &thrust_input_q2 );
ros::Subscriber<std_msgs::Int32> q3_sub("q3_thruster_input", &thrust_input_q3 );
ros::Subscriber<std_msgs::Int32> q4_sub("q4_thruster_input", &thrust_input_q4 );

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

  // Setup I2C
  Wire.begin(SLAVE_ADDR);         // Join I2C bus as slave
  Wire.onRequest(sendMsgs);       // send motor command message when master (high current Arduino) requests
  Wire.onReceive(readVoltageMsg); // read incoming voltage message when master sends it

  // Setup ROS node handle
  nh.initNode();

  // Set ROS Advertisers And Subscribers
  nh.advertise(q1_pub);
  nh.advertise(q2_pub);
  nh.advertise(q3_pub);
  nh.advertise(q4_pub);

  nh.subscribe(q1_sub);      //initialize ros subscriber
  nh.subscribe(q2_sub);      //initialize ros subscriber
  nh.subscribe(q3_sub);      //initialize ros subscriber
  nh.subscribe(q4_sub);      //initialize ros subscriber

}

void loop() {

  // Read the various inputs to the main arduino
  readKillArduino();                // read kill status from kill arduino
  readHandheldReceiver();           // read inputs from handheld receiver
  readLowCurrentBatteryVoltage();   // read low current battery voltage
  readMainBatteryVoltage();         // read main battery voltage

  // Determine operation mode (kill, battery check, and auto/manual) based on inputs
  determineMode();

  // Control light based on operation mode
  controlLight();

  // Execute code based on operation mode
  if (killStatus == 0) {

    // Manual code goes here
    if (mode == 1) {

      // Take joystick readings, and convert to setpoint thrust values
      joy2Setpoint();

      // Write setpoint thruster values to I2C
      createI2cMsg();

    }

    // Autonomous code goes here
    else if (mode == 2) {

      // Checks ROS topics and convert to setpoint values
      leftThrusterSetpoint = q3_thrust;
      rightThrusterSetpoint = q4_thrust;

      // Create messages for I2C comms
      createI2cMsg();

    }

    // Error in mode determination, zero the thrusters (note that this is not necessarily a kill)
    else {

      // Print debug statement
      Serial.println("Error in mode determination; check transmitter connection...");

      // Set thruster setpoints to zero
      zeroSetpoints();

    }
  }

  // Kill code goes here
  else {
    
    // Set thruster setpoints to zero to prevent high current arduino controller from going crazy while system is killed
    zeroSetpoints();

  }

  // Small delay to prevent errors
  delay(5);

  // Spin ROS
  nh.spinOnce();

}
