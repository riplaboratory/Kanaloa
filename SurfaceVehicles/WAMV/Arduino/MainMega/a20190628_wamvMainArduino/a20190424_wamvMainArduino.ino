

/* 
 *  All libraries available from Library Manager built into Arduino IDE.
 *  To access go to Sketch > Include Library > Manage Libraries
 *    ROS: Search for "Rosserial Arduino Library by Michael Ferguson" and install.
 *    Adafruit: Search for "Adafruit PWM Servo Driver Library by Adafruit" and install.
 *    QuickMedian: Search for "QuickMedianLib by Luis Llamas" and install
 *    
 *  Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
 *  Last update: 2019.03.05
 *     
 */

// Library inclusions
#include <Arduino.h>                    // Arduino inclusions
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <Adafruit_PWMServoDriver.h>
#include <QuickMedianLib.h>
//#include <ros.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pin definitions
const int voltMainPin = A1;         // analog in from voltage divider pin for reading main battery voltage
const int tempPin = A2;             // analog in from temperature probe pin for reading temperature (optional)
const byte ch1Pin = 18;             // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
const byte ch4Pin = 19;             // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
const byte ch6Pin = 3;              // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward) 
const byte modeCommPin = 11;        // digital in auto/manual mode state from kill arduino 
const byte revConKillCommPin = 12;  // digital in reverse contactor kill state from kill arduino 
const byte killCommPin = 13;        // digital in kill state from kill arduino 
const byte servoQ1Pin = 4;          // q1 channel on servo shield (note: not the same as a digital pin!)
const byte servoQ2Pin = 5;          // q2 channel on servo shield (note: not the same as a digital pin!)
const byte servoQ3Pin = 6;          // q3 channel on servo shield (note: not the same as a digital pin!)
const byte servoQ4Pin = 7;          // q4 channel on servo shield (note: not the same as a digital pin!)
const byte revConQ1Pin = 22;        // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ2Pin = 24;        // digital out to Q2 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ3Pin = 26;        // digital out to Q3 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ4Pin = 28;        // digital out to Q4 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)

// Global system (constant) variables
const int nMedian = 8;            // number of readings to take to calculate median from controller inputs
const float receiverRR = 50;      // refresh rate of the receiver [Hz] (MUST BE A FLOAT!)
const int ch1PulseMin = 1260;     // minimum expected pulse width of ch1 [us] right stick left right
const int ch1PulseMax = 1820;     // maximum expected pulse width of ch1 [us] 
const int ch4PulseMin = 1040;     // minimum expected pulse width of ch4 [us] left stick left right
const int ch4PulseMax = 2040;     // maximum expected pulse width of ch4 [us]
const int ch6PulseMin = 1048;     // minimum expected pulse width of ch6 [us] left stick up down
const int ch6PulseMax = 2000;     // maximum expected pulse width of ch6 [us]
const int pulseTolerance = 200;   // tolerance range on minimum and maximum expected pulse widths [us]
const int pulseDeadzone = 125;    // deadzone range at the center of each joystick [us]
const int accLimit = 80;          // acceleration limit (maximum change in thrust percentage per second) [%/s]

// Global volatile variables
volatile int ch1PulseRaw = 0;     // channel 1 pulse width (for interrerupt service routine)
volatile int ch1Timer = 0;        // channel 1 pulse width timer (for interrerupt service routine)
volatile int ch4PulseRaw = 0;     // channel 4 pulse width (for interrerupt service routine)
volatile int ch4Timer = 0;        // channel 4 pulse width timer (for interrerupt service routine)
volatile int ch6PulseRaw = 0;     // channel 6 pulse width (for interrerupt service routine)
volatile int ch6Timer = 0;        // channel 6 pulse width timer (for interrerupt service routine)

// Global code variables
int mode;                   // determines manual (1), autonomous (2), or invalid (0) operation mode
float voltMain;             // voltage from the main battery
int killStatus = 1;         // status of the kill switch from kill Arduino
int revConKillStatus = 1;   // status of reverse contactor kill switch from kill Arduino
int modeStatus = 1;         // auto/manual status from kill Arduino
int ch1Filtered = 0;        // filtered output from ch1
int ch4Filtered = 0;        // filtered output from ch4
int ch6Filtered = 0;        // filtered output from ch6
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
int q1Dir = 0;              // Q1 direction (0 = forward, 1 = reverse)
int q2Dir = 0;              // Q2 direction (0 = forward, 1 = reverse)
int q3Dir = 0;              // Q3 direction (0 = forward, 1 = reverse)
int q4Dir = 0;              // Q4 direction (0 = forward, 1 = reverse)
float voltMult = 0.88;      // voltage multiplier to scale output to thrusters (based on battery voltage)
float autoQ1 = 0;           // autonomous command signal from ROS for Q1 thruster (via ROS)
float autoQ2 = 0;           // autonomous command signal from ROS for Q2 thruster (via ROS)
float autoQ3 = 0;           // autonomous command signal from ROS for Q3 thruster (via ROS)
float autoQ4 = 0;           // autonomous command signal from ROS for Q4 thruster (via ROS)

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;         // ROS node handle (allows program to create publishers and subscribers)

// ROS messages
std_msgs::UInt16 q1Msg;         // create Q1 thruster message
std_msgs::UInt16 q2Msg;         // create Q2 thruster message
std_msgs::UInt16 q3Msg;         // create Q3 thruster message
std_msgs::UInt16 q4Msg;         // create Q4 thruster message
std_msgs::Float64 voltMainMsg;  // create main battery voltage message

// ROS publishers
ros::Publisher q1Pub("q1", &q1Msg);                   // Q1 thruster publisher
ros::Publisher q2Pub("q2", &q2Msg);                   // Q2 thruster publisher
ros::Publisher q3Pub("q3", &q3Msg);                   // Q3 thruster publsiher
ros::Publisher q4Pub("q4", &q4Msg);                   // Q4 thruster publisher
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
  nh.advertise(q1Pub);          // Q1 thruster output
  nh.advertise(q2Pub);          // Q2 thruster output
  nh.advertise(q3Pub);          // Q3 thruster output
  nh.advertise(q4Pub);          // Q4 thruster output

  // Initialize ROS subscibers
  nh.subscribe(autoQ1sub);      // autonomous Q1 thruster output
  nh.subscribe(autoQ2sub);      // autonomous Q2 thruster output
  nh.subscribe(autoQ3sub);      // autonomous Q3 thruster output
  nh.subscribe(autoQ4sub);      // autonomous Q4 thruster output

  // Set pin mode
  pinMode(voltMainPin,INPUT);
  pinMode(tempPin,INPUT);
  pinMode(ch1Pin,INPUT);
  pinMode(ch4Pin,INPUT);
  pinMode(ch6Pin,INPUT);
  pinMode(modeCommPin,INPUT_PULLUP);
  pinMode(revConKillCommPin,INPUT_PULLUP);
  pinMode(killCommPin,INPUT_PULLUP);
  pinMode(revConQ1Pin,OUTPUT);
  pinMode(revConQ2Pin,OUTPUT);
  pinMode(revConQ3Pin,OUTPUT);
  pinMode(revConQ4Pin,OUTPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Fall,FALLING);
  
  //Initialize PWM shield
  pwm.begin();
  pwm.setPWMFreq(500);
  
  // Set serial baud rate
  Serial.begin(57600); 
  
}

void loop() {
  
  readKillArduino();      // read kill state from kill arduino

  if (killStatus == 0) {

    // ===== AUTONOMOUS CODE GOES HERE =====
    if (modeStatus == 0) {

      Serial.println("AUTONOMOUS MODE: One day, I shall achieve autonomy! OM NOM NOM NOM!");
      delay(5);
      
    }

    // ===== MANUAL CODE GOES HERE =====
    else {

      // Filter noise from remote control joystick inputs using meadian of multiple measurements
      filterJoy();

      // Take joystick readings, and convert to setpoint thrust values
      joy2Setpoint();

      // Calculate acceleration-limited outputs as a function of the setpoint and send to thrusters
      setpoint2Output();

      // Calculate reverse contactor states and send to reverse contactors
      output2RevCon();

      // Short delay to prevent errors
      delay(5);

//      // Print debug info
//      Serial.print("MANUAL MODE: Q1 (");
//      Serial.print(q1Out);
//      Serial.print(")  Q2 (");
//      Serial.print(q2Out);
//      Serial.print(")  Q3 (");
//      Serial.print(q3Out);
//      Serial.print(")  Q4 (");
//      Serial.print(q4Out);
//      Serial.println(")");
      
    }

  }
  else {
    
    Serial.println("System is killed, waiting for unkill command...");

//          // Filter noise from remote control joystick inputs using meadian of multiple measurements
//      filterJoy();
//
//      // Take joystick readings, and convert to setpoint thrust values
//      joy2Setpoint();
//
//      // Calculate acceleration-limited outputs as a function of the setpoint and send to thrusters
//      setpoint2Output();
//
//      // Calculate reverse contactor states and send to reverse contactors
//      output2RevCon();
//
//      // Short delay to prevent errors
//      delay(5);
//
//    // Short delay to prevent errors
//    delay(5);
    
  }

}
