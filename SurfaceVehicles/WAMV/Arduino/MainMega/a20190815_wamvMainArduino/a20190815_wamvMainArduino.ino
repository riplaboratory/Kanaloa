

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
const byte servoLeftFwd = 4;        // servo shield channel (note: not the same as a digital pin!) for controlling forward speed in the left thrusters
const byte servoLeftRev = 5;        // servo shield channel (note: not the same as a digital pin!) for controlling reverse speed in the left thrusters
const byte servoRightFwd = 6;       // servo shield channel (note: not the same as a digital pin!) for controlling forward speed in the right thrusters
const byte servoRightRev = 7;       // servo shield channel (note: not the same as a digital pin!) for controlling reverse speed in the right thrusters

// Global system (constant) variables
const int nMedian = 8;            // number of readings to take to calculate median from controller inputs
const float receiverRR = 50;      // refresh rate of the receiver [Hz] (MUST BE A FLOAT!)
const int ch1PulseMin = 1050;     // minimum expected pulse width of ch1 [us] right stick left right
const int ch1PulseMax = 2052;     // maximum expected pulse width of ch1 [us] 
const int ch4PulseMin = 1040;     // minimum expected pulse width of ch4 [us] left stick left right
const int ch4PulseMax = 2040;     // maximum expected pulse width of ch4 [us]
const int ch6PulseMin = 1064;     // minimum expected pulse width of ch6 [us] left stick up down
const int ch6PulseMax = 1930;     // maximum expected pulse width of ch6 [us]
const int pulseTolerance = 200;   // tolerance range on minimum and maximum expected pulse widths [us]
const int pulseDeadzone = 125;    // deadzone range at the center of each joystick [us]
const int accLimit = 80;          // acceleration limit (maximum change in thrust percentage per second) [%/s]
const int PWMFreq = 40;           // [Hz], Output frequency of the PWM shield

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
float voltMult = 1;      // voltage multiplier to scale output to thrusters (based on battery voltage)
float autoQ1 = 0;           // autonomous command signal from ROS for Q1 thruster (via ROS)
float autoQ2 = 0;           // autonomous command signal from ROS for Q2 thruster (via ROS)
float autoQ3 = 0;           // autonomous command signal from ROS for Q3 thruster (via ROS)
float autoQ4 = 0;           // autonomous command signal from ROS for Q4 thruster (via ROS)

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;         // ROS node handle (allows program to create publishers and subscribers)

//// ROS messages
//std_msgs::UInt16 q1Msg;         // create Q1 thruster message
//std_msgs::UInt16 q2Msg;         // create Q2 thruster message
//std_msgs::UInt16 q3Msg;         // create Q3 thruster message
//std_msgs::UInt16 q4Msg;         // create Q4 thruster message
//std_msgs::Float64 voltMainMsg;  // create main battery voltage message
//
//// ROS publishers
//ros::Publisher q1Pub("q1", &q1Msg);                   // Q1 thruster publisher
//ros::Publisher q2Pub("q2", &q2Msg);                   // Q2 thruster publisher
//ros::Publisher q3Pub("q3", &q3Msg);                   // Q3 thruster publsiher
//ros::Publisher q4Pub("q4", &q4Msg);                   // Q4 thruster publisher
//ros::Publisher voltMainPub("voltMain", &voltMainMsg); // main battery voltage publisher
//
//// ROS Subscribers
//void autoQ1Cb(const std_msgs::Float64& autoQ1CbMsg) {
//  autoQ1 = autoQ1CbMsg.data;
//}
//ros::Subscriber<std_msgs::Float64> autoQ1sub("autoQ1", &autoQ1Cb);   // autonomous Q1 thruster output
//void autoQ2Cb(const std_msgs::Float64& autoQ2CbMsg) {
//  autoQ2 = autoQ2CbMsg.data;
//}
//ros::Subscriber<std_msgs::Float64> autoQ2sub("autoQ2", &autoQ2Cb);   // autonomous Q2 thruster output
//void autoQ3Cb(const std_msgs::Float64& autoQ3CbMsg) {
//  autoQ3 = autoQ3CbMsg.data;
//}
//ros::Subscriber<std_msgs::Float64> autoQ3sub("autoQ3", &autoQ3Cb);   // autonomous Q3 thruster output
//void autoQ4Cb(const std_msgs::Float64& autoQ4CbMsg) {
//  autoQ4 = autoQ4CbMsg.data;
//}
//ros::Subscriber<std_msgs::Float64> autoQ4sub("autoQ4", &autoQ4Cb);   // autonomous Q4 thruster output

// NEW ANTX ROS CODE

int q1_thrust = 0;
int q2_thrust = 0;
int q3_thrust = 0;
int q4_thrust = 0;

int q1_thrust_pwm;
int q2_thrust_pwm;
int q3_thrust_pwm;
int q4_thrust_pwm;


#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
//#include <std_msgs/Float64.h>

//ros::NodeHandle  nh;      //the ros node handle allows us to create subscribers and publishers and takes care of the serial port
std_msgs::Int32 int_msg;
std_msgs::String str_msg;

void thrust_input_q1( const std_msgs::Int32& input_msg)
{
  q1_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}

void thrust_input_q2(const std_msgs::Int32& input_msg)
{
  q2_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
}

void thrust_input_q3( const std_msgs::Int32& input_msg)
{
  if (modeStatus == 0){
//   q3_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
//    const int pwm_speed = offsetmap(input_msg.data);
//    setSpd(pwm_speed, servoLeftFwd, servoLeftRev); 
    const int q3_out = map(input_msg.data,-1000,1000,-3500,3500); //Map limits
    setSpd(q3_out, servoLeftFwd, servoLeftRev);
  }
  
}

void thrust_input_q4( const std_msgs::Int32& input_msg)
{
  if (modeStatus == 0){
//    q4_thrust = input_msg.data;     //assign subscribed message to variable digitalIn
//    const int pwm_speed = offsetmap(input_msg.data);
//    setSpd(pwm_speed, servoRightFwd, servoRightRev);
    const int q4_out = map(input_msg.data,-1000,1000,-3500,3500); //Map limits
    setSpd(q4_out, servoRightFwd, servoRightRev);
    
  }
}

ros::Subscriber<std_msgs::Int32> q1_sub("q1_thruster_input", &thrust_input_q1 );      //sub to number_topic
ros::Subscriber<std_msgs::Int32> q2_sub("q2_thruster_input", &thrust_input_q2 );      //sub to number_topic
ros::Subscriber<std_msgs::Int32> q3_sub("q3_thruster_input", &thrust_input_q3 );      //sub to number_topic
ros::Subscriber<std_msgs::Int32> q4_sub("q4_thruster_input", &thrust_input_q4 );      //sub to number_topic

// --- END ANTX CCODE

void setup() {
  
  // Initialize ROS node handle
  nh.initNode();

//  // Initialize ROS publishers
//  nh.advertise(voltMainPub);    // main battery voltage
//  nh.advertise(q1Pub);          // Q1 thruster output
//  nh.advertise(q2Pub);          // Q2 thruster output
//  nh.advertise(q3Pub);          // Q3 thruster output
//  nh.advertise(q4Pub);          // Q4 thruster output
//
//  // Initialize ROS subscribers
//  nh.subscribe(autoQ1sub);      // autonomous Q1 thruster output
//  nh.subscribe(autoQ2sub);      // autonomous Q2 thruster output
//  nh.subscribe(autoQ3sub);      // autonomous Q3 thruster output
//  nh.subscribe(autoQ4sub);      // autonomous Q4 thruster output

  // Set pin mode
  pinMode(voltMainPin,INPUT);
  pinMode(tempPin,INPUT);
  pinMode(ch1Pin,INPUT);
  pinMode(ch4Pin,INPUT);
  pinMode(ch6Pin,INPUT);
  pinMode(modeCommPin,INPUT_PULLUP);
  pinMode(revConKillCommPin,INPUT_PULLUP);
  pinMode(killCommPin,INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch1Pin),ch1Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch4Pin),ch4Fall,FALLING);
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch6Pin),ch6Fall,FALLING);
  
  //Initialize PWM shield
  pwm.begin();
  pwm.setPWMFreq(PWMFreq);
  
  // Set serial baud rate
  Serial.begin(57600); 

  //New ROS Code ANTX

  nh.subscribe(q1_sub);      //initialize ros subscriber
  nh.subscribe(q2_sub);      //initialize ros subscriber
  nh.subscribe(q3_sub);      //initialize ros subscriber
  nh.subscribe(q4_sub);      //initialize ros subscriber

//  nh.advertise(arduino_output);     //number output
  

  //---------End ANTX Code
  
}

void loop() {
  
  readKillArduino();      // read kill state from kill arduino

  //Raymond DEBUG
//  killStatus = 0;
//  modeStatus = 0;
  // --- END DEBUG

  if (killStatus == 0) {

    // ===== AUTONOMOUS CODE GOES HERE =====
    if (modeStatus == 0) {

//      Serial.println("AUTONOMOUS MODE: One day, I shall achieve autonomy! OM NOM NOM NOM!");
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
//      output2RevCon();

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

          // Filter noise from remote control joystick inputs using meadian of multiple measurements
      filterJoy();

      // Take joystick readings, and convert to setpoint thrust values
      joy2Setpoint();

      // Calculate acceleration-limited outputs as a function of the setpoint and send to thrusters
      setpoint2Output();

      // Calculate reverse contactor states and send to reverse contactors
//      output2RevCon();

      // Short delay to prevent errors
      delay(5);

    // Short delay to prevent errors
    delay(5);
    
  }

  nh.spinOnce();

}
