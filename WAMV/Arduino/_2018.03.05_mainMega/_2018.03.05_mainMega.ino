// Library inclusions
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
//#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

// Pin definitions
const int voltMainPin = A0;       // analog in from voltage divider pin for reading main battery voltage
const int tempPin = A3;           // analog in from temperature probe pin for reading temperature (optional)
const byte ch1Pin = 4;            // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
const byte ch3Pin = 3;            // PWM in from remote control receiver channel 3 (left stick up-to-down surge/forward)
const byte ch4Pin = 5;            // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
const byte ch5Pin = 7;            // PWM in from remote control receiver channel 5 (manual takeover)
const byte ch8Pin = 11;           // PWM in from remote control receiver channel 8 (remote kill switch)
const byte Q1Pin = 8;             // PWM out to motor driver Q1
const byte Q2Pin = 9;             // PWM out to motor driver Q2
const byte Q3Pin = 10;            // PWM out to motor driver Q3
const byte Q4Pin = 6;             // PWM out to motor driver Q4
const byte yellowLightPin = 37;   // digital out to yellow safety light relay (HIGH to turn on, LOW to turn off)
const byte greenLightPin = 38;    // digital out to green safety light relay (HIGH to turn on, LOW to turn off)
const byte thrusterKillPin = 16;  // digital out to thruster kill switch
const byte armReelKillPin = 18;   // digital out to arm-reel kill switch

// Global code variables
int autoQ1;   // autonomous command signal from Matlab (via ROS)
int autoQ2;
int autoQ3;
int autoQ4;

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;

// ROS messages
std_msgs::UInt16 Q1Msg;         // Q1 thruster message
std_msgs::UInt16 Q2Msg;         // Q2 thruster message
std_msgs::UInt16 Q3Msg;         // Q3 thruster message 
std_msgs::UInt16 Q4Msg;         // Q4 thruster message
std_msgs::Float64 voltMainMsg;  // main battery voltage message

// ROS publishers
ros::Publisher voltMainPub("voltMain",&voltMainMsg);  // main battery voltage
ros::Publisher Q1Pub("Q1", &Q1Msg);                   // Q1 thruster output  
ros::Publisher Q2Pub("Q2", &Q1Msg);                   // Q2 thruster output
ros::Publisher Q3Pub("Q3", &Q1Msg);                   // Q3 thruster output
ros::Publisher Q4Pub("Q4", &Q1Msg);                   // Q4 thruster output

// ROS callback functions (must come before subscibers)
void autoQ1Cb(const std_msgs::UInt16& autoQ1CbMsg) {
  autoQ1 = map(autoQ1CbMsg.data, -100, 100, 31, 63);
}
void autoQ2Cb(const std_msgs::UInt16& autoQ2CbMsg) {
  autoQ2 = map(autoQ2CbMsg.data, -100, 100, 31, 63);
}
void autoQ3Cb(const std_msgs::UInt16& autoQ3CbMsg) {
  autoQ3 = map(autoQ3CbMsg.data, -100, 100, 31, 63);
}
void autoQ4Cb(const std_msgs::UInt16& autoQ4CbMsg) {
  autoQ4 = map(autoQ4CbMsg.data, -100, 100, 31, 63);
}

// ROS Subscribers
ros::Subscriber<std_msgs::UInt16> autoQ1sub("autoQ1", &autoQ1Cb);   // autonomous Q1 thruster output
ros::Subscriber<std_msgs::UInt16> autoQ2sub("autoQ2", &autoQ2Cb);   // autonomous Q2 thruster output 
ros::Subscriber<std_msgs::UInt16> autoQ3sub("autoQ3", &autoQ3Cb);   // autonomous Q3 thruster output
ros::Subscriber<std_msgs::UInt16> autoQ4sub("autoQ4", &autoQ4Cb);   // autonomous Q4 thruster output

void setup() {

  // ROS Node handle
  nh.initNode();

  // Setup publishers
  nh.advertise(voltMainPub);    // main battery voltage
  nh.advertise(Q1Pub);          // Q1 thruster output
  nh.advertise(Q2Pub);          // Q2 thruster output
  nh.advertise(Q3Pub);          // Q3 thruster output
  nh.advertise(Q4Pub);          // Q4 thruster output

  // Setup subscibers
  nh.subscribe(autoQ1sub);    // autonomous Q1 thruster output
  nh.subscribe(autoQ2sub);    // autonomous Q2 thruster output
  nh.subscribe(autoQ3sub);    // autonomous Q3 thruster output
  nh.subscribe(autoQ4sub);    // autonomous Q4 thruster output

  // Handles Arduino timers for PWM frequency (for more info: https://playground.arduino.cc/Main/TimerPWMCheatsheet and http://forum.arduino.cc/index.php/topic,72092.0.html)
  int myEraser = 7;         // this is 111 in binary, and used as an eraser
  TCCR2B &= ~myEraser;      // sets three bits in TCCR2B (timer 2) to zero
  TCCR4B &= ~myEraser;      // sets three bits in TCCR4B (timer 4) to zero
  TCCR4B |= 4;              // Set timer 4 (pins 6 and 8) to prescaler 4 (120 Hz).
  TCCR2B |= 6;              // Set timer 2 (pins 10 and 9) to prescaler 6 (120 Hz).
  
  // Set pins
  pinMode(ch3Pin, INPUT);
  pinMode(ch1Pin, INPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(ch5Pin, INPUT);
  pinMode(ch8Pin, INPUT);
  pinMode(Q1Pin, OUTPUT);
  pinMode(Q2Pin, OUTPUT); 
  pinMode(Q3Pin, OUTPUT);
  pinMode(Q4Pin, OUTPUT);
  pinMode(yellowLightPin, OUTPUT); 
  pinMode(greenLightPin, OUTPUT); 
  pinMode(ch8Pin, INPUT); 
  pinMode(thrusterKillPin, OUTPUT);
  pinMode(armReelKillPin, OUTPUT);

  // Set serial
  Serial.begin(57600);
  
}

void loop() {

  // Check kill states from remote kill switch
  int ch8 = pulseIn(ch8Pin, HIGH, 25000);

  // Kill as necessary
  if (ch8 >= 2000 && ch8 <= 2060) {
    digitalWrite(thrusterKillPin, HIGH);
    digitalWrite(armReelKillPin, HIGH); 
  }
  else if (ch8 >= 1485 && ch8 <= 1500){
    digitalWrite(thrusterKillPin, HIGH);
    digitalWrite(armReelKillPin, LOW); 
  }
  else {
    digitalWrite(thrusterKillPin, LOW);
    digitalWrite(armReelKillPin, LOW); 
  }

  // Main battery voltage reader
  float voltMainBit = analogRead(voltMainPin);      // voltage on voltMainPin [bit]
  float voltMain = voltMainBit*(5*4.85276/1023.0);  // voltage on voltMainPin [V]
  voltMainMsg.data = voltMain;                      // set voltMain message
  voltMainPub.publish( &voltMainMsg);               // publish voltMain topic

  // Check mode (manual or auto) from transmitter
  int ch5 = pulseIn(ch5Pin, HIGH, 25000);

  // ====================
  //     MANUAL CODE  
  // ====================

  if (ch5 <= 1000) {  // if switch A is in the 'up' position, ch5 from transmitter will send pulse lengths less than 1000

    // Set safety light
    digitalWrite(yellowLightPin,HIGH);    // turn on yellow safety light
    digitalWrite(greenLightPin,LOW);      // turn off green safety light

    // Read in receiver values
    int ch1 = pulseIn(ch1Pin, HIGH, 25000);   // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
    int ch3 = pulseIn(ch3Pin, HIGH, 25000);   // PWM in from remote control receiver channel 3 (left stick up-to-down surge/forward)
    int ch4 = pulseIn(ch4Pin, HIGH, 25000);   // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)

    // CHECK THIS!!!!!!!!!!!!!!!
    // Map receiver inputs from -100 to 100
    int ch1Map = map(ch1, 1084, 1909, -100, 100);   // yaw
    int ch3Map = map(ch3, 943, 2052, 100, -100);    // surge
    int ch4Map = map(ch4, 1094, 1909, -100, 100);   // sway

    // Calcuate surge, sway, and yaw components
    int surgeQ1 = ch3Map;
    int surgeQ2 = ch3Map;
    int surgeQ3 = ch3Map;
    int surgeQ4 = ch3Map;
    int swayQ1 = ch4Map;
    int swayQ2 = -ch4Map;
    int swayQ3 = ch4Map;
    int swayQ4 = -ch4Map;
    int yawQ1 = -ch1Map;
    int yawQ2 = ch1Map;
    int yawQ3 = ch1Map;
    int yawQ4 = -ch1Map;

    // Sum surge, sway, and yaw components
    int Q1Map = surgeQ1+swayQ1+yawQ1;
    int Q2Map = surgeQ2+swayQ2+yawQ2;
    int Q3Map = surgeQ3+swayQ3+yawQ3;
    int Q4Map = surgeQ4+swayQ4+yawQ4;

    // Limit components to no less than -100 and no more than 100
    if (Q1Map > 100) { Q1Map = 100; }
    if (Q1Map < -100) { Q1Map = -100; }
    if (Q2Map > 100) { Q2Map = 100; }
    if (Q2Map < -100) { Q2Map = -100; }
    if (Q3Map > 100) { Q3Map = 100; }
    if (Q3Map < -100) { Q3Map = -100; }
    if (Q4Map > 100) { Q4Map = 100; }
    if (Q4Map < -100) { Q4Map = -100; }

    // Remap surge, sway and yaw components to motor driver PWM duty cycle
    // (31 = full reverse, 47 = neutral, 63 = full forward)
    int Q1Out = map(Q1Map, -100, 100, 31, 63);
    int Q2Out = map(Q2Map, -100, 100, 31, 63);
    int Q3Out = map(Q3Map, -100, 100, 31, 63);
    int Q4Out = map(Q4Map, -100, 100, 31, 63);

    // Adjust motor drive output based on main battery voltage
    Q1Out = 47+(Q1Out-47)*12/voltMain;
    Q2Out = 47+(Q2Out-47)*12/voltMain;
    Q3Out = 47+(Q3Out-47)*12/voltMain;
    Q4Out = 47+(Q4Out-47)*12/voltMain; 

    // Set a neutral "buffer" zone for manual transmitter
    if (Q1Out <= 49 && Q1Out >=45) { Q1Out = 47; }
    if (Q2Out <= 49 && Q2Out >=45) { Q2Out = 47; }
    if (Q3Out <= 49 && Q3Out >=45) { Q3Out = 47; }
    if (Q4Out <= 49 && Q4Out >=45) { Q4Out = 47; }

    // Output to motor drivers
    analogWrite(Q1Pin, Q1Out);
    analogWrite(Q2Pin, Q2Out);
    analogWrite(Q3Pin, Q3Out);
    analogWrite(Q4Pin, Q4Out);

    // Publish to ROS
    Q1Msg.data = Q1Out;
    Q2Msg.data = Q2Out;
    Q3Msg.data = Q3Out;
    Q4Msg.data = Q4Out;
    Q1Pub.publish( &Q1Msg);
    Q2Pub.publish( &Q1Msg);
    Q3Pub.publish( &Q1Msg);
    Q4Pub.publish( &Q1Msg);

  }

  // ====================
  //   AUTONOMOUS CODE
  // ====================

  else {

    // Set safety light
    digitalWrite(yellowLightPin,LOW);     // turn off yellow safety light
    digitalWrite(greenLightPin,HIGH);     // turn on green safety light

    // Map autoQn (from ROS) to motor driver PWM duty cycle
    // (31 = full reverse, 47 = neutral, 63 = full forward)
    int Q1Out = map(autoQ1,-100,100,31,63);
    int Q2Out = map(autoQ2,-100,100,31,63);
    int Q3Out = map(autoQ3,-100,100,31,63);
    int Q4Out = map(autoQ4,-100,100,31,63);
    
    // Adjust motor drive output based on main battery voltage
    Q1Out = 47+(Q1Out-47)*12/voltMain;
    Q2Out = 47+(Q2Out-47)*12/voltMain;
    Q3Out = 47+(Q3Out-47)*12/voltMain;
    Q4Out = 47+(Q4Out-47)*12/voltMain; 

    // Output to motor drivers
    analogWrite(Q1Pin, Q1Out);
    analogWrite(Q2Pin, Q2Out);
    analogWrite(Q3Pin, Q3Out);
    analogWrite(Q4Pin, Q4Out);

    // Publish to ROS
    Q1Msg.data = Q1Out;
    Q2Msg.data = Q2Out;
    Q3Msg.data = Q3Out;
    Q4Msg.data = Q4Out;
    Q1Pub.publish( &Q1Msg);
    Q2Pub.publish( &Q1Msg);
    Q3Pub.publish( &Q1Msg);
    Q4Pub.publish( &Q1Msg);

    // Spin once to refresh callbacks
    nh.spinOnce();    // THIS ONE MIGHT NOT BE NEEDED
    
  }

  // Small delay
  delay(10);

  // Spin once to refresh callbacks
  nh.spinOnce();

}
