// Library inclusions
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

// Pin definitiions
const int voltPin = A0;       // voltage divider pin; for reading main battery voltage
const int tempPin = A3;       // temperature probe pin; for reading temperature (optional)
const byte ch1Pin = 4;        // input PWM pin from remote control receiver channel 1; for manual control rotational motion 
const byte ch3Pin = 3;        // input PWM pin from remote controller receiver channel 3; for manual control of linear motion 
const byte ch4Pin = 5;        // input PWM pin from remote control receiver channel 4; for manual control strafing 
const byte ch5Pin = 7;        // input PWM pin from remote control receiver channel 5; for manual takeover 
const byte ch8Pin = 11;       // input PWM pin from remote control receiver channel 8; for remote kill switch
const byte Q1Pin = 8;         // PWM out to thruster Q1; for propulsion 
const byte Q2Pin = 9;         // PWM out to thruster Q2; for propulsion 
const byte Q3Pin = 10;        // PWM out to thruster Q3; for propulsion 
const byte Q4Pin = 6;         // PWM out to thruster Q4; for propulsion 
//const byte safeRed = 48;      // digital out to safety light relay; for red
const byte safeYellow = 37;   // digital out to safety light relay; for color yellow
const byte safeGreen = 38;    // digital out to safety light relay; for color green
const byte killSwitch1 = 16;  // one digital out for kill switch
const byte killSwitch2 = 18;  // second digital out for kill switch

// Code variables
char tempStringC[10];
char tempStringF[10];
char tempStringV[10];
char voltString[10];
unsigned int analog0;
int ch1, rotate, inv_rotate;             // rotation
int ch3, linear, back_linear;            // linear drive
int ch4, strafe, inv_strafe;             // strafing
int ch5, takeover;                       // manual takeover
int ch7;                                 // Being used for other Arduino board so don't mess with channel 7  
int ch8;                                 // killswitch channel 
int auto_q1, auto_q2, auto_q3, auto_q4;  // autonomous variable from Matlab
int auto_q1_butt, auto_q2_butt, auto_q3_butt, auto_q4_butt;
int ch1Decoded, ch3Decoded, ch4Decoded;
int throttleQ1;
int throttleQ2;
int throttleQ3;
int throttleQ4;
int strafeQ1;
int strafeQ2;
int strafeQ3;
int strafeQ4;
int rotateQ1;
int rotateQ2;
int rotateQ3;
int rotateQ4;
int Q1Decoded, Q2Decoded, Q3Decoded, Q4Decoded;
int Q1bit, Q2bit, Q3bit, Q4bit;
int count = 0;

// ROS message type
std_msgs::UInt16 test;
std_msgs::UInt16 test1;
std_msgs::UInt16 test2;
std_msgs::UInt16 test3;
std_msgs::UInt16 test4;
std_msgs::Float64 test6;
ros::NodeHandle nh;
std_msgs::Float64 test5;
std_msgs::Float64 test7;
ros::Publisher volt("volt", &test5);
ros::Publisher temp("temp", &test6);
ros::Publisher voltbox("voltbox", &test7);

// ROS Messages
void q1_cb(const std_msgs::UInt16& cmd_msg) {
  Serial.print("q1");
  //Serial.print(cmd_msg);
  auto_q1= map(cmd_msg.data, 0, 100, 31, 63);
  Serial.print(cmd_msg.data);
//  analogWrite(8, auto_q1);
}
void q2_cb(const std_msgs::UInt16& cmd_msg) {
  Serial.print("q2");
  //Serial.print(cmd_msg);
  auto_q2= map(cmd_msg.data, 0, 100, 31, 63);
  Serial.print(cmd_msg.data);
//  analogWrite(9, auto_q2);
}
void q3_cb(const std_msgs::UInt16& cmd_msg) {
  Serial.print("q3");
  //Serial.print(cmd_msg);
  auto_q3= map(cmd_msg.data, 0, 100, 31, 63);
  Serial.print(cmd_msg.data);
//  analogWrite(10, auto_q3);
}
void q4_cb(const std_msgs::UInt16& cmd_msg) {
    Serial.print("q4");
 // Serial.print(cmd_msg);
  auto_q4= map(cmd_msg.data, 0, 100, 31, 63);
  Serial.print(cmd_msg.data);
//  analogWrite(6, auto_q4);
}

// ROS Subscribers (motor commands from MATLAB)
ros::Subscriber<std_msgs::UInt16> motor_q1_sub("motor_q1", q1_cb);    // 
ros::Subscriber<std_msgs::UInt16> motor_q2_sub("motor_q2", q2_cb);    //
ros::Subscriber<std_msgs::UInt16> motor_q3_sub("motor_q3", q3_cb);    //
ros::Subscriber<std_msgs::UInt16> motor_q4_sub("motor_q4", q4_cb);    //

// ROS Publishers (translated motor commands from here. Topics are from Arduino motor values)
ros::Publisher display_q1("Q1", &test1);
ros::Publisher display_q2("Q2", &test2);
ros::Publisher display_q3("Q3", &test3);
ros::Publisher display_q4("Q4", &test4);

void setup() {

  // Setup subscriber
  nh.initNode();
  nh.subscribe(motor_q1_sub);
  nh.subscribe(motor_q2_sub);
  nh.subscribe(motor_q3_sub);
  nh.subscribe(motor_q4_sub);
  nh.advertise(display_q1);
  nh.advertise(display_q2);
  nh.advertise(display_q3);
  nh.advertise(display_q4);
  
  nh.advertise(volt);
  nh.advertise(temp);
  nh.advertise(voltbox);

  // Handles Arduino timers for PWM frequency (for more info: https://playground.arduino.cc/Main/TimerPWMCheatsheet and http://forum.arduino.cc/index.php/topic,72092.0.html)
  int myEraser = 7;         // this is 111 in binary, and used as an eraser
  TCCR2B &= ~myEraser;      // sets three bits in TCCR2B (timer 2) to zero
  TCCR4B &= ~myEraser;      // sets three bits in TCCR4B (timer 4) to zero
  TCCR4B |= 4;              // Set timer 4 (pins 6 and 8) to prescaler 4 (120 Hz).
  TCCR2B |= 6;              // Set timer 2 (pins 10 and 9) to prescaler 6 (120 Hz).

  // Set pins
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(7, INPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(safeYellow, OUTPUT); 
  pinMode(safeGreen, OUTPUT); 
  pinMode(36, INPUT); 
  pinMode(ch8Pin, INPUT); 
  pinMode(killSwitch1, OUTPUT);
  pinMode(killSwitch2, OUTPUT); 
  Serial.begin(9600);
  }
 
void loop(){
  
  ch1 = pulseIn(4, HIGH, 25000);           // PWM values coming in from remote control receiver channel 1 
  ch3 = pulseIn(3, HIGH, 25000);           // PWM values coming in from remote control receiver channel 3
  ch4 = pulseIn(5, HIGH, 25000);           // PWM values coming in from remote control receiver channel 4
  ch5 = pulseIn(7, HIGH, 25000);           // PWM values coming in from remote control receiver channel 5
  ch8 = pulseIn(ch8Pin, HIGH, 25000);      // PWM values coming in from remote control receiver channel 8 
  takeover = ch5;
  
  if(ch8 >= 2020 && ch8 <= 2060){
    digitalWrite(killSwitch1, HIGH);
    digitalWrite(killSwitch2, HIGH); 
  }

  else if(ch8 >= 1485 && ch8 <= 1500){
    digitalWrite(killSwitch1, HIGH);
    digitalWrite(killSwitch2, LOW);
  }

  else{
    digitalWrite(killSwitch1, LOW);
    digitalWrite(killSwitch2, LOW);
  }
  // Set 'takeover' variable equal to PWM values coming from channel 5
  /*
   * How 'map' function works: Variable = map(value, fromLow, fromHigh, toLow, toHIgh);
   * More information can be found at : <https://www.arduino.cc/en/Reference/Map>
   * 
   * The variables below are set to mapped down values of the PWM signals coming from the remote control receiver. 
   * The values 'fromLow' and 'fromHigh' for each channel was determined experimentally by using Serial.print(ch#); 
   * 'fromLow' should be the lowest PWM value from the channel, 'fromHigh' should be the highest PWM value from the signal. 
   * 
   * The 'toLow' and 'toHigh' values were also determined experimentally. The range of these values are actually fixed and depends 
   * on the frequency the Arduino sets the respective pins at.  
   * 
   * For example: 
   *    'ch1' is associated with the x-axis of the right joystick on the remote control. When moving the right joystick all the
   *    way to the left, 'Serial.println(ch1);' prints a value of 1084 to the serial monitor; when moving the right joystick all 
   *    the way to the right, 'Serial.println(ch1);' prints a value of 1909 to the serial monitor. The 'toLow' and 'toHigh' values 
   *    were also determiend experimentally. The frequency of 'ch1' (pin4) is 120 Hz which fixes the range of 'toLow' to 'toHigh' 
   *    at 31 to 63.   
   */
  
  ch1Decoded = map(ch1, 1084, 1909, -100, 100);     // 'rotate' is assigned a value within the range of 31 to 63 from mapped down range of channel 1
  ch3Decoded = map(ch3, 943, 2052, 100, -100);      // 'linear' is assigned a value within the range of 63 to 31 (The PWM value is highest when the left joystick is all the way down and at its lowest when the left joystick is all the way up. To rectify this, we switch the 'toLow' and 'toHigh' range in reverse.)
  ch4Decoded = map(ch4, 1094, 1909, -100, 100);     // 'strafe' is assigned a value within the rnage of 31 to 63 from mapped down range of channel 1
  
// Measure Temperature
  int tempRead = analogRead(tempPin);
  float tempVolt = float(tempRead) / 205;
  float tempCelsius = (100 * tempVolt) - 50;
  float tempF = (tempCelsius*1.8)+32;
  dtostrf(tempCelsius, 8, 2, tempStringC);
  dtostrf(tempF, 8,2, tempStringF);
  dtostrf(tempVolt, 8,2, tempStringV);
  test6.data=tempCelsius;
  temp.publish( &test6);
    
//Measuring Voltage for Box
    int voltRead = analogRead(voltPin);
      float voltVolt = voltRead*(5*4.85276 / 1023.0);
    dtostrf(voltVolt, 8, 2, voltString);    
    test7.data = voltVolt;
    voltbox.publish (&test7);
    
// Voltage reader code
  int sensorValue = analogRead(A0);
  float voltage = sensorValue*(5*4.85276 / 1023.0);
  test5.data = voltage;
  volt.publish( &test5);
      
      linear = 47+(linear-47)*12/voltage;
    
      back_linear = 47+(back_linear-47)*12/voltage;
    
      rotate = 47+(rotate-47)*12/voltage;
    
      inv_rotate = 47+(inv_rotate-47)*12/voltage;
    
      strafe = 47+(strafe-47)*12/voltage;
    
      inv_strafe = 47+(inv_strafe-47)*12/voltage;
    
//  analogWrite(11, 51); //fan at 25% duty cycle

 
  // --------------------
  // Manual Takeover Mode
  // --------------------
  if(takeover<=1000){         //  If switch A is in "up" position, the remote control receiver will consistently send PWM values less than 1000
    digitalWrite(safeYellow,HIGH);     //  Turns on yellow safety light
    digitalWrite(safeGreen,LOW);    //  Turns off green safety light
    
    throttleQ1 = ch3Decoded;
    throttleQ2 = ch3Decoded;
    throttleQ3 = ch3Decoded;
    throttleQ4 = ch3Decoded;
    strafeQ1 = ch4Decoded;
    strafeQ2 = -ch4Decoded;
    strafeQ3 = ch4Decoded;
    strafeQ4 = -ch4Decoded;
    rotateQ1 = -ch1Decoded;
    rotateQ2 = ch1Decoded;
    rotateQ3 = ch1Decoded;
    rotateQ4 = -ch1Decoded;

    Q1Decoded = throttleQ1 + strafeQ1 + rotateQ1;
    Q2Decoded = throttleQ2 + strafeQ2 + rotateQ2;
    Q3Decoded = throttleQ3 + strafeQ3 + rotateQ3;
    Q4Decoded = throttleQ4 + strafeQ4 + rotateQ4;

    // 
    if(Q1Decoded > 100){
      Q1Decoded = 100;
    }
    if(Q1Decoded < -100){
      Q1Decoded = -100;
    }
    if(Q2Decoded > 100){
      Q2Decoded = 100;
    }
    if(Q2Decoded < -100){
      Q2Decoded = -100;
    }
    if(Q3Decoded > 100){
      Q3Decoded = 100;
    }
    if(Q3Decoded < -100){
      Q3Decoded = -100;
    }
    if(Q4Decoded > 100){
      Q4Decoded = 100;
    }
    if(Q4Decoded < -100){
      Q4Decoded = -100;
    } 

    // "Squeezes" motor command range from [-100, 100](ROS) to [31, 63].
    Q1bit = map(Q1Decoded, -100, 100, 31, 63);
    Q2bit = map(Q2Decoded, -100, 100, 31, 63);
    Q3bit = map(Q3Decoded, -100, 100, 31, 63);
    Q4bit = map(Q4Decoded, -100, 100, 31, 63);

    // Ensure the motors are in neutral most of the time
    // By being close to 47. 
    if(Q1bit <= 49 && Q1bit >= 45){
      Q1bit = 47;
    }
    if(Q2bit <= 49 && Q2bit >= 45){
      Q2bit = 47;
    }
    if(Q3bit <= 49 && Q3bit >= 45){
      Q3bit = 47;
    }
    if(Q4bit <= 49 && Q4bit >= 45){
      Q4bit = 47;
    } 

    analogWrite(Q1Pin, Q1bit);
    analogWrite(Q2Pin, Q2bit);
    analogWrite(Q3Pin, Q3bit);
    analogWrite(Q4Pin, Q4bit);
  }

  // ---------------
  // Autonomous Mode
  // ---------------
  else {
    interrupts();
    digitalWrite(safeGreen,HIGH);   // turns on green safety light
    digitalWrite(safeYellow,LOW);  // turns off yellow safety light

    // Thruster Q1
    if (auto_q1 > 47)
    {
      auto_q1_butt = 47+(auto_q1-47)*12/voltage;
    }
    else if (auto_q1 < 47)
    {
      auto_q1_butt = 47-(47-auto_q1)*12/voltage;
    }
    
    if( auto_q2 > 47)
    {
      auto_q2_butt = 47+(auto_q2-47)*12/voltage;
    }
    else if (auto_q2 < 47)
    {
      auto_q2_butt = 47-(47-auto_q2)*12/voltage;
    } 
    
    if( auto_q3 > 47)
    {
      auto_q3_butt = 47+(auto_q3-47)*12/voltage;
    }
    else if (auto_q3 < 47)
    {
      auto_q3_butt = 47-(47-auto_q3)*12/voltage;
    } 
    
    if( auto_q4 > 47)
    {
      auto_q4_butt = 47+(auto_q4-47)*12/voltage;
    }
    else if (auto_q4 < 47)
    {
      auto_q4_butt = 47-(47-auto_q4)*12/voltage;
    }

    // Send command to motor controllers
    analogWrite(8,auto_q1_butt);
    analogWrite(9,auto_q2_butt);
    analogWrite(10,auto_q3_butt);
    analogWrite(6,auto_q4_butt);

    
//    if (auto_q1 > 47)
//    {
//      auto_q1 = 47+(auto_q1-47)*12/voltage;
//    }
//    else if (auto_q1 < 47)
//    {
//      auto_q1 = 47-(47-auto_q1)*12/voltage;
//    }
//    
//    if( auto_q2 > 47)
//    {
//      auto_q2 = 47+(auto_q2-47)*12/voltage;
//    }
//    else if (auto_q2 < 47)
//    {
//      auto_q2 = 47-(47-auto_q2)*12/voltage;
//    } 
//    
//    if( auto_q3 > 47)
//    {
//      auto_q3 = 47+(auto_q3-47)*12/voltage;
//    }
//    else if (auto_q3 < 47)
//    {
//      auto_q3 = 47-(47-auto_q3)*12/voltage;
//    } 
//    
//    if( auto_q4 > 47)
//    {
//      auto_q4 = 47+(auto_q4-47)*12/voltage;
//    }
//    else if (auto_q4 < 47)
//    {
//      auto_q4 = 47-(47-auto_q4)*12/voltage;
//    }
//
//    // Send command to motor controllers
//    analogWrite(8,auto_q1);
//    analogWrite(9,auto_q2);
//    analogWrite(10,auto_q3);
//    analogWrite(6,auto_q4);

    // Used to be test1,1,1,1. Now it's test1,2,3,4.
    // Display arduino value sending to the motor 
//    test1.data = auto_q1;
//    test2.data = auto_q2;
//    test3.data = auto_q3;
//    test4.data = auto_q4;
//    display_q1.publish( &test1);
//    display_q2.publish( &test2);
//    display_q3.publish( &test3);
//    display_q4.publish( &test4);

    // Diaply arduino values sending to the motor on ROS
    test1.data = auto_q1_butt;
    test2.data = auto_q2_butt;
    test3.data = auto_q3_butt;
    test4.data = auto_q4_butt;
    display_q1.publish( &test1);
    display_q2.publish( &test2);
    display_q3.publish( &test3);
    display_q4.publish( &test4);
    
    nh.spinOnce();
  }
  // Delay ignores the zero-value motor commands, which gets rid of the pulsing.
  //delay(500);
  //nh.spinOnce();
  //delay(1);
}
