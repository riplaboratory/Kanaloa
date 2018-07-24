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

// Pin definitions
const byte receiverPin = 2;   // output PWM pin to Q1 motor controller
const byte motorPin = 6;      // output digital pin to kill switch relay

//ROS message type
std_msgs::UInt16 test;
ros::NodeHandle nh;

//Publish message to ROS topic "motorOut"
ros::Publisher motor("motorOut", &test);

void setup() {

  //Setup subscriber
  nh.initNode();
  nh.advertise(motor);

    //Set pins
    pinMode(motorPin, OUTPUT);
    digitalWrite(motorPin, LOW);

    // Handles Arduino timers for PWM frequency (for more info: https://playground.arduino.cc/Main/TimerPWMCheatsheet and http://forum.arduino.cc/index.php/topic,72092.0.html)
    int myEraser = 7;         // this is 111 in binary, and used as an eraser
    TCCR4B &= ~myEraser;      // sets three bits in TCCR4B (timer 4) to zero
    TCCR3B &= ~myEraser;      // sets three bits in TCCR3B (timer 3) to zero
    TCCR4B |= 4;              // Set timer 4 (pins 6 and 8) to prescaler 4 (120 Hz).
    TCCR3B |= 1;              // Set timer 3 (pins 10 and 9) to prescaler 1 (31000 Hz).


    Serial.begin(57600);

}

void loop() {
{
  // full up = 1085; full down = 1920; neutral = 1505
  int receiverPinPulseWidth = pulseIn(receiverPin, HIGH); // read pulse width from receiver

  int motorOut = map(receiverPinPulseWidth, 982, 2000, 63, 31);
  analogWrite(motorPin, motorOut);
  Serial.println(motorOut); 
  test.data = motorOut;
  motor.publish(&test);

  nh.spinOnce(); 
}
  
  nh.spinOnce();
}

