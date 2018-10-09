//Note: On serial, you will see a line bunch of zeros before the thruster info. Those 0's mean that I2c communication is working. (Serial.println(endtransmission))
//Sometimes, you may see a 4 or another number pop up, this is where normally a failure would occur, but the timeout function catches it. After a bit, the serial should continue

// Arduino library inclusions
#if (ARDUINO >= 100)            // Arduino
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>                // ROS inclusions
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <Adafruit_MCP4725.h>   // Adafruit DAC, must install and add "Adafruit_MCP4725" library to your IDE to compile
Adafruit_MCP4725 dac;           // Adafruit DAC constructor
#include<Wire.h>                // To communicate with the DACs
#include<I2C.h>                 //For managing the freezing problem. To install this library go to the following link, download the library from the website (or on our Git), open the Arduino folder, find the library folder, then extract the library.
//http://dsscircuits.com/articles/arduino-i2c-master-library
//An example of the I2C library in use. https://arduino.stackexchange.com/questions/30353/i2c-connection-freezes-master-if-slave-disconnects

// I2C definitions
#define TCAADDR 0x70

// Pin definitions
// Note: Pin 3 and Pin 2 on main arduino is not working.
const int voltMainPin = A0;     // analog in from voltage divider pin for reading main battery voltage
const int tempPin = A3;         // analog in from temperature probe pin for reading temperature (optional)
const byte ch1Pin = 4;          // PWM in from remote control receiver channel 1 (right stick left-to-right yaw/rotation)
const byte ch6Pin = 6;          // PWM in from remote control receiver channel 6 (left stick up-to-down surge/forward)
const byte ch4Pin = 5;          // PWM in from remote control receiver channel 4 (left stick left-to-right sway/strafe)
const byte ch5Pin = 7;          // PWM in from remote control receiver channel 5 (switches between manual and autonomous mode)
const byte ch7Pin = 3;          // PWM in from remote control receiver channel 7 (switches reverse contactor on and off)
const byte ch8Pin = 8;          // PWM in from remote control receiver channel 8 (kill signal)
const byte stateOutPin = 10;    // digital out to kill-switch Arduino in high current box (HIGH for autonomous, LOW for manual)
const byte killOutPin = 11;     // digital out to kill-switch Arduino in high current box (HIGH for kill, LOW for unkill)
const byte revConQ1Pin = 14;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ2Pin = 19;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ3Pin = 16;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConQ4Pin = 17;    // digital out to Q1 reversing contactor (HIGH for motor to spin forward, LOW to spin in reverse)
const byte revConKillPin = 18;  // digital out to kill power to reversing contactors (HIGH to allow reversing contactors, LOW to put motors into BRAKE mode)

// Global code variables
int mode;                           // determines manual (1), autonomous (2), or invalid (0) operation mode
float voltMain;                     // voltage from the main battery
const int voltOutMax = 36;          // maximum output voltage to thrusters [V]
unsigned long timeNow = 0;          // time tracking variable
unsigned long timeLast = 0;         // time tracking variable
const unsigned int pulseMax = 1891; // maximum pulse length from receiver
const unsigned int pulseMin = 1080; // minimum pulse length from receiver
float autoQ1;                         // autonomous command signal from Matlab for Q1 thruster (via ROS)
float autoQ2;                         // autonomous command signal from Matlab for Q2 thruster (via ROS)
float autoQ3;                         // autonomous command signal from Matlab for Q3 thruster (via ROS)
float autoQ4;                         // autonomous command signal from Matlab for Q4 thruster (via ROS)
float Q1Setpoint = 0;                 // Setpoint for Q1 thruster output
float Q2Setpoint = 0;                 // Setpoint for Q2 thruster output
float Q3Setpoint = 0;                 // Setpoint for Q3 thruster output
float Q4Setpoint = 0;                 // Setpoint for Q4 thruster output
float Q1Out = 0;                      // Q1 thruster output (acceleration limited)
float Q2Out = 0;                      // Q2 thruster output (acceleration limited)
float Q3Out = 0;                      // Q3 thruster output (acceleration limited)
float Q4Out = 0;                      // Q4 thruster output (acceleration limited)
float Q1Last = 0;                     // last Q1 thruster output
float Q2Last = 0;                     // last Q2 thruster output
float Q3Last = 0;                     // last Q3 thruster output
float Q4Last = 0;                     // last Q4 thruster output
float Q1DacOut = 0;                   // bit output to Q1 DAC
float Q2DacOut = 0;                   // bit output to Q2 DAC
float Q3DacOut = 0;                   // bit output to Q3 DAC
float Q4DacOut = 0;                   // bit output to Q4 DAC

// ROS node handle (allows program to create publishers and subscribers)
ros::NodeHandle nh;

// ROS messages
std_msgs::UInt16 Q1Msg;         // Q1 thruster message
std_msgs::UInt16 Q2Msg;         // Q2 thruster message
std_msgs::UInt16 Q3Msg;         // Q3 thruster message
std_msgs::UInt16 Q4Msg;         // Q4 thruster message
std_msgs::Float64 voltMainMsg;  // main battery voltage message

// ROS publishers
ros::Publisher voltMainPub("voltMain", &voltMainMsg); // main battery voltage
ros::Publisher Q1Pub("Q1", &Q1Msg);                   // Q1 thruster output
ros::Publisher Q2Pub("Q2", &Q2Msg);                   // Q2 thruster output
ros::Publisher Q3Pub("Q3", &Q3Msg);                   // Q3 thruster output
ros::Publisher Q4Pub("Q4", &Q4Msg);                   // Q4 thruster output

// ROS callback functions (must come before subscibers)
void autoQ1Cb(const std_msgs::Float64& autoQ1CbMsg) {
  //autoQ1 = map(autoQ1CbMsg.data, -100, 100, fullreverse, fullforward);
  autoQ1 = autoQ1CbMsg.data;
}
void autoQ2Cb(const std_msgs::Float64& autoQ2CbMsg) {
  //autoQ2 = map(autoQ2CbMsg.data, -100, 100, fullreverse, fullforward);
  autoQ2 = autoQ2CbMsg.data;
}
void autoQ3Cb(const std_msgs::Float64& autoQ3CbMsg) {
  //autoQ3 = map(autoQ3CbMsg.data, -100, 100, fullreverse, fullforward);
  autoQ3 = autoQ3CbMsg.data;
}
void autoQ4Cb(const std_msgs::Float64& autoQ4CbMsg) {
  //autoQ4 = map(autoQ4CbMsg.data, -100, 100, fullreverse, fullforward);
  autoQ4 = autoQ4CbMsg.data;
}

// ROS Subscribers
ros::Subscriber<std_msgs::Float64> autoQ1sub("autoQ1", &autoQ1Cb);   // autonomous Q1 thruster output
ros::Subscriber<std_msgs::Float64> autoQ2sub("autoQ2", &autoQ2Cb);   // autonomous Q2 thruster output
ros::Subscriber<std_msgs::Float64> autoQ3sub("autoQ3", &autoQ3Cb);   // autonomous Q3 thruster output
ros::Subscriber<std_msgs::Float64> autoQ4sub("autoQ4", &autoQ4Cb);   // autonomous Q4 thruster output

void setup() {

  // Initialize ROS node handle
  nh.initNode();

  // Initialize ROS publishers
  nh.advertise(voltMainPub);    // main battery voltage
  nh.advertise(Q1Pub);          // Q1 thruster output
  nh.advertise(Q2Pub);          // Q2 thruster output
  nh.advertise(Q3Pub);          // Q3 thruster output
  nh.advertise(Q4Pub);          // Q4 thruster output

  // Initialize ROS subscibers
  nh.subscribe(autoQ1sub);      // autonomous Q1 thruster output
  nh.subscribe(autoQ2sub);      // autonomous Q2 thruster output
  nh.subscribe(autoQ3sub);      // autonomous Q3 thruster output
  nh.subscribe(autoQ4sub);      // autonomous Q4 thruster output

  // Set pins
  pinMode(ch6Pin, INPUT);
  pinMode(ch1Pin, INPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(ch5Pin, INPUT);
  pinMode(stateOutPin, OUTPUT);
  pinMode(killOutPin, OUTPUT);
  pinMode(revConQ1Pin, OUTPUT);
  pinMode(revConQ2Pin, OUTPUT);
  pinMode(revConQ3Pin, OUTPUT);
  pinMode(revConQ4Pin, OUTPUT);
  pinMode(revConKillPin, OUTPUT);

  // Initialize I2C hardware
  I2c.begin();
  tcaselect(0);
  dac.begin(0x62);
  TWDR = 72; //Sets speed to 100khz (default) Change the 12 to a 72 for high speed (400khz)
  // Set serial baud rate
  Serial.begin(57600);
  I2c.timeOut(100); //This should prevent freezing
}

void loop() {

  // Read mode, revCon, and battery voltage pins
  unsigned int ch7Raw = pulseIn(ch7Pin, HIGH);    // PWM in from remote control receiver channel 7 (SB switch) for switching reverse contactor on and off
  unsigned int ch5Raw = pulseIn(ch5Pin, HIGH);    // PWM in from remote control receiver channel 5 (SF switch) for selecting manual (down) or autonomous (up) mode. down = 1080; up = 1890.
  unsigned int ch8Raw = pulseIn(ch8Pin, HIGH);    // PWM in from remote control receiver channel 5 (SD switch) for selecting manual (down) or autonomous (up) mode. down = 1080; up = 1890.
  readMainBatteryVoltage();                       // reads and smooths battery voltage by taking multiple analogReads and taking the median.

  // Kill switch state
  if (ch8Raw > (pulseMin - 100) && ch8Raw < (pulseMin + 100)) {   // switch is in the UP position, unkill 
    digitalWrite(killOutPin, LOW);
  }
  else {
    digitalWrite(killOutPin, HIGH);                               // switch is in the DOWN position, kill
  }

  // Reverse contactor kill state
  if (ch7Raw > (pulseMin - 100) && ch7Raw < (pulseMin + 100)) {   // switch is in the UP position, unkill
    digitalWrite(revConKillPin, HIGH);
  }
  else {
    digitalWrite(revConKillPin, LOW);                             // switch is in the UP position, kill
  }

  int tol = 150;

  // Determine mode
  if (ch5Raw > (pulseMin - tol) && ch5Raw < (pulseMin + tol)) {
    mode = 1;
  }
  else if (ch5Raw > (pulseMin - tol) && ch5Raw < (pulseMax + tol)) {
    mode = 2;
  }
  else {
    mode = 0;
  }

  // Manual Code
  if (mode == 1) {

    // Send WAM-V state to kill-switch Arduino
    digitalWrite(stateOutPin, LOW);    // LOW means WAM-V is in manual-control state

    // Read signals from manual controller and encode to thruster setpoint components
    joy2setpoint();

    // Calculate thruster output components based thruster setpoint components and acceleration limit
    setpoint2output();

    // Write output thrust components
    output2dacRevCon();

    // Print
    Serial.print("Thruster setpoints:   ");
    Serial.print(Q1Setpoint);
    Serial.print(" ");
    Serial.print(Q2Setpoint);
    Serial.print(" ");
    Serial.print(Q3Setpoint);
    Serial.print(" ");
    Serial.println(Q4Setpoint);
    Serial.print("Thruster outputs:     ");
    Serial.print(Q1Out);
    Serial.print(" ");
    Serial.print(Q2Out);
    Serial.print(" ");
    Serial.print(Q3Out);
    Serial.print(" ");
    Serial.println(Q4Out);
    Serial.print("Thruster bit output:  ");
    Serial.print(Q1DacOut);
    Serial.print(" ");
    Serial.print(Q2DacOut);
    Serial.print(" ");
    Serial.print(Q3DacOut);
    Serial.print(" ");
    Serial.println(Q4DacOut);
    Serial.println(" ");
    delay(1000); //Comment out after testing

  }

  // Autonomous Code
  else if (mode == 2) {

    // Send WAM-V state to kill-switch Arduino
    digitalWrite(stateOutPin, HIGH);    // HIGH means WAM-V is in autonomous state

    Serial.println("AUTONOMOUS MODE ENGAGED (WHICH DOES NOTHING OM NOM NOM NOM)");
    Serial.println(" ");

  }

  // Invalid input code
  else {

    // Send WAM-V state to kill-switch Arduino
    digitalWrite(stateOutPin, LOW);     // LOW means WAM-V is in manual-control state

    Serial.println("INVALID MODE SELECTION ENGAGED. CHECK YOUR TRANSMITTER CONNECTION!");
    Serial.println(" ");

  }

  // Small delay to prevent errors
  delay(5);

  // Spin once to refresh ROS callbacks
  nh.spinOnce();

}

void readMainBatteryVoltage() {
  // Reads and smooths battery voltage by taking multiple analogReads and taking the median.
  // Note that reading is still very rough (+/-5V) even when doing this.

  float calib = 0.062;
  unsigned long total = 0;
  int voltMainRawMean = 0;
  int voltMainRawMedian = 0;
  const unsigned int nReadings = 50;
  int readings[nReadings];

  // Instantiate readings array with zeros
  for (unsigned int thisReading = 0; thisReading < nReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Take readings
  for (unsigned int thisReading = 0; thisReading < nReadings; thisReading++) {
    total = total - readings[thisReading];
    readings[thisReading] = analogRead(voltMainPin);
    total = total + readings[thisReading];
    delay(1);
  }

  // Sort readings
  sortArray(readings, nReadings);               // sort array to take median

  // Calculate mean and median
  voltMainRawMean = total / nReadings;          // mean (doesn't get used)
  voltMainRawMedian = readings[nReadings / 2];  // median

  // Convert to calibrated voltage
  voltMain = voltMainRawMedian * calib;         // convert voltMainRaw to voltMainRaw

  // Catching code for zero read voltage
  if (voltMain < 0.05 && voltMain > -0.05) {
    //    Serial.println("CAUTION: Main battery voltage not detected!  Main battery is probably not plugged in!");
    //    Serial.println("Setting voltMain = 50 for debugging purposes...");
    voltMain = 50;
  }

  // Publish to ROS
  voltMainMsg.data = voltMain;          // set voltMain message
  voltMainPub.publish(&voltMainMsg);    // publish voltMain topic

}

void sortArray(int a[], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
      }
    }
  }
}

void joy2setpoint() {

  // Function code variables
  unsigned int deadZone = 10;     // +/- percentage of middle area of joysticks for zero input [%] (min = -100; max = 100)

  // Read signals from transmitter and encode it for motors
  unsigned int ch1Raw = pulseIn(ch1Pin, HIGH); // PWM in from remote control receiver channel 1 (right stick left-to-right) for yaw control. left = 1080, right = 1891.
  unsigned int ch6Raw = pulseIn(ch6Pin, HIGH); // PWM in from remote control receiver channel 6 (left stick up-to-down) for surge control. up = 1075, right = 1900.
  unsigned int ch4Raw = pulseIn(ch4Pin, HIGH); // PWM in from remote control receiver channel 4 (left stick left-to-right) for sway control. left = 1075, right = 1900.

  // Calculate voltage multiplier based on battery voltage
  float voltMult = voltOutMax / voltMain;
  if (voltMult >= 1) {
    Serial.println("Serious problem encountered with main battery voltage calculation...disabling thruster output");
    voltMult = 0;
  }

  // Map receiver inputs from -100 to 100
  int yawIn = map(ch1Raw, pulseMin, pulseMax, 100, -100); // yaw (positive = CCW, negative = CW)
  int surgeIn = map(ch6Raw, pulseMin, pulseMax, -100, 100); // surge (positive = forward, negative = backward)
  int swayIn = map(ch4Raw, pulseMin, pulseMax, -100, 100); // sway (positive = right, negative = left)

  // Apply dead zone to controller components
  if (yawIn <= deadZone || yawIn >= -deadZone) {
    yawIn = 0;
  }
  if (surgeIn <= deadZone || surgeIn >= -deadZone) {
    surgeIn = 0;
  }
  if (swayIn <= deadZone || swayIn >= -deadZone) {
    swayIn = 0;
  }

  // Calculate surge, sway, and yaw components
  int surgeQ1 = surgeIn;
  int surgeQ2 = surgeIn;
  int surgeQ3 = surgeIn;
  int surgeQ4 = surgeIn;
  int swayQ1 = -swayIn;
  int swayQ2 = swayIn;
  int swayQ3 = -swayIn;
  int swayQ4 = swayIn;
  int yawQ1 = yawIn;
  int yawQ2 = -yawIn;
  int yawQ3 = -yawIn;
  int yawQ4 = yawIn;

  // Sum surge, sway, and yaw components
  Q1Setpoint = surgeQ1 + swayQ1 + yawQ1;
  Q2Setpoint = surgeQ2 + swayQ2 + yawQ2;
  Q3Setpoint = surgeQ3 + swayQ3 + yawQ3;
  Q4Setpoint = surgeQ4 + swayQ4 + yawQ4;

  // Limit setpoint components to no less than -100 and no more than 100
  Q1Setpoint = constrain(Q1Setpoint, -100, 100);
  Q2Setpoint = constrain(Q2Setpoint, -100, 100);
  Q3Setpoint = constrain(Q3Setpoint, -100, 100);
  Q4Setpoint = constrain(Q4Setpoint, -100, 100);

  // Scale setpoint components based on battery voltage
  Q1Setpoint = Q1Setpoint * voltMult;
  Q2Setpoint = Q2Setpoint * voltMult;
  Q3Setpoint = Q3Setpoint * voltMult;
  Q4Setpoint = Q4Setpoint * voltMult;

}

void setpoint2output() {

  // Function code variables
  const byte accLimit = 35;     // acceleration limit (maximum change in thrust percentage per second) [%/s]

  // Calculate time delta since last loop
  timeNow = millis();
  float timeDelta = (timeNow - timeLast) * 0.001;

  // Calculate maximum thrust delta
  float thrustDelta = accLimit * timeDelta;   // maximum change in thrust component percentage in this loop [%]

  // Calculate actual thrust delta
  int Q1Delta = Q1Setpoint - Q1Last;
  int Q2Delta = Q2Setpoint - Q2Last;
  int Q3Delta = Q3Setpoint - Q3Last;
  int Q4Delta = Q4Setpoint - Q4Last;

  // Limit Q1 output thrust if change in setpoint is greater than allowable delta
  if (abs(Q1Delta) > thrustDelta) {
    if (Q1Delta > 0) {
      Q1Out = Q1Last + thrustDelta;
    }
    else {
      Q1Out = Q1Last - thrustDelta;
    }
  }
  else {
    Q1Out = Q1Setpoint;
  }

  // Limit Q2 output thrust if change in setpoint is greater than allowable delta
  if (abs(Q2Delta) > thrustDelta) {
    if (Q2Delta > 0) {
      Q2Out = Q2Last + thrustDelta;
    }
    else {
      Q2Out = Q2Last - thrustDelta;
    }
  }
  else {
    Q2Out = Q2Setpoint;
  }

  // Limit Q3 output thrust if change in setpoint is greater than allowable delta
  if (abs(Q3Delta) > thrustDelta) {
    if (Q3Delta > 0) {
      Q3Out = Q3Last + thrustDelta;
    }
    else {
      Q3Out = Q3Last - thrustDelta;
    }
  }
  else {
    Q3Out = Q3Setpoint;
  }

  // Limit Q4 output thrust if change in setpoint is greater than allowable delta
  if (abs(Q4Delta) > thrustDelta) {
    if (Q4Delta > 0) {
      Q4Out = Q4Last + thrustDelta;
    }
    else {
      Q4Out = Q4Last - thrustDelta;
    }
  }
  else {
    Q4Out = Q4Setpoint;
  }

  // Save timeNow and thruster output components for calculation in next loop
  timeLast = timeNow;
  Q1Last = Q1Out;
  Q2Last = Q2Out;
  Q3Last = Q3Out;
  Q4Last = Q4Out;

  // Publish to ROS
  Q1Msg.data = Q1Out;
  Q2Msg.data = Q2Out;
  Q3Msg.data = Q3Out;
  Q4Msg.data = Q4Out;
  Q1Pub.publish( &Q1Msg);
  Q2Pub.publish( &Q2Msg);
  Q3Pub.publish( &Q3Msg);
  Q4Pub.publish( &Q4Msg);

}

void output2dacRevCon() {
  // Outputs appropriate signal to DACs and reverse contactors based on thruster outputs

  // Function code variables
  const byte Q1Port = 0;
  const byte Q2Port = 1;
  const byte Q3Port = 2;
  const byte Q4Port = 3;
  const int dacRes = 4095;

  // Output analog signal to DACs
  Wire.begin();
  tcaselect(Q1Port);                        // open the port to Q1 DAC.
  Q1DacOut = abs(Q1Out * dacRes / 100);     // calculate the bit output to Q1 DAC
  dac.setVoltage(Q1DacOut, false);          // Set the voltage to send to the motor
  tcaselect(Q2Port);                        // open the port to Q2 DAC.
  Q2DacOut = abs(Q2Out * dacRes / 100);     // calculate the bit output to Q2 DAC
  dac.setVoltage(Q2DacOut, false);          // Set the voltage to send to the motor
  tcaselect(Q3Port);                        // open the port to Q3 DAC.
  Q3DacOut = abs(Q3Out * dacRes / 100);     // calculate the bit output to Q3 DAC
  dac.setVoltage(Q3DacOut, false);          // Set the voltage to send to the motor
  tcaselect(Q4Port);                        // open the port to Q4 DAC.
  Q4DacOut = abs(Q4Out * dacRes / 100);     // calculate the bit output to Q4 DAC
  dac.setVoltage(Q4DacOut, false);          // Set the voltage to send to the motor

  // Control direction of reversing contactor
  if (Q1Out >= 0) {
    digitalWrite(revConQ1Pin, LOW);
    //    Serial.println("Q1 FORWARD");
  }
  else {
    digitalWrite(revConQ1Pin, HIGH);
    //    Serial.println("Q1 REVERSE");
  }
  if (Q2Out >= 0) {
    digitalWrite(revConQ2Pin, LOW);
    //    Serial.println("Q2 FORWARD");
  }
  else {
    digitalWrite(revConQ2Pin, HIGH);
    //    Serial.println("Q2 REVERSE");
  }
  if (Q3Out >= 0) {
    digitalWrite(revConQ3Pin, LOW);
    //    Serial.println("Q3 FORWARD");
  }
  else {
    digitalWrite(revConQ3Pin, HIGH);
    //    Serial.println("Q3 REVERSE");
  }
  if (Q4Out >= 0) {
    digitalWrite(revConQ4Pin, LOW);
    //    Serial.println("Q4 FORWARD");
  }
  else {
    digitalWrite(revConQ4Pin, HIGH);
    //    Serial.println("Q4 REVERSE");
  }

}


void tcaselect(uint8_t i) {
  // Opens Ports on the multiplexer to communicate with individual DACs
  // i is the number of the port
  // Provided by Adafruit Documentation (https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test)

  if (i > 7) {
    Serial.println("Invalid multiplexer port selected");
    return;
  }
  I2c.write(TCAADDR,1 << i);
}

int scaledaccel(int oval, int nval, bool isaccel) {
  //Auxillary function to accellimit
  //Controls how quickly the DAC needs to change voltage depending on the difference in voltage represented using 12 bit resolution.
  int diff = 0;
  diff = abs(nval - oval);
  if (isaccel) { //If the WAMV is decelerating
    if (diff >= 1000) {
      return oval + 200;
    }
    if (diff >= 100) {
      return oval + 50;
    }
    if (diff >= 10) {
      return oval + 5;
    }
    if (diff >= 1) {
      return oval + 1;
    }
  }
  else { //If WAMV is accelerating
    if (diff >= 1000) {
      return oval - 200;
    }
    if (diff >= 100) {
      return oval - 50;
    }
    if (diff >= 10) {
      return oval - 5;
    }
    if (diff >= 1) {
      return oval - 1;
    }
  }
}
