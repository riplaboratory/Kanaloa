/*
 * I2C master. This program takes the motor controller outputs within 
 * the range of 0-4096 (with 2048 being neutral), creates a string for 
 * each output, and sends it to the high current Arduino via I2C
 * communication protocol. 
 * 
 * Msg Template: 
 *  Q    : start flag
 *  1-4  : thruster number
 *  XXXX : thruster speed magnituderu
 *  F/R  : direction & end flag
 * 
 * Example Output per line: Q13000FQ2500R Q33000FQ4500R 
 * 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

// -------- CONSTANTS ----------------------
#define SLAVE_ADDR 9        // arbitrary address number to identify slave Arduino
#define ANSWERSIZE 14       // number of bits expected to come from slave Arduino
#define ROS_BAUDRATE 57600
#define FREQ 50
#define Vmax 12
#define ORANGE 0x48  // GND
#define GREEN 0x49   // VDD
#define BROWN 0x4B   // SCL
#define BLUE 0x4A    // SDA 
#define MC1_F 4
#define MC1_R 5
#define MC2_F 0
#define MC2_R 1
#define MC3_F 6
#define MC3_R 7
#define MC4_F 2
#define MC4_R 3
#define LF MC3_F
#define LR MC3_R
#define RF MC1_F
#define RR MC1_R

// --------- MESSAGE VARIABLES -------------
volatile char message[28];    // array used to store motor command message from slave Arduino as characters
String q1Msg;                 // string for parsing Q1 motor command
String q2Msg;                 // string for parsing Q2 motor command
String q3Msg;                 // string for parsing Q3 motor command
String q4Msg;                 // string for parsing Q4 motor command
String voltageMsg = "V";      // string for sending Voltage message to slave Arduino
int q1Motor;                  // Q1 motor command to send to motor controller
int q2Motor;                  // Q2 motor command to send to motor controller
int q3Motor;                  // Q3 motor command to send to motor controller
int q4Motor;                  // Q4 motor command to send to motor controller
char q1Dir;                   // variable to store character of Q1's motor direction
char q2Dir;                   // variable to store character of Q2's motor direction
char q3Dir;                   // variable to store character of Q3's motor direction
char q4Dir;                   // variable to store character of Q4's motor direction
bool debug = true;            // set to true to see printouts for messages

// Setup the PWM Shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Setup the ADCs
Adafruit_ADS1115 adc_left(BROWN);
Adafruit_ADS1115 adc_right(GREEN);
int16_t motorVoltage_adc_left = 0;   //actual voltage sent to the motors
int16_t motorVoltage_adc_right = 0;
const float motorVoltageCalibration = 20.1;


// Set up PID controller
int spd_left = 0; // pwm command [-4095, 4095] - thing we want to change to control
int spd_right = 0;
double motorVoltage_left, motorVoltage_right, pidSpd_left, pidSpd_right, voltageSetpoint_left = 0., voltageSetpoint_right = 0.;
double Kp = 210, Ki = 50, Kd = 5;
PID motor_left_pid(&motorVoltage_left, &pidSpd_left, &voltageSetpoint_right, Kp, Ki, Kd, DIRECT);
PID motor_right_pid(&motorVoltage_right, &pidSpd_right, &voltageSetpoint_left, Kp, Ki, Kd, DIRECT);


// Timer for sending main battery voltage
int motorCmdTimer = millis();    // timer variable to store time for requesting motor commands 
float motorCmdFreq = 10;         // number of times to request motorCmds from slave Arduino [Hz]
int voltageTimer = millis();     // timer variable to store time for sending voltage measurement
float voltageSendFreq = 4;       // number of times to send voltage measurement to slave Arduino [Hz]


// Variables for high battery voltage monitor
const int voltagePin = A1;    // pin for voltage measurement
int sensorValue;              // voltage measurement in bits (0-255)
float voltage;                // voltage measurement


void setup() {
  Wire.begin();         // join the I2C bus as a master (master device has no paramter)

    // Set up serial connection(s)
  Serial.begin(ROS_BAUDRATE);

    // Initialize servo shield
  pwm.begin();
  pwm.setPWMFreq(FREQ);
    // Make sure all the outputs are off initially.
    pwm.setPWM(byte(LF),0,4096);
    pwm.setPWM(byte(LR),0,4096);
    pwm.setPWM(byte(RF),0,4096);
    pwm.setPWM(byte(RR),0,4096);

      // Initialize ADC
  adc_left.begin();
  adc_right.begin();

  // Initialize the PID controller
  motor_left_pid.SetMode(AUTOMATIC);
  motor_left_pid.SetOutputLimits(-4095,4095);
  motor_left_pid.SetSampleTime(100);
  motor_right_pid.SetMode(AUTOMATIC);
  motor_right_pid.SetOutputLimits(-4095,4095);
  motor_right_pid.SetSampleTime(100);
}

void loop() {
  setSpd(spd_left,LF,LR);
  setSpd(spd_right,RF,RR);
  
  // Read voltages from the ADC
  motorVoltage_adc_left = adc_left.readADC_Differential_0_1();
  motorVoltage_adc_right = adc_right.readADC_Differential_0_1();

    // Convert bits to voltage
  motorVoltage_left = motorVoltage_adc_left*motorVoltageCalibration*0.1875/1000;
  motorVoltage_right = motorVoltage_adc_right*motorVoltageCalibration*0.1875/1000;

    // Execute the pid
  motor_left_pid.Compute();
  motor_right_pid.Compute();
  spd_left = (int) pidSpd_left;
  spd_right = (int) pidSpd_right;

    if (abs(voltageSetpoint_left) <= 0.5){
    spd_left = 0;
    spd_right = 0;
  }

  //Update target voltage
  vupdate();

  // Request motor commands from slave Arduino at a set frequency
  if((millis()-motorCmdTimer) > (1/motorCmdFreq)){
    // Receive motor commands from slave Arduino
    requestFromSlave();
    parseMotorCmds();

    // Reset timer
    motorCmdTimer = millis();
  }

  // Send main battery voltage to slave Arduino at a set frequency
  if((millis()-voltageTimer) > (1/voltageSendFreq)){
    // Measure mainBattVoltage
    getVoltage();
    
    // Send main battery voltage to slave Arduino every 
    sendVoltageMsg();
    
    // Reset timer
    voltageTimer = millis();
  }
}
