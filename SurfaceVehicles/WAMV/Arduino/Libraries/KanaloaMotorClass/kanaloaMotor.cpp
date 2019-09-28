/*kanaloaMotor.cpp
    Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
    LICENSE: Internal Kanaloa use only

    PID feed back control of MinnKota Motor controllers output voltage
    See header file, kanaloaMotor.h, for more information

    # Version History
      2019.09.13 - A Trimble (atrimble@hawaii.edu)
        Initial Creation
*/

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>
#include "kanaloaMotor.h"

kanaloaMotor::kanaloaMotor(Adafruit_PWMServoDriver& pwm, 
                           byte forwardPin, 
                           byte reversePin, 
                           byte color)
                           :
                           pwm(pwm),
                           forwardPin(forwardPin), 
                           reversePin(reversePin),
                           pwmSpd(0),
                           adcData(0),
                           adc(color), 
                           motorVoltage(0), 
                           pidPWMSpd(0), 
                           voltageSetpoint(0),
                           Kp(175),
                           Ki(300),
                           Kd(0),
                           motor_pid(&motorVoltage, 
                                     &pidPWMSpd,
                                     &voltageSetpoint,
                                     Kp,
                                     Ki,
                                     Kd,
                                     DIRECT){
}

void kanaloaMotor::init() {
  setPWMSpd(0);
  adc.begin();
  adc.setGain(GAIN_TWOTHIRDS);
  motor_pid.SetMode(AUTOMATIC);
  motor_pid.SetOutputLimits(PWM_SPD_MIN, PWM_SPD_MAX);
  motor_pid.SetSampleTime(PID_SAMPLE_TIME);
}

void kanaloaMotor::pidUpdate() {
  readVoltage();
  motor_pid.Compute();
  pwmSpd = (int) pidPWMSpd;
  if (abs(voltageSetpoint) <= DEAD_ZONE_LIMIT) {
    pwmSpd = 0;
  }
  setPWMSpd(pwmSpd);
}

void kanaloaMotor::setPWMSpd(int pwmCommand) {
  //Avoid shoot-through
  pwm.setPWM(forwardPin, 0, 4096);
  pwm.setPWM(reversePin, 0, 4096);
  delayMicroseconds(50);

  if (pwmCommand == 0) {
    pwm.setPWM(forwardPin, 0, 4096);
    pwm.setPWM(reversePin, 0, 4096);
  }
  else if (pwmCommand > 0 && pwmCommand < 4096) {
    pwm.setPWM(forwardPin, 0, pwmCommand);
    pwm.setPWM(reversePin, 0, 0);
  }
  else if (pwmCommand < 0 && abs(pwmCommand) < 4096) {
    pwm.setPWM(forwardPin, 0, 0);
    pwm.setPWM(reversePin, 0, abs(pwmCommand));
  }
  else {
    pwm.setPWM(forwardPin, 0, 4096);
    pwm.setPWM(reversePin, 0, 4096);
    Serial.println("Error: Speed command is out of bounds.");
    Serial.print("I tried to set the servo command to: ");
    Serial.print(pwmCommand);
    Serial.println(" I'm not sure why");
  }
}

void kanaloaMotor::setVoltageSetpoint(double vin) {
  voltageSetpoint = vin;
}

void kanaloaMotor::setVoltageSetpointFromQcommand(int qin) {
  setVoltageSetpoint(convert_motorSpeed_to_voltage(qin));
}

double kanaloaMotor::getMotorVoltage() {
  readVoltage();
  return motorVoltage;
}

double kanaloaMotor::getVoltageSetpoint() const {
  return voltageSetpoint;
}

void kanaloaMotor::debugOutput() {
  Serial.print("Voltage (Vsp, Vm): ");
  Serial.print(getVoltageSetpoint());
  Serial.print(", ");
  Serial.print(getMotorVoltage());
  Serial.print("PWM Command: ");
  Serial.println(pwmSpd);
}

void kanaloaMotor::printGains() {
  Serial.print("Gains (Kp, Ki, Kd): ");
  Serial.print(motor_pid.GetKp()); Serial.print(", ");
  Serial.print(motor_pid.GetKi()); Serial.print(", ");
  Serial.print(motor_pid.GetKd()); Serial.println(")");
}

void kanaloaMotor::printPWMcommands() {
  Serial.print("PWMcommands (pidPWMSpd, pwmSpd): ");
  Serial.print(pidPWMSpd); Serial.print(" ,");
  Serial.print(pwmSpd); Serial.println(")");
}

//PRIVATE FUNCTIONS
void kanaloaMotor::readVoltage() {
  adcData = adc.readADC_Differential_0_1();
  motorVoltage = adcData * CALIBRATION_CONSTANT;
}

double kanaloaMotor::convert_motorSpeed_to_voltage(int q_speed) {
  return VMAX / QMAX * q_speed;
}
