/*kanaloaMotor.h
 *  Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
 *  LICENSE: Internal Kanaloa use only
 *  
 *  PID feed back control of MinnKota Motor controllers output voltage
 *  # Hardware - hardware the code is designed to control
 *      - PWM Servo shield
 *      - 2.5 V constant ouput control board. Requires 2 channels one to 
 *          provide forward commands and one to provide reverse commands
 *      - ADS1115 ADC circuit
 *  # Libraries
 *      ## Library Manager (Sketch ->Include Library -> Manage Libraries)
 *          - "Adafruit PWM Servo Driver Library" by Adafruit
 *          - "Adafruit ADS1X15" by Adafruit
 *      ## Downloaded
 *          - PID_v1 by Brett Beauregard, ADD GITHUB WEBSITE
 *  
 *  # Version History
 *    2019.09.13 - A Trimble (atrimble@hawaii.edu)
 *      Initial Creation
 */

#ifndef kanaloaMotor_h
#define kanaloaMotor_h

#include <PID_v1.h>

#define PWM_SPD_MAX 4095
#define PWM_SPD_MIN -4095
#define PID_SAMPLE_TIME 100
#define CALIBRATION_CONSTANT 0.003768 //20.1*0.1875/1000
#define VMAX 24.0
#define QMAX 1000.0
#define DEAD_ZONE_LIMIT 2 


class kanaloaMotor {  
  public:
    // Constructors
    kanaloaMotor(Adafruit_PWMServoDriver& pwm, 
    byte forwardPin,
    byte reversePin,
    byte color);

    // Functions
    void init();
    void pidUpdate();
    void setPWMSpd(int vel);
    void setVoltageSetpoint(double vin);
    void setVoltageSetpointFromQcommand(int qin);
    double getMotorVoltage();
    double getVoltageSetpoint() const;
    void debugOutput();
    void printGains();
    void printPWMcommands();

  private:
    // Servo shield stuff
    Adafruit_PWMServoDriver& pwm;
    byte forwardPin;
    byte reversePin;
    int pwmSpd;
    
    // ADC stuff
    int16_t adcData;
    Adafruit_ADS1115 adc;
    
    // PID stuff
    double motorVoltage;
    double pidPWMSpd;
    double voltageSetpoint;
    double Kp;
    double Ki;
    double Kd;
    PID motor_pid;
    
    // Functions
    void readVoltage();
    double convert_motorSpeed_to_voltage(int q_speed);
};

#endif
