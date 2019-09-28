/*kanaloaMotor.h
 *  Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
 *  LICENSE: Internal Kanaloa use only
 *  
 *  Learn how to control a mtor output voltage
 *  
 *  # Version History
 *    2019.09.13 - A Trimble (atrimble@hawaii.edu)
 *      Initial Creation
 */

#ifndef kanaloaMotor_h
#define kanaloaMotor_h

//#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

#define PWM_SPD_MAX 4095
#define PWM_SPD_MIN -4095
#define PID_SAMPLE_TIME 100
#define CALIBRATION_CONSTANT 0.003768 //20.1*0.1875/1000
#define VMAX 24.0
#define QMAX 1000.0


class kanaloaMotor {  
  public:
    // Constructors
    kanaloaMotor(Adafruit_PWMServoDriver& pwm, byte forwardPin, byte reversePin, byte color);

    // Functions
    void init();
    void pidUpdate();
    void setPWMSpd(int vel);
    void setVoltageSetpoint(double vin);
    void setVoltageSetpointFromQcommand(int qin);
    double getMotorVoltage();
    double getVoltageSetpoint() const;
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
