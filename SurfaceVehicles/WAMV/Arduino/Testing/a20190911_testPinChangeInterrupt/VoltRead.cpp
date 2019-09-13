/*
 * Print battery voltage
 * With Voltage Divider 21.4 kOhms resister
 * help from https://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
 * Jordan Dalessandro Sept 11 2019
 */
//#include <std_msgs/UInt6.h>
#include "Arduino.h"
#include "VoltRead.h"


//this is a test constructor
VoltRead::VoltRead(int pin) {
  pinMode(pin, OUTPUT)
  _pin = pin;
}






//int led_pin = 13;


int ledPin1 = 11;
int ledPin2 = 10;
int ledPin3 = 9;
int ledPin4 = 6;
int ledPin5 = 5;
//float for the difference between the max and min voltage values

//Initialize the maximum voltage the battery has
//and the minumum voltage the battery should be considered dead
float batteryMax = 29.5; //the max battery voltage keep this value higher than batteryMin
float batteryMin = 22.0; //the min battery voltage. 
float maxMinDif = (batteryMax - batteryMin)/4;
float batteryLow = batteryMin + maxMinDif;
float batteryMidLow = batteryLow + maxMinDif;
float batteryMidMax = batteryMidLow + maxMinDif;


void setup()
{
  
  Serial.begin(9600);

  //initialize the LED pins
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  //digitalWrite(led_pin, LOW);
}

void loop()
{
  
  //Store resistance value
  int sensorValue = analogRead(A0);
  
  //change the values from 0-1023 to a range that 
  //corresponds to the voltage the pin is reading, 
  //float to scale the numbers between 0.0 and 5.0, 
  //divide 5.0 by 1023.0 and multiply that by sensorValue :

  float voltage= 0.0292 * sensorValue + 0.022;
  
  //print to screen
  Serial.println(voltage);
  
  //Dead battery level
   if (voltage < batteryMin) {      
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin4, LOW); 
      digitalWrite(ledPin3, LOW); 
      digitalWrite(ledPin2, LOW); 
      digitalWrite(ledPin5, LOW); 
    }
    //Low battery power
    else if (voltage >= batteryMin && voltage < batteryLow) {   
      digitalWrite(ledPin2, HIGH);
      digitalWrite(ledPin4, LOW); 
      digitalWrite(ledPin3, LOW); 
      digitalWrite(ledPin5, LOW); 
      digitalWrite(ledPin1, LOW); 
    }
    //Medium low battery level
    else if (voltage >= batteryLow && voltage < batteryMidLow) {    
      digitalWrite(ledPin3, HIGH);
      digitalWrite(ledPin4, LOW); 
      digitalWrite(ledPin5, LOW); 
      digitalWrite(ledPin2, LOW); 
      digitalWrite(ledPin1, LOW); 
    }
    //Medium full battery level
    else if (voltage >= batteryMidLow && voltage < batteryMidMax) {  
      digitalWrite(ledPin4, HIGH);
      digitalWrite(ledPin5, LOW); 
      digitalWrite(ledPin3, LOW); 
      digitalWrite(ledPin2, LOW); 
      digitalWrite(ledPin1, LOW); 
    }
    //highest battery level
    else if (voltage >= batteryMidMax && voltage < batteryMax) {  
      digitalWrite(ledPin5, HIGH);
      digitalWrite(ledPin4, LOW); 
      digitalWrite(ledPin3, LOW); 
      digitalWrite(ledPin2, LOW); 
      digitalWrite(ledPin1, LOW); 
    }
    //above the maximum
    else {
      Serial.println("Voltage Too High");
      
      digitalWrite(ledPin5, LOW);
      digitalWrite(ledPin4, LOW); 
      digitalWrite(ledPin3, LOW); 
      digitalWrite(ledPin2, LOW); 
      digitalWrite(ledPin1, LOW); 
    }
  
}
