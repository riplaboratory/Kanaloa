/*
 * This program is used to measure the analog voltage from the batteries and send this information to the kill-switch Arduino in the low-current box via 
 * PWM waves. 
 * 
 * Created by Kai Jones
 * Date: 04/10/2019
 *
 * Updates:
 * - Implemented code to send PWM waves out. 
*/

// Pin Definitions
const int voltagePin = A1;
const int voltageOutPin = 3; 

// System Variables
float voltageBits; 
float voltage;
int   voltageOutput; 

void setup() {
  pinMode(voltagePin, INPUT);
  pinMode(voltageOutPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  voltageBits = analogRead(voltagePin);
  voltage = 0.0401*voltageBits + 0.0591; 
  Serial.print("Voltage: ");
  Serial.println(voltage);
  delay(100); 
  
  voltageOutput = map(voltage, 0.00, 40.00, 0, 255);
  analogWrite(voltageOutPin, voltageOutput); 
}
