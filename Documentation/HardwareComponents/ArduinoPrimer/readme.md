# Arduino Primer

## Youtube Tutorials Here:  
<https://www.youtube.com/channel/UCDGAg1pqcy8IScMqHHTy9Gg>

## 1. Hardware

Pins with the tilde symbol (~) sends out PWM waves. 
Pins 0 and 1 are specifically for receiving and transferring data (respectively) to other devices.
Analog pins are used to convert analog inputs into digital inputs by changing them into small “steps.” 
The ARDUINO can only give out up to 3.3V or 5V with the respective pins. 
Reset button on the board simply resets the program (NOT TO BE MISTAKEN FOR RESETTING THE BOARD)
The ARDUINO board can be reset by applying 0V to the reset pin. 

## 2.  Installing ARDUINO IDE (Integrated Development Environment)

Visit ARDUINO’s website, download, and install. Easy peasy. 

## 3. ARDUINO IDE and Sketch Overview

Language is a simpler version of C++
Compiling is reading human readable code and translating it to machine code
Save VERY FREQUENTLY (like every line of code)
Verify (CTRL+R) checks the code to see if there are any errors 
Upload (CTRL+U) transmits information to the arduino board
Serial Monitor Window (Shift+CTRL+M) allows you to see the communication between the computer and arduino 
By going to File>>Examples has examples of sample codes to help the user get started with a specific code----”Standing on the shoulders of giants”

## 4. Understanding Arduino Syntax

Syntax is basically grammar for computer coding languages
Comments can be made in two ways:
//   -----     is used to make comments in one line
/*-------*/    is used to make comments in multiple lines
End every statement with a semicolon ; 
Functions are basically codes simplified into one keyword
Two main functions:
1. Setup: Everything here is run once. Gets everything “set up.”
2. Loop: Runs the code over and over. 

Void is a function that does not return any values from running the code. 
Curly bracket (or braces) encloses around functions.


## 5. Understanding Variables 

Variables are similar to variables in mathematics. 
Variables can only hold data types that you specify.
To declare a variable
Specify data type >> Name variable (i.e. int myNum;)
Try to make variable names descriptive of its function
The int data type stands for integer. An integer can only be a number from -32,768 to 32,768
If you cross the threshold, it will rollover and get the negative value of the other end.

## 6. How to Blink an LED

Before the two main functions (setup and loop,) declare and initialize variables. 
Within void setup() function, set mode of pins.
Function for setting pin modes is pinMode(variable, mode);      where mode is either INPUT or OUTPUT. 
Within the void loop() function, statements will tell the board what to do over and over.  
 Function digitalWrite(variable, HIGH/LOW) applies a voltage to the pin
delay(ms)   delays the next statement to run for ‘x’ amount of time in milliseconds.

## 7. digitalRead() and the Serial Port

Serial.begin(9600) is a function within a library where it initialize serial communication at 9600 bits per second  
Within setup() function. 
“Serial” is the name of the library. 
“begin” is the name of the function in the library 
The period is used for accessing libraries 
Always 9600 (for now) 
 pinMode will be set to “INPUT” mode because we want to read the voltage on the pin 
buttonState = digitalRead(pushButton);
pushButton is the digital pin
buttonState is a variable that will hold the state of the pushButton
digitalRead() function will return a 1 if voltage is HIGH and return a 0 if voltage is LOW
Serial.println(buttonState);
Prints the value of buttonState

## 8. analogRead() and the Serial Port

Analog inputs are labeled A0-A5 pins on the board
Analog inputs are wide ranges of voltages that are converted into digital inputs by dividing the range into smaller portions after going through the digital-analog converter
int sensorValue = analogRead(A0);
analogRead reads the voltage being applied at the pin and return an integer value
Will return a voltage 0<x<1023 where 5 volts will return a value of 1023 (scaled with the voltage)
Serial.println(sensorValue) prints the value of sensorValue to the serial monitor 

## 9. How to read voltages with analogRead()

Serial.begin(9600); is in setup to initialize serial communication at 9600 bits per second
Int sensorValue = analogRead(A0);
sensorValue is variable set to the voltage value received from pin A0 
sensorValue will be a value between 0<x<1023
analogRead is the function that reads the analog input from pin A0
float  voltage = sensorValue * (5.0/1023.0);
Voltage is set to the conversion of sensorValue from bits to voltage
float is a data type that has decimals
Float can also be a giant number (2x10^38)
Only precise up to 7 total digits (characters)
Slow computations
Can return weird numbers

## 10. Fade an LED

int led = 9;  sets the LED to pin 9 
int brightness = 0;  determines the “brightness” level of the LED as an integer
int fadeAmount = 5;  is a number to how many points to fade the LED by
Within void setup() 
pinMode(led, OUTPUT); sets LED pin to OUTPUT
analogWrite(pin, value);
Has nothing to do with the analog pins
Can be used with pins 3,5,6,10 to adjust power output of Arduino board by sending out a PWM (Pulse Width Modulation) 
pin defines which pin to send power to
value determines how long a voltage of 5 Volts will be applied for and thus sending out a “voltage” in-between 0 and 5 volts
Voltage can only be 5 Volts (ON) or 0 Volts (OFF)
value is an integer between 0<x<255 where 255 means 100% duty cycle, 125 means 50% duty cycle, and 0 means 0% duty cycle
brightness = brightness + fadeAmount ; 
Sets value of brightness equal to itself plus fadeAmount
Loops back to have new value of brightness increase by fadeAmount again...and again
if (brightness == 0 || brightness == 255) { fadeAmount = -fadeAmount; }
If brightness equals 0 or 255, sets fadeAmount to the negative value of itself
This will cause the loop to reverse so the LED will increase/decrease in brightness 


