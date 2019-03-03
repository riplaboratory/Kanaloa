// MRUH main arduino code
// Last update: 2019.03.03

// Pin definitions
const byte ch1Pin = 2;        // input PWM pin from manual controller receiver channel 1 (left stick up-down)
const byte ch2Pin = 3;        // input PWM pin from manual controller receiver channel 2 (left stick left-right)
const byte ch3Pin = 4;        // input PWM pin from manual controller receiver channel 4 (right stick left-right)
const byte ch7Pin = 12;       // input PWM pin from manual controller gear channel 7 (remote kill switch)
const byte physKillPin = A10; // input analog pin from physical kill switch (this should really be a digital pin, but whatever for now)
const byte killRelayPin = 30; // output digital pin to kill switch relay
const byte q1Pin = 6;         // output PWM pin to Q1 motor controller
const byte q2Pin = 8;         // output PWM pin to Q2 motor controller
const byte q3Pin = 9;         // output PWM pin to Q3 motor controller
const byte q4Pin = 10;        // output PWM pin to Q4 motor controller

// Global variables
int ch1PulseWidth = 0;
int ch2PulseWidth = 0;
int ch3PulseWidth = 0;
int ch7PulseWidth = 0;
int physKill = 0;
int killState = 1;
int ch1Map = 0;
int ch2Map = 0;
int ch3Map = 0;
int surgeQ1 = 0;
int surgeQ2 = 0;
int surgeQ3 = 0;
int surgeQ4 = 0;
int swayQ1 = 0;
int swayQ2 = 0;
int swayQ3 = 0;
int swayQ4 = 0;
int yawQ1 = 0;
int yawQ2 = 0;
int yawQ3 = 0;
int yawQ4 = 0;
int q1Map = 0;
int q2Map = 0;
int q3Map = 0;
int q4Map = 0;
int q1Bit = 0;
int q2Bit = 0;
int q3Bit = 0;
int q4Bit = 0;

void setup() {

  // Set pin modes
  pinMode(ch1Pin,INPUT_PULLUP);
  pinMode(ch2Pin,INPUT_PULLUP);
  pinMode(ch3Pin,INPUT_PULLUP);
  pinMode(ch7Pin,INPUT_PULLUP);
  pinMode(physKillPin,INPUT);     // this pin is physically pulled down to ground in box
  pinMode(killRelayPin,OUTPUT);
  pinMode(q1Pin,OUTPUT);
  pinMode(q2Pin,OUTPUT);
  pinMode(q3Pin,OUTPUT);
  pinMode(q4Pin,OUTPUT);
  
  // Set kill relay HIGH to KILL
  digitalWrite(killRelayPin,HIGH);
  
  // Serial setup
  Serial.begin(9600);

  // Handles Arduino timers for PWM frequency (for more info: https://playground.arduino.cc/Main/TimerPWMCheatsheet and http://forum.arduino.cc/index.php/topic,72092.0.html)
  int myEraser = 7;         // this is 111 in binary, and used as an eraser
  TCCR2B &= ~myEraser;      // sets three bits in TCCR2B (timer 2) to zero
  TCCR4B &= ~myEraser;      // sets three bits in TCCR4B (timer 4) to zero
  TCCR4B |= 4;              // Set timer 4 (pins 6 and 8) to prescaler 4 (120 Hz).
  TCCR2B |= 6;              // Set timer 2 (pins 10 and 9) to prescaler 6 (120 Hz).

}

void loop() {

  // Read all inputs
  ch1PulseWidth = pulseIn(ch1Pin,HIGH);     // ch 1 (surge joystick channel)
  ch2PulseWidth = pulseIn(ch2Pin,HIGH);     // ch 2 (sway joystick channel)
  ch3PulseWidth = pulseIn(ch3Pin,HIGH);     // ch 3 (yaw joystick channel)
  ch7PulseWidth = pulseIn(ch7Pin,HIGH);     // ch 7 (remote kill switch channel)
  physKill = digitalRead(physKillPin);      // physical kill switch channel

  // Local variables describing pulse information
  int pulseMax = 1988;        // maximum pulseWidth [us] (it won't be EXACTLY 2000 us because of imperfections in the signal...should measure this)
  int pulseMin = 993;         // minimum pulseWidth [us] (it won't be EXACTLY 1000 us because of imperfections in the signal...should measure this)
  int pulseTolerance = 150;   // tolerance to pulseWidth logic [us]
  int pulseDeadzone = 100;    // deadzone around center of joystick throw [us] (increase for more deazone, decrease for less)

  // Check kill state based on kill switches
  if (physKill == HIGH && ch7PulseWidth > (pulseMax - pulseTolerance)) {
    killState = 0;
  }
  else {
    killState = 1;
  }

  // This code executes when system is killed
  if (killState == 1) {
    
    // Set kill relay HIGH to KILL
    digitalWrite(killRelayPin,HIGH);
    
  }

  // This code executes when system is unkilled
  else if (killState == 0) {
    
    // Set kill relay to LOW to UNKILL
    digitalWrite(killRelayPin,LOW);

    // Remove set joystick inputs to neutral if within joystick deadzone
    int pulseNeutral = pulseMin+round((pulseMax-pulseMin)/2);   // calculate neutral pulse width
    if (ch1PulseWidth < pulseNeutral+round(pulseDeadzone/2) && ch1PulseWidth > pulseNeutral-round(pulseDeadzone/2)) {
      ch1PulseWidth = pulseNeutral;
    }
    if (ch2PulseWidth < pulseNeutral+round(pulseDeadzone/2) && ch2PulseWidth > pulseNeutral-round(pulseDeadzone/2)) {
      ch2PulseWidth = pulseNeutral;
    }
    if (ch3PulseWidth < pulseNeutral+round(pulseDeadzone/2) && ch3PulseWidth > pulseNeutral-round(pulseDeadzone/2)) {
      ch3PulseWidth = pulseNeutral;
    }

    // Map joystick inputs from -100 to 100
    ch1Map = map(ch1PulseWidth,pulseMin,pulseMax,-100,100);
    ch2Map = map(ch2PulseWidth,pulseMin,pulseMax,-100,100);
    ch3Map = map(ch3PulseWidth,pulseMin,pulseMax,-100,100);

    // Calculate surge, sway, and yaw components
    surgeQ1 = ch1Map;   // surge (forward positive)
    surgeQ2 = ch1Map;
    surgeQ3 = ch1Map;
    surgeQ4 = ch1Map;
    swayQ1 = -ch2Map;   // sway (right positive)
    swayQ2 = ch2Map;
    swayQ3 = -ch2Map;
    swayQ4 = ch2Map;
    yawQ1 = -ch3Map;    // yaw (CLOCKWISE positive)
    yawQ2 = ch3Map;
    yawQ3 = ch3Map;
    yawQ4 = -ch3Map;

    // Map thruster components from -100 to 100
    q1Map = surgeQ1 + swayQ1 + yawQ1;
    q2Map = surgeQ2 + swayQ2 + yawQ2;
    q3Map = surgeQ3 + swayQ3 + yawQ3;
    q4Map = surgeQ4 + swayQ4 + yawQ4;
  
    if (q1Map > 100) {
      q1Map = 100;
    }
    if (q1Map < -100) {
      q1Map = -100;
    }
    if (q2Map > 100) {
      q2Map = 100;
    }
    if (q2Map < -100) {
      q2Map = -100;
    }
    if (q3Map > 100) {
      q3Map = 100;
    }
    if (q3Map < -100) {
      q3Map = -100;
    }
    if (q4Map > 100) {
      q4Map = 100;
    }
    if (q4Map < -100) {
      q4Map = -100;
    }

    // Map thsuter components to proper pulse width
    // reverse = 31, neutral = 47, forward = 63  
    q1Bit = map(q1Map,-100,100,31,63);
    q2Bit = map(q2Map,-100,100,31,63);
    q3Bit = map(q3Map,-100,100,31,63);
    q4Bit = map(q4Map,-100,100,31,63);

    // Write to motor controller pins
    analogWrite(q1Pin,q1Bit);
    analogWrite(q2Pin,q2Bit);
    analogWrite(q3Pin,q3Bit);
    analogWrite(q4Pin,q4Bit);
    
  }

  // This code executes when a fault occurs in the kill state detection
  else {
    
    // Error in kill state calculation, KILL
    digitalWrite(killRelayPin,HIGH);
    Serial.println("Error in kill state calculation, KILLING");
    
  }
}
