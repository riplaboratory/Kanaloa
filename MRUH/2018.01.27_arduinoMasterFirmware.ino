// Pin definitions
const byte Q1Pin = 6;     // output PWM pin to Q1 motor controller
const byte Q2Pin = 8;     // output PWM pin to Q2 motor controller
const byte Q3Pin = 9;     // output PWM pin to Q3 motor controller
const byte Q4Pin = 10;    // output PWM pin to Q4 motor controller
const byte gearPin = 12;   // int PWM pin from manual controller gear channel
const byte ch1Pin = 2;    // input PWM pin from manual controller receiver channel 1 (left stick up down)
const byte ch2Pin = 3;    // input PWM pin from manual controller receiver channel 2 (left stick left right)
const byte ch3Pin = 4;    // input PWM pin from manual controller receiver channel 3 (right stick left right)
const byte relayPin = 30; // output digital pin to kill switch relay

// Code vars
int gearPulseWidth = 0;
int ch1PulseWidth = 0;
int ch2PulseWidth = 0;
int ch3PulseWidth = 0;
int ch1Decoded = 0;
int ch2Decoded = 0;
int ch3Decoded = 0;
int throttleQ1 = 0;
int throttleQ2 = 0;
int throttleQ3 = 0;
int throttleQ4 = 0;
int strafeQ1 = 0;
int strafeQ2 = 0;
int strafeQ3 = 0;
int strafeQ4 = 0;
int rotateQ1 = 0;
int rotateQ2 = 0;
int rotateQ3 = 0;
int rotateQ4 = 0;
int Q1Decoded = 0;
int Q2Decoded = 0;
int Q3Decoded = 0;
int Q4Decoded = 0;
int Q1bit = 0;
int Q2bit = 0;
int Q3bit = 0;
int Q4bit = 0;

void setup() {

  pinMode(gearPin,INPUT_PULLUP);
  pinMode(ch1Pin,INPUT_PULLUP);
  pinMode(ch2Pin,INPUT_PULLUP);
  pinMode(ch3Pin,INPUT_PULLUP);
  pinMode(relayPin,OUTPUT);

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

  gearPulseWidth = pulseIn(gearPin,HIGH);   // gear (kill)
  ch1PulseWidth = pulseIn(ch1Pin,HIGH);     // throttle
  ch2PulseWidth = pulseIn(ch2Pin,HIGH);     // strafe
  ch3PulseWidth = pulseIn(ch3Pin,HIGH);     // rotate

  // unkill
  if (gearPulseWidth > 1080 && gearPulseWidth < 1100) { 

    digitalWrite(relayPin,HIGH);
    
    // Ch1: 965 (down) to 1991 (up)
    // Ch2: 1991 (left) to 965 (right)
    // Ch3: 2007 (left) to 973 (right)
  
    ch1Decoded = map(ch1PulseWidth,965,1991,-100,100);
    ch2Decoded = map(ch2PulseWidth,1991,965,-100,100);
    ch3Decoded = map(ch3PulseWidth,2007,973,-100,100);
  
    throttleQ1 = ch1Decoded;      // forward
    throttleQ2 = ch1Decoded;
    throttleQ3 = ch1Decoded;
    throttleQ4 = ch1Decoded;
    strafeQ1 = -ch2Decoded;       // right strafe
    strafeQ2 = ch2Decoded;
    strafeQ3 = -ch2Decoded;
    strafeQ4 = ch2Decoded;
    rotateQ1 = -ch3Decoded;       // right (clockwise) rotation
    rotateQ2 = ch3Decoded;
    rotateQ3 = ch3Decoded;
    rotateQ4 = -ch3Decoded;
  
    Q1Decoded = throttleQ1 + strafeQ1 + rotateQ1;
    Q2Decoded = throttleQ2 + strafeQ2 + rotateQ2;
    Q3Decoded = throttleQ3 + strafeQ3 + rotateQ3;
    Q4Decoded = throttleQ4 + strafeQ4 + rotateQ4;
  
    if (Q1Decoded > 100) {
      Q1Decoded = 100;
    }
    if (Q1Decoded < -100) {
      Q1Decoded = -100;
    }
    if (Q2Decoded > 100) {
      Q2Decoded = 100;
    }
    if (Q2Decoded < -100) {
      Q2Decoded = -100;
    }
    if (Q3Decoded > 100) {
      Q3Decoded = 100;
    }
    if (Q3Decoded < -100) {
      Q3Decoded = -100;
    }
    if (Q4Decoded > 100) {
      Q4Decoded = 100;
    }
    if (Q4Decoded < -100) {
      Q4Decoded = -100;
    }
  
    // reverse = 31
    // neutral = 47
    // forward = 63
  
    Q1bit = map(Q1Decoded,-100,100,31,63);
    Q2bit = map(Q2Decoded,-100,100,31,63);
    Q3bit = map(Q3Decoded,-100,100,31,63);
    Q4bit = map(Q4Decoded,-100,100,31,63);
    
    analogWrite(Q1Pin,Q1bit);
    analogWrite(Q2Pin,Q2bit);
    analogWrite(Q3Pin,Q3bit);
    analogWrite(Q4Pin,Q4bit);

  }

  // KILL
  else {
    digitalWrite(relayPin,LOW);
  }
}
