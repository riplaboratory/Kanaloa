// Pin definitions
const byte Q1Pin = 6;     // output PWM pin to Q1 motor controller
const byte Q2Pin = 8;     // output PWM pin to Q2 motor controller
const byte Q3Pin = 9;     // output PWM pin to Q3 motor controller
const byte Q4Pin = 10;    // output PWM pin to Q4 motor controller
const byte ch1Pin = 2;    // input PWM pin from manual controller receiver channel 1 (left stick up down)
const byte ch2Pin = 3;    // input PWM pin from manual controller receiver channel 2 (right stick left right)
const byte ch4Pin = 4;    // input PWM pin from manual controller receiver channel 4 (left stick left right)
const byte ch7Pin = 12;   // int PWM pin from manual controller gear channel
const byte relayPin = 30; // output digital pin to kill switch relay

// Code vars
int ch1PulseWidth = 0;
int ch2PulseWidth = 0;
int ch4PulseWidth = 0;
int ch7PulseWidth = 0;
int ch1Decoded = 0;
int ch2Decoded = 0;
int ch4Decoded = 0;
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

  pinMode(ch1Pin,INPUT_PULLUP);
  pinMode(ch2Pin,INPUT_PULLUP);
  pinMode(ch4Pin,INPUT_PULLUP);
  pinMode(ch7Pin,INPUT_PULLUP);
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
  /***
  * Note: You NEED to have the 25000 wait time when using pulseIn to accurately read the PWM coming in. If the wait time is too low, the Arduino can't read the PWM signal consistently (it can't read the fastest PWM signal of 894 us properly). 
  * 
  * Max PWM signal: 1824
  * Mid/Neutral PWM signal: 1356-1362
  * Min PWM signal: 894
  *
  **/
    
  ch1PulseWidth = pulseIn(ch1Pin,HIGH, 25000 );    // throttle
  ch2PulseWidth = pulseIn(ch2Pin,HIGH, 25000);     // strafe
  ch4PulseWidth = pulseIn(ch4Pin,HIGH, 25000);     // rotate
  ch7PulseWidth = pulseIn(ch7Pin,HIGH, 25000);     // kill signal
  
  // unkill
  if (ch7PulseWidth > 700 && ch7PulseWidth < 1000) { 

    digitalWrite(relayPin,HIGH);
    
    // Ch1: 890 (down) to 1810 (up)
    // Ch2: 890 (left) to 1810 (right)
    // Ch4: 890 (left) to 1810 (right)
  
    ch1Decoded = map(ch1PulseWidth,890,1810,-100,100);
    ch2Decoded = map(ch2PulseWidth,890,1810,-100,100);
    ch4Decoded = map(ch4PulseWidth,890,1810,-100,100);
  
    throttleQ1 = ch1Decoded;      // forward
    throttleQ2 = ch1Decoded;
    throttleQ3 = ch1Decoded;
    throttleQ4 = ch1Decoded;
    rotateQ1 = -ch2Decoded;       // right (clockwise) rotation
    rotateQ2 = ch2Decoded;
    rotateQ3 = ch2Decoded;
    rotateQ4 = -ch2Decoded;
    strafeQ1 = ch4Decoded;       // right strafe
    strafeQ2 = -ch4Decoded;
    strafeQ3 = ch4Decoded;
    strafeQ4 = -ch4Decoded;
  
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
