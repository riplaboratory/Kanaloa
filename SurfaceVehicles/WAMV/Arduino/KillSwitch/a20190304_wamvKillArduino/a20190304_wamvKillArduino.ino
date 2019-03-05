
/*
   This code is used to run the killswitch system. The killswitch Arduino is located in the low current box and receives power from its own power source. The physical
   Arduino monitors the kill state of the WAM-V by reading if the physical killswitches are KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) and if the PWM signal coming
   from the remote killswitch receiver is sending the kill signal. The killswitch Arduino also reads the WAM-V's manual or autonomous state by receving a HIGH or LOW
   signal respectively. If the system is killed, the killswitch Arduino sends a kill signal to the high current relays located in the high current box and changes the
   visual feedback light to RED. If the system is unkilled, the Arduino changes the visual feedback light to YELLOW (manual) or BLUE (autonomous).

   NOTES:
   - Physical Killswitches:       HIGH = UNKILLED, LOW = KILLED
   - Visual Feedback Light Pins:  HIGH = Light is OFF, LOW = Light is ON
   - Remote Kill switch PWM:       Switch DOWN = 1032, Switch MID = 1532, Switch UP = 2068

   !!!IMPORTANT!!!
   There are two kill switches on each side of the WAM-V. The two switches on each side are wired in series with each other. In other words, the two switches on each
   side of the WAM-V depend on each other to be KILLED or UNKILLED. This is just a temporary fix for the 2018 RobotX Maritime Challenge. Eventually, we will want to
   isolate each physical kill switch in the kill switch system.

   Created by: Kai Jones
   Revisions by: Jordan Romanelli and Kai Jones
   Date: 2018.12.09

*/

// Pin definitions
const byte ch5Pin = 2;             // PWM in from receiver channel 5 (auto/manual mode switch)
const byte ch7Pin = 18;            // PWM in from receiver channel 7 (reverse contactor kill switch)
const byte ch8Pin = 19;            // PWM in from receiver channel 8 (remote kill switch)
const byte physicalKillPin = 3;    // digital in from physical kill switch
const byte killRelayPin = 6;       // digital out to kill relay
const byte modeCommPin = 7;        // digital out to main arduino communicating auto/manual mode
const byte revConKillCommPin = 8;  // digital out to main arduino communicating reverse contactor kill stats
const byte killCommPin = 9;        // digital out to main arduino communicating kill state
const byte blueLightPin = 11;      // digital out to blue light relay
const byte greenLightPin = 12;     // digital out to green light relay
const byte redLightPin = 13;       // digital out to red light relay

// Global variables
int physicalKillStatus = 1;     // physical kill status variable
int remoteKillStatus = 1;       // remote kill status variable
int killStatus = 1;             // overall kill status based on kill switches (HIGH for kill, LOW for UNKILL)
int revConKillStatus = 1;       // reverse contactor kill status variable
int modeStatus = 1;             // auto/manual mode status variable
volatile int ch5PulseWidth = 0; // channel 5 pulse width (for interrerupt service routine)
volatile int ch5Timer = 0;      // channel 5 pulse width timer (for interrerupt service routine)
volatile int ch7PulseWidth = 0; // channel 5 pulse width (for interrerupt service routine)
volatile int ch7Timer = 0;      // channel 5 pulse width timer (for interrerupt service routine)
volatile int ch8PulseWidth = 0; // channel 5 pulse width (for interrerupt service routine)
volatile int ch8Timer = 0;      // channel 5 pulse width timer (for interrerupt service routine)

void setup() {

  // Set pin mode
  pinMode(ch5Pin,INPUT);
  pinMode(ch7Pin,INPUT);
  pinMode(ch8Pin,INPUT);
  pinMode(physicalKillPin,INPUT);
  pinMode(killRelayPin,OUTPUT);
  pinMode(modeCommPin,OUTPUT);
  pinMode(revConKillCommPin,OUTPUT);
  pinMode(killCommPin,OUTPUT);
  pinMode(blueLightPin,OUTPUT);
  pinMode(greenLightPin,OUTPUT);
  pinMode(redLightPin,OUTPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ch5Pin), ch5Rise, RISING);
  attachInterrupt(digitalPinToInterrupt(ch5Pin), ch5Fall, FALLING);
  attachInterrupt(digitalPinToInterrupt(ch7Pin), ch7Rise, RISING);
  attachInterrupt(digitalPinToInterrupt(ch7Pin), ch7Fall, FALLING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin), ch8Rise, RISING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin), ch8Fall, FALLING);

  // Set buad rate
  Serial.begin(9600);

}

void loop() {

  // Grab ch5, ch7, and ch8 from interrupt global variables
  int ch5Pulse = ch5PulseWidth;   // mode switch
  int ch7Pulse = ch7PulseWidth;   // reverse contactor kill switch
  int ch8Pulse = ch8PulseWidth;   // remote kill switch

  // Pulse width variables
  int ch5PulseMin = 1170;
  int ch5PulseMax = 2150;
  int ch5PulseNeutral = round((ch5PulseMax - ch5PulseMin) / 2) + ch5PulseMin;
  int ch7PulseMin = 1140;
  int ch7PulseMax = 2140;
  int ch7PulseNeutral = round((ch7PulseMax - ch7PulseMin) / 2) + ch7PulseMin;
  int ch8PulseMin = 1250;
  int ch8PulseMax = 2250;
  int ch8PulseNeutral = round((ch8PulseMax - ch8PulseMin) / 2) + ch8PulseMin;
  int tolerance = 100;

  // Determine kill switch (ch8) state
  if (ch8Pulse + tolerance > ch8PulseMin && ch8Pulse - tolerance < ch8PulseMin) {

    // Switch on transmitter is in the up position (unkill)
    remoteKillStatus = 0;

  }
  else if (ch8Pulse + tolerance > ch8PulseNeutral && ch8Pulse - tolerance < ch8PulseNeutral) {

    // Switch on transmitter is in the middle position (kill)
    remoteKillStatus = 1;

  }
  else if (ch8Pulse + tolerance > ch8PulseMax && ch8Pulse - tolerance < ch8PulseMax) {

    // Switch on transmitter is in the down position (kill)
    remoteKillStatus = 1;

  }
  else {

    // Invalid input from ch8
    Serial.println("Remote kill swich (channel 8) reporting invalid input; check transmitter connection. KILLING...");
    remoteKillStatus = 2;

  }

  // Determine reverse contactor kill switch (ch7) state
  if (ch7Pulse + tolerance > ch7PulseMin && ch7Pulse - tolerance < ch7PulseMin) {

    // Switch on transmitter is in the up position (unkill)
    revConKillStatus = 0;

  }
  else if (ch7Pulse + tolerance > ch7PulseNeutral && ch7Pulse - tolerance < ch7PulseNeutral) {

    // Switch on transmitter is in the middle position (kill)
    revConKillStatus = 1;

  }
  else if (ch7Pulse + tolerance > ch7PulseMax && ch7Pulse - tolerance < ch7PulseMax) {

    // Switch on transmitter is in the down position (kill)
    revConKillStatus = 1;

  }
  else {

    // Invalid input from ch8
    Serial.println("Remote reverse contactor kill swich (channel 7) reporting invalid input; check transmitter connection. KILLING...");
    revConKillStatus = 2;

  }

  // Determine auto/manual mode switch state
  if (ch5Pulse + tolerance > ch5PulseMax && ch5Pulse - tolerance < ch5PulseMax) {

    // Switch on transmitter is in the up position (auto)
    modeStatus = 0;
    
  }
  else if (ch5Pulse + tolerance > ch5PulseMin && ch5Pulse - tolerance < ch5PulseMin) {

    // Switch on transmitter is in the down position (manual)
    modeStatus = 1;
    
  }
  else {

    // Invalid input from ch5
    Serial.println("Remote mode swich (channel 5) reporting invalid input; check transmitter connection. Setting to MANUAL...");
    modeStatus = 1;

  }

  // Read state of physical kill switches
  if (digitalRead(physicalKillPin) == HIGH) {

    // Physical kill switch is unpressed (unkill)
    physicalKillStatus = 0;
    
  }
  else {

    // Physical kill switch is pressed (kill)
    physicalKillStatus = 1;
    
  }

  // Full statement to kill or unkill system
  if (physicalKillStatus == 0 && remoteKillStatus == 0) {

    killStatus = 0;
    digitalWrite(killRelayPin,LOW);   // pull LOW to unkill
    
  }
  else {

    killStatus = 1;
    digitalWrite(killRelayPin,HIGH);   // pull HIGH to unkill
    
  }

  // Communicate stuff to main arduino
  if (killStatus == 0) {
    digitalWrite(killCommPin,LOW);        // pull low to indicate unkill
  }
  else {
    digitalWrite(killCommPin,HIGH);       // pull high to indicate kill
  }
  if (revConKillStatus == 0) {
    digitalWrite(revConKillCommPin,LOW);  // pull low to indicate unkill
  }
  else {
    digitalWrite(revConKillCommPin,HIGH); // pull high to indicate kill
  }
  if (modeStatus == 0) {
    digitalWrite(modeCommPin,LOW);        // pull low to indicate autonomous
  }
  else {
    digitalWrite(modeCommPin,HIGH);        // pull high to indicate manual
  }

  // Control light
  ledController(killStatus, modeStatus);

}

void ch5Rise() {
  attachInterrupt(digitalPinToInterrupt(ch5Pin), ch5Fall, FALLING);
  ch5Timer = micros();
}

void ch5Fall() {
  attachInterrupt(digitalPinToInterrupt(ch5Pin), ch5Rise, RISING);
  ch5PulseWidth = micros() - ch5Timer;
}

void ch7Rise() {
  attachInterrupt(digitalPinToInterrupt(ch7Pin), ch7Fall, FALLING);
  ch7Timer = micros();
}

void ch7Fall() {
  attachInterrupt(digitalPinToInterrupt(ch7Pin), ch7Rise, RISING);
  ch7PulseWidth = micros() - ch7Timer;
}

void ch8Rise() {
  attachInterrupt(digitalPinToInterrupt(ch8Pin), ch8Fall, FALLING);
  ch8Timer = micros();
}

void ch8Fall() {
  attachInterrupt(digitalPinToInterrupt(ch8Pin), ch8Rise, RISING);
  ch8PulseWidth = micros() - ch8Timer;
}

bool ledController(bool killStatus, bool manualControlSignal) {

  if (killStatus) {
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, HIGH);
    digitalWrite(blueLightPin, HIGH);
//    Serial.println("RED");
  }

  else if (!killStatus && manualControlSignal) {
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, LOW);
    digitalWrite(blueLightPin, HIGH);
//    Serial.println("YELLOW");
  }

  else if (!killStatus && !manualControlSignal) {
    digitalWrite(redLightPin, HIGH);
    digitalWrite(greenLightPin, HIGH);
    digitalWrite(blueLightPin, LOW);
//    Serial.println("BLUE");
  }
  else {
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, LOW);
    digitalWrite(blueLightPin, LOW);
//    Serial.println("WHITE, ERROR");
  }
}
