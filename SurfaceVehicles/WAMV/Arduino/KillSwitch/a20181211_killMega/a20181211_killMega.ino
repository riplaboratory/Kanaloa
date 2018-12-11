/*
 * This code is used to run the killswitch system. The killswitch Arduino is located in the low current box and receives power from its own power source. The physical
 * Arduino monitors the kill state of the WAM-V by reading if the physical killswitches are KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) and if the PWM signal coming 
 * from the remote killswitch receiver is sending the kill signal. The killswitch Arduino also reads the WAM-V's manual or autonomous state by receving a HIGH or LOW 
 * signal respectively. If the system is killed, the killswitch Arduino sends a kill signal to the high current relays located in the high current box and changes the 
 * visual feedback light to RED. If the system is unkilled, the Arduino changes the visual feedback light to YELLOW (manual) or BLUE (autonomous). 
 * 
 * NOTES:
 * - Physical Killswitches:       HIGH = UNKILLED, LOW = KILLED
 * - Visual Feedback Light Pins:  HIGH = Light is OFF, LOW = Light is ON 
 * - Remote Kill switch PWM:       Switch DOWN = 1032, Switch MID = 1532, Switch UP = 2068
 * 
 * !!!IMPORTANT!!! 
 * There are two kill switches on each side of the WAM-V. The two switches on each side are wired in series with each other. In other words, the two switches on each
 * side of the WAM-V depend on each other to be KILLED or UNKILLED. This is just a temporary fix for the 2018 RobotX Maritime Challenge. Eventually, we will want to  
 * isolate each physical kill switch in the kill switch system. 
 * 
 * Created by: Kai Jones
 * Revisions by: Jordan Romanelli and Kai Jones
 * Date: 2018.12.09
 * 
 */

//-----------------
// Pin Definitions
//-----------------
const int physicalKill1Pin = 2;     // Physical kill switch number 1 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) 
const int physicalKill2Pin = 3;     // Physical kill switch number 2 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH)  
const int safetyRelayPin = 6;       // Pin which Arduino sends a HIGH/LOW signal to relay
const int stateInPin = 9;           // digital in from mainMega for manual mode state (LOW for autonomous, HIGH for manual)
const int remoteKillOutPin = 8;     // digital out to mainMega for kill state (HIGH for kill, LOW for UNKILL)
const int ch8Pin = 19;              // PWM coming in from channel 8 of receiver
const int blueLightPin = 11;        // Blue LED light on safety pole (HIGH to turn light OFF, LOW to turn light ON)
const int greenLightPin = 12;       // Pin which Arduino sends a HIGH/LOW signal to relay for green safety light
const int redLightPin = 13;         // Red LED light on safety pole (HIGH to turn light OFF, LOW to turn light ON)

//------------------
// System Variables
//------------------
bool physicalKill1Status = true;    // physical kill 1 status variable. true means KILL is detected, false means UNKILL is detected.
bool physicalKill2Status = true;    // physical kill 2 status variable. true means KILL is detected, false means UNKILL is detected.
bool remoteKillStatus = true;       // digital out to mainMega for kill state (HIGH for kill, LOW for UNKILL)
bool stateStatus = true;            // digital in from mainMega for manual mode state (LOW for autonomous, HIGH for manual)
bool killStatus = true;             // overall kill status based on kill switches (HIGH for kill, LOW for UNKILL)
int tolerance = 200;                // Tolerance for PWM signal 
unsigned long timeNow = 0;          // time tracking variable
unsigned long timeLast = 0;         // time tracking variable
volatile int ch8PulseWidth = 0;     // channel 8 pulse width (for interrerupt service routine)
volatile int ch8Timer = 0;          // channel 8 pulse width timer (for interrerupt service routine)
volatile int ch8GoodReading = 0;    // good reading from channel 8

const int killCount = 10;
int counter = 0;

//---------------
// SETUP FUNCTION
//---------------
void setup() {
  pinMode(physicalKill1Pin,INPUT);          // Reads whether physical killswitch 1 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(physicalKill2Pin,INPUT);          // Reads whether physical killswitch 2 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(safetyRelayPin,OUTPUT);           // Arduino sends a HIGH/LOW signal to relay depending on killswitches's states
  pinMode(stateInPin,INPUT_PULLUP);         // digital in from mainMega for manual mode state (HIGH for autonomous, LOW for manual)
  pinMode(ch8Pin,INPUT);                    // PWM coming in from channel 8 of receiver
  pinMode(remoteKillOutPin,OUTPUT);         // digital out to mainMega for kill state (HIGH for kill, LOW for UNKILL)
  pinMode(redLightPin,OUTPUT);              // Arduino sends signal out to red safety light
  pinMode(greenLightPin,OUTPUT);            // Arduino sends HIGH/LOW signal to green safety light
  pinMode(blueLightPin, OUTPUT);            // Arduino sends HIGH/LOW signal to blue safety light

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(ch8Pin),ch8Fall,FALLING);

  Serial.begin(57600);

}

//--------------
// LOOP FUNCTION
//--------------
void loop() {

//  int temp = 0;
//  while (true) {
//    temp = pulseIn(ch8Pin,HIGH);
//    Serial.println(temp);
//    if (temp > 750 && temp < 2250) {
//      Serial.print("^^^ is a good value, breaking out");
//      Serial.println(" ");
//      break;
//    }
//  } 
//  if (temp > 1350) {
//    // remote unkill
//    remoteKillStatus = false;
//    digitalWrite(remoteKillOutPin, LOW); 
//    Serial.println("stayin alive, resetting counter");
//    counter = 0;
//  }
//  else {
//    counter = counter + 1;
//    Serial.println(counter);
//    if (counter >= killCount) {
//      // remote kill
//      remoteKillStatus = true;
//      Serial.println("killin");
//      digitalWrite(remoteKillOutPin, HIGH);
//      counter = 0;
//    }
//  }

  int temp = 0;
  temp = pulseIn(ch8Pin,HIGH);
  
  // Determine kill switch (ch8) state
  if(temp + tolerance > 1000 && temp - tolerance < 1000){
    // SD switch on transmitter is in the up position (unkill)
    remoteKillStatus = false;
    digitalWrite(remoteKillOutPin, LOW); 
  }
  else if (temp + tolerance > 1500 && temp - tolerance < 1500) {
    // SD switch on transmitter is in the middle position (kill)
    remoteKillStatus = true; 
    digitalWrite(remoteKillOutPin, HIGH); 
  }
  else if (temp + tolerance > 2000 && temp - tolerance < 2000) {
    // SD switch on transmitter is in the down position (kill)
    remoteKillStatus = true; 
    digitalWrite(remoteKillOutPin, HIGH);
  }
  else {
    // Invalid input from ch8
    Serial.println("Remote kill swich (channel 8) reporting invalid input; check transmitter connection. KILLING...");   
    remoteKillStatus = true; 
    digitalWrite(remoteKillOutPin, HIGH); 
  }

  // Read states
  physicalKill1Status = digitalRead(physicalKill1Pin);
  physicalKill2Status = digitalRead(physicalKill2Pin);
  stateStatus = digitalRead(stateInPin);

  // If statement to kill the system
  if (physicalKill1Status == HIGH && physicalKill2Status == HIGH && remoteKillStatus == false) {   // All switches are unkilled
    killStatus = false;    
    digitalWrite(safetyRelayPin, LOW);
  }
  else {
    killStatus = true;
    digitalWrite(safetyRelayPin, HIGH);
  }
    
  // Control LED lights
  ledController(killStatus,stateStatus);
  
  delay(50);  // Delay is for data to settle (Wonky without delay)
  
}

//------------
//  FUNCTIONS
//------------

bool ledController(bool killStatus, bool manualControlSignal) {
  
  if(killStatus) {
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, HIGH);
    digitalWrite(blueLightPin, HIGH);
//    Serial.println("red");
  }
  
  else if(!killStatus && manualControlSignal) {
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, LOW);
    digitalWrite(blueLightPin, HIGH);
//    Serial.println("yellow");
  }
  
  else if(!killStatus && !manualControlSignal) {
    digitalWrite(redLightPin, HIGH);
    digitalWrite(greenLightPin, HIGH);
    digitalWrite(blueLightPin, LOW);
//    Serial.println("blue");
  }
  else {
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, LOW);
    digitalWrite(blueLightPin, LOW);
  }
}
