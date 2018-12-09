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
 * - Remote Killswitch PWM:       Switch DOWN = 1032, Switch MID = 1532, Switch UP = 2068
 * 
 * Created by: Kai Jones
 * Revisions by: Jordan Romanelli and Kai Jones
 * Date: 2018.12.08
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
const int ch8Pin = 10;              // PWM coming in from channel 8 of receiver
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
int ch8PulseLength = 0;             // Pulse length of the PWM signal coming from channel 8 of the receiver.
int tolerance = 50;                 // Tolerance for PWM signal 

//---------------
// SETUP FUNCTION
//---------------
void setup() {
  pinMode(physicalKill1Pin,INPUT);          // Reads whether physical killswitch 1 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(physicalKill2Pin,INPUT);          // Reads whether physical killswitch 2 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(safetyRelayPin,OUTPUT);           // Arduino sends a HIGH/LOW signal to relay depending on killswitches's states
  pinMode(stateInPin,INPUT_PULLUP);         // digital in from mainMega for manual mode state (HIGH for autonomous, LOW for manual)
  pinMode(ch8Pin, INPUT_PULLUP);            // PWM coming in from channel 8 of receiver
  pinMode(remoteKillOutPin,OUTPUT);         // digital out to mainMega for kill state (HIGH for kill, LOW for UNKILL)
  pinMode(redLightPin,OUTPUT);              // Arduino sends signal out to red safety light
  pinMode(greenLightPin,OUTPUT);            // Arduino sends HIGH/LOW signal to green safety light
  pinMode(blueLightPin, OUTPUT);            // Arduino sends HIGH/LOW signal to blue safety light

  Serial.begin(57600);

}

//--------------
// LOOP FUNCTION
//--------------
void loop() {

  // Read PWM coming in from channel 8 of receiver and set remote kill state 
  ch8PulseLength = pulseIn(ch8Pin, HIGH, 25000); 

  // Determine kill switch (ch8) state
  if(ch8PulseLength + tolerance > 2068 && ch8PulseLength - tolerance < 2068){
    // SD switch on transmitter is in the up position (unkill)
    remoteKillStatus = false;
    digitalWrite(remoteKillOutPin, LOW); 
  }
  else if (ch8PulseLength + tolerance > 1532 && ch8PulseLength - tolerance < 1532) {
    // SD switch on transmitter is in the middle position (kill)
    remoteKillStatus = true; 
    digitalWrite(remoteKillOutPin, HIGH); 
  }
  else if (ch8PulseLength + tolerance > 1032 && ch8PulseLength - tolerance < 1032) {
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
  ledController(killStatus, stateStatus);
  
  delay(100);  // Delay is for data to settle (Wonky without delay)
  
}


//------------
//  FUNCTIONS
//------------
bool ledController(bool killStatus, bool manualControlSignal){
  if(killStatus){
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, HIGH);
    digitalWrite(blueLightPin, HIGH);
  }
  else if(!killStatus && manualControlSignal){
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, LOW);
    digitalWrite(blueLightPin, HIGH);
  }
  else if(!killStatus && !manualControlSignal){
    digitalWrite(redLightPin, HIGH);
    digitalWrite(greenLightPin, HIGH);
    digitalWrite(blueLightPin, LOW);
  }
  else{
    digitalWrite(redLightPin, LOW);
    digitalWrite(greenLightPin, LOW);
    digitalWrite(blueLightPin, LOW);
  }
}
