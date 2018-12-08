/*
 * This code is used to run the killswitch system. The Arduino is powered by the main power supply in the high current box going through a 12 V regulator. 
 * 5 V goes out to the two physical kill switches which loops back into two pins on the Arduino. When the switches are KILLED, the circuit breaks and the Arduino 
 * reads the pin as LOW. If the switches are UNKILLED, the Arduino reads the pin as HIGH. For the remote killswitch, the Arduino interprets the PWM signal coming 
 * from the receiver to infer the remote killswitch state. If any of these switches are KILLED, a software switch is KILLED, and the system is KILLED. 
 * 
 * Created by: Kai Jones
 * Revisions by: Jordan Romanelli and Kai Jones
 * Date: 2018.12.07
 * 
 * Update Notes:
 * - Changed code so that Arduino reads PWM signal coming straight from receiver
 * 
 */

//-----------------
// Pin Definitions
//-----------------
const int physicalKill1Pin = 2;     // Physical kill switch number 1 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) 
const int physicalKill2Pin = 3;     // Physical kill switch number 2 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH)  
const int safetyRelayPin = 6;       // Pin which Arduino sends a HIGH/LOW signal to relay
const int ch8Pin = 8;               // PWM signal coming in from receiver
const int blueLightPin = 11;        // Blue LED light on safety pole (HIGH to turn light OFF, LOW to turn light ON)
const int greenLightPin = 12;       // Pin which Arduino sends a HIGH/LOW signal to relay for green safety light
const int redLightPin = 13;         // Red LED light on safety pole (HIGH to turn light OFF, LOW to turn light ON)

//------------------
// System Variables
//------------------
bool physicalKill1Status = true;    // physical kill 1 status variable. true means KILL is detected, false means UNKILL is detected.
bool physicalKill2Status = true;    // physical kill 2 status variable. true means KILL is detected, false means UNKILL is detected.
bool remoteKillStatus = true;       // remote kill status variable. true means KILL is detected, false means UNKILL is detected. 
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
  pinMode(ch8Pin,INPUT_PULLUP);             // PWM signal coming in from ch8 of receiver
  pinMode(redLightPin,OUTPUT);              // Arduino sends signal out to red safety light
  pinMode(greenLightPin,OUTPUT);            // Arduino sends HIGH/LOW signal to green safety light
  pinMode(blueLightPin, OUTPUT);            // Arduino sends HIGH/LOW signal to blue safety light

  Serial.begin(57600);

}

//--------------
// LOOP FUNCTION
//--------------
void loop() {
  
  // Read incoming PWM signal from channel 8 
  ch8PulseLength = pulseIn(ch8Pin, HIGH, 25000);

  // Determine kill switch (ch8) state
  if(ch8PulseLength + tolerance > 2068 && ch8PulseLength - tolerance < 2068){
    // SD switch on transmitter is in the up position (unkill)
    remoteKillStatus = false;
  }
  else if (ch8PulseLength + tolerance > 1532 && ch8PulseLength - tolerance < 1532) {
    // SD switch on transmitter is in the middle position (kill)
    remoteKillStatus = true; 
  }
  else if (ch8PulseLength + tolerance > 1032 && ch8PulseLength - tolerance < 1032) {
    // SD switch on transmitter is in the down position (kill)
    remoteKillStatus = true; 
  }
  else {
    // Invalid input from ch8
    remoteKillStatus = true; 
    Serial.println("Remote kill swich (channel 8) reporting invalid input; check transmitter connection. KILLING...");   
  }

  // Read physical kill states
  physicalKill1Status = digitalRead(physicalKill1Pin);
  physicalKill2Status = digitalRead(physicalKill2Pin);
  
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
