/*
 * This code is used to run the killswitch system. The Arduino sends 5V out to the two physical kill switches which loops back into a pin on the Arduino. When the switches are 
 * KILLED, the circuit breaks and the Arduino reads the pin as LOW. If the switches are UNKILLED, the Arduino reads the pin as HIGH. For the remote killswitch, the Arduino
 * interprets the PWM signal coming from the receiver to infer the remote killswitch state. If any of these switches are KILLED, a software switch is KILLED, and the system 
 * is killed. 
 * 
 * Created by: Kai Jones
 * Revisions by: Brennan Yamamoto
 * Date: 2018.07.25
 * 
 * Update Notes:
 * - Included section of code to control red LED on safety light. 
 * - Verified in hardware.  Note that relay board is reverse logic, i.e. the board sits at 5V in normal state, and requires pull down to LOW on signal pin to change system out of normal state.  This sounds unsafe, but is OK, because if arduino dies, signal pin gets naturally pulled up to 5V and relay board reverts to normal (killed) state.  If 5V dies, relay board loses power altogether, and relay reverts to normal (killed) state.  
 */

//-----------------
// Pin Definitions
//-----------------
const int physicalKill1Pin = 2;     // Physical kill switch number 1 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) 
const int physicalKill2Pin = 3;     // Physical kill switch number 2 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH)
const int relayPin = 6;             // Pin which Arduino sends a HIGH/LOW signal to relay
const int ch8Pin = 8;               // PWM signal from the 8th channel of the R9 receiver in the wireless pole; this is the remote kill switch signal
const int redLightPin = 13;         // Red LED light on safety pole (HIGH to turn light ON, LOW to turn light OFF)

//-----------------
// String Variables
//-----------------
String switch1Statement;
String switch2Statement;
String remoteStatement;

//------------------
// System Variables
//------------------
bool kill = true;                   // master kill status variable.  true will KILL the system, false will UNKILL system.
bool physicalKill1Status = true;    // physical kill 1 status variable. true means KILL is detected, false means UNKILL is detected.
bool physicalKill2Status = true;    // physical kill 2 status variable. true means KILL is detected, false means UNKILL is detected.
bool ch8KillStatus = true;          // ch8 (remote) kill status variable. true means KILL is detected, false means UNKILL is detected.
bool physicalKill1PinState = LOW;
bool physicalKill2PinState = LOW;
int ch8PulseLength = 0;

//-------------
// VERBOSE MODE
//-------------
bool verbose_mode = false;          // setting verbose_mode to true will print debug information to serial monitor

//---------------
// SETUP FUNCTION
//---------------
void setup() {
  pinMode(physicalKill1Pin, INPUT);     // Reads whether physical killswitch 1 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(physicalKill2Pin, INPUT);     // Reads whether physical killswitch 2 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(relayPin, OUTPUT);            // Arduino sends a HIGH/LOW signal to relay depending on killswitches's states
  pinMode(ch8Pin, INPUT_PULLUP);        // Arduino reads incoming PWM signal for remote kill switch
  pinMode(redLightPin, OUTPUT);         // Arduino sends signal out to red safety light
  Serial.begin(9600);
}

//--------------
// LOOP FUNCTION
//--------------
void loop() {
  main_function();              // main_function() is always called in each loop (see farther below)
  if(verbose_mode == true) {    // Check to see if boolean variable called TEST_MODE is set to true
    verbose_function();         // If true, call "verbose_function()" (see farther down)
  }
}

//-----------
// Functions
//-----------

void main_function() {

  // Check pin states
  physicalKill1PinState = digitalRead(physicalKill1Pin);  // check physical kill 1 pin state
  physicalKill2PinState = digitalRead(physicalKill2Pin);  // check physical kill 2 pin state
  ch8PulseLength = pulseIn(ch8Pin,HIGH,250000);           // check ch8 from transmitter (remote kill channel), 0.25s timeout

  // Low-level logic
  if(physicalKill1PinState == HIGH) {
    physicalKill1Status = false;          // physical kill 1 is UNKILLED
  }
  else { 
    physicalKill1Status = true;           // physical kill 1 is KILLED
  }
  if(physicalKill2PinState == HIGH) {
    physicalKill2Status = false;          // physical kill 2 is UNKILLED
  }
  else { 
    physicalKill2Status = true;           // physical kill 2 is KILLED
  }
  if(ch8PulseLength > 1825 && ch8PulseLength < 1925) { 
    ch8KillStatus = false;                // ch8 (remote kill) is UNKILLED
  }
  else {
    ch8KillStatus = true;                 // ch8 (remote kill is KILLED
  }

  // Kill logic
  if(physicalKill1Status == false && physicalKill2Status == false && ch8KillStatus == false) {
    kill = false;       // all kill switches are UNKILLED, safe to set kill status to false
  }
  else { 
    kill = true;        // something is KILLED, set kill status to true
  }

  // Send kill or unkill signal
  // Note that relay board is reverse logic, so KILLING (normal state) requires that you pull the signal HIGH, and UNKILLING requires that you pull the signal LOW.  This seems unsafe, but is protexted by reverse logic.
   if(kill == true) {
    digitalWrite(relayPin,HIGH);        // KILLING system is desired, pull relay signal HIGH
    digitalWrite(redLightPin, LOW);     // Turn red saftey light ON to indicate system is KILLED
   }
   else{
    digitalWrite(relayPin,LOW);         // UNKILLING system is desired, pull relay signal HIGH
    digitalWrite(redLightPin, HIGH);    // Turn red safety light OFF to indicate system is LIVE
   }  
}

void verbose_function(){

  Serial.print("Physical kill 1 status: ");
  if(physicalKill1Status == true) {
    Serial.println("KILLED");
  }
  else {
    Serial.println("UNKILLED");
  }
  Serial.print("Physical kill 2 status: ");
  if(physicalKill2Status == true) {
    Serial.println("KILLED");
  }
  else {
    Serial.println("UNKILLED");
  }
Serial.print("ch8 (remote kill) status: ");
  Serial.print(ch8PulseLength);
  Serial.print("us -> ");
  if(ch8KillStatus == true) {
    Serial.println("KILLED");
  }
  else {
    Serial.println("UNKILLED");
  }
  Serial.println(" ");
  Serial.println("Master kill status: ");
  if(kill == true) {
    Serial.println("KILLED");
  }
  else {
    Serial.println("UNKILLED");
  }
  Serial.println(" ");
   
}

