/*
 * This code is used to run the killswitch system. The Arduino sends 5V out to the two physical kill switches which loops back into a pin on the Arduino. When the switches are 
 * KILLED, the circuit breaks and the Arduino reads the pin as LOW. If the switches are UNKILLED, the Arduino reads the pin as HIGH. For the remote killswitch, the Arduino
 * interprets the PWM signal coming from the receiver to infer the remote killswitch state. If any of these switches are KILLED, a software switch is KILLED, and the system 
 * is killed. 
 * 
 * Created by: Kai Jones
 * Revisions by: Kai Jones
 * Date: 2018.09.06
 * 
 * Update Notes:
 * - Implemented code to control all safety lights (green, yellow, and red safety lights) 
 * 
 */

//-----------------
// Pin Definitions
//-----------------
const int physicalKill1Pin = 2;     // Physical kill switch number 1 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) 
const int physicalKill2Pin = 3;     // Physical kill switch number 2 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH)  
const int safetyRelayPin = 6;       // Pin which Arduino sends a HIGH/LOW signal to relay
const int manualControlPin = 7;     // Pin which Arduino reads a HIGH/LOW signal to determine whether WAM-V is either in autonomous or manual control state. Signal is sent from Arduino within low current box.  
const int ch8Pin = 8;               // PWM signal from the 8th channel of the R9 receiver in the wireless pole; this is the remote kill switch signal
const int greenLightPin = 12;       // Pin which Arduino sends a HIGH/LOW signal to relay for green safety light
const int redLightPin = 13;         // Red LED light on safety pole (HIGH to turn light ON, LOW to turn light OFF)



//------------------
// System Variables
//------------------
bool physicalKill1Status = true;    // physical kill 1 status variable. true means KILL is detected, false means UNKILL is detected.
bool physicalKill2Status = true;    // physical kill 2 status variable. true means KILL is detected, false means UNKILL is detected.
bool ch8KillStatus = true;          // ch8 (remote) kill status variable. true means KILL is detected, false means UNKILL is detected.
bool manualControlSignal = HIGH;    // Variable is either HIGH/LOW which correlates to whether WAM-V is in autonomous or manual control state.
int ch8PulseLength = 0;


//---------------
// SETUP FUNCTION
//---------------
void setup() {
  pinMode(physicalKill1Pin, INPUT);     // Reads whether physical killswitch 1 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(physicalKill2Pin, INPUT);     // Reads whether physical killswitch 2 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(safetyRelayPin, OUTPUT);            // Arduino sends a HIGH/LOW signal to relay depending on killswitches's states
  pinMode(ch8Pin, INPUT_PULLUP);        // Arduino reads incoming PWM signal for remote kill switch
  pinMode(redLightPin, OUTPUT);         // Arduino sends signal out to red safety light
  pinMode(greenLightPin, OUTPUT);       // Arduino sends HIGH/LOW signal to green safety light
  pinMode(manualControlPin, INPUT);  // Arduino reads HIGH/LOW signal to determine if WAM-V's state is in autonomous or manual control. 
}

//--------------
// LOOP FUNCTION
//--------------
void loop() {

  // Check the two physical kill switches and set to booleans
  physicalKill1Status = digitalRead(3);
  physicalKill2Status = digitalRead(2);

  // Get the Pulse length from reciever
  ch8PulseLength = pulseIn(ch8Pin,HIGH,250000);           

  // If statement to kill the system
  if (!physicalKill1Status || !physicalKill2Status || !(ch8PulseLength > 1825 && ch8PulseLength < 1925)) {
    digitalWrite(safetyRelayPin, LOW);
  }
  else {
    digitalWrite(safetyRelayPin, HIGH);
  }

  delay(50);  // Delay is for data to settle (Wonky without delay)
}
