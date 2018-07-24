/*
 * This code is used to run the killswitch system. The Arduino sends 5V out to the two physical kill switches which loops back into a pin on the Arduino. When the switches are 
 * KILLED, the circuit breaks and the Arduino reads the pin as LOW. If the switches are UNKILLED, the Arduino reads the pin as HIGH. For the remote killswitch, the Arduino
 * interprets the PWM signal coming from the receiver to infer the remote killswitch state. If any of these switches are KILLED, a software switch is KILLED, and the system 
 * is killed. 
 * 
 * Creaeted by: Kai Jones
 * Date: 07/21/2018
 * 
 * Update Notes: 
 * - Changed some naming conventions to be more clear (i.e. "unkilled" instead of "disengaged")
 * - Changed structure of code for verbose mode 
 * 
 */

//-----------------
// Pin Definitions
//-----------------
const int physicalSwitch1Pin = 2;   // Physical kill switch number 1 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH) 
const int physicalSwitch2Pin = 3;   // Physical kill switch number 2 that reads whether switch is KILLED (OPEN/LOW) or UNKILLED (CLOSED/HIGH)
const int relayPin = 6;             // Pin which Arduino sends a HIGH/LOW signal to relay
const int ch8Pin = 8;               // PWM signal from the 8th channel of the R9 receiver in the wireless pole; this is the remote kill switch signal

//-----------------
// String Variables
//-----------------
String switch1Statement;
String switch2Statement;
String remoteStatement;

//------------------
// System Variables
//------------------
bool softwareSwitch = true;  // A "switch" in software. If this switch is false, one of the kill switches are KILLED.
bool switch1 = false;        // Physical killswitch variable. If killswitch is KILLED, Arduino will read LOW on physicalSwitch1Pin and set this variable to true.       
bool switch2 = false;        // Physical killswitch variable. If kill switch is KILLED, Arduino will read LOW on physicalSwitch1Pin and set this variable to true.
bool remote = false;         // Remote killswitch variable. If remote switch is KILLED, Arduino will read PWM kill signal and set this variable to true.

//-------------
// VERBOSE MODE
//-------------
// Basically, by setting the following variable 'verbose_mode' to true, the code will run the same main function, except it will print inputs and outputs in the serial monitor.
bool verbose_mode = false;             // For troubleshooting purposes. Set to true to see what values each pin is receiving and outputting. 

void setup() {
  pinMode(physicalSwitch1Pin, INPUT);     // Reads whether physical killswitch 1 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(physicalSwitch2Pin, INPUT);     // Reads whether physical killswitch 2 is KILLED (LOW) or UNKILLED (HIGH)
  pinMode(relayPin, OUTPUT);              // Arduino sends a HIGH/LOW signal to relay depending on killswitches's states
  pinMode(ch8Pin, INPUT_PULLUP);          // Arduino reads incoming PWM signal for remote kill switch
  Serial.begin(9600);
}


//------------
//  Main Loop
//------------
void loop() {

   main_function();            // main_function() is always called in each loop (see farther below)
   
   if(verbose_mode == true){   // Check to see if boolean variable called TEST_MODE is set to true
    verbose_function();        // If true, call "verbose_function()" (see farther down)
   }
   else{
                               // If verbose_mode is not true, simply do not call the function
   }
}


//-----------
// Functions
//-----------

void verbose_function(){
    
   if(digitalRead(physicalSwitch1Pin) == HIGH){      // Check to if physical killswitch is UNKILLED (should be HIGH since ciruit is normally closed.)
      switch1Statement = "Switch 1 is UNKILLED.";  // Set string variable to say switch's state.
   }
   else{
      switch1Statement = "Switch 1 is KILLED.";    // Set string variable to say switch's state.
   }
   
   if(digitalRead(physicalSwitch2Pin) == HIGH){     // Check to if physical killswitch is UNKILLED (should be HIGH since ciruit is normally closed.)
      switch2Statement = "Switch 2 is UNKILLED."; // Set string variable to say switch's state.
   }
   else{
      switch2Statement = "Switch 2 is KILLED.";    // Set string variable to say switch's state.
   }
   
   if(pulseIn(ch8Pin, HIGH, 32000) >= 1100){                // Read PWM signal to see if remote switch is UNKILLED.
      remoteStatement = "Remote killswitch is UNKILLED."; // Set string variable to say switch's state.
   }
   else {
      remoteStatement = "Remote killswitch is KILLED.";    // Set string variable to say switch's state.
   }
   
   //------------------
   // Print statements
   //------------------

   Serial.println(switch1Statement);               // Print state of physical killswitch 1.
   Serial.println(switch2Statement);               // Print state of physical killswitch 2.
   Serial.print("ch8 PWM: ");                      // Print PWM signal from receiver.
   Serial.println(pulseIn(ch8Pin, HIGH, 32000));   // PWM signal. 
   Serial.println(remoteStatement);                // Print state of remote kill switch. 
   delay(500);                                     // Delay for half a second for readability.
   
}


void main_function(){
  
  if(digitalRead(physicalSwitch1Pin) == HIGH){  // Check to if physical killswitch is UNKILLED (should be HIGH since ciruit is normally closed.)   
    switch1 = false;                            // Set boolean variable to false meaning switch is UNKILLED.
   }
   else{
    switch1 = true;                             // Set boolean variable to true meaning switch is KILLED.
   }
   
   if(digitalRead(physicalSwitch2Pin) == HIGH){ // Check to if physical killswitch is UNKILLED (should be HIGH since ciruit is normally closed.)
    switch2 = false;                            // Set boolean variable to false meaning switch is UNKILLED.
   }
   else{
    switch2 = true;                             // Set boolean variable to true meaning switch is KILLED.
   }
   
   if(pulseIn(ch8Pin, HIGH, 32000) >= 1100){    // Read PWM signal to see if remote switch is UNKILLED.
    remote = false;                             // Set boolean variable to false meaning switch is UNKILLED.
   }
   else{
    remote = true;                              // Set boolean variable to true meaning switch is KILLED.
   }

   if((switch1 == true || switch2 == true) || remote == true){ // Check to see if any of the three switches are KILLED (true).
    softwareSwitch = true;                                     // Set software switch to true. 
   }
   else{
    softwareSwitch = false;                                    // Otherwise, software switch is false. 
   }
   
   //--------------------------------------------------------------
   // Send kill signal to relay depending on software switch state
   //--------------------------------------------------------------
   
   if(softwareSwitch == false){    // Check to see if software switch is KILLED
    digitalWrite(relayPin, HIGH);  // If software switch is KILLED, trigger relay 
   }
   else{
    digitalWrite(relayPin, LOW);  // Otherwise, leave relay alone
   }  
}
