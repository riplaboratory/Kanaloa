/*
 * This code is used to run the killswitch system. The Arduino sends 5V out to the two physical kill switches which loops back into a pin on the Arduino. When the switches are 
 * engaged, the circuit breaks and the Arduino reads the pin as LOW. If the switches are disengaged, the Arduino reads the pin as HIGH. For the remote killswitch, the Arduino
 * interprets the PWM signal coming from the receiver to infer the remote killswitch state. If any of these switches are engaged, a software switch is engaged, and the system 
 * is killed. 
 * 
 * Creaeted by: Kai Jones
 * Date: 07/19/2018
 */

//-----------------
// Pin Definitions
//-----------------
const int physicalSwitch1Pin = 2;   // Physical kill switch number 1 that reads whether switch is engaged (OPEN/LOW) or disengaged (CLOSED/HIGH) 
const int physicalSwitch2Pin = 4;   // Physical kill switch number 2 that reads whether switch is engaged (OPEN/LOW) or disengaged (CLOSED/HIGH)
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
bool softwareSwitch = true;  // A "switch" in software. If this switch is false, one of the kill switches are engaged.
bool switch1 = false;        // Physical killswitch variable. If killswitch is engaged, Arduino will read LOW on physicalSwitch1Pin and set this variable to true.       
bool switch2 = false;        // Physical killswitch variable. If kill switch is engaged, Arduino will read LOW on physicalSwitch1Pin and set this variable to true.
bool remote = false;         // Remote killswitch variable. If remote switch is engaged, Arduino will read PWM kill signal and set this variable to true.

//-----------
// TEST MODE
//-----------
// Basically, by setting the following variable 'TEST_MODE' to true, the code will run the same main function, except it will print inputs and outputs in the serial monitor.
bool TEST_MODE = true;             // For troubleshooting purposes. Set to true to see what values each pin is receiving and outputting. 

void setup() {
  pinMode(physicalSwitch1Pin, INPUT);     // Reads whether physical killswitch 1 is engaged (LOW) or disengaged (HIGH)
  pinMode(physicalSwitch2Pin, INPUT);     // Reads whether physical killswitch 2 is engaged (LOW) or disengaged (HIGH)
  pinMode(relayPin, OUTPUT);              // Arduino sends a HIGH/LOW signal to relay depending on killswitches's states
  pinMode(ch8Pin, INPUT_PULLUP);          // Arduino reads incoming PWM signal for remote kill switch
  Serial.begin(9600);
}


//------------
//  Main Loop
//------------
void loop() {
   
   if(TEST_MODE == true){   // Check to see if boolean variable called TEST_MODE is set to true
    test_mode();            // If true, call "test_mode()" function (see farther down)
   }
   else{
    main_function();       // Otherwise, call function named "main_function()"
   }
}


//-----------
// Functions
//-----------

void test_mode(){
    
   if(digitalRead(physicalSwitch1Pin) == HIGH){      // Check to if physical killswitch is disengaged (should be HIGH since ciruit is normally closed.)
      switch1Statement = "Switch 1 is disengaged.";  // Set string variable to say switch's state.
      switch1 = false;                               // Set boolean variable to false meaning switch is disengaged.
   }
   else{
      switch1Statement = "Switch 1 is engaged.";    // Set string variable to say switch's state.
      switch1 = true;                               // Set boolean variable to true meaning swithc is engaged.
   }
   
   if(digitalRead(physicalSwitch2Pin) == HIGH){     // Check to if physical killswitch is disengaged (should be HIGH since ciruit is normally closed.)
      switch2Statement = "Switch 2 is disengaged."; // Set string variable to say switch's state.
      switch2 = false;                              // Set boolean variable to false meaning switch is disengaged.
   }
   else{
      switch2Statement = "Switch 2 is engaged.";    // Set string variable to say switch's state.
      switch2 = true;                               // Set boolean variable to true meaning switch is engaged.
   }
   
   if(pulseIn(ch8Pin, HIGH, 32000) >= 1200){                 // Read PWM signal to see if remote switch is disengaged.
      remoteStatement = "Remote killswitch is disengaged.";  // Set string variable to say switch's state.
      remote = false;                                        // Set boolean variable to false meaning switch is disengaged.
   }
   else {
      remoteStatement = "Remote killswitch is engaged.";    // Set string variable to say switch's state.
      remote = true;                                        // Set boolean variable to true meaning switch is engaged.
   }

   if((switch1 == true || switch2 == true) || remote == true){  // Check to see if any of the three switches are engaged (true).
      softwareSwitch = true;                                    // Set software switch to true.
   }
   else{
      softwareSwitch = false;                                   // If none of the switches are engaged, set software switch to false.
   }
   
   //------------------
   // Print statements
   //------------------

   Serial.println(switch1Statement);                  // Print state of physical killswitch 1.
   Serial.println(switch2Statement);                  // Print state of physical killswitch 2.
   Serial.print("ch8 PWM: ");                         // Print PWM signal from receiver.
   Serial.println(pulseIn(ch8Pin, HIGH, 32000));      // PWM signal. 
   Serial.println(remoteStatement);                   // Print state of remote kill switch. 
   delay(500);                                        // Delay for half a second for readability.
   
   //--------------------------------------------------------------
   // Send kill signal to relay dependingo on software switch state
   //--------------------------------------------------------------
   
   if(softwareSwitch == false){                    // Check to see if software switch is engaged
    digitalWrite(relayPin, LOW);                   // If software switch is engaged, trigger relay 
   }
   else{
    digitalWrite(relayPin, HIGH);                  // Otherwise, leave relay alone
   }
}



void main_function(){
  
  if(digitalRead(physicalSwitch1Pin) == HIGH){  // Check to if physical killswitch is disengaged (should be HIGH since ciruit is normally closed.)   
    switch1 = false;                            // Set boolean variable to false meaning switch is disengaged.
   }
   else{
    switch1 = true;                             // Set boolean variable to true meaning switch is engaged.
   }
   
   if(digitalRead(physicalSwitch2Pin) == HIGH){ // Check to if physical killswitch is disengaged (should be HIGH since ciruit is normally closed.)
    switch2 = false;                            // Set boolean variable to false meaning switch is disengaged.
   }
   else{
    switch2 = true;                             // Set boolean variable to true meaning switch is engaged.
   }
   
   if(pulseIn(ch8Pin, HIGH, 30000) > 1000){     // Read PWM signal to see if remote switch is disengaged.
    remote = false;                             // Set boolean variable to false meaning switch is disengaged.
   }
   else{
    remote = true;                              // Set boolean variable to true meaning switch is engaged.
   }

   if((switch1 == true || switch2 == true) || remote == true){ // Check to see if any of the three switches are engaged (true).
    softwareSwitch = true;                                     // Set software switch to true. 
   }
   else{
    softwareSwitch = false;                                    // Otherwise, software switch is false. 
   }
   
   //--------------------------------------------------------------
   // Send kill signal to relay depending on software switch state
   //--------------------------------------------------------------
   
   if(softwareSwitch == false){    // Check to see if software switch is engaged
    digitalWrite(relayPin, LOW);   // If software switch is engaged, trigger relay 
   }
   else{
    digitalWrite(relayPin, HIGH);  // Otherwise, leave relay alone
   }  
}
