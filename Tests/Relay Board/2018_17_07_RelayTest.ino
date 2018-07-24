/*
 * The purpose of this program is to test a relay switch. The Arduino will output a 5V signal (HIGH or LOW) to the relay board's
 * input signal pins. When Arduino is sending HIGH signal, the LED indicator on the relay turns OFF. When the Arduino is sending 
 * LOW signal, the LED indicator on the relay turns ON. This can be interpreted as a 'Red Light' which means 'Stop' or in this 
 * case, 'No Power Going To This Relay.'
 * 
 * Creaeted by: Kai Jones
 * Date: 07/17/18
 */

int relay_1 = 5; // Relay 1 is set to the Arduino's pin numbered 5
int relay_2 = 6; // Relay 2 is set to the Arduino's pin numbered 6

void setup() {
  pinMode(relay_1, OUTPUT); // Relay 1 pin is set to 'OUTPUT' a signal
  pinMode(relay_2, OUTPUT); // Relay 2 pin is set to 'OUTPUT' a signal 
}

void loop() {
  digitalWrite(relay_1, HIGH); // Arduino sends a HIGH signal (5V) to relay 1
  digitalWrite(relay_2, HIGH);  // Arduino sends a LOW signal (0V) to relay 2
  delay(5000);                 // Delay for 5 seconds so we can tell what happens to each relay 
  digitalWrite(relay_1, LOW);  // Arduino sends a LOW signal (0V) to relay 1
  digitalWrite(relay_2, HIGH); // Arduino sends a HIGH signal (5V) to relay 2
  delay(5000); 
}
