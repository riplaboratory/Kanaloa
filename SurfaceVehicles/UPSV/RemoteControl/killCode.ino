void killCode() {

  Serial.println("kill");

  // Kill system
  digitalWrite(killPortRelayPin,HIGH);   // set PORT SIDE kill relay pin to HIGH to kill
  digitalWrite(killStarRelayPin,HIGH);   // set STARBOARD SIDE kill relay pin to HIGH to kill

/*
  // Read battery voltage
  battRead();

  // Perform battery check if user desires it
  batteryCheck();  
*/
  // Change light to red
  changeLight(2);
  
}
