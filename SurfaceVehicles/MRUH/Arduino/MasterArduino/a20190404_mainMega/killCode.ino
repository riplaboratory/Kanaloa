void killCode() {

  // Kill system
  digitalWrite(killRelayPin,HIGH);   // set kill relay pin to HIGH to kill

  // Change light to red
  changeLight(2);
  
}
