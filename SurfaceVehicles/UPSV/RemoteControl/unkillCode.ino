void unkillCode() {

  Serial.println("unkill");

  // Unkill system
  digitalWrite(killPortRelayPin,LOW);   // set PORT SIDE kill relay pin to LOW to unkill
  digitalWrite(killStarRelayPin,LOW);   // set STARBOARD SIDE kill relay pin to LOW to unkill

/*
  // Read battery voltage
  battRead();

  // Perform battery check if user desires it
  batteryCheck();  
*/
  // Change light to yellow
  changeLight(5);

  // Read joysticks and calculate thruster setpoint values
  joy2Setpoint();

  // Calculate acceleration-limited outputs as a function of the setpoint and send to thrusters
  setpoint2Output();

  // Scale output to thrusters by a multiplier (typically to limit voltage output to thruster)
  scaleOutput();

  // Write output thruster values to motor controllers
  output2Thruster();
  
}
