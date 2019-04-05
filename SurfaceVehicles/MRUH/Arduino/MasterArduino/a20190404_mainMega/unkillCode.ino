void unkillCode() {

  // Unkill system
  digitalWrite(killRelayPin,LOW);   // set kill relay pin to LOW to unkill

  // Change light to yellow
  changeLight(3);

  // Read joysticks and calculate thruster setpoint values
  joy2Setpoint();

  // Calculate acceleration-limited outputs as a function of the setpoint and send to thrusters
  setpoint2Output();

  // Scale output to thrusters by a multiplier (typically to limit voltage output to thruster)
  scaleOutput();

  // Write output thruster values to motor controllers
  output2Thruster();
  
}
