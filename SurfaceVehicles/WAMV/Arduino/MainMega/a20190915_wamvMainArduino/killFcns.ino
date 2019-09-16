void zeroSetpoints() {
  /*
     Sets the thruster setpoints to zero.  This should be used when the system is killed to prevent the high current arduino controller from going crazy while system is killed.
  */

  // Set thruster components to zero
  leftThrusterSetpoint = 0;
  rightThrusterSetpoint = 0;

  // Create messages for I2C comms
  createI2cMsg();

}
