void setSpd(int vel, byte fwdpin, byte revpin) {
    //Avoid shoot-through
    pwm.setPWM(fwdpin,0,4096);
    pwm.setPWM(revpin,0,4096);
    delayMicroseconds(50);
    
    if (vel == 0) {
      pwm.setPWM(fwdpin,0,0);
      pwm.setPWM(revpin,0,0);
    }
    else if (vel > 0 && vel < 4096) {
      pwm.setPWM(fwdpin,0,vel);
      pwm.setPWM(revpin,0,0);
    }
    else if (vel < 0 && abs(vel) < 4096) {
      pwm.setPWM(fwdpin,0,0);
      pwm.setPWM(revpin,0,abs(vel));
    }
    else {
      pwm.setPWM(fwdpin,0,4096);
      pwm.setPWM(revpin,0,4096);
      Serial.println("Error: Speed command is out of bounds.");
      Serial.print("I tried to set the servo command to: ");
      Serial.print(vel);
      Serial.println(" I'm not sure why");
    }
}

void vupdate(){
  voltageSetpoint_left = Vmax/1000.0*q2Motor;
  voltageSetpoint_right = Vmax/1000.0*q1Motor;
}
