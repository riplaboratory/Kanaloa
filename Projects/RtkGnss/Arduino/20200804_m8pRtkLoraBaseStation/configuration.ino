void configureRfm9x(RH_RF95 &lora) {
  /*
      Configures the wireless settings for the RFM9x board. See the datasheet here: https://cdn.sparkfun.com/assets/f/d/1/1/6/RFM95_96_97_98W.pdf
      Important SPI registers for RFM9x:
        0x01: RegOpMode, we want 10000001: LoRa mode, Access LoRa registers, high frequency mode, STDBY
        0x09: RegPaConfig, we want 10001111: PA_BOOST, max RFO power (doesn't matter with PA_BOOST on), output power 17-(15-15)
        0x1d: RegModemConfig1, we want 01001000: 31.25 kHz, 4/8, explicit
        0x1e: RegModemConfig2, we want 10010100: 512 c/s, normal mode, CRC on, reset timeout
        0x26: RegModemConfig3, we want 00000100: static node, LNA gain set internally
  */
  
  // Reset RFM9x module
  digitalWrite(rfm9xRstPin, HIGH);
  delay(100);
  digitalWrite(rfm9xRstPin, LOW);
  delay(10);
  digitalWrite(rfm9xRstPin, HIGH);
  delay(10);

  // Initialize RFM9x module with default values
  while (!lora.init()) { Serial.println(F("Failed to communicate with the RFM9x module. Retrying...")); delay(100); }
  Serial.println(F("Sucessful communication with the RFM9x module..."));

  // Set frequency to 915 MHz using built-in function
  lora.setFrequency(915);            // set frequency to 915 MHz using built-in function
  lora.setTxPower(20);               // set transmitter power to 100 mW (20 dBm) using built in function
  lora.spiWrite(0x1d, B01001000);    // 31.25 kHz, 4/8, explicit
  lora.spiWrite(0x1e, B10010100);    // 512 c/s, normal mode, CRC on, reset timeout
  lora.spiWrite(0x26, B00000100);    // static node, LNA gain set internally
//  lora.spiWrite(0x26,B00001100);     // mobile node, LNA gain set internally
  
}

void configureNeoM8p(SFE_UBLOX_GPS &gnss) {
  
  // Connect to NEO-M8P
  gnss.begin(Wire);
  while (!gnss.isConnected()) { Serial.println(F("Failed to communicate with the NEO-M8P module. Retrying...")); delay(100); }
  Serial.println(F("Sucessful communication with the NEO-M8P module..."));

  // Check survey status of NEO-M8P
  while (!gnss.getSurveyStatus(2000)) { Serial.println(F("Failed to get survey status of NEO-M8P module. Retrying...")); delay(100); }
  Serial.println(F("Sucessful survey status check with NEO-M8P module..."));

  // Configure NEO-M8P module
  gnss.setI2COutput(COM_TYPE_UBX);
  gnss.saveConfiguration();

}

void launchNeoM8pSurvey(SFE_UBLOX_GPS &gnss) {

  // Local vars
  int minSurveyTime = 10;     // minimum amount of time for survey [s]       (eventually set this to 300 s)
  int minSurveyAcc = 100;     // minimum positional accuracy for survey [m]   (eventually set this to 2 m)

  // Start survey
  while (!gnss.enableSurveyMode(minSurveyTime, minSurveyAcc)) { Serial.println(F("Failed to start survey on NEO-M8P module. Retrying...")); delay(100); }
  Serial.println(F("Sucessful survey start on NEO-M8P module..."));
  Serial.print(F("  min elapsed time: "));
  Serial.print(minSurveyTime);
  Serial.print(F(" [s]; min accuracy: "));
  Serial.print(minSurveyAcc);
  Serial.println(F(" [m].")); 

  // Wait for survey to complete
  do {
    if (gnss.getSurveyStatus(2000)) {
      Serial.print(F("  elapsed time = "));
      Serial.print(gnss.svin.observationTime);
      Serial.print(F(" [s]; accuracy = "));
      Serial.print(gnss.svin.meanAccuracy);
      Serial.println(F(" [m]."));
    }
    else { Serial.println(F("Survey request failed. Retrying...")); }
  } while (!gnss.svin.valid);
  Serial.println(F("Parameters achieved, NEO-M8P survey complete..."));
  
}

void enableRTCM(SFE_UBLOX_GPS &gnss) {

  // Local vars
  boolean rtcmChecks = true;
  
  // RTCM checks
//  do {
    rtcmChecks &= gnss.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);   // enable message 1005 to output through I2C port, message every second
    rtcmChecks &= gnss.enableRTCMmessage(UBX_RTCM_1077, COM_PORT_I2C, 1);
    rtcmChecks &= gnss.enableRTCMmessage(UBX_RTCM_1087, COM_PORT_I2C, 1);
    rtcmChecks &= gnss.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10);  // enable message every 10 seconds
    if (!rtcmChecks) { Serial.println(F("RTCM messages failed to enable. Retrying...")); delay(100); }
//  } while (!rtcmChecks);
  Serial.println(F("RTCM message checks passed, NEO-M8P is now broadcasting RTCM messages..."));

  // Broadcast RTCM
  gnss.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  
}
