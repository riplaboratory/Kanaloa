/*
 * RFM9x LoRa radio test TX code for Arduino Nano
 * 
 * To install Adafruit fork of RadioHead library, download and install from this link: https://cdn-learn.adafruit.com/assets/assets/000/031/670/original/RadioHead-1.59.zip?1460574831
 * 
 * Other useful links:
 *   https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts
 *   https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
*/

// Library inclusions
#include <SPI.h>
#include <RH_RF95.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Wire.h>

// Pin definitions
#define RFM95_CS 4    // chip select pin (CS on RFM9x board)
#define RFM95_RST 2   // radio reset pin (RST on RFM9x board)
#define RFM95_INT 3   // radio GPIO 0 interrupt pin (G0 on RFM9x board)

// Set specific 900 MHz frequency
#define RF95_FREQ 915
#define RF95_PWR 23

// Create instance of radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Create instance of rtk gnss object
SFE_UBLOX_GPS baseStation;                    
void setup()
{ 
  // Set pin mode
  pinMode(RFM95_RST, OUTPUT);

  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  // Reset RFM9x module
  Serial.println("Resetting RFM9x module...");
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize communication with RFM9x module
  while (!rf95.init()) {
    Serial.println("Failed to communicate with RFM9x module!");
    while (1);
  }
  Serial.println("Sucessful communication with RFM9x module...");

  // Set frequency to 915 MHz
  rf95.setFrequency(RF95_FREQ);

  // Set output power to 23 dBm (ranges from 5 to 23 dBm)
  rf95.setTxPower(RF95_PWR, false);

    // Initialize I2C comms bus
  Wire.begin();

  // setup GNSS
  setupGNSS(baseStation); 

  // wait for Survey-In to finish
  waitForSurvey();        

  // enable RTCM messages
  enableRTCM();           
}

void loop()
{
  
  // see if new data is available. Process bytes as they come in. 
  baseStation.checkUblox();                 

  Serial.println("DOOODOOOOO");
}


// This function gets called from the SparkFun Ublox Arduino Library.
// As each RTCM byte comes in you can specify what to do with it
// Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  //Let's just pretty-print the HEX values for now
  if (baseStation.rtcmFrameCounter % 16 == 0) Serial.println();
  Serial.print(" ");
  if (incoming < 0x10) Serial.print("0");
  Serial.print(incoming, HEX);

  // Delay
  delay(250); // Wait 1 second between transmits, could also 'sleep' here!

  // Send new packet
  Serial.print("Sent to RX: '");
  rf95.send(incoming , 50);

  // Wait for packet confirmation
  delay(10);
  rf95.waitPacketSent();

  // Wait up to 500 ms for reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Received from RX: '");
      Serial.print((char*)buf);
      Serial.print("' at RSSI = ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
}
