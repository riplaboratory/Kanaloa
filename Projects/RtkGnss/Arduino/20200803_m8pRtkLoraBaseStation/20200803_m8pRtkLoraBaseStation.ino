/*
 * TX code for NEO-M8P RTK GPS with RFM9x LoRa radio on an Arduino Nano
 * 
 * To install Adafruit fork of RadioHead library, download and install from this link: https://cdn-learn.adafruit.com/assets/assets/000/031/670/original/RadioHead-1.59.zip?1460574831
 * 
 * Other useful links:
 *   https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts
 *   https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
 *   https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#a46330e9d7ec87255b91d0e4297cc8814
 *   http://development.wombatsecurity.com/development/2018/10/05/modem-mystery/
 *   https://electronics.stackexchange.com/questions/278192/understanding-the-relationship-between-lora-chips-chirps-symbols-and-bits
*/

// Library inclusions
#include <SPI.h>                              // arduino SPI library (built in)
#include <RH_RF95.h>                          // adafruit fork of RadioHead library (get it here: https://cdn-learn.adafruit.com/assets/assets/000/031/670/original/RadioHead-1.59.zip?1460574831)
#include <SparkFun_Ublox_Arduino_Library.h>   // sparkfun ublox arduino library (get it from arduino ide library manager)
#include <Wire.h>                             // arduino I2C library (built in)

// Pin definitions
#define rfm9xCsPin 4    // chip select pin (CS on RFM9x module)
#define rfm9xRstPin 2   // radio reset pin (RST on RFM9x module)
#define rfm9xIntPin 3   // radio GPIO 0 interrupt pin (G0 on RFM9x module)

// Objects
RH_RF95 rfm9x(rfm9xCsPin, rfm9xIntPin);   // RFM9x radio instance
SFE_UBLOX_GPS neom8p;                     // NEO-M8P RTK GPS object instance

void setup()
{ 
  
  // Set pin mode
  pinMode(rfm9xRstPin, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);                     // if using hardware serial, 115200, but for software serial 9600 is advisable
  while (!Serial) { delay(1); }

  // Configure RFM9x module
  configureRfm9x(rfm9x);

  // Initialize I2C
  Wire.begin();

  // Configure NEO-M8P module
  configureNeoM8p(neom8p); 

  // Launch survey on NEO-M8P module
  launchNeoM8pSurvey(neom8p);
  
  // enable RTCM messages
  enableRTCM(neom8p);
        
}

void loop()
{
  
  // Check NEO-M8P module for new data
  neom8p.checkUblox();                 

  // Set delay to match maximum refresh rate of module (don't take up too many I2C cycles)
  delay(250);
  
}

// This function gets called from the SparkFun Ublox Arduino Library as an interrupt each time RTCM correction data is available
void SFE_UBLOX_GPS::processRTCM(uint8_t hexByte)
{
  
  // For now, just printing the HEX values to serial... eventually
  if (neom8p.rtcmFrameCounter % 16 == 0) { Serial.println(); }
  Serial.print(F(" "));
  if (hexByte < 0x10) { Serial.print(F("0")); }
  Serial.print(hexByte, HEX);

  
//  Test code...
//  // Send new packet
//  Serial.print(F("Sent to RX: '"));
//  rfm9x.send(incoming , 50);
//
//  // Wait for packet confirmation
//  delay(10);
//  rfm9x.waitPacketSent();
//
//  // Wait up to 500 ms for reply
//  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//  uint8_t len = sizeof(buf);
//  if (rfm9x.waitAvailableTimeout(500))
//  { 
//    // Should be a reply message for us now   
//    if (rfm9x.recv(buf, &len))
//   {
//      Serial.print(F("Received from RX: '"));
//      Serial.print((char*)buf);
//      Serial.print(F("' at RSSI = "));
//      Serial.println(rfm9x.lastRssi(), DEC);
//    }
//    else
//    {
//      Serial.println(F("Receive failed"));
//    }
//  }
//  else
//  {
//    Serial.println("No reply, is there a listener around?");
//  }


}
