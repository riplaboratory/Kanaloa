/*
   TX code for NEO-M8P RTK GPS with RFM9x LoRa radio on an Arduino Nano

   To install Adafruit fork of RadioHead library, download and install from this link: https://cdn-learn.adafruit.com/assets/assets/000/031/670/original/RadioHead-1.59.zip?1460574831

   Other useful links:
     https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts
     https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
     https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#a46330e9d7ec87255b91d0e4297cc8814
     http://development.wombatsecurity.com/development/2018/10/05/modem-mystery/
     https://electronics.stackexchange.com/questions/278192/understanding-the-relationship-between-lora-chips-chirps-symbols-and-bits
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

// Globals
boolean configChecks = false;   // status of configuration checks
unsigned long tLoopStart = 0;   // time now [ms]
unsigned int tLoop = 0;         // time for loop execution [ms]
unsigned int tElapsedRtk = 0;   // time elapsed since last RTK grab [ms]

// Volatile globals
volatile byte rtcmByteNow = 0;    // number of bytes in current RTCM sentence
volatile byte rtcmSenNow[50];     // most current RTCM sentence
volatile byte rtcmByteLast = 0;   // number of bytes in current RTCM sentence
volatile byte rtcmSenLast[50];    // most current RTCM sentence

// Objects
RH_RF95 rfm9x(rfm9xCsPin, rfm9xIntPin);   // RFM9x radio instance
SFE_UBLOX_GPS neom8p;                     // NEO-M8P RTK GPS object instance

void setup()
{

  // Set pin mode
  pinMode(rfm9xRstPin, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

  // Configure RFM9x module
  configureRfm9x(rfm9x);

  // Initialize I2C
  Wire.begin();

  // Configure NEO-M8P module
  configureNeoM8p(neom8p);

  // Launch survey on NEO-M8P module
  launchNeoM8pSurvey(neom8p);

  // Enable RTCM messages
  enableRTCM(neom8p);

  // Set configChecks to true
  configChecks = true;

  // Zero instantiate arrays
  memset(rtcmSenNow, 0, sizeof(rtcmSenNow));
  memset(rtcmSenLast, 0, sizeof(rtcmSenLast));

}

void loop()
{

  // Loop start time calcuations
  tLoopStart = millis();

  // Read data from GPS approximately every 250 ms
  if (tElapsedRtk > 250) {
    
    // Check NEO-M8P module for new data
    neom8p.checkUblox();

    // Reset tElapsedRtk back to zero
    tElapsedRtk = 0;

  }

  
//  Serial.println(rtcmByteLast);


  // Delay 1 ms (to avoid timing errors)
  delay(1);

  // Loop end time calculations
  tLoop = tLoopStart - millis();
  tElapsedRtk += tLoop;

}

// This function gets called from the SparkFun Ublox Arduino Library as an interrupt each time RTCM correction data is available
void SFE_UBLOX_GPS::processRTCM(uint8_t hexByte)
{

  if (configChecks) {

//    // Save hexByte to sentence
//    if (neom8p.rtcmFrameCounter % 16 == 0) {      // if this is true, then the PREVIOUS bit was the end of the sentence
//      rtcmByteLast = rtcmByteNow;                 // set rtcmByteLast as rtcmByteNow
//      memcpy(rtcmSenLast, rtcmSenNow, 50);        // set rtcmSenLast as rtcmSenNow
//      rtcmByteNow = 0;                            // set rtcmByteNow to zero
//      memset(rtcmSenNow, 0, sizeof(rtcmSenNow));  // set rtcmSenNow to zero
//    }
//    rtcmSenNow[rtcmByteNow] = hexByte;
//    rtcmByteNow++;

    // Print RTCM correction data over serial
//    if (neom8p.rtcmFrameCounter % 16 == 0) { Serial.println(); }
    if (hexByte == 0xD3) { Serial.println(); }
    Serial.print(F(" "));
    if (hexByte < 0x10) { Serial.print(F("0")); }
    Serial.print(hexByte, HEX);

    //Send RTCM over RFM9x module
    //    rfm9x.send((byte)hexByte,1);
    // need to construct into a sentence in loop and send outside this interrupt function, since sending a radio packet takes a few hundred ms.

  }


  //  // Send new packet
  //  Serial.print(F("Sent to RX: '"));
  //  rfm9x.send(incoming, 50);
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
