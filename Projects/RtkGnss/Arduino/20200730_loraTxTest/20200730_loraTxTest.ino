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

// Pin definitions
#define RFM95_CS 4    // chip select pin (CS on RFM9x board)
#define RFM95_RST 2   // radio reset pin (RST on RFM9x board)
#define RFM95_INT 3   // radio GPIO 0 interrupt pin (G0 on RFM9x board)

// Set specific 900 MHz frequency
#define RF95_FREQ 915
#define RF95_PWR 23

// Create instance of radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

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
  
}

void loop()
{
  
  // Delay
  delay(250); // Wait 1 second between transmits, could also 'sleep' here!

  // Send new packet
  char radiopacket[10] = "RNG =    ";
  itoa(random(0,10), radiopacket+6, 10);
  itoa(random(0,10), radiopacket+7, 10);
  itoa(random(0,10), radiopacket+8, 10);
  itoa(random(0,10), radiopacket+9, 10);
  Serial.print("Sent to RX: '");
  Serial.print(radiopacket);
  Serial.println("'");
  rf95.send((uint8_t *)radiopacket, 20);

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
