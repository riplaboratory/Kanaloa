//Changelog
//03-22-2021: ESP32 now reboots when WiFi fails to connect. Blue LED turns on when ROS connects. Formatting and documentation edits.
//04-12-2021: Documentation edits.
//04-21-2021: Updated WiFi and IP credentials. Moved LED off function to setup so it actually turns the LED off.
//04-26-2021: Added frame_id, changed coordinate data type to double
//04-28-2021: Added covariance
//05-02-2021: Added ESP restart subscriber, comment/formatting/documentation edits, added ESP ROS connectivity check
//05-03-2021: Documentation and formatting, added connectivity check to the end of the loop as well
//05-05-2021: Added timestamp

//Arduino Libraries
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <Wire.h> //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS

//WiFi Definitions -- EDIT THIS IF WIFI CREDENTIALS CHANGE
const char* ssid = "MRUH_2G";
const char* password = "Aut0m@tion";
IPAddress server(10, 10, 10, 12); //Kanaloa IP, this must be set to the IP address of master
IPAddress ip_address;

//Dayne's hotspot for testing credentials
//const char* ssid = "June";
//const char* password = "zmxncbvv";
//IPAddress server(192,168,13,26); //Kanaloa IP, this must be set to the IP address of master (Hostname -I in terminal)
//IPAddress ip_address;

long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.
char frame[] = "zed_gps"; //Name frame ID
int status = WL_IDLE_STATUS; //has WiFi.status() return idle
#define ONBOARD_LED  2
WiFiClient client; //Initialize the client library
SFE_UBLOX_GPS myGPS; //Initialize GPS library

class WiFiHardware //Enables the onboard WiFi to act as a serial port
{
  public:
    WiFiHardware() {};

    void init()
    {
      //TCP server/client setup -- one port per sensor
      client.connect(server, 11411);
    }

    // read a byte from the serial port. -1 = failure
    int read()
    {
      // implement this method so that it reads a byte from the TCP connection and returns it
      //  you may return -1 is there is an error; for example if the TCP connection is not open
      return client.read();
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
      // implement this so that it takes the arguments and writes or prints them to the TCP connection
      for (int i = 0; i < length; i++)
        client.write(data[i]);
    }

    // returns milliseconds since start of program
    unsigned long time()
    {
      return millis();
    }
};

//ROS and data stuff here
sensor_msgs::NavSatFix fix_msg; //Assign fix_msg to NavSatFix class
ros::Publisher Fix("/fix", &fix_msg); //Declare ROS topic name "/fix", assign Publisher to Fix
ros::NodeHandle_<WiFiHardware> nh; //Set NodeHandle class as custom WiFiHardware class

//ESP32 restart subscriber
void esp_restart( const std_msgs::Empty& toggle_msg)
{
  ESP.restart(); //restart the ESP
}

ros::Subscriber<std_msgs::Empty> sub("esp_restart", &esp_restart);

//Connect to WiFi function
void setupWiFi() //Connects to WiFi using <WiFi.h>. SSID and password defined above.
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500); //Reboots ESP if WiFi connection fails
  if (i == 21)
  {
    Serial.print("Could not connect to"); Serial.println(ssid);
    Serial.print("Rebooting in 5 seconds");
    delay(5000);
    ESP.restart();
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup()
{
  Serial.begin(115200);

  //Initialize WiFi and LED
  setupWiFi();
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  //Initialize GPS
  Wire.begin();
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //Set up GPS
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  delay(2000);

  //Initialize ROS
  nh.initNode(); //Initializes ROS
  nh.advertise(Fix); //Publish ROS topic -- Change for different topics
  nh.subscribe(sub); //Subscribe to ROS esp restart topic
}

void loop()
{
  if (nh.connected())
  {
    //Blue LED on if ROS is connected
    //Also reboots the ESP if ROS connection is lost (usually)
    digitalWrite(ONBOARD_LED, HIGH);
  }

  for (int i = 0; i > 5; i++) //waits for 10 loops before checking the if statement
  {
    if (nh.connected() == 0) //if ROS connection is lost:
    {
      //Reboots ESP
      ESP.restart();
    }
  }

  fix_msg.header.frame_id = frame;
  fix_msg.header.stamp = nh.now();

  //TODO: Check to see if time stamp publisher updates correctly
  //Otherwise fix it

  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    double latitude = myGPS.getLatitude() * pow(10, -7); //get latitude data and assign it to long latitude
    fix_msg.latitude = latitude; //fix_msg.latitude is the ROS topic, = latitude is the GPS variable
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    double longitude = myGPS.getLongitude() * pow(10, -7);
    fix_msg.longitude = longitude;
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees)"));

    double altitude = myGPS.getAltitude() * pow(10, -3);
    fix_msg.altitude = altitude;
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (m)"));

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);
    Serial.println();

    //Placeholder covariance for localization
    fix_msg.position_covariance[0] = 0.0001;
    fix_msg.position_covariance[1] = 0.0001;
    fix_msg.position_covariance[2] = 0.0001;
    fix_msg.position_covariance[3] = 0.0001;
    fix_msg.position_covariance[4] = 0.0001;
    fix_msg.position_covariance[5] = 0.0001;
    fix_msg.position_covariance[6] = 0.0001;
    fix_msg.position_covariance[7] = 0.0001;
    fix_msg.position_covariance[8] = 0.0001;

    Fix.publish( &fix_msg ); //Publish all the data (&fix_msg) to ROS topic Fix
  }

  for (int i = 0; i > 5; i++) //I have two of these because sometimes it doesn't reboot properly idk
  {
    if (nh.connected() == 0) //if ROS connection is lost
    {
      //Reboots ESP
      ESP.restart();
    }
  }

  Serial.print("TCP Connection Status:"); Serial.println(nh.connected()); //1 = connected
  nh.spinOnce();
  delay(500);
}
