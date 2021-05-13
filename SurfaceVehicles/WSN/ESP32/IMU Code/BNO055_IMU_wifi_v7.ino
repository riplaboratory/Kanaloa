//Changelog:
//04-16-2021: Added LED function when ROS is connected
//04-21-2021: Updated WiFi and IP credentials. Moved LED off function to setup so it actually turns the LED off.
//04-26-2021: Added frame_id
//04-28-2021: Added covariance
//05-02-2021: Added ESP restart subscriber, comment/formatting/documentation edits, increased BNO055_SAMPLERATE_DELAY_MS to 100, added ROS connectivity check
//05-03-2021: Documentation and formatting changes
//05-05-2021: Added timestamp

//Arduino Libraries
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//WiFi Definitions
const char* ssid = "MRUH_2G";
const char* password = "Aut0m@tion";
IPAddress server(10, 10, 10, 12); //Kanaloa IP, this must be set to the IP address of master (Hostname -I in terminal)
IPAddress ip_address;

//Dayne's hotspot for testing credentials
//const char* ssid = "June";
//const char* password = "zmxncbvv";
//IPAddress server(192,168,13,26);
//IPAddress ip_address;

#define BNO055_SAMPLERATE_DELAY_MS (100) //void loop delay
char frame[] = "bno055"; //Name frame ID
int status = WL_IDLE_STATUS; //has WiFi.status() return idle
#define ONBOARD_LED  2
WiFiClient client; //Initialize the client library

class WiFiHardware //Enables the onboard WiFi to act as a serial port
{
  public:
    WiFiHardware() {};

    void init()
    {
      //TCP server/client setup
      client.connect(server, 11412);
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
      // implement this so that it takes the arguments and writes orprints them to the TCP connection
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
sensor_msgs::Imu data_msg;
ros::Publisher Data("/imu_data", &data_msg); //Set up topic
ros::NodeHandle_<WiFiHardware> nh; //Set NodeHandle class as custom WiFiHardware class

//ESP32 restart subscriber
void esp_restart( const std_msgs::Empty &toggle_msg)
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

void setup(void)
{
  Serial.begin(115200);
  
  //Initialize WiFi and LED
  setupWiFi();
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  //Initialize IMU
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(2000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  //Initialize ROS
  nh.initNode(); //Enable TCP connection
  nh.advertise(Data); //Publish ROS topic -- Change for different topics
  nh.subscribe(sub); //Subscribe to ROS esp restart topic
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  if (nh.connected())
  {
    //Blue LED on if ROS is connected
    digitalWrite(ONBOARD_LED, HIGH);
  }

  for (int i = 0; i > 10; i++) //waits for 10 loops before checking the if statement
  {
    if (nh.connected() == 0) //if ROS connection is lost:
    {
      //Reboots ESP
      ESP.restart();
    }
  }

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angvelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  data_msg.header.frame_id = frame;
  data_msg.header.stamp = nh.now(); //Gets unix time from ROS and includes it in the data message for synchronization purposes

  //TODO: Check to see if time stamp publisher updates correctly
  //Otherwise fix it

  /* Display the floating point data, (euler) */
    data_msg.orientation.x = euler.x();
    Serial.print("X: ");
    Serial.print(euler.x());
  
    data_msg.orientation.y = euler.y();
    Serial.print(" Y: ");
    Serial.print(euler.y());
  
    data_msg.orientation.z = euler.z();
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t");

//  //  /* Display IMU data in QUATERNION */
//  data_msg.orientation.w = quat.w(), 4;
//  data_msg.orientation.x = quat.x(), 4;
//  data_msg.orientation.y = quat.y(), 4;
//  data_msg.orientation.z = quat.z(), 4;

  data_msg.linear_acceleration.x = linearaccel.x();
  data_msg.linear_acceleration.y = linearaccel.y();
  data_msg.linear_acceleration.z = linearaccel.z();

  data_msg.angular_velocity.x = angvelocity.x();
  data_msg.angular_velocity.y = angvelocity.y();
  data_msg.angular_velocity.z = angvelocity.z();

  //Placeholder covariance for localization
  data_msg.orientation_covariance[0] = 0.002;
  data_msg.orientation_covariance[1] = 0.002;
  data_msg.orientation_covariance[2] = 0.002;
  data_msg.orientation_covariance[3] = 0.002;
  data_msg.orientation_covariance[4] = 0.002;
  data_msg.orientation_covariance[5] = 0.002;
  data_msg.orientation_covariance[6] = 0.002;
  data_msg.orientation_covariance[7] = 0.002;
  data_msg.orientation_covariance[8] = 0.002;

  //Placeholder covariance for localization
  data_msg.linear_acceleration_covariance[0] = 0.003;
  data_msg.linear_acceleration_covariance[1] = 0.003;
  data_msg.linear_acceleration_covariance[2] = 0.003;
  data_msg.linear_acceleration_covariance[3] = 0.003;
  data_msg.linear_acceleration_covariance[4] = 0.003;
  data_msg.linear_acceleration_covariance[5] = 0.003;
  data_msg.linear_acceleration_covariance[6] = 0.003;
  data_msg.linear_acceleration_covariance[7] = 0.003;
  data_msg.linear_acceleration_covariance[8] = 0.003;

  //Placeholder covariance for localization
  data_msg.angular_velocity_covariance[0] = 0.001;
  data_msg.angular_velocity_covariance[1] = 0.001;
  data_msg.angular_velocity_covariance[2] = 0.001;
  data_msg.angular_velocity_covariance[3] = 0.001;
  data_msg.angular_velocity_covariance[4] = 0.001;
  data_msg.angular_velocity_covariance[5] = 0.001;
  data_msg.angular_velocity_covariance[6] = 0.001;
  data_msg.angular_velocity_covariance[7] = 0.001;
  data_msg.angular_velocity_covariance[8] = 0.001;
  /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */

  /* Display calibration status for each sensor to serial monitor */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  Data.publish( &data_msg ); //Publishes all the data in the message

  if (nh.connected()) nh.spinOnce(); //I don't know how this works and I'm afraid to touch it.
  nh.spinOnce(); 
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
