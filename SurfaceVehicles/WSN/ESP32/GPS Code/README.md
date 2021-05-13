## General SOP for Using the ESP32

### First-Time Setup
1. Open most recent version of esp_ZEDGPS_wifi.ino
2. Make sure the ssid and password between the quotation marks of `const char* ssid = “”;` and `const char* password = “”;` match the router settings
3. Obtain the local IP address by entering `Hostname -I` in the terminal
4. Enter the IP within the parenthesis of `IPAddress server();` separated by commas instead of periods, i.e. `(192,168,0,103)`
5. Repeat steps 2 through 4 for most recent version of `BNO055_IMU_wifi.ino` and any other sensor sketches

### With Launch File (Normal Operation)
1. Power sensors and ESP32
2. Open linux terminal
3. Enter `rosrun wsn_launch.launch` (or the name of your launch file) in the terminal
4. If the ESP32 successfully connects, wait at least 10 seconds before closing the TCP port, otherwise the ESP32 may need to be power cycled

### Without Launch File (Testing, Troubleshooting, Etc.)
1. Power sensors and ESP32
2. Open linux terminal
3. Enter `roscore` in terminal
4. Open new terminal window or tab
5. Enter `rosrun rosserial_python serial_node.py tcp 11411 __name:=GPS` in the terminal
6. Open new terminal window or tab
7. Enter `rosrun rosserial_python serial_node.py tcp 11412 __name:=IMU in the terminal`
8. If the ESP32 successfully connects, wait at least 10 seconds before closing the TCP port, otherwise the ESP32 may need to be power cycled

**For additional sensors, change the port number (the number between tcp and __name) to a number above 11411, and change the __name: to anything**

### Viewing Data
1. Enter `rostopic list` in a new terminal to view a list of ROS topics being published
2. Enter `rostopic echo <TOPIC>` to view data, I.E. rostopic echo Fix to view GPS data

### Recording data
Enter `rosbag record <TOPIC>` to record one specific topic, or `rosbag record -a` if you need to record all topics (***this is generally not recommended as the bagfiles could end up being gigantic***).

Enter `rosbag record -o <desired filename> <TOPIC>`  to name the bagfile that is recorded.
