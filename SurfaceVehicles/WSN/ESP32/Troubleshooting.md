## Troubleshooting
The following documentation is a compiled list of errors encountered and their solutions when coding and implementing the ESP32 microcontrollers in WSN.

----
### Problem:
ESP32 continuously reboots when trying to use rosserial. The serial monitor output on the Arduino IDE looks like this:

```
rst:0xc (SW_CPU_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0018,len:4
load:0x3fff001c,len:1216
ho 0 tail 12 room 4
load:0x40078000,len:9720
ho 0 tail 12 room 4
load:0x40080400,len:6364
entry 0x400806b8
/home/runner/work/esp32-arduino-lib-builder/esp32-arduino-lib-builder/esp-idf/components/freertos/queue.c:1442 (xQueueGenericReceive)- assert failed!
abort() was called at PC 0x40087c45 on core 1

Backtrace: 0x4008b734:0x3ffb1d80 0x4008b965:0x3ffb1da0 0x40087c45:0x3ffb1dc0 0x400f93e2:0x3ffb1e00 0x400f95c6:0x3ffb1e20 0x400eba4c:0x3ffb1e40 0x400ebab5:0x3ffb1e60 0x400e8c52:0x3ffb1e80 0x400e7adc:0x3ffb1ea0 0x400eb2d0:0x3ffb1ee0 0x400d254e:0x3ffb1f00 0x400d1f46:0x3ffb1f50 0x400d1a87:0x3ffb1f80 0x400d2e07:0x3ffb1fb0 0x40087f19:0x3ffb1fd0

```

### Resolution:
"ROS opts to use a wifi connection over the serial connection if you program an ESP32". Edit the ros.h file in the ros_lib library:

```
vi ~/Arduino/libraries/ros_lib/ros.h
```

Change:

```
#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif
```

Into:

```
#if defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif
```

[Source](https://github.com/espressif/arduino-esp32/issues/4807#issuecomment-782414911)

----

### Problem:
GURU MEDITATION ERROR, REBOOT LOOP
Error: Core 1 panic'ed (LoadProhibited). Exception was unhandled.

### Resolution:
Core 1 errors also happen when rosserial is not installed properly. There are several ways to reinstall rosserial, but the following has worked the best for me:

  ```
	cd path/to/your/catkin/ws/src
	rm -r rosserial
	git clone https://github.com/ros-drivers/rosserial.git -b melodic-devel
	catkin_make
  ```

Change melodic-devel to whatever distro you’re on, Core 1 errors are usually a problem with ROS.

[Source](https://answers.ros.org/question/355801/rosserial-python-on-ros-noetic/)

----

### Problem:
ROSRUN HANGS AND DOESN’T CONNECT:
[INFO] [1512416583.799159]: ROS Serial Python Node Fork_server is: False [INFO] [1512416583.816673]: Waiting for socket connections on port 11411 waiting for socket connection 

### Resolution:
Resolved by increasing the void setup() delay and putting client.read(); back.

[Source](https://github.com/agnunez/espros/issues/3)

----

### Problem:
ROSRUN ERROR
ImportError: No module named queue

### Resolution:
In your catkin workspace, src/rosserial/rosserial_python/src/rosserial_python/SerialClient.py, try changing the line `import queue` to the following:
try: 
```
    import queue
except ImportError:
    import Queue as queue
```

Caused by Python 2 and Python 3 mismatch

[Source](https://answers.ros.org/question/362043/importerror-no-module-named-queue/)

----
