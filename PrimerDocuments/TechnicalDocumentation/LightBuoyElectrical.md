
## **Light Buoy (Electrical Component)**

Made by: Yong Li

The light buoy is created for the purposes of testing and collecting data in a real life environment for color recognition. The main goal is for the light buoy to flash three colors (Red, Blue, and Green) in a random sequence.  

The light buoy will appear black when it is off. The light assembly on the buoy will successively display colors one at a time to generate a sequential pattern of three colors (e.g., red-green-red). Each individual color will appear for 1 second, after which the lights will remain off (black) for 2 seconds before repeating the same pattern. A color may be repeated in the pattern, but the same color will not appear twice in a row.

[Link to RobotX 2018 competition details
](https://www.robotx.org/images/RobotX-2018-Tasks_v2.0.pdf)

### **Prerequisites**

 - C/C++
 - Arduino knowledge
 - Relay knowledge 
 - Know how to solder 

### **Supplies**

 - Arduino Mega
 - Atleast 3-Channel Relay Board
 - Portable Power Supply (12V+)
 - Voltage Regulator (If needed)
 - Husky Box or any water proof container/box.
 - 90+ ft. of LED strips. 
 - Push Button
 - Water proof ethernet cable

### **Software**

 - [Arduino IDE ](https://www.arduino.cc/en/Main/Software)(Preferably)
 or
 - [Arduino Web Editor](https://create.arduino.cc/) (Must have internet connection)
 
 ### **Arduino**
 
![Arduino MEga](https://www.arduino.cc/en/uploads/Main/ArduinoMega.jpg)

An Arduino is required to control the color sequences. Further below will contain the Arduino code to control the color sequences of the LED strips. The Arduino will output three signals that will control relay boards to turn colors on/off on the LED strip. The Arduino will also take in an input from a push button that will signal when to randomize the color sequence.
[Link for more about Arduino
](https://www.arduino.cc/en/Tutorial/HomePage)

### **Power**

The light buoy will need to be fully functional out in the middle of the ocean. For this to be possible a portable battery is required. A 12V or higher portable battery is recommended to power the LED strips. If the power supply is higher than 12V an voltage regulator is required because the Arduino Mega has a max input of 12V. *Note: The power supply for this project will not have an on/off switch. Disconnect or connect power supply manually to turn power on/off.

### **Relay Board**
 
![enter image description here](https://www.inventelectronics.com/wp-content/uploads/2017/04/3-channel-relay-2.jpg)
 
 A 3-Channel relay board is required for the light buoy to create multiple color sequences. The relay board will connect to the three individual wires of the LED strip. Each wire is associated with a color (Red, Blue, and Green) and each wire will be connected to a individual channel on the relay board. An Arduino will send a signal to the relay board to control which lights to turn on/off. 
 
[Link for more information about Relay Boards](https://howtomechatronics.com/tutorials/arduino/control-high-voltage-devices-arduino-relay-tutorial/)



### **Schematics**

Below is the basic overall schematic of the electrical components. 

Note: If voltage of power supply higher than 12 V a voltage regulator is required. When assembling the actual components into a husky do not use a bread board. 
 
![enter image description here](https://lh3.googleusercontent.com/oK2wUOcoyEaFhKDlpQ-Dj-eCkadwH8e-oe7Gd6JenzbL-Vspa9L-EAWKxP4K5cVqQU1F2sPwuRGq)

Below is a close up of the schematic for the Arduino Mega. From this schematic there are 2 inputs, 3 outputs, and 3 ground. The 12V battery supply will connect to the VIN pin. The 5V pin is an output to power the relay board. Pins 4-6 are outputs that connect to the relay board. The pins send a signal to turn the relay on/off. Pin 2 is an input that connects to the push button. The input will signal the code when to randomize the color pattern. 

![enter image description here](https://lh3.googleusercontent.com/JOhRVOfFbIGac3HqZIaRFvxmVnimNKUcZFjI--oJ71ckJSwrU2Cg5v9i-6og7xXrWdNaZAsZpNto)

Below is a close up of the schematic for the relay board. The three outputs coming from the Arduino's pins 4-6 are received as inputs on the relay board IN 1-3. The relay board is also powered by the 5V output from the Arduino. Relays K 1-3 output 3 wires to the led light strip. *Note: The schematic shows a LED light to represent the LED strip. 

![enter image description here](https://lh3.googleusercontent.com/vOm3j_ioa97sHyetp2GTIU1-SRyaDiq9Ij365_kClWMecO1_mnfdXGt7gWO0VDkQ75LfTE7BLT4G)

### **Code**

Below are two different codes that will work for the light buoy. Both codes act accordingly, each individual color will appear for 1 second, after which the lights will remain off (black) for 2 seconds before repeating the same pattern. A color may be repeated in the pattern, but the same color will not appear twice in a row.

The reasoning behind having two different codes are for the different methods of randomizing the color sequences. There will be a push button on the outside of the electrical components when held will randomize the color sequence. However, if there are ever issues with the push button during testing. Below contains code that will avoid using the button and randomize the color patterns after four sequences. 

[Link for code to randomize after four consecutive patterns](https://github.com/riplaboratory/Kanaloa/blob/master/Projects/DeepLearning/ScanTheCode/Arduino/a20181013/a20181013_scanTheCode.ino)

[Link for code to randomize when button is held](https://github.com/riplaboratory/Kanaloa/blob/master/Projects/DeepLearning/ScanTheCode/Arduino/STCRandButton)

Software is required to import code onto the Arduino. Either the IDE or the web editor will work. 

### **Assembly**

 Below is an example of how assembling the Husky Box should look like. All electrical components are attached to the box with velcro and on the base of the box NOT the cover of the box. If you look at the sides of the box there is a button and water proof ethernet cable. Ethernet cable were chosen to connect the electrical components to the actual light buoy because it is water proof and long enough to connect the two. Other water proofing is needed such as super gluing around the button as well as the ethernet cable. 

*Note: In the example box there is a voltage regulator. This was used to drop the portable battery(14.4V) to 12V. 
 
![enter image description here](https://lh3.googleusercontent.com/8ICdDS54b4UjO9Jfksjj4J_tzWmSYEsAXu7PtX8rVLjdSAefE6AmS2m3e1ofwjDcOXZ99jCgvJye)
