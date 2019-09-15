# EzUHF JR Module Transmitter and 8 CH Receiver Primer

The ImmersionRC EzUHF [JR Module Transmitter](https://www.immersionrc.com/fpv-products/ezuhf-jr-module/) and [8 Channel Diversity Receiver](https://www.immersionrc.com/fpv-products/ezuhf-8-channel-diversity-receiver/) are 431 to 450 MHz transmitter receiver modules that are compatible with the FrSky Taranis handheld remote controllers that we use in the lab.  The full operation manuals for each module can be found in their respective links.  

The 433 MHz frequency band is illegal to operate in the USA in this manner without the proper license; however, it is a common frequency band worldwide.  

## IMPORTANT NOTES

### Pairing

Transmitter TX1 should already be paired with receiver RX1, and transmitter TX2 should already be paired with receiver RX2.  Pairing is somewhat difficult, so if you happen to lose pairing for a transmitter-receiver pair in the field (e.g. if TX1 and RX1 become unpaired somehow), then you're probably better off switching to the TX2-RX2 transmitter-receiver pair for the duration of your mission.  The pairing between a transmitter and receiver should retain until either of the two are paired with a different device.

If you still need to pair a transmitter and receiver, see section on "Pairing Process" below. 

### Setting Failsafe Mode

The failsafe mode dictactes what actions the receiver will take in the event of a connection loss with the transmitter.  This is crucially important to consider for safe robot operation.  To set this failsafe, the transmitter and receiver combo should already be paired and powered on, and the transmitter should be mounted in the external RF slot of the Taranis handheld remote controller.  Move the joysticks and switches on the Taranis handheld remote controller to the positions you would like the robot to fail in, then hold down the "Fail Safe Bind" button on the back of the transmitter module.  The transmitter module will beep several times to confirm your settings.

__In particular, the most important fail safe setting is the remote kill switch channel on the handheld remote controller.__  This should be set to the KILL position (whichever physical switch that happens to be on the handheld remote controller) when setting the failsafe.  

To test your settings, most the switches and joysticks on the handheld remote controller, then power off the handheld remote controller.  If the failsafe setting worked, the robot should revert to the failsafe settings.

## Optional Notes

### Software Configuration (Optional)

The ImmersionRC software is called "ImmersionRC Tools" [available here](https://www.immersionrc.com/?download=4894), which runs on Windows.  It is used for firmware update and configuration of the transmitter and receiver modules.  You should not need to use this often, particularly if the transmitter and receiver modules are already working well.  Note that you will need a MicroUSB cable for programming the JR Transmitter module, and a __MiniUSB cable for programming the 8 Channel Diversity Receiver__; at the time of writing, MicroUSB cables are still quite common, but MiniUSB cables are very uncommon.

The devices connect as serial COM devices in Windows.  The softare interface is fairly straightforward.  The JR Tranmitter module falls under the "EzUHF: Tx, 500mW, 2W, JR" family of devices, and the 8 Channel Diversity Receiver falls under the "EzUHF: 8ch. Rx" family of devices.

To update the firmware, you need to download the firmware from the product website; this is a .fw file.  Then you need to put the device in "program mode" before connecting it to the computer.  Do this by holding down the bind button before plugging it into the computer.  Once connected to the computer, navigate to the device family, click on the "Program" tab, and click on the "Update Firmware" button.  This will open another window, where you should browse to where the .fw firmware file is stored on your computer.  Click OK and wait until the software tells you that the firmware update was successful.  __Note that the firmware version number for the transmitter module and the receiver module must match if you want to pair the two devices together__.

In addition to firmware update, we make two changes to the default settings.  First, we set the RF Channel Count to 8CH (from 12CH); this setting is under the "EzUHF Advanced" tab.  It is possible to get as many as 12 channels from the PPM receiver mode; however, this requires some amount of hardware modification, as receiver module only has 8 physical outputs.  If you require more than 8 channels on your handheld remote controller, we recommend using a system based on serial communication, like SBUS or DSM2 (read more on this topic [here](https://oscarliang.com/pwm-ppm-sbus-dsm2-dsmx-sumd-difference/)).  Second, we set the Frequency band to 

### Pairing Process

As previously mentioned, the pairing process can be quite inconsistent for this particular system, so this process can be challenging to perform in the field.  The TX1 transmitter should already be paired to the RX1 receiver, and the TX2 transmitter should already be paired to the RX2 receiver.  The systems should not become unpaired under normal circumstances.  

If you do need to pair the two devices, the process that works for us is as follows:
 1. Ensure both the transmitter and receiver module are powered off.
 2. Plug the transmitter module into the socket in the back of your Taranis handheld remote controller.  Ensure the transmitter module is set to "LO" power (physical switch on the rear of the transmitter module).
 3. Hold down the "Fail Safe Bind" button on the rear of the transmitter module, and then power on the Taranis handheld remote controller while continuing to hold down the "Fail Safe Bind" button.  As soon as the transmitter module begins beeping, let go of the "Fail Safe Bind" button.
 4. Within 10s, power on the receiver module, and then hold down the "BIND" button on the receiver module for at least 15 seconds.

__If you are having problems pairing, note the following:__ 
 - On a fresh transmitter-receiver combo, it took us 5-10 tries to pair successfully.  So keep trying!
 - The manual tells you to hold down the bind button for 5 seconds (step 4); however, we found that this should be more like 15 seconds.
 - It is actually difficult to tell if the pairing process was successful.  According to the manual, the receiver should blink 3 times of the pairing was successful and 6 times if it was not; however, we did not ever find this to be the case.  Your best method of ensuring pairing is to look at the channel outputs on an osciloscope and see if they respond accordingly to inputs to the handheld remote controller.  In addition, a paired receiver has a distinct LED flash pattern, which dims between on and off, as opposed to pulsing on an off; however, it will often not demonstrate this pattern until the power to the receiver is cycled after a sucessful pairing.  
 - The transmitter and receiver must be updated to the same firmware version number (e.g. 1.53).  You can update this in the ImmersionRC Tools software.
 - The transmitter and receiver should both be set to "430-450MHz Extreme Hopping" frequency band setting.  You can configure this in the ImmersionRC Tools software; however, the devices should already be configured to this setting.  You can try other configurations; however, we had the most success with this setting.
