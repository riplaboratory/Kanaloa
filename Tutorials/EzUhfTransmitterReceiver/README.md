# EzUHF JR Module Transmitter and 8 CH Receiver Operating and Pairing

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

## Pairing Process
