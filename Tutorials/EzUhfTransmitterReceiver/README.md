# EzUHF JR Module Transmitter and 8 CH Receiver Operating and Pairing

The ImmersionRC EzUHF [JR Module Transmitter](https://www.immersionrc.com/fpv-products/ezuhf-jr-module/) and [8 Channel Diversity Receiver](https://www.immersionrc.com/fpv-products/ezuhf-8-channel-diversity-receiver/) are 431 to 450 MHz transmitter receiver modules that are compatible with the FrSky Taranis transmitters that we use in the lab.

The 433 MHz frequency band is illegal to operate in the US in this manner without the proper license; however, it is a common frequency band worldwide.  

## IMPORTANT NOTES

### Pairing

Transmitter TX1 should already be paired with receiver RX1, and transmitter TX2 should already be paired with receiver RX2.  Pairing is somewhat difficult, so if you happen to lose pairing for a transmitter-receiver pair in the field (e.g. if TX1 and RX1 become unpaired somehow), then you're probably better off switching to the TX2-RX2 transmitter-receiver pair for the duration of your mission.  The pairing between a transmitter and receiver should retain until either of the two are paired with a different device.

If you still need to pair a transmitter and receiver, see section on "Pairing Process" below. 

### Setting Failsafe

The failsafe mode dictactes what actions the receiver will take in the event of a connection loss.  This is crucially important to consider for safe robot operation.  To set this failsafe, the transmitter and receiver combo should already be paired and powered on, and the transmitter should be mounted in the external RF slot of the Taranis handheld remote controller.  Move the joysticks and switches on the Taranis handheld remote controller to the positions you would like the robot to fail in, e.g. if you would like the robot to "kill" upon a connection loss, then move your kill switch channel to the "kill" position.  Press and hold the "Fail Safe Bind" button on the back of the transmitter module.  The transmitter module will beep several times to confirm your settings.

Test your settings by moving switches and joysticks into random positions, then power off the transmitter.  If the failsafe setting worked, the robot should revert to the settings 

## Software Configuration

## Pairing Process
