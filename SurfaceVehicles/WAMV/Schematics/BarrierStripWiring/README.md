# Barrier Strip Wiring
This document describes the barrier strip wiring pats for the WAM-V.  All barrier strips start at the low current box and connect 1:1 to a barrier strip somewhere else in the WAM-V; therefore, all strips are listed based on their relationship with the low current box barrier strip it connects to.

## LCBS1 to WPBS2/WPBS3
Low current barrier strip 1 to wireless pole barrier strip 2/3.
For connecting LC power (GND+5V) and mainMega to the WP power and receiver/SBUS decoder.

| Position (Color) | LC description | WP description |
| :---: | :---: | :---: |
| 1 (stripe orange) | Low current GND | Wireless pole GND (goes to WPBS3-1) | 
| 2 (solid orange) | Low current 5V | Wireless pole 5V (goes to WPBS2-2) | 
| 3 (stripe green) | empty | empty |
| 4 (solid blue) | empty | empty |
| 5 (stripe blue) | empty | empty |
| 6 (solid green) | empty | empty |
| 7 (stripe brown) | empty | empty |
| 8 (solid brown) | empty | empty |
 
## LCBS2 to WPBS1
Low current barrier strip 2 (LCBS2) to wireless pole barrier strip 1 (WPBS1).
For connecting the LC mainMega to WP recevier/SBUS decoder.

| Position (Color) | LC description | WP description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega digital pin 18 | SBUS decoder channel 1 |
| 2 (solid orange) | empty | empty |
| 3 (stripe green) | empty | empty |
| 4 (solid blue) | mainMega digital pin 19 | SBUS decoder channel 4 |
| 5 (stripe blue) | mainMega digital pin 20 | SBUS decoder channel 5 |
| 6 (solid green) | mainMega digital pin 21 | SBUS decoder channel 6 |
| 7 (stripe brown) | mainMega digital pin 3 | SBUS decoder channel 7 |
| 8 (solid brown) | mainMega digital pin 2 | SBUS decoder channel 8 |
 
## LCBS3 to HCBS1
Low current barrier strip 3 to high current barrier strip 1.
For connecting LC mainMega to HC optocoupler motor controller inputs.

| Position (Color) | LC description | HC description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega PWM servo sheild ch 0 signal (Q1) | optocoupler motor controller input Q1 |
| 2 (solid orange) | mainMega PWM servo sheild ch 1 signal (Q2) | optocoupler motor controller input Q2 |
| 3 (stripe green) | mainMega PWM servo sheild ch 2 signal (Q3) | optocoupler motor controller input Q3 |
| 4 (solid blue) | mainMega PWM servo sheild ch 3 signal (Q4) | optocoupler motor controller input Q4 |
| 5 (stripe blue) | empty | empty |
| 6 (solid green) | empty | empty |
| 7 (stripe brown) | empty | empty |
| 8 (solid brown) | empty | empty |

## LCBS4 to HCBS2
Low current barrier strip 4 to high current barrier strip 2.
For connecting LC mainMega to HC optocoupler reversing contactor inputs _and_ LC small kill relay switching contacts to HC big kill relay coil contacts.

| Position (Color) | LC description | HC description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega digital pin 22 (Q1) | optocoupler reversing contactor input Q1 |
| 2 (solid orange) | mainMega digital pin 24 (Q2) | optocoupler reversing contactor input Q2 |
| 3 (stripe green) | mainMega digital pin 26 (Q3) | optocoupler reversing contactor input Q3 |
| 4 (solid blue) | mainMega digital pin 28 (Q4) | optocoupler reversing contactor input Q4 |
| 5 (stripe blue) | mainMega digital pin 30 (revConKill) | optocoupler reversing contactor input revConKill |
| 6 (solid green) | empty | empty |
| 7 (stripe brown) | small kill relay switching contacts common | kill relay 1 and 2 coil (left) |
| 8 (solid brown) | small kill relay switching contacts normally open | kill relay 1 and 2 coil (right) |
 
## LCBS5 to HCBS3
Low current barrier strip 5 to high current barrier strip 3.
For connecting LC mainMega to HC optocoupler batteryMega output.

| Position (Color) | LC description | HC description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega pin (UNDETERMINED) | optocoupler batteryVoltage output |
| 2 (solid orange) | empty | empty |
| 3 (stripe green) | empty | empty |
| 4 (solid blue) | empty | empty |
| 5 (stripe blue) | empty | empty |
| 6 (solid green) | empty | empty |
| 7 (stripe brown) | empty | empty |
| 8 (solid brown) | empty | empty |
