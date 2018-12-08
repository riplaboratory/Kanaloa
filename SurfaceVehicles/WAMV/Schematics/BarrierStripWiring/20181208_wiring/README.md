# Barrier Strip Wiring
This document describes the barrier strip wiring pats for the WAM-V.  All barrier strips start at the low current box and connect 1:1 to a barrier strip somewhere else in the WAM-V; therefore, all strips are listed based on their relationship with the low current box barrier strip it connects to.

## LCBS1 to WPBS2/WPBS3
Low current barrier strip 1 to wireless pole barrier strip 2/3.  For connecting LC power (GND+5V) and mainMega to the WP power and receiver/SBUS decoder.

| Position (Color) | LCBS1 description | WPBS2/3 description |
| :---: | :---: | :---: |
| 1 (stripe orange) | LC GND | LC GND (WPBS3-1) | 
| 2 (solid orange) | LC 5V | LC 5V (WPBS3-2) | 
| 3 (stripe green) | empty | empty (WPBS2-3) |
| 4 (solid blue) | empty | empty (WPBS2-4) |
| 5 (stripe blue) | empty | empty (WPBS2-5) |
| 6 (solid green) | empty | empty (WPBS2-6) |
| 7 (stripe brown) | empty | empty (WPBS2-7) |
| 8 (solid brown) | empty | empty (WPBS2-8) |
 
## LCBS2 to WPBS1
Low current barrier strip 2 (LCBS2) to wireless pole barrier strip 1 (WPBS1).  For connecting the LC mainMega to WP recevier/SBUS decoder.

| Position (Color) | LCBS2 description | WPBS1 description |
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
Low current barrier strip 3 to high current barrier strip 1.  For connecting LC mainMega to HC optocoupler motor controller inputs.

| Position (Color) | LCBS3 description | HCBS1 description |
| :---: | :---: | :---: |
| 1 (stripe orange) | LC GND (short to LCBS1-1) | optocoupler LC GND |
| 2 (solid orange) | LC 5V (short to LCBS1-2) | octocoupler LC 5V |
| 3 (stripe green) | HC GND | HC GND (from main battery GND) |
| 4 (solid blue) | HC 5V | HC 5V (from 5V regulator GND) |
| 5 (stripe blue) | mainMega PWM servo shield ch 0 signal (Q1) | optocoupler motor controller input Q1 |
| 6 (solid green) | mainMega PWM servo shield ch 1 signal (Q2) | optocoupler motor controller input Q2 |
| 7 (stripe brown) | mainMega PWM servo shield ch 2 signal (Q3) | optocoupler motor controller input Q3 |
| 8 (solid brown) | mainMega PWM servo shield ch 3 signal (Q4) | optocoupler motor controller input Q4 |

## LCBS4 to HCBS2
Low current barrier strip 4 to high current barrier strip 2.  For connecting LC mainMega to HC optocoupler reversing contactor inputs _and_ LC small kill relay switching contacts to HC big kill relay coil contacts.

| Position (Color) | LCBS4 description | HCBS2 description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega PWM servo shield ch 0 signal (Q1) | optocoupler motor controller input Q1 |
| 2 (solid orange) | mainMega PWM servo shield ch 1 signal (Q2) | optocoupler motor controller input Q2 |
| 3 (stripe green) | mainMega PWM servo shield ch 2 signal (Q3) | optocoupler motor controller input Q3 |
| 4 (solid blue) | mainMega PWM servo shield ch 3 signal (Q4) | optocoupler motor controller input Q4 |
| 5 (stripe blue) | mainMega digital pin (undetermined) | optocoupler stepper direction Q1 |
| 6 (solid green) | mainMega digital pin (undetermined) | optocoupler stepper direction Q2 |
| 7 (stripe brown) | mainMega digital pin (undetermined) | optocoupler stepper direction Q3 |
| 8 (solid brown) | mainMega digital pin (undetermined) | optocoupler stepper direction Q4 |
 
## LCBS5 to HCBS3
Currently unassigned.  LCBS5 currently exists in the low current box, but HCBS3 does not exist in the high current box.

| Position (Color) | LCBS5 description | HCBS3 description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega digital pin (undetermined) | optocoupler stepper pulse Q1 |
| 2 (solid orange) | mainMega digital pin (undetermined) | optocoupler stepper pulse Q2 |
| 3 (stripe green) | mainMega digital pin (undetermined) | optocoupler stepper pulse Q3 |
| 4 (solid blue) | mainMega digital pin (undetermined) | optocoupler stepper pulse Q4 | 
| 5 (stripe blue) | mainMega digital pin (undetermined) | optocoupler stepper enable Q1 |
| 6 (solid green) | mainMega digital pin (undetermined) | optocoupler stepper enable Q2 |
| 7 (stripe brown) | mainMega digital pin (undetermined) | optocoupler stepper enable Q3 |
| 8 (solid brown) | mainMega digital pin (undetermined) | optocoupler stepper enable Q4 |

## LCBS6 to WP light
Low current barrier strip 5 to wireless pole light.  For connecting kill switch mainMega to HC optocoupler batteryMega output.

| Position (Color) | LCBS6 description | WP light description |
| :---: | :---: | :---: |
| 1 (stripe orange) | kill switch 12V | led strip 12V (black) |
| 2 (solid orange) | empty | empty |
| 3 (stripe green) | kill switch green relay contacts | led strip green (green) |
| 4 (solid blue) | kill switch red relay contacts | led strip red (red) |
| 5 (stripe blue) | empty | empty |
| 6 (solid green) | empty | empty |
| 7 (stripe brown) | empty | empty |
| 8 (solid brown) | empty | empty |
