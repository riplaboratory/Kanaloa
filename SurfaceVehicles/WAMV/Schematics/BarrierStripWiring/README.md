# Barrier Strip Wiring
This document describes the barrier strip wiring pats for the WAM-V.  All barrier strips start at the low current box and connect 1:1 to a barrier strip somewhere else in the WAM-V; therefore, all strips are listed based on their relationship with the low current box barrier strip it starts with.

## Color Convention
Because all barrier strips are used to organize signal the wiring out from RJ45 ethernet cables, all barrier strips will follow the EIA/TIA T568B color standard (used in north america); i.e. from top to bottom (or left to right): 

| Position | Color |
| :---: | :---: |
| 1 | stripe orange |
| 2 | solid orange |
| 3 | stripe green |
| 4 | solid blue |
| 5 | stripe blue |
| 6 | solid green |
| 7 | stripe brown |
| 8 | solid brown |

## Low current barrier strip 1 (LCBS1) to wireless pole barrier strip 2 and 3 (WPBS2 and WPBS3)
For hosting the primary low current box ground location, and connecting the mainMega to the wireless pole (WPBS2 and WPBS3).

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
 
## Low current barrier strip 2 (LCBS2) to wireless pole barrier strip 1 (WPBS1)
For connecting the LC mainMega to WP recevier/SBUS decoder.

| Position (Color) | LC description | WP description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega digital pin 18 | SBUS decoder channel 1 |
| 2 (solid orange) | empty |
| 3 (stripe green) | empty |
| 4 (solid blue) | mainMega digital pin 19 | SBUS decoder channel 4 |
| 5 (stripe blue) | mainMega digital pin 20 | SBUS decoder channel 5 |
| 6 (solid green) | mainMega digital pin 21 | SBUS decoder channel 6 |
| 7 (stripe brown) | mainMega digital pin 3 | SBUS decoder channel 7 |
| 8 (solid brown) | mainMega digital pin 2 | SBUS decoder channel 8 |
 
## Low current barrier strip 3 (LCBS3) to high current barrier strip 1 (HCBS1)
For connecting LC mainMega to HC optocoupler motor controller inputs.

| Position (Color) | LC Description | HC description |
| :---: | :---: | :---: |
| 1 (stripe orange) | mainMega PWM servo sheild ch 0 signal (Q1) |
| 2 (solid orange) | mainMega PWM servo sheild ch 1 signal (Q2) |
| 3 (stripe green) | mainMega PWM servo sheild ch 2 signal (Q3) |
| 4 (solid blue) | mainMega PWM servo sheild ch 3 signal (Q4) |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

## High Current Box Barrier Strip 1 (HCBS1)
For connecting optocoupler motor controller inputs to low current box LCBS3 (mainMega).

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | optocoupler motor controller input Q1 |
| 2 (solid orange) | optocoupler motor controller input Q2 |
| 3 (stripe green) | optocoupler motor controller input Q3 |
| 4 (solid blue) | optocoupler motor controller input Q4 |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

## Low Current Box Barrier Strip 4 (LCBS4)
For connecting mainMega and small kill relay to high current box HCBS2 (optocoupler reversing contactor inputs and big kill relay coil).

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | mainMega digital pin 22 (Q1) |
| 2 (solid orange) | mainMega digital pin 24 (Q2) |
| 3 (stripe green) | mainMega digital pin 26 (Q3) |
| 4 (solid blue) | mainMega digital pin 28 (Q4) |
| 5 (stripe blue) | mainMega digital pin 30 (revConKill) |
| 6 (solid green) | empty |
| 7 (stripe brown) | small kill relay switching contacts common |
| 8 (solid brown) | small kill relay switching contacts normally open |
 
## Low Current Box Barrier Strip 5 (LCBS5)
For connecting mainMega to batteryMega in high current box HCBS3 (for reading main battery voltage). 

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | empty |
| 2 (solid orange) | empty |
| 3 (stripe green) | empty |
| 4 (solid blue) | empty |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

## High Current Box Barrier Strip 1 (HCBS1)
For connecting optocoupler motor controller inputs to low current box LCBS3 (mainMega).

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | optocoupler motor controller input Q1 |
| 2 (solid orange) | optocoupler motor controller input Q2 |
| 3 (stripe green) | optocoupler motor controller input Q3 |
| 4 (solid blue) | optocoupler motor controller input Q4 |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

## High Current Box Barrier Strip 2 (HCBS2)
For connecting optocoupler reversing contactor inputs and big kill relay coil to low current box LCBS4 (mainMega and small kill relay)

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | optocoupler reversing contactor input Q1 |
| 2 (solid orange) | optocoupler reversing contactor input Q2 |
| 3 (stripe green) | optocoupler reversing contactor input Q3 |
| 4 (solid blue) | optocoupler reversing contactor input Q4 |
| 5 (stripe blue) | optocoupler reversing contactor input revConKill |
| 6 (solid green) | empty |
| 7 (stripe brown) | kill relay 1 and 2 coil (left) |
| 8 (solid brown) | kill relay 1 and 2 coil (right) |

## High Current Box Barrier Strip 3 (HCBS3)
For connecting batteryMega to mainMega in low current box LCBS5. 

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | empty |
| 2 (solid orange) | empty |
| 3 (stripe green) | empty |
| 4 (solid blue) | empty |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

## Wireless Pole Barrier Strip 1 (WPBS1)
For connecting recevier/SBUS decoder to mainMega in low current box (LCBS1)

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | SBUS decoder channel 1 |
| 2 (solid orange) | empty |
| 3 (stripe green) | empty |
| 4 (solid blue) | SBUS decoder channel 4 |
| 5 (stripe blue) | SBUS decoder channel 5 |
| 6 (solid green) | SBUS decoder channel 6 |
| 7 (stripe brown) | SBUS decoder channel 7 |
| 8 (solid brown) | SBUS decoder channel 8 |

## Wireless Pole Barrier Strip 2 (WPBS2)
For connecting recevier/SBUS decoder to mainMega in low current box (LCBS2)

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | empty |
| 2 (solid orange) | empty |
| 3 (stripe green) | empty |
| 4 (solid blue) | empty |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

## Wireless Pole Barrier Strip 3 (WPBS3)
For providing GND and 5V from low current box to wireless pole

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | GND |
| 2 (solid orange) | 5V |
