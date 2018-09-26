# Barrier Strip Wiring
This document describes the barrier strip wiring pats for the WAM-V.

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

## Low Current Box

### Low Current Box Barrier Strip 1
For connecting mainMega to wireless pole

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | PRIMARY GND (any component that needs ground should ground here, and only here) |
| 2 (solid orange) | 5V (from voltage regulator) |
| 3 (stripe green) | empty |
| 4 (solid blue) | empty |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |
 
### Low Current Box Barrier Strip 2
For connecting the mainMega to wireless pole

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | mainMega digital pin 4 |
| 2 (solid orange) | empty |
| 3 (stripe green) | empty |
| 4 (solid blue) | mainMega digital pin 5 |
| 5 (stripe blue) | empty |
| 6 (solid green) | mainMega digital pin 6 |
| 7 (stripe brown) | mainMega digital pin 3 |
| 8 (solid brown) | Low current box barrier strip 3, position 3 |
 
### Low Current Box Barrier Strip 3
For connecting the mainMega to the kill switch system in the high current box

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | GND (from low current box barrier strip 1, position 1) |
| 2 (solid orange) | mainMega digital pin 10 |
| 3 (stripe green) | empty |
| 4 (solid blue) | empty |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |

### Low Current Box Barrier Strip 4
For connecting mainMega to reversing contactor system in high current vox

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | mainMega digital pin 14 |
| 2 (solid orange) | mainMega digital pin 15 |
| 3 (stripe green) | mainMega digital pin 16 |
| 4 (solid blue) | mainMega digital pin 17 |
| 5 (stripe blue) | mainMega digital pin 18 |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |
 
### Low Current Box Barrier Strip 5
For connecting mainMega to DACs in high current box


| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | mainMega 5V |
| 2 (solid orange) | empty |
| 3 (stripe green) | mainMega SDA pin |
| 4 (solid blue) | mainMega SCL pin |
| 5 (stripe blue) | voltage divider red (main battery HOT) |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |
 
 ## Wireless Pole
 
### Wireless Pole Barrier Strip 1
For connecting SBUS decoder to mainMega in low current box

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

### Wireless Pole Barrier Strip 2
For connecting SBUS decoder to mainMega in low current box

| Position (Color) | Description |
| :---: | :---: |
| 1 (stripe orange) | ground |
| 2 (solid orange) | 5V |
| 3 (stripe green) | empty |
| 4 (solid blue) | empty |
| 5 (stripe blue) | empty |
| 6 (solid green) | empty |
| 7 (stripe brown) | empty |
| 8 (solid brown) | empty |
