# Barrier Strip Wiring
This document describes the barrier strip wiring pats for the WAM-V.

## Color Convention
Because all barrier strips are used to organize signal the wiring out from RJ45 ethernet cables, all barrier strips will follow the EIA/TIA T568B color standard (used in north america); i.e. from top to bottom (or left to right): 

| Position | Color |
| --- | --- |
| 1. | stripe orange |
| 2. | solid orange |
| 3. | stripe green |
| 4. | solid blue |
| 5. | stripe blue |
| 6. | solid green |
| 7. | stripe brown |
| 8. | solid brown |

## Low Current Box

### Low Current Box Barrier Strip 1
For connecting mainMega to wireless pole

 1. (stripe orange): ground
 2. (solid orange): 5V
 3. (stripe green): empty
 4. (solid blue): empty
 5. (stripe blue): empty
 6. (solid green): empty
 7. (stripe brown): empty
 8. (solid brown): empty
 
### Low Current Box Barrier Strip 2
For connecting the mainMega to wireless pole

 1. mainMega digital pin 4
 2. empty
 3. empty
 4. mainMega digital pin 5
 5. empty
 6. mainMega digital pin 6
 7. mainMega digital pin 3
 8. Low Current Box Barrier Strip 3 position 3 (solid blue)
 
### Low Current Box Barrier Strip 3
For connecting the mainMega to the kill switch system in the high current box

 1. Low Current Box Barrier Strip 1 position 1 (stripe orange)
 2. mainMega digital pin 10
 3. empty
 4. empty
 5. empty
 6. empty
 7. empty
 8. empty

### Low Current Box Barrier Strip 4
For connecting mainMega to reversing contactor system in high current vox

 1. mainMega digital pin 14
 2. mainMega digital pin 15
 3. mainMega digital pin 16
 4. mainMega digital pin 17
 5. mainMega digital pin 18
 6. empty
 7. empty
 8. empty
 
### Low Current Box Barrier Strip 5
For connecting mainMega to DACs in high current box

 1. mainMega 5V
 2. empty
 3. mainMega SDA pin
 4. mainMega SCL pin
 5. voltage divider red (main battery HOT)
 6. empty
 7. empty
 8. empty
 
 ## Wireless Pole
 
### Wireless Pole Barrier Strip 1
For connecting SBUS decoder to mainMega in low current box

 1. SBUS decoder channel 1
 2. empty
 3. empty
 4. SBUS decoder channel 4
 5. SBUS decoder channel 5
 6. SBUS decoder channel 6
 7. SBUS decoder channel 7
 8. SBUS decoder channel 8

### Wireless Pole Barrier Strip 2
For connecting SBUS decoder to mainMega in low current box

 1. ground
 2. 5V
 3. empty
 4. empty
 5. empty
 6. empty
 7. empty
 8. empty
 

