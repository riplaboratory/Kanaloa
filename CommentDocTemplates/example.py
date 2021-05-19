""" 
  Name: example.py
    
  Description: What does this file do?
    
  # HARDWARE - hardware the code is designed to control or use
      - PWM Servo shield
      - 2.5 V constant ouput control board. Requires 2 channels one to 
          provide forward commands and one to provide reverse commands
      - ADS1115 ADC circuit
  # LIBRARIES
      ## Library Manager (Sketch ->Include Library -> Manage Libraries)
          - "Adafruit PWM Servo Driver Library" by Adafruit
          - "Adafruit ADS1X15" by Adafruit
      ## Downloaded
          - PID_v1 by Brett Beauregard, https://github.com/br3ttb/Arduino-PID-Library/
  # INPUTS
      ## Input 1: [unit], description 
      ## Input 2: [unit], description
  # OUTPUTS
      ## Output 1: [unit], description (if function return)
      ## other output description, plots, actions, ROS topics, etc.
  
  # VERSION HISTORY
    @date     2019.09.13 
    @author   Example Author (example@hawaii.edu)
    @brief    Initial creation
  
    @date     Date file is done being modified
    @author   Modified by
    @brief    What changes were made?
              You can hard wrap the changes if the first line runs long
 
  # COPYWRITE: Created by and for Team Kanaloa. All rights reserved.
               (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
  
  # LICENSE: Internal Kanaloa use only
"""

def myExample(a, b):
    """ This function adds two numbers together

    Parameters
    ----------
    a : integer
        The first addend
    b : integer
        The second addend
    
    Returns
    -------
    sum
        The sum of the addends
    """
    sum = a + b;
    return sum;
