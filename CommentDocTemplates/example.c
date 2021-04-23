/** 
 *  @file example.c
 *  Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
 *  LICENSE: Internal Kanaloa use only
 *  
 *  @brief What does this file do?
 *  
 *  # Hardware - hardware the code is designed to control
 *      - PWM Servo shield
 *      - 2.5 V constant ouput control board. Requires 2 channels one to 
 *          provide forward commands and one to provide reverse commands
 *      - ADS1115 ADC circuit
 *  # Libraries
 *      ## Library Manager (Sketch ->Include Library -> Manage Libraries)
 *          - "Adafruit PWM Servo Driver Library" by Adafruit
 *          - "Adafruit ADS1X15" by Adafruit
 *      ## Downloaded
 *          - PID_v1 by Brett Beauregard, https://github.com/br3ttb/Arduino-PID-Library/
 *  # INPUTS
 *      ## Input 1: [unit], description 
 *      ## Input 2: [unit], description
 *  # OUTPUTS
 *      ## Output 1: [unit], description (if function return)
 *      ## other output description, plots, actions, ROS topics, etc.
 *
 *  # Version History
 *    @date     2019.09.13 
 *    @author   Example Author (example@hawaii.edu)
 *    @brief    Initial creation
 * 
 *    @date     Date file is done being modified
 *    @author   Modified by
 *    @brief    What changes were made?
 *              You can hard wrap the changes if the first line runs long
 */

/**
 * @brief This function adds two numbers together
 * 
 * @param a first addend
 * @param b second addend
 * @return  Sum of the addends
 * SOURCE: https://www.cs.cmu.edu/~410/doc/doxygen.html
 */
int myAddition (int a, int b) {
    int result;
    result = a + b;
    return result;
}