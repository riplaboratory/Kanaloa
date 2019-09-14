/*
 *  This program is to test how to convert bytes to numbers. 
 *  
 *  Created by Kai Jones
 *   April 8, 2019
 * 
*/
#include <iostream>
#include <math.h>

int bytes[10] = {1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
int ans = 0; 
int j = 0; 

int main(void){
    for(int i=; i>=0; i--){
        if(bytes[i] > 0){
            ans = ans + pow(2,j);
        }
     j++;
    }

    std::cout << ans << std::endl;

    return 0; 
}