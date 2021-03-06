/*
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.derekmolloy.ie
 */

#include <iostream>
#include <string>
#include <unistd.h>
#include <poll.h>
#include "SimpleGPIO.h"
//using namespace std;

uint16_t MUX_SEL_PIN1 = 117;    //  Pin P9_25
uint16_t MUX_SEL_PIN2 = 115;    //  Pin P9_27

int main(int argc, char *argv[]){

	//cout << "Testing the GPIO Pins" << endl;

    gpio_export(MUX_SEL_PIN1);    // Tell OS to use this pin
    gpio_export(MUX_SEL_PIN2);
    usleep(1000*1000); // wait 1 s !!IMPORTANT!! the OS needs this time!
    gpio_set_dir(MUX_SEL_PIN1, OUTPUT_PIN);   // Set pin as output direction
	gpio_set_dir(MUX_SEL_PIN2, OUTPUT_PIN); 

	// Flash the LED 5 times
	
    while(true){
        gpio_set_value(MUX_SEL_PIN1, HIGH);
        gpio_set_value(MUX_SEL_PIN2, HIGH);
	usleep(600);
	int i = 2;
        gpio_set_value(MUX_SEL_PIN1, LOW);
        gpio_set_value(MUX_SEL_PIN2, LOW);
	usleep(600);
	i = 5;
    }
    

	gpio_unexport(MUX_SEL_PIN1);     // free pins
    gpio_unexport(MUX_SEL_PIN2);
	return 0;
}

