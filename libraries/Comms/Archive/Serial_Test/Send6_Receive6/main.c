//////////////////////COMMON SHARED FILE///////////////////////////////////
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "Serial.h"

////////////////////////DESKTOP COMPUTER///////////////////////////////////
int main() {
	//Initialize Serial Port
	//using this baudrate
	printf("Initializing the dev Port \n");
	//Make sure Arduino is on this port
	//If this doesn't work even if you are sure that you have the right port.
	//Try manually sending w/r (carriage return) via the Arduino Serial monitor
	//To be sure that Serial communication is actually working 
	my = SerialInit("/dev/ttyACM0",115200); 

	//Wait 5 seconds because for some reason the Arduino reboots when you 
	//run this code
	//https://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection/
	printf("Sleeping for 5 seconds to let the Arduino Reboot \n");
	sleep(5);

	//Send w\r to Arduino
	SerialPutHello(&my,1);

	//Consume w\r\n
	SerialGetHello(&my,1);

	//Create fictitious float
	float number1 = -3.6;
	float number2 = 4.5;
	float number3 = 2.4;
	float number4 = 9.8;
	float number5 = -2.8;
	float number6 = 0.3;
	float number_array[MAXFLOATS]; //MAXFLOATS is set to 10 in Serial.h right now
	number_array[0] = number1;
	number_array[1] = number2;
	number_array[2] = number3;
	number_array[3] = number4;
	number_array[4] = number5;
	number_array[5] = number6;
	int number_of_numbers = 6;

	//Send to Arduino
	SerialPutArray(&my,number_array,number_of_numbers);

	//Read everything (just for debugging)
	//SerialGetAll(&my);

	//Now Read from Arduino
	float rec_number_array[MAXFLOATS];
	int number_of_rec_numbers = 3;
	SerialGetArray(&my,rec_number_array,number_of_rec_numbers);

	//Extract Data
	float rec_number = 0;
	for (int i = 0;i<number_of_rec_numbers;i++) {
		rec_number = rec_number_array[i];	
		printf("Number Received = %lf \n",rec_number);
	}

} //end main loop desktop computer
