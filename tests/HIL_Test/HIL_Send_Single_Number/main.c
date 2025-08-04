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
	printf("Sending w slash r \n");
	SerialPutc(&my,'w');
	SerialPutc(&my,'\r');
	printf("Sent \n");

	//Consume w\r\n
	printf("Reading the Serial Buffer for w slash r slash n \n");
	char inchar = '\r';
	for (int i = 0;i<3;i++) {
	  inchar = SerialGetc(&my);
	  printf("%d \n",int(inchar));
	}

	//Create fictitious float
	float number = -3.6;
	float number_array[MAXFLOATS];
	number_array[0] = number;
	int number_of_numbers = 1;

	//Send to Arduino
	SerialPutArray(&my,number_array,number_of_numbers);

	//Now Read from Arduino
	SerialGetArray(&my,number_array,number_of_numbers);

	//Extract Data
	float rec_number = number_array[0];

	printf("Number Received = %lf \n",rec_number);

} //end main loop desktop computer
