#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdio.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#ifdef RPI
//sudo apt-get install wiringpi
#include <wiringPi.h>
#include <wiringSerial.h>
//to compile run
//g++ -o run.exe Telemetry.cpp -lwiringPi (this is already in the makefile)
#endif

//This is needed to convert from floats to longs
#ifndef INPARSER_H
#define INPARSER_H
union inparser {
	long inversion;
	float floatversion;
};
#endif

#define MAXFLOATS 10
#define MAXLINE 120

class Telemetry {
    private:
    	int hComm;
	public:
		float lastTime = 0.0;
		float period = 1.0; //Default is second
		///THE FORMAT FOR SAYING HELLO AND RESPONDING AS WELL AS PUTTING 
		//AN ARRAY AND READING AN ARRAY REALLY NEEDS TO BE THE SAME
		//THERE IS NO REASON REALLY TO HAVE THEM BE DIFFERENT.
		//And well unfortunately since sending is w\r and responding is w\r\n
		//we have multiple routines for sending and receiving.
		//In addition, the desktop software sends 3 hex numbers followed by \r
		//while the board sends 1 hex number followed by a \r. This software was also written
		//with cross platform in mind and unfortunately this means that WIN32, Linux, RPI and Arduino
		//all have different methods. Again my long term goal is to have the same sending and
		//receive format for both board and desktop but since this Serial library is used in multiple
		//repos I'm leaving all of these routines for backwards compatibility knowing full well that this is
		//absurdly inefficient and utterly confusing.

		//Serial Functions

		////////////SERIAL INITIALIZATION//////////////
		//Use this if you want to use default ComPortName and BaudRate
		void InitSerialPort(void);
		//Use this if you want to specify ComPortName and BaudRate
		void SerialInit(char *ComPortName, int BaudRate);
		///Get and Receive 1 character
		char SerialGetc();
		void SerialPutc(char txchar);
		
		//This sends a string rather than 1 character
		void SerialPutString(char *string);
		//this is an overloaded function that uses a for loop instead of a while loop
		void SerialPutString(char *string,int len); 

		//This routine reads 1 character and prints to the screen what you received
		void SerialDebug();
		//This routine reads everything from serial and prints it to screen. not just 1 character
		void SerialGetAll();

		//This is where things get confusing
		///Let's assume you have a drone that is constantly listening to a ground station and then
		//responding to the groundstation if it receives a "hello" (w\r)
		//In this case SerialListen is listening for w\r
		int SerialListen(int echo);
		int SerialListen();
		//Serial respond then sends hello, sir (w\r\n)
		void SerialRespond(int echo);
		void SerialRespond();

		//Now let's assume you are the ground station and you are saying hello and then
		//listening for the response from the drne.
		//In this case you need SerialPutHello (w\r)
		void SerialPutHello(int echo);
		void SerialPutHello();
		//and then get the response from the drone or whatever (w\r\n)
		int SerialGetHello(int echo);
		//So basically SerialPutHello and SerialRepond are different because SerialPutHello send w\r
		//and SerialRespond sends w\r\n which is fucking annoying because if we just sent w\r both ways
		//we wouldn't need both of these functions

		////Ok then SerialPutArray sends an entire array to a board but uses the format
		//put 3 Hex numbers followed by a \r
		void SerialPutArray(float array[],int num);
		void SerialPutArray(float array[],int num,int echo);
		//SerialGetArray though assumes the numbers are being received in the format 1 hex number \r
		void SerialGetArray(float array[],int num);
		void SerialGetArray(float array[],int num,int echo);
		//Again this is so fucking annoying because since we have 3 hex \r and 1 hex \r we now need
		//different routines for a drone

		//So this routine here sends an array in the 1 Hex \r format
		void SerialSendArray(float array[],int num);
		void SerialSendArray(float array[],int num,int echo);
		//You might be wondering where the read 3 hex \r format function and well I've never needed it
		//Right now the MultiSAT++/HIL simulation uses SerialPutArray to send 3 Hex \r to an Arduino.
		//The SerialReadArray which read 3 Hex \r is actually currently in a *.ino function. Ugh.
		//The Arduino then sends 1 hex \r using it's own Arduino functions and of course we then just
		//use SerialGetArray to decode 1 hex \r format. Big sigh. Cmontalvo 10/13/2020
};


#endif
