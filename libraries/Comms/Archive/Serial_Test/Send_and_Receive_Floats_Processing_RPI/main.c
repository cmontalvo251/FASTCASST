//////////////////////COMMON SHARED FILE///////////////////////////////////
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "Serial.h"

////////////////////////DESKTOP COMPUTER///////////////////////////////////
int main() {
  //Initialize Serial Port
  //using this baudrate of 57600
  printf("Initializing the dev Port \n");
  my = SerialInit("/dev/ttyAMA0",57600);  //This is the UART port on the RPI
  printf("Dev Port Initialized. If no errors present we are currently listening \n");

  //After we setup the while loop we need to create an infinite while loop
  //to emulate what an autopilot or robot routine would look like on an RPi
  while (1) {

    //From here we basically need to constantly read the serial port at least once
    //in the while loop and check for w\r from the computer so I'll need to write
    //set to 0 to turn echo off, 1 = all echos on, 2 = only echo if you receive anything
    int ok = SerialListen(&my,1); 

    // w\r was received it means we need to respond
    if (ok == (119+13)) { //119 is ASCII for w and 13 is ASCII for \r

      //Once we get 119 and 13 we need to tell the groundstation that we heard them
      //so we send 119,13,10 - w\r\n
      printf("w slash r received!! \n");
      SerialRespond(&my,1);
      
      //Once we've responded we must send whomever is talking to us some data
      //Create fictitious floats
      float number1 = -3.6;
      float number2 = 4.5;
      float number3 = -2.8;
      float number_array[MAXFLOATS]; //MAXFLOATS is set to 10 in Serial.h right now
      number_array[0] = number1;
      number_array[1] = number2;
      number_array[2] = number3;
      int number_of_numbers = 3;
      //Send over Serial (make sure to use the 1 hex \r format)
      SerialSendArray(&my,number_array,number_of_numbers,1); //The trailing 1 is for echo
    }
      
  } //end robot while loop
} //end main loop robot
