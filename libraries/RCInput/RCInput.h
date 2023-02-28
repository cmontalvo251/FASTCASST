#ifndef RCIN_H
#define RCIN_H

#ifndef ARDUINO
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <err.h>
#include <sys/ioctl.h>
#include <iostream>
#else
#include <Arduino.h>
#include <stdio.h>
#include "PWMSIGNALS.h"
#endif
#include <stdlib.h>

//////////Here are the iterations

////SIMONLY - no rx at all
////SIL,HIL,AUTO - Need Receiver or Joystick
////    RPI - Receiver
////    Arduino - Receiver Ard
////    Desktop - Joystick 

///Running SIL/HIL/AUTO on RPI - Use Receiver
//You don't need these hooks or Arduino
#ifndef ARDUINO
#if defined (SIL) || (HIL) || (AUTO)
#ifdef RPI
#define RECEIVER
#endif
#endif

//?Running in Realtime on Desktop
//If HIL - RPI handles comms
#if defined (SIL) && (DESKTOP)
#ifndef KEYBOARD
#define JOYSTICK
#endif //KEYBOARD
#endif //SIL DESKTOP
#endif //ARDUINO

#ifdef RECEIVER
//Using a receiver on the Raspberry Pi
#include <Util/Util.h>
#endif

#ifdef JOYSTICK
#include <linux/joystick.h>
#endif

//Leaving these defines here just in case. Some are for RPi and some are for joystick
#define RCIN_SYSFS_PATH "/sys/kernel/rcio/rcin"
#define JOY_DEV "/dev/input/js0"
#define NAME_LENGTH 80

///DEFAULT STICK_MAX AND MIN
#define STICK_MAX 2016. //uS
#define STICK_MIN 992. //uS
#define STICK_MID 1500. //uS
#define IDLE 1200. //uS

#ifdef RCTECH
#define BIT_RANGE 16000
#else
#define BIT_RANGE 32768
#endif

#ifdef ARDUINO
#define RECV_CHAN0PIN 	2
#define RECV_CHAN1PIN 	3
#define RECV_CHAN2PIN 	4
#define RECV_CHAN3PIN 	5
#define RECV_CHAN4PIN 	6
#define RECV_CHAN5PIN 	7
static volatile int timeLastChange[RECV_N_CHANNEL];
static volatile int rx_array_static[RECV_N_CHANNEL];
#endif

class RCInput {
public:
  void initialize();
  void readRCstate();
  void printRCstate(int);
  void setStick(int val);
  void setStickNeutral();
  int bit2PWM(int val);
  void mapjoy2rx();
  void saturation_block();
  void RangeCheck();
  int invert(int);
  //Constructor
  RCInput();
  //Destructor
  ~RCInput();
  int joy_fd=-1,*joycomm=NULL,*rx_array=NULL,*axis_id=NULL,num_of_axis=0,num_of_buttons=0,x;
  char *button = NULL,name_of_joystick[NAME_LENGTH];
  double keyboard[8]; //do max 8 just in case
  #ifdef JOYSTICK
  struct js_event js;
  #endif
private:
  int open_axis(int ch);
  int read_axis(int ch);
  void LostCommCheck();
  // interrupt handlers
  static void ch0Handler();
  void ch1Handler();
  void ch2Handler();
  void ch3Handler();
  void ch4Handler();
  void ch5Handler();
  static void pwmHandler(int chann, int pin);
  static int getRXvalue(int chann);
};

#endif
