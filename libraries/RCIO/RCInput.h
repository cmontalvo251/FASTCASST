#ifndef RCIN_H
#define RCIN_H

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <err.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <iostream>

//////////Here are the iterations

////SIMONLY - no rx at all
////SIL,HIL,AUTO - Need Receiver or Joystick
////    RPI - Receiver
////    Arduino - Receiver Ard
////    Desktop - Joystick 

///Running SIL/HIL/AUTO on RPI - Use Receiver
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
#endif
#endif

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

class RCInput {
public:
    void initialize();
    void readRCstate();
    void printRCstate(int);
    void setStick(int val);
    void setStickNeutral();
    int bit2PWM(int val);
    void mapjoy2rx();
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
};

#endif
