#ifndef _PWDSIGNALS_H_
#define _PWDSIGNALS_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#ifdef ARDUINO
#include <Servo.h>
#define RECV_N_CHANNEL 	6
#else
#include <Util/Util.h>
#endif

class PWMSIGNALS {
public:
    PWMSIGNALS();

    bool init(unsigned int channel);
    bool enable(unsigned int channel);
    bool set_period(unsigned int channel, unsigned int freq);
    bool set_duty_cycle(unsigned int channel, float period);
 private:
    #ifdef ARDUINO
    Servo outservo[RECV_N_CHANNEL];
    #endif

};

#endif //_PWD_H_

