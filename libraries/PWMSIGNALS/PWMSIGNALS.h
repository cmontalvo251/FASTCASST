#ifndef _PWDSIGNALS_H_
#define _PWDSIGNALS_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#ifndef ARDUINO
#include <Util/Util.h>
#endif

class PWMSIGNALS {
public:
    PWMSIGNALS();

    bool init(unsigned int channel);
    bool enable(unsigned int channel);
    bool set_period(unsigned int channel, unsigned int freq);
    bool set_duty_cycle(unsigned int channel, float period);
};

#endif //_PWD_H_

