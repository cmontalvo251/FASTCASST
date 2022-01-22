#ifndef RCOUTPUT_H
#define RCOUTPUT_H

#include "PWM.h"
#include <unistd.h> //This is for sleep()
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>

#define OUTMIN 1100
#define OUTMAX 2000

class RCOutput {
public:
    RCOutput(); //constructor
    void initialize(int);
    void saturation_block();
    void write();
    int *pwm_array=NULL;
    int NUMSIGNALS;
    void print();
private:
    PWM pwm;
};

#endif // RCOUTPUT_H
