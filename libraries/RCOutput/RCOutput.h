#ifndef RCOUTPUT_H
#define RCOUTPUT_H

#ifdef ARDUINO
#include "PWMSIGNALS.h"
#else
#include "PWMSIGNALS/PWMSIGNALS.h"
#include <unistd.h> //This is for sleep()
#endif
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>

#define OUTMIN 992
#define OUTMID 1500
#define OUTMAX 2000

class RCOutput {
public:
    RCOutput(); //constructor
    void initialize(int);
    int RangeCheck();
    void setOutputNeutral();
    void saturation_block();
    void write();
    int *pwm_array=NULL;
    int *pwm_array_prev=NULL;
    int NUMSIGNALS;
    void print();
    void backup();
    void revert();
private:
    PWMSIGNALS pwmsignals;
};

#endif // RCOUTPUT_H