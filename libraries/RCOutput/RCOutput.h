#ifndef RCOUTPUT_H
#define RCOUTPUT_H

#ifdef ARDUINO
#include "PWMSIGNALS.h"
#include "Datalogger.h"
#else
#include "PWMSIGNALS/PWMSIGNALS.h"
#include <unistd.h> //This is for sleep()
#include <Datalogger/Datalogger.h>
#endif
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>

#define OUTMIN 992
#define OUTMID 1500
#define OUTMAX 1950

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
