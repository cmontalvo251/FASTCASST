#ifndef AIRSPEED_H
#define AIRSPEED_H

#include "ADC_Navio2.h" //for the analog_converter
#include <MATLAB/MATLAB.h> //for the ADCVEC
#include <sys/time.h>
#include <stdlib.h>
#include <ctime> ///Btw. I really just need sleep().
#include <memory> //I just don't know which one its' in
#include <cstdio>
#include <Common/Util.h>
#include <stdint.h>
#include <unistd.h>

#define PITOTPERIOD 0.1
#define Pascals 101.325
#define gamma 1.4
#define R 287.0
#define READ_FAILED -1
#define NAVERAGES 100
#define MAXPORTS 10

class Airspeed {
 public:
  MATLAB ADCVEC[MAXPORTS]; //These will be the vectors that hold information
  ADC *analog_converter;
  float windspeed[MAXPORTS];
  float windspeed_filtered[MAXPORTS];
  float windspeed_average;
  float adc_result[MAXPORTS];
  float a_inf,B,T_avg;
  float V_avg[MAXPORTS];
  int ctr[MAXPORTS];
  int channel[MAXPORTS];
  int numADCs=0;
  unsigned long lastTime = 0;
  Airspeed(); //contructor -- the input is the channel
  void setup(int*,int,float);
  void ComputeWindSpeed(int i);
  void FilterWindSpeed(int i);
  void readADC(int i);
  void computeADCAverages();
  void poll(int fileopen);
};

#endif
