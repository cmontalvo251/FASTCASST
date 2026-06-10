// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
#ifndef TIMER_H
#define TIMER_H

#include <time.h>
#ifndef _WIN32
#ifndef ARDUINO
#include <unistd.h>
#endif
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

void cross_sleep(double);
void cross_sleep(double,int);
void PAUSE();
void error(char *);

class TIMER {
 private:
  double start_sec_,prevTime_;
  time_t rawtime_;
  clock_t t_;
  ////////////////////////THESE FUNCTIONS HAVE BEEN MOVED TO PRIVATE FUNCTIONS
  ////////////////////////BECAUSE UPDATETIME IS THE ONLY THING YOU NEED
  ////////////////////////ONLY USE THESE FUNCTIONS AND MOVE THEM TO PUBLIC IF
  /////////////////////////YOU KNOW WHAT YOU'RE DOING
  void getCurrentTime();
  double getSeconds();
  double getTimeElapsed(); //give time elapsed from subsequent function calls in seconds
  double getTimeSinceStart(); //gives time from when the start_sec_ was created.
  void resetStartTime(); //use this function to reset the start timer
  ///////////////////////////////////////////////////////////////////////
  struct tm* ptm_;
  #ifndef ARDUINO
  struct timespec ts;
  #else
  double ts;
  #endif
 public:
  double currentTime,elapsedTime;
  void updateTime();
  void init(double);
  void incrementTime(double);
  void printTime();
  //Constructor
  TIMER();

};

#endif
