// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner

#include "timer.h"
#include "time.h"

#ifdef _WIN32
#include "windows.h"
#endif

#ifdef ARDUINO
#include <Arduino.h>
#endif

void error(char *output)
{
  printf("Error -> %s \n",output);
  printf("Program Quit \n");
  exit(1);
}

void PAUSE()
{
  printf("%s","Function Paused\n");
  printf("Type in any number(1-inf) and press enter to continue\n");
  int  a;
  scanf("%i",&a);
}

void cross_sleep(double length) {
  #ifdef _WIN32
  //Code for Windows
  Sleep(length*1000);
  #else
  //Not on windows
  #ifdef ARDUINO
  //If on Arduino
  delay(length*1000);
  #else
  //Otherwise you're probably on linux or unix
  //#ifdef __unix__
  //sleep(length*1000000); 
  //#else
  usleep(length*1000000);
  //#endif
  #endif //ARDUINO
  #endif //_WIN32
}

void cross_sleep(double length,int val) {
  if (val == 6) {
    //This means you want to run usleep at the actual value
    //The problem is that usleep only works on linux computers
    #ifdef ARDUINO
    cross_sleep((length*10e-6)/1000.0);
    #else
    usleep(length);
    #endif
  } else {
    cross_sleep(length);
  }
}

TIMER::TIMER() {
  //Initialize timer
  resetStartTime();
}

void TIMER::resetStartTime() {
  getCurrentTime();
  start_sec_ = getSeconds();
  prevTime_ = 0.0;
}

double TIMER::getSeconds() {
  //http://linux.die.net/man/3/clock_gettime
  #ifndef ARDUINO
  double sec = ptm_->tm_sec + (ptm_->tm_min)*60 + ((ptm_->tm_hour-4)%24)*3600 + (ptm_->tm_mday)*24*3600 + (double)ts.tv_nsec*pow(10,-9);
  #else
  double sec = ts;  
  #endif
  //double sec = 20*((double)t_)/CLOCKS_PER_SEC;
  //printf("Current Time = %lf \n",start_sec_);
  return sec;
}

void TIMER::getCurrentTime() {
  //t_ = clock();
  #ifdef _WIN32
  time(&rawtime_);
  ptm_ = gmtime(&rawtime_);
  #else
  #ifdef ARDUINO
  ts = millis()/1000.0;
  #else
  clock_gettime(CLOCK_MONOTONIC,&ts);
  ptm_ = gmtime(&ts.tv_sec);
  #endif
  #endif
}

double TIMER::getTimeElapsed() {
  double last_sec,end_sec;
  //Compute Time from last call
  last_sec = getSeconds();
  //printf("Last Call to getSeconds = %lf \n",last_sec);
  //Get Current Time
  getCurrentTime();
  //Compute Current Time in Seconds
  end_sec = getSeconds();

  return (end_sec-last_sec);
}

double TIMER::getTimeSinceStart() {
  double end_sec;
  getCurrentTime();
  end_sec = getSeconds();
  return (end_sec-start_sec_);
}

void TIMER::printTime() {
  printf("%lf ",currentTime);
}

void TIMER::updateTime() {
  //Get Current Time
  getCurrentTime();
  //Compute Current Time in Seconds
  currentTime = getSeconds()-start_sec_;
  //Compute Elapsed Time 
  elapsedTime = currentTime - prevTime_;
  //Reset prevTime
  prevTime_ = currentTime;
}

void TIMER::incrementTime(double TIMESTEP) {
  currentTime += TIMESTEP;
  //Compute Elapsed Time 
  elapsedTime = currentTime - prevTime_;
  //Reset prevTime
  prevTime_ = currentTime;
}

void TIMER::init(double time) {
  resetStartTime();
  currentTime = time;
  elapsedTime = 0;
  prevTime_ = 0;
}
