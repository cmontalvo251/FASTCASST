// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner

#include "timer.h"
#include "time.h"

#ifdef _WIN32
#include "windows.h"
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
  Sleep(length*1000);
  #else
  usleep(length*1000000);
  #endif
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
  double sec = ptm_->tm_sec + (ptm_->tm_min)*60 + ((ptm_->tm_hour-4)%24)*3600 + (ptm_->tm_mday)*24*3600 + (double)ts.tv_nsec*pow(10,-9);
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
  clock_gettime(CLOCK_MONOTONIC,&ts);
  ptm_ = gmtime(&ts.tv_sec);
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