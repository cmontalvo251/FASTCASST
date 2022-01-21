#include "sensors.h"

//Constructor
sensors::sensors() {
}

//Polling routine to read all sensors
void sensors::poll(double currentTime) {
  //First Analog to Digital Converter
  analog.get_results();
  
  //Then we poll the barometer and temperature sensor (needs current time)
  atm.poll(currentTime);
}

//Printing Routine
void sensors::print() {
  //Analog signals
  analog.print_results();
  //Barometer and Temperature
  printf("%lf %lf %lf ",atm.pressure,atm.altitude,atm.temperature);
}
