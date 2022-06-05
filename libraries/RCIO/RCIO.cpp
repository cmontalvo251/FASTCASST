#include "RCIO.h"

//Constructor
RCIO::RCIO() {
  //Initialize RCInput
  in.initialize();
}

//Read receiver signals
void RCIO::read() {
  in.readRCstate();
}

//Print Receiver signals
void RCIO::printIn(int numChannels) {
  in.printRCstate(numChannels);
}

//Initialize PWM channels
void RCIO::outInit(int num) {
  out.initialize(num);
}

//Write PWM signals
void RCIO::write() {
  out.write();
}

//Print PWM Channels
void RCIO::printOut() {
  out.print();
}
