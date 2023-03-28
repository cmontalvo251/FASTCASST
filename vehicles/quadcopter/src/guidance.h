#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <MATLAB.h>
#include <RCIO/RCIO.h>
#include <Datalogger/Datalogger.h>

class guidance() {
 public:
  guidance();
  loop();
  init();
  MATLAB guidance_matrix;
};

#endif GUIDANCE
