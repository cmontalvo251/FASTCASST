#ifndef _RGBLED_H_
#define _RGBLED_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdint>
#include <GPIO/gpio.h>
#include <LED/Led.h>

class RGBled {
public:
    RGBled();

    bool initialize();
    void setColor(Colors color);

private:
    Navio::Pin *pinR;
    Navio::Pin *pinG;
    Navio::Pin *pinB;
};

#endif //_RGBLED_H_
