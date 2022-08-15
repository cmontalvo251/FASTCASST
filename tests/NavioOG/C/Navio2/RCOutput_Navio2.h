#ifndef RCOUTPUT_NAVIO2_H
#define RCOUTPUT_NAVIO2_H

#include "PWM.h"
#include <Common/RCOutput.h>
#include <unistd.h> //This is for sleep()

#define SERVO_START 1100 /*mS*/ //initialize motor pwm

class RCOutput_Navio2 : public RCOutput
{
public:
    RCOutput_Navio2();
    bool initialize(int channel) override;
    bool enable(int channel) override;
    bool set_frequency(int channel, float frequency) override;
    bool set_duty_cycle(int channel, float period) override;
    int rcoutSetup() override;
private:
    PWM pwm;
};

#endif // RCOUTPUT_NAVIO2_H
