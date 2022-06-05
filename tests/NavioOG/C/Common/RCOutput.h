#ifndef RCOUTPUT
#define RCOUTPUT

class RCOutput
{
public:
    virtual bool initialize(int channel) = 0;
    virtual bool enable(int channel) = 0;
    virtual bool set_frequency(int channel, float frequency) = 0;
    virtual bool set_duty_cycle(int channel, float period) = 0;
    virtual int rcoutSetup() = 0;
    int num_motors;
    double PWMOUTPUTS[9];
};

#endif // RCOUTPUT

