#ifndef RCINPUT
#define RCINPUT

#define NUM_CHANNELS 8 //Moved this to here on 3/6/2019. Hope I don't break anything

//Ok so this is the lower level kernel for all things RCInput
//So if you want to add stuffs to RCInput do it here.
class RCInput {

public:
    virtual void initialize() = 0;
    virtual int read(int c) = 0;
    virtual int pollReceiver(double) = 0;
    virtual void checkLostComms(double cT) = 0;
    virtual double getLevels() = 0;
    int channel_count,NUMRX;
    int MOVELEVEL=-1; //if the sticks move more than -1 which is all the time the timer will reset
    int RESETTIME=1000; //This default says you have 1000 seconds before the transmitter will turn off
    int SIGNALSOFF = 0;
    double timestop = 0;
    double currentPWMLevel = 0;
    double RXCHANNELS[NUM_CHANNELS],RXVALS[NUM_CHANNELS],OLDRXVALS[NUM_CHANNELS];
};

#endif // RCINPUT

