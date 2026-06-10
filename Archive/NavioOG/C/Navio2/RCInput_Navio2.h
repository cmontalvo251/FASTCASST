#pragma once

#include <cstddef>

///wait. what's this??? -- Turns out this header file overrides a fundemental header file
//in the Common block.
#include <Common/RCInput.h> //Head over to this header file for even more of a shit show.

//#define NUM_CHANNELS 8 -- Moved to Common block

///if you notice this line below. It says the class is RCINput_Navio2 but the actual class
//is RCInput. So. don't edit any of the variables in here.
//Go change shit in Common/RCInput.h
class RCInput_Navio2 : public RCInput
{
public:
    void initialize() override;
    int read(int ch) override;
    int pollReceiver(double) override;
    void checkLostComms(double cT) override;
    double getLevels() override;
    //Might need to move this to Common/RCInput.h
    //int channel_count override;
    RCInput_Navio2();
    ~RCInput_Navio2();

private:
    int open_channel(int ch);
    
    static const size_t CHANNEL_COUNT = NUM_CHANNELS;
    int channels[CHANNEL_COUNT];
};
