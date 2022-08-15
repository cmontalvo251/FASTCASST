#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <err.h>

#include "RCInput_Navio2.h" //this adds the Navio2 header file bit it also adds the Common header file as well
#include <Common/Util.h>

#define RCIN_SYSFS_PATH "/sys/kernel/rcio/rcin"


//constructor class
RCInput_Navio2::RCInput_Navio2()
{
  //Run the initialize routine
  initialize();
  //set the channel count
  channel_count = NUM_CHANNELS;
  printf("There are %d channels \n",channel_count);

  for (int idx = 0;idx<NUM_CHANNELS;idx++){
    OLDRXVALS[idx] = 0;
  }
}
//empty destructor class
RCInput_Navio2::~RCInput_Navio2()
{
}
//Here's the initialize routine
void RCInput_Navio2::initialize()
{
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        channels[i] = open_channel(i);
        if (channels[i] < 0) {
	  printf("Error opening a channel \n");
            perror("open");
        }
    }
}

//I wrote this so that I could poll the receiver on any plane car or rover
//Montalvo 3/5/2019
int RCInput_Navio2::pollReceiver(double currentTime)
{
  double signal;
  if (SIGNALSOFF == 0) {
    //printf("Checking Receiver %lf \n",currentTime);
    for (int idx = 0;idx<NUMRX;idx++) {
      signal = read(RXCHANNELS[idx]);
      RXVALS[idx] = signal;
    }
  } else {
    printf("Motors off for good \n");
  }
  //It's possible that the transmitter has gone out in which case when we poll the reciever the signals
  //are constant. To check for this you can run a routine called checkLostComms
  checkLostComms(currentTime); 
  return SIGNALSOFF;
}

double RCInput_Navio2::getLevels() {
  double level = 0;
  for (int idx = 0;idx<NUMRX;idx++){
    level += abs(OLDRXVALS[idx] - RXVALS[idx]);
    OLDRXVALS[idx] = RXVALS[idx]; 
  }
  return level;
}

void RCInput_Navio2::checkLostComms(double currentTime) {
  //printf("Checking Comms \n");
  //This is the nth time we've entered this loop
  double level = getLevels();
  //printf("checkComms %lf %lf (%lf) %d \n",currentTime,timestop,level,SIGNALSOFF);
  if (level > MOVELEVEL) {
    //We've wiggled the sticks enough to prove that the receiver is still on
    //or there is enough noise in the transmitter to prove it's still on
    //So reset the timer
    timestop = currentTime;
  } else {
    //This means we haven't wiggled the sticks enough which means we're sitting on the ground
    //or we've lost comms or you didn't allow for enough noise
    if (currentTime - timestop > RESETTIME) {
      //printf("RESET TIME = %d \n",RESETTIME);
      //printf("Sending shutdown signal \n");
      //So set all the RXVALS to zero
      SIGNALSOFF = 1;
      for (int idx = 0;idx<NUMRX;idx++){
	RXVALS[idx] = 0;
      }
    }
  }
}

//This is where we read the channel
int RCInput_Navio2::read(int ch)
{
    if (ch > ARRAY_SIZE(channels) )
	{	
        fprintf(stderr,"Channel number too large\n");
        return -1;
	}

    char buffer[10];
    //so this is some fancy bit shit bullcrap that only computer scientist know how this works
    if (::pread(channels[ch], buffer, ARRAY_SIZE(buffer), 0) < 0) {
        perror("pread");
    }
    //this function atoi converts a char to a integer
    return atoi(buffer);
}

int RCInput_Navio2::open_channel(int channel)
{
    char *channel_path;
    if (asprintf(&channel_path, "%s/ch%d", RCIN_SYSFS_PATH, channel) == -1) {
        err(1, "channel: %d\n", channel);
    }

    int fd = ::open(channel_path, O_RDONLY);

    free(channel_path);

    return fd;
}

