#include "PWMSIGNALS.h"

PWMSIGNALS::PWMSIGNALS()
{
}

bool PWMSIGNALS::init(unsigned int channel)
{
    int err;
    #ifdef ARDUINO
    outservo[channel].attach(13-channel);
    return true;
    #else
    err = write_file("/sys/class/pwm/pwmchip0/export", "%u", channel);
    if (err >= 0 || err == -EBUSY)
    {
        return true;
    }
    else 
    {
        printf("Can't init channel %u\n", channel);
        return false;
    }
    return true;
    #endif
}

#ifndef ARDUINO
bool PWMSIGNALS::enable(unsigned int channel)
{
    char path[60] = "/sys/class/pwm/pwmchip0";
    char path_ch[20];
    sprintf(path_ch, "/pwm%u/enable", channel);
    strcat(path, path_ch);
    
    if (write_file(path, "1") < 0)
    {
        printf("Can't enable channel %u\n", channel);
        return false;
    }
    return true;
}

bool PWMSIGNALS::set_period(unsigned int channel, unsigned int freq)
{
    int period_ns;
    char path[60] = "/sys/class/pwm/pwmchip0";
    char path_ch[20];
    sprintf(path_ch, "/pwm%u/period", channel);
    strcat(path, path_ch);

    period_ns = 1e9 / freq;
    if (write_file(path, "%u", period_ns) < 0)
    {
        printf("Can't set period to channel %u\n", channel);
        return false;
    }
    return true;
}
#endif

bool PWMSIGNALS::set_duty_cycle(unsigned int channel, float period)
{
  #ifdef ARDUINO
  outservo[channel].writeMicroseconds(period*1000);
  return true;
  #else
  int period_ns;
  char path[60] = "/sys/class/pwm/pwmchip0";
  char path_ch[20];
  sprintf(path_ch, "/pwm%u/duty_cycle", channel);
  strcat(path, path_ch);
  
  period_ns = period * 1e6;
  if (write_file(path, "%u", period_ns) < 0)
    {
      printf("Can't set duty cycle to channel %u\n", channel);
      return false;
    }
  return true;
  #endif
}
