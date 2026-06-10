#ifndef ADC_H
#define ADC_H

#include <cstddef>
#include <MATLAB/MATLAB.h>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <err.h>
#include <Util/Util.h>
#define ADC_SYSFS_PATH "/sys/kernel/rcio/adc"

class ADC
{
	private:
	    void initialize();
	    int get_channel_count(void);
	    int read(int ch);
        int open_channel(int ch);
	    static const size_t CHANNEL_COUNT = 6;
	    int channels[CHANNEL_COUNT]={-1};
	public:
		//Constructor
		ADC(); 
		void get_results();
		void print_results();
		int channel_count;
		MATLAB results;
};

#endif