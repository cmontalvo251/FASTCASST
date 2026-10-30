#ifndef STARTRACKER_H
#define STARTRACKER_H

#include <MATLAB/MATLAB.h>

class STARTRACKER {
    private:
        int status;
    public:
        STARTRACKER();
        //~STARTRACKER();
        MATLAB ptp,ptprad;
        double roll=0,pitch=0,yaw=0;
        void loop();
};

#endif
