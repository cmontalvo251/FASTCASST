#include "startracker.h"

STARTRACKER::STARTRACKER() {
    ptp.zeros(3,1,"Startracker Roll Pitch Yaw (deg)");
    ptprad.zeros(3,1,"Startracker Roll Pitch Yaw (rad)");
}

void STARTRACKER::loop() {
    ptp.set(1,1,roll);
    ptp.set(2,1,pitch);
    ptp.set(3,1,yaw);
    ptprad.overwrite(ptp);
    ptprad.mult_eq(PI/180);
}