#include "../../libraries/Timer/timer.h"

TIMER watch;


int main () {
	watch.init(0);
	watch.updateTime();
	printf("Time = %lf \n",watch.currentTime);
	cross_sleep(1.0);
	watch.updateTime();
	printf("Time + Elapsed %lf %lf \n",watch.currentTime,watch.elapsedTime);
	exit(1);
}