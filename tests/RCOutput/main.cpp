#include <RCIO/RCOutput.h>

RCOutput rcout; //this calls the constructor which does nothing

int main(int argc,char* argv[]) {

  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    exit(1);
  }
  
  //You then need to initialize the pwm ports 
  rcout.initialize(4); //The default is 9 pwm signals
  
  int output = OUTMIN;
  int i = 0;

  //SEND OUTMIN AND WAIT LIKE 10 SECONDS TO MAKE SURE EVERYTHING IS INITIALIZED
  for (int j = 0;j<rcout.NUMSIGNALS;j++) {
    rcout.pwmcomms[j] = output;
    printf("%d ",output);
  }
  printf("\n");
  rcout.write();
  usleep(3*1000000);
  
  while (1) {
    printf("i = %d ",i);
    //Add 100 to signal
    output += 100;
    if (output > OUTMAX) {
      output = OUTMIN;
    }
    //Set every value in matrix to output
    for (int j = 0;j<rcout.NUMSIGNALS;j++) {
      rcout.pwmcomms[j] = output;
      printf("%d ",output);
    }
    rcout.write();
    printf("\n");
    i++;
    usleep(1000000);
  }
  return 0;
};
