//////////////////////COMMON SHARED FILE///////////////////////////////////
#include <stdlib.h>
#include <iostream>
#include "Serial.h"

////////////////////////DESKTOP COMPUTER///////////////////////////////////
int main(int argc, char* argv[]) {
	//Initialize Serial Port
	//Make sure Arduino is on this port and 
	//using this baudrate
	//my = SerialInit("/dev/ttyACM0",115200); 
	my = SerialInit("/dev/ttyUSB0",57600);

	/*
	for (int i = 1;i<argc;i++) {
	  printf("%s \n",argv[i]);
	  char c = ' ';
	  int j = 0;
	  while (c != '\0') {
	    c = argv[i][j];
	    if (int(c) != 0) {
	      printf("Sending Character = %c ASCII Code = %d \n",c,int(c));
	      SerialPutc(&my,c);
	    }
	    j++;
	  }
	}
	*/
	printf("Persistently in Listen Mode \n");
	char inchar;
	//for (int i = 0;i<3;i++) {
	while (1) {
	  inchar = SerialGetc(&my);
	  printf("%d \n",int(inchar));
	}

	//SerialPutc(&my,argv[1]); //1
	//SerialPutc(&my,'a'); //2
	//SerialPutc(&my,'a'); //3
	//SerialPutc(&my,'a'); //4
	//SerialPutc(&my,'a'); //5
	//SerialPutc(&my,'b');
	
	//SerialPutc(&my,'a'); //1
	//SerialPutc(&my,'a'); //2
	//SerialPutc(&my,'a'); //3
	//SerialPutc(&my,'a'); //4
	//SerialPutc(&my,'a'); //5
	//SerialPutc(&my,'b'); 

	//Send w\r to Arduino
	//printf("Sending w slash r \n");
	/* SerialPutc(&my,'\r'); //1
	SerialPutc(&my,'\r'); //2
	SerialPutc(&my,'\r'); //3
	SerialPutc(&my,'\r'); //4
	SerialPutc(&my,'\r'); //5
	SerialPutc(&my,'a'); //actual character?

	SerialPutc(&my,'\r'); //1
	SerialPutc(&my,'\r'); //2
	SerialPutc(&my,'\r'); //3
	SerialPutc(&my,'\r'); //4
	SerialPutc(&my,'\r'); //5
	SerialPutc(&my,'b'); //actual character?

	SerialPutc(&my,'\r'); //1
	SerialPutc(&my,'\r'); //2
	SerialPutc(&my,'\r'); //3
	SerialPutc(&my,'\r'); //4
	SerialPutc(&my,'\r'); //5
	SerialPutc(&my,'c'); //actual character? */
	//printf("Sent \n");
} //end main loop desktop computer
