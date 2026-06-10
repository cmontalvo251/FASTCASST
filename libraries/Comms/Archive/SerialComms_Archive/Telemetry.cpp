#include <Serial/Telemetry.h>

//Call this for defaults
void Telemetry::InitSerialPort(void)
{
  // VERY IMPORTANT: Edit this line of code to designate which COM port the ADCS board is using!!
  int BaudRate = 115200;
  #ifdef __WIN32__
    char *port = "\\\\.\\COM12";
  #endif
  #if defined __linux__ || __APPLE__ || RPI
    char *port = "/dev/ttyUSB0";
  #endif
  SerialInit(port,BaudRate);
}


//Call this for higher level control
void Telemetry::SerialInit(char *ComPortName, int BaudRate) 
{
  #ifdef RPI
  printf("Opening Com Port on Raspberry Pi \n");
  if(wiringPiSetup() == -1) {
      fprintf(stdout, "Unable to start wiringPi: %s\n", strerror (errno));
    }
  hComm = serialOpen(ComPortName,BaudRate);
  if (hComm < 0) {
      fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
    }
  return;
  #endif
  
  //On linux you need to open the tty port
  #if defined __linux__ || __APPLE__
  printf("Opening Com Port on Linux \n");
  hComm = open(ComPortName,  O_RDWR | O_NOCTTY);
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  // Read in existing settings, and handle any error
  if(tcgetattr(hComm, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be whatever the baudRate variable is
  cfsetispeed(&tty, BaudRate);
  cfsetospeed(&tty, BaudRate);

  // Save tty settings, also checking for error
  if (tcsetattr(hComm, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  #endif
} 
 
char Telemetry::SerialGetc()
{
  char rxchar;

  #ifdef RPI
  if (serialDataAvail(hComm)) {
      rxchar = serialGetchar(hComm);
      //fflush(stdout);
    }
  return rxchar;
  #else
  
  #if defined __linux__ || __APPLE__
    // Allocate memory for read buffer, set size according to your needs
    memset(&rxchar, '\0', sizeof(rxchar));
    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = read(hComm, &rxchar, sizeof(rxchar));
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes <= 0) {
      //printf("Error reading: %s", strerror(errno));
      rxchar = '\0';
    }
    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
    // print it to the screen like this!)
    //printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
    //printf("Read %i bytes, rxchar = %c, ASCII = %d ",num_bytes,rxchar,int(rxchar));
    return rxchar;
  #endif

  #endif

}
 
void Telemetry::SerialPutc(char txchar)
{
  #ifdef RPI
  serialPutchar(hComm,txchar);
  fflush(stdout);
  return;
  #endif
  #if defined __linux__ || __APPLE__
  // Write to serial port
  write(hComm,&txchar,sizeof(txchar));
  return;
  #endif
}

void Telemetry::SerialPutString(char *string,int len) {
  char outchar;
  outchar = *string++;
  for (int idx = 0;idx<len;idx++){
    SerialPutc(outchar);
    //printf("%c",outchar);
    outchar = *string++;
  }
}

void Telemetry::SerialPutString(char *string)
{
  char outchar;
  outchar = *string++;
  //This while loop. Does it always break?
  //I would rather have a for loop
  //I would rather have an input to the code be the length of
  //the string so we just use a for loop
  //The routine above is overloaded with length as an input
  while (outchar!=NULL){ 
    SerialPutc(outchar);
    //printf("%c",outchar);
    outchar = *string++;
  }
}

void Telemetry::SerialSendArray(float number_array[],int num) {
  SerialSendArray(number_array,num,1);
}

void Telemetry::SerialSendArray(float number_array[],int num,int echo) {
  union inparser inputvar;
  char outline[20];
  for (int i = 0;i<num;i++) {
    inputvar.floatversion = number_array[i];
    int int_var = inputvar.inversion;
    if (echo) {
      printf("Sending = %lf %d \n",number_array[i],int_var);
    }
    sprintf(outline,"%d:%08x",i,int_var);
    if (echo) {
      printf("Hex = %s \n",outline);
    }
    //This routine uses a while loop until it hits a NULL char
    //SerialPutSrting(hComm,outline);
    //This routine uses a for loop
    SerialPutString(outline,11); //The 11 here is the number of characters in the string
    //H: - 2 chars
    //followed by 8 hex chars
    //followed by a space - 1
    //11 total
    //Send a slash r after every number
    SerialPutc('\r');
  }
  if (echo) {
    printf("Numbers Sent \n");
  }
}

///////////This is really annoying but when this Serial library was first written, the desktop
////side would send 3 hex numbers at a time and then a \r at the end. The board would then
///respond with 1 hex number at a time with \r at the end. Because of that the SerialPutArray is
///for the desktop to send an array where 3 numbers are followed by \r
///the SerialSendArray is literally the exact same code but it sends 1 number at a time with \r
///at the end. In my opinion, it would be better to send 1 hex number and then \r back and forth
///that way there's no confusion on which routine to use. Problem is that MultiSAT++/HIL is using
//the 3 hex \r format and the RPI Groundstation is using 1 hex \r format. In an effort to not break
//other people's code I have kept SerialPutArray and SerialSendArray. If we can ever get the MultiSAT
//and HIL members in the room together and have a coding party I suggest we change everything to 1 hex \r
//format but for now we will leave this here. CMontalvo 10/13/2020 (This was a Tuesday. Not a Friday)

void Telemetry::SerialPutArray(float number_array[],int num) {
  SerialPutArray(number_array,num,1);
}

void Telemetry::SerialPutArray(float number_array[],int num,int echo) {
  union inparser inputvar;
  char outline[20];
  int slashr = 0;
  for (int i = 0;i<num;i++) {
    inputvar.floatversion = number_array[i];
    int int_var = inputvar.inversion;
    if (echo) {
      printf("Sending = %lf %d \n",number_array[i],int_var);
    }
    sprintf(outline,"H:%08x ",int_var);
    if (echo) {
      printf("Hex = %s \n",outline);
    }
    SerialPutString(outline);
    slashr++;
    //Send a slash r after every 3rd set of numbers
    if (slashr == 3) {
      SerialPutc('\r');
      slashr=0;
    }
  }
  if (echo) {
    printf("Numbers Sent \n");
  }
}

//This function will just read everything from the Serial monitor and print it to screen
void Telemetry::SerialGetAll() {
  char inchar = '\0';
  printf("Waiting for characters \n");
  int i = 0;
  do {
    do {
      inchar = SerialGetc();
      //printf("i = %d inchar = %c chartoint = %d \n",i,inchar,int(inchar));
    } while (inchar == '\0');
    printf("Receiving: i = %d char = %c chartoint = %d \n",i,inchar,int(inchar));
    i++;
  } while ((i<MAXLINE));
  printf("Response received \n");
}

void Telemetry::SerialGetArray(float number_array[],int num) {
  SerialGetArray(number_array,num,1);
}

void Telemetry::SerialGetArray(float number_array[],int num,int echo) {
  union inparser inputvar;
  for (int d = 0;d<num;d++) {
    int i = 0;
    char inLine[MAXLINE];
    char inchar = '\0';
    if (echo) {
      printf("Waiting for characters \n");
    }
    do {
      do {
        inchar = SerialGetc();
      } while (inchar == '\0');
      if (echo) {
      printf("Receiving: i = %d char = %c chartoint = %d \n",i,inchar,int(inchar));
      }
      inLine[i++] = inchar;
    } while ((inchar != '\r') && (i<MAXLINE));
    if (echo) {
      printf("Response received \n");
    }

    // Format from Arduino:
    // H:nnnnnnnn 

    // Now Convert from ASCII to HEXSTRING to FLOAT
    if (echo) {
      printf("Converting to Float \n");
    }
    inputvar.inversion = 0;
    for(i=2;i<10;i++){
      if (echo) {
        printf("Hex Digit: i = %d char = %c \n",i,inLine[i]);
      }
      inputvar.inversion <<= 4;
      inputvar.inversion |= (inLine[i] <= '9' ? inLine[i] - '0' : toupper(inLine[i]) - 'A' + 10);
    }
    if (echo) {
      printf("Integer Received = %d \n",inputvar.inversion);
      printf(" \n");
    }
    number_array[d] = inputvar.floatversion;
  }
}

void Telemetry::SerialPutHello() {
  SerialPutHello(1); //echo is on by default unless you turn it off
}

void Telemetry::SerialPutHello(int echo) {
  if (echo) {
    printf("Sending w slash r \n");
  }
  SerialPutc('w');
  SerialPutc('\r');
  if (echo) {
    printf("Sent \n");
  }
}

int Telemetry::SerialGetHello(int echo) {
  //Consume w\r\n
  if (echo) {
    printf("Reading the Serial Buffer for w slash r slash n \n");
  }
  char inchar;
  int err = 0;
  for (int i = 0;i<3;i++) {
    inchar = SerialGetc();
    int val = int(inchar);
    err+=val;
    if (echo) {
      printf("%d \n",val);
    }
  }
  return err;
}

int Telemetry::SerialListen() {
  return SerialListen(1); //default to having echo on
}

int Telemetry::SerialListen(int echo) {
  //Listen implies that this is a drone/UAV/robot that is simply
  //listening on the airwaves for anyone sending out w \r
  //Listen w\r
  
  ///////////////THIS WORKS DO NOT TOUCH (RPI ONLY)
  /* char dat;
  if(serialDataAvail(hComm))
    {
      dat = serialGetchar(hComm);
      printf("char = %c int(char) = %d \n", dat,int(dat));
    }
    return 0;*/
  ///////////////////////////////////////

  int ok = 0;
  char inchar;
  inchar = SerialGetc();
  int val = int(inchar);
  if ((echo) && (val > 0) && (val < 255)) {
    printf("SerialListen => char = %c int(char) = %d \n", inchar,val);
  }
  
  if (val == 119) { //That's a w!
    ok += 119;
    //If we received a w we need to read say 10 times and see if we get a \r
    //remember that \r is a 13 in ASCII and \n is 10 in ASCII
    inchar = SerialGetc();
    val = int(inchar);
    //If we received a 13 or reach max we will break out of this loop
    //There is nothing more we need to do so we will just print val
    //to the screen
    if ((echo) && (val == 13)){
      printf("Slash R Received!!!! \n");
    }
    if (echo) {
      printf("Character Received = %c ASCII Code = %d \n",inchar,val);
    }
    //and then increment ok
    ok+=val;
  }
//either way we shall return ok
  return ok;
}

void Telemetry::SerialDebug() {
  char inchar;
  inchar = SerialGetc();
  int val = int(inchar);
  printf("Character Received = %c ASCII Code = %d \n",inchar,val);
}

void Telemetry::SerialRespond() {
  //overloaded function just calls the echo on version by default
  SerialRespond(1);
}

//Responding is very much like SerialPutHello except this is
//board side so this implies that a drone/uav/robot is responding
//to a groundstation computer saying hi.
//the response to hello (w\r) is hello, sir (w\r\n)
void Telemetry::SerialRespond(int echo) {

  /* THIS WORKS DO NOT TOUCH (RPI ONLY)
  char dat;
  dat = 'w';
  printf("Sending char %c \n",dat);
  serialPutchar(my, dat);
  dat = '\r';
  printf("Sending char %c \n",dat);
  serialPutchar(my, dat);
  dat = '\n';
  printf("Sending char %c \n",dat);
  serialPutchar(my, dat);
  */
  
  if (echo) {
    printf("Sending w slash r slash n \n");
  }
  SerialPutc('w');
  SerialPutc('\r');
  SerialPutc('\n');
  if (echo) {
    printf("Sent \n");
  }
}

  
  
