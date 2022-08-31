#include "hardware.h"

//Constructor
hardware::hardware() {
}

//Initialization Routine
void hardware::init(char root_folder_name[],int NUMSIGNALS) {
  printf("Root Folder Name = %s \n",root_folder_name);

  //Initialize the RCIO
  rc.outInit(NUMSIGNALS);

  //The hardware initialization needs to import the Simulation.txt file
  //and the config.txt file
  //This will create the in_configuration_matrix and in_simulation_matrix
  
  //////////////////////////SIMULATION.TXT///////////////////////
  char simfile[256]={NULL};
  strcat(simfile,root_folder_name);
  strcat(simfile,"Input_Files/Simulation.txt");
  int ok = logger.ImportFile(simfile,&in_simulation_matrix,"simulation_matrix",-99);
  if (!ok) { exit(1); }// else {in_simulation_matrix.disp();}
  //////////////////////////CONFIG.TXT////////////////////////////
  char configfile[256]={NULL};
  strcat(configfile,root_folder_name);
  strcat(configfile,"Input_Files/Config.txt");
  ok = logger.ImportFile(configfile,&in_configuration_matrix,"configuration_matrix",-99);
  if (!ok) { exit(1); }// else {in_configuration_matrix.disp();}

  /////EXTRACT DATA FROM CONFIG FILE
  PRINTRATE = in_configuration_matrix.get(1,1);
  LOGRATE = in_configuration_matrix.get(2,1);
  RCRATE = in_configuration_matrix.get(3,1);
  TELEMRATE = in_configuration_matrix.get(4,1);
  HILRATE = in_configuration_matrix.get(4,1);

  sense.init(in_configuration_matrix,in_simulation_matrix);

  //Extract Number of SIGNALS and set headers
  pwmnames = (char**)malloc((NUMSIGNALS)*sizeof(char*));
  for (int i = 1;i<=NUMSIGNALS;i++) {
    pwmnames[i-1] = (char*)malloc((18)*sizeof(char));
    sprintf(pwmnames[i-1],"PWM Hardware Out %d",i);
  }

  //Initialize Logger
  logger.init("data/",sense.getNumVars()+1+NUMSIGNALS); //+1 for time
  //Set and log headers
  logger.appendheader("Time (sec)");
  logger.appendheaders(sense.headernames,sense.getNumVars());
  logger.appendheaders(pwmnames,NUMSIGNALS);
  logger.printheaders();

  //What data do I want sent to RPI???
  //1 - roll
  //2 - pitch
  //3 - yaw
  //4 - lat
  //5 - lon
  //6 - alt
  //7 - gx
  //8 - gy
  //9 - gz
  //10 - pressure
  
  //What data do I want send to Desktop????
  //1 - rx 1
  //2 - rx 2
  //3 - rx 3
  //4 - rx 4
  //5 - rx 5
  //6 - control matrix 1
  //7 - control matrix 2
  //8 - control matrix 3
  //9 - control matrix 4

  //Initialize the Telemetry
  //What do I want sent?
  //1 - Time
  //2 - roll
  //3 - pitch
  //4 - yaw
  //5 - latitude
  //6 - longitude
  
  NUMTELEMETRY = 6; //Set the actual values in the loop function
  NUMSENSE = 10;
  NUMCTL = 9;
  telemetry_matrix.zeros(NUMTELEMETRY,1,"Telemetry Matrix HW");
  uart_sense_matrix.zeros(NUMSENSE,1,"Serial Sense Matrix");
  uart_ctl_matrix.zeros(NUMCTL,1,"Serial Control Matrix");
  ser.init(NUMTELEMETRY,NUMSENSE,NUMCTL);
}

//This version of the loop runs assuming you are saving data from the simulation 
//environment
void hardware::send(double currentTime,MATLAB model_matrix,double keyboardVars[]) {

  //Send Model Matrix to sensor class
  sense.send(currentTime,model_matrix);

  //We also need to set the temperature in the modeling matrix
  //I don't think we really need this in the model but whatever
  model_matrix.set(30,1,sense.getTemperature());
  
  //Is using the keyboard you need to copy over the keyboard values
  //to rcin
  #ifdef KEYBOARD
  for (int i = 0;i<4;i++) {
    rc.in.keyboard[i] = keyboardVars[i];
  }
  //printf("Key = ");
  //for (int i = 0;i<4;i++) {
  //printf("%lf ",keyboardVars[i]);
  //}
  //printf("\n");
  #endif
}

//This version of the loop runs assuming you are polling from real hardware
void hardware::loop(double currentTime,double elapsedTime,MATLAB control_matrix) {
  //Here we poll the receiver
  if (currentTime >= nextRCtime) {
    //printf("Read RX %lf \n",currentTime);
    rc.read();
    //rc.in.printRCstate(-4);
    //printf("\n");
    nextRCtime=currentTime+RCRATE;
  }

  //Poll all as quickly as possible sensors - This puts all raw data into the sense matrix
  sense.poll(currentTime,elapsedTime);

  //I then need to populate the pwm_array with the control signals as quickly as possible
  for (int i = 0;i<rc.out.NUMSIGNALS;i++) {
    rc.out.pwm_array[i] = round(control_matrix.get(i+1,1));
  }
  //However, the control routine is created by the user and doesn't necessarily have a
  //saturation filter built in so we need to do that here
  rc.out.saturation_block();
  //printf("ELEVATOR ARRAY = %d \n",rc.out.pwm_array[2]);

  //Check to see if it's time to log
  if (currentTime >= nextLOGtime) {
    //printf("Hardware Logging %lf \n",currentTime);
    logger.printvar(currentTime);
    logger.print(sense.sense_matrix);
    logger.printarrayln(rc.out.pwm_array,rc.out.NUMSIGNALS);
    nextLOGtime=currentTime+LOGRATE;
  }

  //Send telemetry data. Always running to make sure
  //it works in SIL mode and SIMONLY.
  //Unfortunately I have to turn off telemetry in HIL mode
  //Because we are using the radios for HIL right now
  //If I find another way to send data from a computer
  //to an RPI I will change this.
  if (currentTime >= nextTELEMtime) {
    //printf("Sending Telemetry %lf \n",currentTime);
    //For right now let's send RPY and GPS coordinates
    telemetry_matrix.set(1,1,currentTime);
    telemetry_matrix.set(2,1,sense.orientation.roll);
    telemetry_matrix.set(3,1,sense.orientation.pitch);
    telemetry_matrix.set(4,1,sense.orientation.yaw);
    telemetry_matrix.set(5,1,sense.satellites.latitude);
    telemetry_matrix.set(6,1,sense.satellites.longitude);
    ser.sendTelemetry(telemetry_matrix,0);
    nextTELEMtime=currentTime+TELEMRATE;
  }

  //And then send the pwm_array to the servos and ESCs as quickly as possible
  //The write function has a built in saturation block no need to worry there
  //Right now we have to shut this off when using the satellite because
  //Satellites don't use servos
  #ifndef satellite
  rc.out.write();
  #endif
}

void hardware::hil(double currentTime,double elapsedTime) {
    if (currentTime < nextHILtime) {
      return;
    } else {
      nextHILtime = currentTime + HILRATE;
    }

    #if defined (HIL) && (DESKTOP)
    //Normally this line of code would run in the hardware::loop routine but since that's running on the
    //pi we need to run it here. The poll just simply puts everything in the sense_matrix from the model
    //matrix
    sense.poll(currentTime,elapsedTime);
    if (sendOK) {
      printf("Send Data to RPI \n");
      //What data do I want sent to RPI???
      //1 - roll
      uart_sense_matrix.set(1,1,sense.sense_matrix.get(4,1));
      //2 - pitch
      uart_sense_matrix.set(2,1,sense.sense_matrix.get(5,1));
      //3 - yaw
      uart_sense_matrix.set(3,1,sense.sense_matrix.get(6,1));
      //4 - lat
      uart_sense_matrix.set(4,1,sense.sense_matrix.get(16,1));
      //5 - lon
      uart_sense_matrix.set(5,1,sense.sense_matrix.get(17,1));
      //6 - alt
      uart_sense_matrix.set(6,1,sense.sense_matrix.get(18,1));
      //7 - gx
      uart_sense_matrix.set(7,1,sense.sense_matrix.get(10,1));
      //8 - gy
      uart_sense_matrix.set(8,1,sense.sense_matrix.get(11,1));
      //9 - gz
      uart_sense_matrix.set(9,1,sense.sense_matrix.get(12,1));
      //10 - pressure
      uart_sense_matrix.set(10,1,sense.sense_matrix.get(27,1));
      uart_sense_matrix.disp();
      ser.sendSense(uart_sense_matrix);

      //If you set this to sendOk = 1 the DESKTOP will continually send
      //data to the RPI. It'd be a good way to test one way communication from
      //the desktop to the RPI
      sendOK = 0;
    } else {
      printf("READING CONTROL MATRIX FROM SERIAL \n");
      //rec error code not operational at the moment
      int rec = ser.readControl(uart_ctl_matrix);
      uart_ctl_matrix.disp();
      sendOK = 1;
      //We then need to populate this into the appropriate vectors
      rc.in.rx_array[0] = uart_ctl_matrix.get(1,1); //wait. Why do we need these rcinputs?
      rc.in.rx_array[1] = uart_ctl_matrix.get(2,1);
      rc.in.rx_array[2] = uart_ctl_matrix.get(3,1);
      rc.in.rx_array[3] = uart_ctl_matrix.get(4,1);
      //Before we copy uart_ctl_matrix over to pwm_array we need to create a backup
      //printf("\n RCOUTPUT before uart update \n");
      //rc.out.print();
      //printf("\n ----- \n");
      rc.out.backup();

      rc.out.pwm_array[0] = uart_ctl_matrix.get(5,1); //don't we just need the rcoutputs?
      rc.out.pwm_array[1] = uart_ctl_matrix.get(6,1);
      rc.out.pwm_array[2] = uart_ctl_matrix.get(7,1);
      rc.out.pwm_array[3] = uart_ctl_matrix.get(8,1);
      
      //printf("\n RCOUTPUT after uart update \n");
      //rc.out.print();
      //printf("\n ----- \n");

      //Again the control matrix is coming from the control routine on the pi
      //The control routine is written by the user. since we can't expect the user
      //to worry about saturation we need to put in that block here
      rc.in.saturation_block();
      //printf("RCINPUT \n");
      //rc.in.printRCstate(-4);

      //We then will run the RangeCheck and it will return an error code
      int err = rc.out.RangeCheck(); //This is a bit more robust to Serial errors than just the saturation block
      //printf("\n RCOUTPUT after range check \n");
      //rc.out.print();
      //printf("\n ---- \n");

      if (err) {
        //This means that the rangecheck through an error which means the values received from
        //uart were invalid and we need to revert back to our previous values
        rc.out.revert();
      }

      //printf("\n RCOUTPUT after potential revert \n");
      //rc.out.print();
      //printf("\n ---- \n");

    }
    #endif
    
    //If we're running HIL on RPI we need to receive data from serial via desktop
    #if defined (HIL) && (RPI)
    if (recOK) {
      printf("Receive Data from Desktop \n");
      ser.readSense(uart_sense_matrix);
      uart_sense_matrix.disp();
      //Again we need to populate this into the appropriate vectors
      //Roll
      sense.sense_matrix.set(4,1,uart_sense_matrix.get(1,1));
      //Pitch
      sense.sense_matrix.set(5,1,uart_sense_matrix.get(2,1));
      //Yaw
      sense.sense_matrix.set(6,1,uart_sense_matrix.get(3,1));
      //Lat
      sense.sense_matrix.set(16,1,uart_sense_matrix.get(4,1));
      //Lon
      sense.sense_matrix.set(17,1,uart_sense_matrix.get(5,1));
      //Alt
      sense.sense_matrix.set(18,1,uart_sense_matrix.get(6,1));
      //gx
      sense.sense_matrix.set(10,1,uart_sense_matrix.get(7,1));
      //gy
      sense.sense_matrix.set(11,1,uart_sense_matrix.get(8,1));
      //gz
      sense.sense_matrix.set(12,1,uart_sense_matrix.get(9,1));
      //pressure
      sense.sense_matrix.set(27,1,uart_sense_matrix.get(10,1));
      //sense.sense_matrix.disp();

      //If you set this variable recOK to 1 the RPI will continually receive data from the DESKTOP
      //computer. It would be a good way to test 1 way communication
      recOK = 1;
    } else {
      //Then send data to desktop
      //printf("Send data to Desktop \n");
      //What data do I want send to Desktop????
      //1 - rx 1
      uart_ctl_matrix.set(1,1,rc.in.rx_array[0]);
      //2 - rx 2 
      uart_ctl_matrix.set(2,1,rc.in.rx_array[1]);
      //3 - rx 3
      uart_ctl_matrix.set(3,1,rc.in.rx_array[2]);
      //4 - rx 4
      uart_ctl_matrix.set(4,1,rc.in.rx_array[3]);
      //5 - rx 5
      uart_ctl_matrix.set(5,1,rc.in.rx_array[4]);
      //6 - control matrix 1
      uart_ctl_matrix.set(6,1,rc.out.pwm_array[0]);
      //7 - control matrix 2
      uart_ctl_matrix.set(7,1,rc.out.pwm_array[1]);
      //8 - control matrix 3
      uart_ctl_matrix.set(8,1,rc.out.pwm_array[2]);
      //9 - control matrix 4
      uart_ctl_matrix.set(9,1,rc.out.pwm_array[3]);
      //ser.sendControl(uart_ctl_matrix);
      recOK = 1;
    }
    #endif

}
