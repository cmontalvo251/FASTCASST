#include "hardware.h"

#ifdef HIL
boost::mutex HILmutex; //Mutex for passing data b/t HIL asynchronous threads
//Global variables for uart_sense matrix and uart_ctl_matrix
MATLAB uart_sense_matrix,uart_ctl_matrix,uart_sense_matrix_copy,uart_ctl_matrix_copy;
#endif

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
  //HILRATE IS NOW HARDCODED. Note that HILRATE is just the time that the sense matrices
  //are updated. The serial hil loop runs as fast as possible to read data and not miss anything  
  //HILRATE = in_configuration_matrix.get(4,1); 

  sense.init(in_configuration_matrix,in_simulation_matrix);

  //Extract Number of SIGNALS and set headers
  pwmnames = (char**)malloc((NUMSIGNALS)*sizeof(char*));
  for (int i = 1;i<=NUMSIGNALS;i++) {
    pwmnames[i-1] = (char*)malloc((18)*sizeof(char));
    sprintf(pwmnames[i-1],"PWM Hardware Out %d",i);
  }

  //Initialize Logger
  logger.init("data/",sense.getNumVars()+6+NUMSIGNALS); //+6 for time and 5 RC channels
  //Set and log headers
  //TIME
  logger.appendheader("Time (sec)");
  //SENSE VARS
  logger.appendheaders(sense.headernames,sense.getNumVars());
  //RC IN SIGNALS
  rcnames = (char**)malloc((5)*sizeof(char*));
  for (int i = 1;i<=5;i++) {
    rcnames[i-1] = (char*)malloc((18)*sizeof(char));
    sprintf(rcnames[i-1],"RC Channel #%d",i);
  }
  logger.appendheaders(rcnames,5);
  //RC OUT SIGNALS
  logger.appendheaders(pwmnames,NUMSIGNALS);
  logger.printheaders();
  /////////IF YOU ADD TO THIS HEADER YOU NEED TO MAKE SURE YOU INCREMENT
  ////////THE LOGGER.INIT FUNCTION BY 1

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
  // All the control signals

  //Initialize the Telemetry
  //What do I want sent?
  //1 - Time
  //2 - roll
  //3 - pitch
  //4 - yaw (compass)
  //5 - latitude
  //6 - longitude
  //7 - altitude (barometer)
  //8 - speed (GPS)
  
  NUMTELEMETRY = 8; //Set the actual values in the loop function
  telemetry_matrix.zeros(NUMTELEMETRY,1,"Telemetry Matrix HW");
  //Telemetry is always on an working in some sort of configuration
  serTelem.TelemInit(NUMTELEMETRY);


  /////////////////////////HIL/////////////////////////////
  #ifdef HIL

  //LOOP RATES
  //HILRATE IS HOW OFTEN THE MATRICES ARE UPDATED
  //SERIALLOOPRATE IS HOW OFTEN THE DATA IS SENT VIA UART
  #ifdef DESKTOP
  HILRATE = 0.001;
  SERIALLOOPRATE = 0.001;
  #elif RPI
  HILRATE = 0.001;
  SERIALLOOPRATE = 0.001;
  #endif
  
  //NUMSENSE = 10;
  //NUMSENSE = 3; //For tank RPY
  NUMSENSE = 4; //For apprentice innerloop RP GXGY
  NUMCTL = NUMSIGNALS; //Same as control signals
  uart_sense_matrix.zeros(NUMSENSE,1,"Serial Sense Matrix");
  uart_sense_matrix_copy.zeros(NUMSENSE,1,"Serial Sense Matrix Copy");
  uart_ctl_matrix.zeros(NUMCTL,1,"Serial Control Matrix");
  uart_ctl_matrix_copy.zeros(NUMCTL,1,"Serial Control Matrix Copy");
  serHIL.HILInit(NUMSENSE,NUMCTL);
  //In this branch, rather than kicking off the hil routine as a thread we're going to call it in
  //the actual function call
  //boost::thread hilloop(hil,serHIL,SERIALLOOPRATE);
  #endif
  ////////////////////////////////////////////////////////
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
  //If we are runnning on the RPI in HIL mode we don't run this
  int POLL = 1;
  #if defined (RPI) && (HIL)
      POLL = 0;
  #endif
  if (POLL) {
      sense.poll(currentTime,elapsedTime);
  }

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
    //Time
    logger.printvar(currentTime);
    //All sense states
    logger.print(sense.sense_matrix);
    //RC Channels
    for (int i = 0;i<5;i++) {
      logger.printint(rc.in.rx_array[i]);
    }
    //RC Out Channels
    logger.printarrayln(rc.out.pwm_array,rc.out.NUMSIGNALS);
    //DEBUG PRINT
    /*printf("Ail RX Ch = %d ",rc.in.rx_array[1]);
    printf(" Ail RX Out = %d ",rc.out.pwm_array[1]);
    int aileron = rc.in.rx_array[1];
    double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    printf(" Phi_c from RX Ch = %lf ",roll_command);
    int aileron_out = rc.out.pwm_array[1];
    double kpa = 2.5;
    double kda = 0.05;
    double roll = sense.sense_matrix.get(4,1);
    double roll_rate = sense.sense_matrix.get(10,1);
    double roll_command_out = roll - (-(aileron_out - OUTMID) - kda*roll_rate)/kpa;
    printf(" Phi_c from RX Out = %lf \n",roll_command_out);*/
    nextLOGtime=currentTime+LOGRATE;
  }

  //Send telemetry data. Always running to make sure
  //it works in SIL mode and SIMONLY.
  //Note that this code opens two serial ports when running in HIL mode
  //Also note that this function loop() only runs on the pi in HIL mode
  //So if HIL mode is on and you're on DESKTOP, this entire function won't
  //run.
  if (currentTime >= nextTELEMtime) {
    //printf("Sending Telemetry %lf \n",currentTime);
    //PAUSE();
    //For right now let's send RPY and GPS coordinates

    //This is the telemetry data needed for ground station command line
    /*telemetry_matrix.set(1,1,currentTime);
    telemetry_matrix.set(2,1,sense.satellites.latitude);
    telemetry_matrix.set(3,1,sense.satellites.longitude);
    telemetry_matrix.set(4,1,sense.compass);
    telemetry_matrix.set(5,1,sense.orientation.yaw);
    telemetry_matrix.set(6,1,sense.satellites.heading);*/

    //This is the telemetry data needed for ground station serial (GUI)
    telemetry_matrix.set(1,1,currentTime);
    telemetry_matrix.set(2,1,sense.orientation.roll);
    telemetry_matrix.set(3,1,sense.orientation.pitch);
    telemetry_matrix.set(4,1,sense.compass);
    telemetry_matrix.set(5,1,sense.satellites.latitude);
    telemetry_matrix.set(6,1,sense.satellites.longitude);
    telemetry_matrix.set(7,1,sense.atm.altitude); 
    telemetry_matrix.set(8,1,sense.satellites.speed);
    
    serTelem.sendTelemetry(telemetry_matrix,0);
    nextTELEMtime=currentTime+TELEMRATE;
  }

  //And then send the pwm_array to the servos and ESCs as quickly as possible
  //The write function has a built in saturation block no need to worry there
  //Right now we have to shut this off when using the satellite because
  //Satellites don't use servos
  #ifndef satellite
  //printf("WRITING SIGNALS \n");
  rc.out.write();
  #endif

}

#ifdef HIL
void hardware::hilsend(double currentTime) {
  //Because this uart_sense_matrix is controlled in two asynchronous threads
  //we need to place a mutex here

  //Now we only need to send data to these matrices at like 10 Hz
  //So HILRATE is hardcoded for the moment
  //Note that this HILRATE is different than how fast the serial
  //threaded loop is running. 
  if (currentTime < nextHILtime) {
      return;
    } else {
      nextHILtime = currentTime + HILRATE;
      //printf("Processing HIL MATRICES = %lf \n",currentTime);
  }

  //HILmutex.lock(); //In this branch we don't need the mutex locks

  #ifdef DESKTOP
  //What data do I want sent to RPI???
  //1 - X
  /*uart_sense_matrix.set(1,1,sense.sense_matrix.get(1,1));
  //2 - Y
  uart_sense_matrix.set(2,1,sense.sense_matrix.get(2,1));
  //3 - Z - this is fused pressure and GPS altitude again combined on SIL DESKTOP
  uart_sense_matrix.set(3,1,sense.sense_matrix.get(3,1));
  //4 - roll
  uart_sense_matrix.set(4,1,sense.sense_matrix.get(4,1));
  //5 - pitch
  uart_sense_matrix.set(5,1,sense.sense_matrix.get(5,1));
  //6 - compass value - IMU and GPS are fused in SIL on DESKTOP
  uart_sense_matrix.set(6,1,sense.sense_matrix.get(6,1));
  //7 - u - forward flight speed
  uart_sense_matrix.set(7,1,sense.sense_matrix.get(7,1));
  //uart_sense_matrix.disp();
  //8 - gx
  uart_sense_matrix.set(8,1,sense.sense_matrix.get(10,1));
  //9 - gy
  uart_sense_matrix.set(9,1,sense.sense_matrix.get(11,1));
  //10 - gz
  uart_sense_matrix.set(10,1,sense.sense_matrix.get(12,1));*/

  //For tank
  /*
  //1 - roll
  uart_sense_matrix.set(1,1,sense.sense_matrix.get(4,1));
  //2 - pitch
  uart_sense_matrix.set(2,1,sense.sense_matrix.get(5,1));
  //3 - IMU yaw
  uart_sense_matrix.set(3,1,sense.sense_matrix.get(20,1));
  */

  //For Apprentice Inner loop
  //Roll
  //uart_sense_matrix.set(1,1,sense.sense_matrix.get(4,1));
  if (currentTime < 10) {
    uart_sense_matrix.set(1,1,50);
  } else {
    printf("SWAPPED \n");
    uart_sense_matrix.set(1,1,-50);
  }
  //Pitch
  uart_sense_matrix.set(2,1,sense.sense_matrix.get(5,1));
  //Roll Rate
  uart_sense_matrix.set(3,1,2);
  //uart_sense_matrix.set(3,1,sense.sense_matrix.get(10,1));
  //Pitch Rate
  uart_sense_matrix.set(4,1,sense.sense_matrix.get(11,1));

  //Fir this HIL branch, once we put the matrices where they need to go we call the hilroutine
  hil(currentTime);

  //Data received from PI
  //Before we copy uart_ctl_matrix over to pwm_array we need to create a backup
  //printf("\n RCOUTPUT before uart update \n");
  //rc.out.print();
  //printf("\n ----- \n");
  rc.out.backup();
  for (int i = 1;i<=NUMCTL;i++) {
    rc.out.pwm_array[i-1] = uart_ctl_matrix.get(i,1); //don't we just need the rcoutputs?
  }

  printf("rec -> aileron = %d \n",rc.out.pwm_array[1]);
      
  //printf("\n RCOUTPUT after uart update \n");
  //rc.out.print();
  //printf("\n ----- \n");

  //We then will run the RangeCheck and it will return an error code
  int err = rc.out.RangeCheck(); //This is a bit more robust to Serial errors than just the saturation block
  //printf("\n RCOUTPUT after range check \n");
  //rc.out.print();
  //printf("\n ---- \n");
  
  if (err) {
    //This means that the rangecheck through an error which means the values received from
    //uart were invalid and we need to revert back to our previous values
    rc.out.revert();
    //printf("INVALID REVERTING \n");
  }

  //printf("\n RCOUTPUT after potential revert \n");
  //rc.out.print();
  //printf("\n ---- \n");
  #endif //DESKTOP

  #ifdef RPI
    //Again we need to populate this into the appropriate vectors
    // 1 - X
    /*sense.sense_matrix.set(1,1,uart_sense_matrix.get(1,1));
    sense.satellites.X = sense.sense_matrix.get(1,1);
    // 2 - Y
    sense.sense_matrix.set(2,1,uart_sense_matrix.get(2,1));
    sense.satellites.Y = sense.sense_matrix.get(2,1);
    // 3 - Z
    sense.sense_matrix.set(3,1,uart_sense_matrix.get(3,1));
    sense.satellites.Z = sense.sense_matrix.get(3,1);
    sense.atm.altitude = -sense.sense_matrix.get(3,1); //Just assume that the altitude is negative -Z
    //Need to update the GPS Latitude and Longitude
    sense.satellites.decodeXYZ();
    // 4 - roll
    sense.sense_matrix.set(4,1,uart_sense_matrix.get(4,1));
    sense.orientation.roll = sense.sense_matrix.get(4,1);
    // 5 - pitch
    sense.sense_matrix.set(5,1,uart_sense_matrix.get(5,1));
    sense.orientation.pitch = sense.sense_matrix.get(5,1);
    // 6 - yaw (compass)
    sense.sense_matrix.set(6,1,uart_sense_matrix.get(6,1));
    sense.sense_matrix.set(20,1,uart_sense_matrix.get(6,1));
    sense.sense_matrix.set(19,1,uart_sense_matrix.get(6,1));
    sense.orientation.yaw = sense.sense_matrix.get(6,1); //For now we set everything to this value
    sense.compass = sense.sense_matrix.get(6,1); //For now we set everything to this value
    sense.satellites.heading = sense.sense_matrix.get(6,1);
    // 7 - u (forward flight speed)
    sense.sense_matrix.set(7,1,uart_sense_matrix.get(7,1));
    sense.satellites.speed = sense.sense_matrix.get(7,1);
    // 8 - gx
    sense.sense_matrix.set(10,1,uart_sense_matrix.get(8,1));
    sense.orientation.roll_rate = sense.sense_matrix.get(10,1);
    // 9 - gy
    sense.sense_matrix.set(11,1,uart_sense_matrix.get(9,1));
    sense.orientation.pitch_rate = sense.sense_matrix.get(11,1);
    // 10 - gz 
    sense.sense_matrix.set(12,1,uart_sense_matrix.get(10,1));
    sense.orientation.yaw_rate = sense.sense_matrix.get(12,1);
    */

    //FOR TANK
    /*
    // 4 - roll
    sense.sense_matrix.set(4,1,uart_sense_matrix.get(1,1));
    sense.orientation.roll = sense.sense_matrix.get(4,1);
    // 5 - pitch
    sense.sense_matrix.set(5,1,uart_sense_matrix.get(2,1));
    sense.orientation.pitch = sense.sense_matrix.get(5,1);
    // 6 - yaw (compass)
    sense.sense_matrix.set(6,1,uart_sense_matrix.get(3,1));
    sense.sense_matrix.set(20,1,uart_sense_matrix.get(3,1));
    sense.sense_matrix.set(19,1,uart_sense_matrix.get(3,1));
    sense.orientation.yaw = sense.sense_matrix.get(6,1); //For now we set everything to this value
    sense.compass = sense.sense_matrix.get(6,1); //For now we set everything to this value
    sense.satellites.heading = sense.sense_matrix.get(6,1);
    */

    //For Apprentice Innerloop
    sense.sense_matrix.set(4,1,uart_sense_matrix.get(1,1));
    sense.orientation.roll = sense.sense_matrix.get(4,1);
    // 5 - pitch
    sense.sense_matrix.set(5,1,uart_sense_matrix.get(2,1));
    sense.orientation.pitch = sense.sense_matrix.get(5,1);
    // 8 - gx
    sense.sense_matrix.set(10,1,uart_sense_matrix.get(3,1));
    sense.orientation.roll_rate = sense.sense_matrix.get(10,1);
    // 9 - gy
    sense.sense_matrix.set(11,1,uart_sense_matrix.get(3,1));
    sense.orientation.pitch_rate = sense.sense_matrix.get(11,1);

    //Fir this HIL branch, once we put the matrices where they need to go we call the hilroutine
    hil(currentTime);

    //We also need to populate the rc out matrices
    for (int i = 1;i<=rc.out.NUMSIGNALS;i++) {
      //control matrix - i
      uart_ctl_matrix.set(i,1,rc.out.pwm_array[i-1]);
    }
  #endif //RPI
  
  //In this branch we don't need to mutex locks
  //HILmutex.unlock();

  //printf("Done Processing HIL Matrices = %lf \n",currentTime);
}

void hardware::hil(double currentTime) {
    //This code needs to operate on a listen as often as possible
    //and only send when you've received a response  
    //In Debug mode we will have the desktop continually send data to 
    //the pi and have the pi continually listen

  //All vars in here must be globals or locally defined -- Not anymore after we moved to synchronous
  //bool sendOK = 0; //Set to 0 and DESKTOP will receive first
  //bool recOK = 0; //Set to 0 and RPI will send first

  //This needs to be an infinite loop or it will only run once
  //In this branch we will run this function call one at a time
  //while (1) {

    #ifdef DESKTOP
    //printf("!!!!!!!!!!!!!!!!!! Send Data to RPI HIL !!!!!!!!!!!!!!!!!!!!!! \n");

    //This uart_sense_matrix is set in another asynchronous thread. Therefore we need
    //to lock the mutex -- See comment below
    //HILmutex.lock(); -- No need for lock anymore in synchronous function calls
    uart_sense_matrix_copy.overwrite(uart_sense_matrix);
    //HILmutex.unlock();

    //Then we send the copy
    printf("send -> time = %lf phi,p = %lf %lf ",currentTime,uart_sense_matrix_copy.get(1,1),uart_sense_matrix_copy.get(3,1));
    serHIL.sendSense(uart_sense_matrix_copy);

    //So the sendSense and ReadSense functions enter an infinite while loop until data is read.
    //Because of this we actually need to have a copy be used for the uart comms and then 
    //lock the mutex once we're ready to copy. Otherwise we enter into a thread lock in 
    //an infinite while loop and never send data.
    //HILmutex.lock(); -- No need for locks anymore in synchronous function calls
    uart_sense_matrix.overwrite(uart_sense_matrix_copy);
    //HILmutex.unlock();

    //If you set this to sendOk = 1 the DESKTOP will continually send
    //data to the RPI. It'd be a good way to test one way communication from
    //the desktop to the RPI
    //printf("READING CONTROL MATRIX FROM SERIAL \n");
    //This uart_ctl_matrix is set in another asynchronous thread. Therefor we need
    //to lock the mutex -- see comment below on thread safety
    serHIL.readControl(uart_ctl_matrix_copy);

    //So the sendControl and readControl functions enter an inifinite while loop until data is read
    //Because of this we actually need to have a copy be used for the uart comms and then
    //lock the mutex once we're ready to copy. Otherwise we enter into a thread lock in
    //an infinite while loop and never send data
    //HILmutex.lock(); -- no need for locks in synchronous function calls
    uart_ctl_matrix.overwrite(uart_ctl_matrix_copy);
    //HILmutex.unlock();
    //if you set this sendOK = 0 the DESKTOP will conitnually be in read Mode. It'd be a good way to
    //test one way communication from the Desktop to the RPI
    #endif
    
    //If we're running HIL on RPI we need to receive data from serial via desktop
    #ifdef RPI
    //printf("Receive Data from Desktop \n");

    //This uart_sense_matrix is set in another asynchronous thread. Therefore we need
    //to lock the mutex -- See comment below on thread safety. 
    serHIL.readSense(uart_sense_matrix_copy);
    //uart_sense_matrix_copy.disp();
    //So the sendSense and ReadSense functions enter an infinite while loop until data is read.
    //Because of this we actually need to have a copy be used for the uart comms and then 
    //lock the mutex once we're ready to copy. Otherwise we enter into a thread lock in 
    //an infinite while loop and never send data.
    //HILmutex.lock(); -- moving to synchronous
    uart_sense_matrix.overwrite(uart_sense_matrix_copy);
    //HILmutex.unlock();

    //If you set this variable recOK to 1 the RPI will continually receive data from the DESKTOP
    //computer. It would be a good way to test 1 way communication
    //Then send data to desktop
    //printf("!!!!!!!!!!!!!!!!!!Send data to Desktop HIL !!!!!!!!!!!!!!!!!!! \n");

    //This uart_ctl_matrix is set in another asynchronous thread therefor we need
    //to lock the mutex -- See comment below
    //HILmutex.lock(); -- moving to synchronous
    uart_ctl_matrix_copy.overwrite(uart_ctl_matrix);
    //HILmutex.unlock();

    //Then we send the copy
    serHIL.sendControl(uart_ctl_matrix_copy);

    //So the sendControl and readControl functions enter an infinite while loop until
    //data is read. Because of this we actually need to have a copy be used for the uart
    //comms and then lock the mutex once we're ready to copy. Otherwise we enter into a thread
    //lock in an infinite loop and never can advance the rest of the code
    //HILmutex.lock(); -- moving to synchronous
    uart_ctl_matrix.overwrite(uart_ctl_matrix_copy);
    //HILmutex.unlock();

    //if you set this to recOK = 0 the RPI will contiunally send
    //data to the DESKTOP. It'd be a good way to test one way communication
    //from the RPi to the desktop
    #endif

    //We need a cross sleep here or the code will blast serial data faster than we can read
    //At the same time on the RPI side we might need a different SERIALLOOPRATE to make sure 
    //we read as fast as possible
    //This cross_sleep will eventually go away or reduce to like 0.001 so that we read as fast
    //as possible for now we're in debug mode so we're only sending every 1.0 seconds
    //cross_sleep(SERIALLOOPRATE); -- No longer need this because function is synchronous now

  //} //End of inifinite while loop -- Removed for synchronouse function call

} //end of hil() function

#endif //HIL
