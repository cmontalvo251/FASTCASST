#include <iostream> //these are standard includes
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
using namespace std;

//Include the datalogger class
#include <Datalogger/Datalogger.h>
Datalogger logger; //Creating a variable called Datalogger. just like int or double I can do Datalogger

//include barometer class
#include <Baro/BaroTemp.h>
BaroTemp barotemp;

//Include the GPS Class
#include <GPS/GPS.h>
GPS satellites;

//Include the IMU Class
#include <IMU/IMU.h>
IMU orientation;

//Analog to Digital Converter to read Analog Signals
#include <ADC/ADC.h>
ADC analog; 

//Include a Class for CPU Temperature
#include <Sensors/temperature.h>
temperature cputemp;

//Get a Timer
//time and clock are reserved variables so I used watch
#include <Timer/timer.h>
TIMER watch;

//Telemetry 
#ifdef TELEMETRY
//Import telemetry class
#include <Serial/Telemetry.h>
Telemetry serial;
#endif

//THE MAIN FUNCTION THAT this cpp file will run
//main has to return an integer
//int is an integer - 2 bytes 
//double is a double precision floating point number - 8 bytes
//float is a single precision floating point number - 4 bytes
//char is a single character - 2 bytes
//char* is a vector of characters which is basically a string like 'c' is a char
//but 'carlos' is a char*
//char** it's a list of char* so you can do 'carlos','collin'
// argc is the number of input arguments
// argv is the input arguments
int main(int argc,char** argv) {
	printf("Running Datalogger Test Script \n");

	//Select an IMU
	orientation.init(0); //0 for MPU and 1 for LSM

	////////////TELEMTRY SETUP//////////////
  	#ifdef TELEMETRY
	printf("Opening Serial Port ttyAMA0 \n");
  	serial.SerialInit("/dev/ttyAMA0",57600);
  	printf("If no errors present, serial port is open \n");
	float number_array[MAXFLOATS]; //MAXFLOATS is set to 10 in Telemetry.h as of 7/16/2021
	int number_Telemetry_vars = 7;
	serial.period = 1.0; //Set the period of telemetry to 1 second
	#endif

	//////////////DATALOGGING SETUP///////////////////
	printf("Looking for File in %s \n",argv[1]);
	logger.findfile(argv[1]);
	//Then we open it
	logger.open();
	//Let's make a MATLAB variable for output
	MATLAB outdata;
	outdata.zeros(19+analog.channel_count,1,"outdata");
	////////////////////////////////////////////////////

	//We create a loop to write stuff
	watch.resetStartTime();

	for (int i = 0;i<10000;i++){
		//Update Timer
		watch.updateTime();

		//POLL GPS
		satellites.poll(watch.currentTime,0);

		//POLL BAROMETER AND TEMPERATURE
		barotemp.poll(watch.currentTime);

		//Get Temperature of internal temp sensor
		cputemp.get();

		//Read ADC
		analog.get_results();

		//Poll IMU
		double s = 0.0; //0 for no filtering and 1.0 for overfiltering
		orientation.loop(watch.elapsedTime,s);

		/////////////////POLL TELEMETRY//////////////////////////
        #ifdef TELEMETRY
        if ((watch.currentTime - serial.lastTime) > serial.period) {
			number_array[0] = orientation.roll;
			number_array[1] = orientation.pitch;
			number_array[2] = orientation.yaw;
			number_array[3] = satellites.longitude;
			number_array[4] = satellites.latitude;
			number_array[5] = satellites.altitude;
			number_array[6] = watch.currentTime;
			serial.SerialSendArray(number_array,number_Telemetry_vars,1); //the trailing zero is to turn off echo
			serial.lastTime = watch.currentTime;
		}
		#endif

		///PRINT CURRENT DATA STREAMS
		#ifdef DEBUG
		printf("i = %d ",i);
		printf(" Time = %lf %lf ",watch.currentTime,watch.elapsedTime);
		printf("Lat/Lon = %lf %lf ",satellites.latitude,satellites.longitude);
		printf("Alt (GPS,Baro) = %lf %lf ",satellites.altitude,barotemp.altitude);
		printf("P = %lf ",barotemp.pressure);
		printf("Temp (Baro,IMU,CPU) = %lf %lf %lf ",barotemp.temperature,orientation.temperature,cputemp.temp);
		printf("RPY = %lf %lf %lf ",orientation.roll,orientation.pitch,orientation.yaw);
		printf("PQR = %lf %lf %lf ",orientation.roll_rate,orientation.pitch_rate,orientation.yaw_rate);
		printf("ACC = %lf %lf %lf ",orientation.ax,orientation.ay,orientation.az);
		analog.print_results();
		printf("\n");
		#endif

		///////////////LOG TO DISC/////////////////////
		//Populate the outdata Matrix to log to sd card
		outdata.set(1,1,watch.currentTime);
		outdata.set(2,1,watch.elapsedTime);
		outdata.set(3,1,barotemp.pressure);
		outdata.set(4,1,barotemp.temperature);
		outdata.set(5,1,orientation.temperature);
		outdata.set(6,1,cputemp.temp);
		outdata.set(7,1,satellites.latitude);
		outdata.set(8,1,satellites.longitude);
		outdata.set(9,1,satellites.altitude);
		outdata.set(10,1,barotemp.altitude);
		outdata.set(11,1,orientation.roll);
		outdata.set(12,1,orientation.pitch);
		outdata.set(13,1,orientation.yaw);
		outdata.set(14,1,orientation.roll_rate);
		outdata.set(15,1,orientation.pitch_rate);
		outdata.set(16,1,orientation.yaw_rate);
		outdata.set(17,1,orientation.ax);
		outdata.set(18,1,orientation.ay);
		outdata.set(19,1,orientation.az);
		outdata.vecset(20,20+analog.channel_count-1,analog.results,1);
		logger.println(outdata);

		cross_sleep(.05);
	}

	logger.close();
	///The int at the top means this function will return an integer
	return 0;
}
