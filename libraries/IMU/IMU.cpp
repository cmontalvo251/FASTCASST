#include "IMU.h"

///Constructor
IMU::IMU() {
}

void IMU::init(int sensor_type_in){
  sensor_type = sensor_type_in;
  #ifndef DESKTOP
  if (sensor_type == 0) {
    printstdout("Selected: MPU9250\n");
    imu_sensor = new MPU9250();
    TEMP_SCALE = 1.0;
  }
  if (sensor_type == 1) {
    printstdout("Selected: LSM9DS1\n");
    imu_sensor = new LSM9DS1();
    TEMP_SCALE = 10.0;
  }
  if (sensor_type == 2) {
    printstdout("Selected: BNO055\n");
    #ifdef ARDUINO
    imu_sensor = new BNO055();
    #else
    printstdout("BNO055 Currently only supported on Arduino. You need to select a different IMUTYPE\n");
    exit(1);
    #endif
    TEMP_SCALE = 1.0;
  }
  if ((sensor_type > 2) || (sensor_type < 0)) { 
    printstdout("INCORRECT IMUTYPE Check your input files \n");
    exit(1);
  }

  printstdout("Probing Sensor....\n");

  if (!imu_sensor->probe()) {
    printstdout("Sensor not enabled. Exiting prematurely \n");
    return;
  } else {
    printstdout("Sensor enabled properly \n");
    printstdout("Running initialization procedure.....\n");
  }
  imu_sensor->initialize();
  printstdout("Beginning Gyro calibration...\n");
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  int N = 100;
  for(int i = 0; i<N; i++)
    {
      imu_sensor->update();
      imu_sensor->read_gyroscope(&gx, &gy, &gz);
      offset[0] += gx;
      offset[1] += gy;
      offset[2] += gz;
      cross_sleep(10000,6);
      /*printstdout("Counter = ");
      printstdoutint(i);
      printstdout("\n");*/
    }
  offset[0]/=N;
  offset[1]/=N;
  offset[2]/=N;
  printstdout("Offsets are: ");
  printstdoutdbl(offset[0]);
  printstdout(" ");
  printstdoutdbl(offset[1]);
  printstdout(" ");
  printstdoutdbl(offset[2]);
  printstdout(" ");
  //ahrs.setGyroOffset(offset[0],offset[1],offset[2]);
  #endif
}

void IMU::setOrientation(double rollin,double pitchin,double yawin,double gxin,double gyin,double gzin) {
  roll = rollin;
  pitch = pitchin;
  yaw = yawin;
  //The sensor is oriented in a different orientation then standard 3-2-1
  gx = gyin;
  gy = gxin;
  gz = -gzin;
}

void IMU::setTemperature(double tempin) {
  temperature = tempin;
}

void IMU::loop(double elapsedTime){
  #ifndef DESKTOP
  //This routine reads the accelerometer, magnetometer and rate_gyro
  imu_sensor->update();
  //These functions here just put them into local memory. Really they should be name get_accelerometer
  //And the update routine above should be read();
  imu_sensor->read_gyroscope(&gx, &gy, &gz);
  imu_sensor->read_accelerometer(&ax, &ay, &az);
  imu_sensor->read_magnetometer(&mx ,&my ,&mz);
  //Comment out temperature to save time.
  //temperature = imu_sensor->read_temperature()/TEMP_SCALE;

  ///SUBSTRACT OFFSETS HERE
  gx-=offset[0];
  gy-=offset[1];
  gz-=offset[2];
  #endif //ENDIF DESKTOP
      
  //Filter the Gyro
  filterGyro();
  
  #ifndef DESKTOP
  //printf(" gx,gy,gz B = %lf %lf %lf ",gx,gy,gz);
  //ahrs.update(ax,ay,az,gx,gy,gz,mx,my,mz,elapsedTime);
  if (sensor_type == 2) {
    //BNO already has quaternions
    imu_sensor->read_quaternion(&q0,&q1,&q2,&q3);
    ahrs.setQuaternions(q0,q1,q2,q3);
  } else {
    ahrs.updateNOMAG(ax,ay,az,gx,gy,gz,elapsedTime);
  }
  #endif

  //Convert Quaternions to Euler Angles
  #ifdef DESKTOP
  ahrs.getEuler(&roll,&pitch,&yaw); //For some reason the angles are swapped and yaw is negative
  yaw=-yaw;
  #else
  ahrs.getEuler(&pitch,&roll,&yaw);
  #endif

  //printf("(PTP) = %lf %lf %lf \n",pitch,roll,yaw);

  //Now get Heading just from the magnetometer
  getMagHeading();
}

void IMU::getMagHeading() {
  #ifndef DESKTOP
  ///THIS MAY THROW AN ERROR THE FIRST TIME YOU COMPILE ON RPI. JUST COMMENT IT OUT IF YOU NEED TO
  double sphi = sin(roll*PI/180.0);
  double cphi = cos(roll*PI/180.0);
  double stheta = sin(pitch*PI/180.0);
  double mnorm = sqrt(mx*mx + my*my + mz*mz);
  double mxbar = mx/mnorm;
  double mybar = my/mnorm;
  double mzbar = mz/mnorm;
  double top = mzbar*sphi - mybar*cphi;
  double bottom = mxbar*cphi + mybar*stheta*sphi + mzbar*cphi*stheta;
  magyaw = atan2(top,bottom);
  #else
  magyaw = yaw;
  #endif
}

void IMU::filterGyro() {
  //Bumblebee has a first order filter using a Tustin Transformation
  //but that is for the motor signals. Apparently the filter works well for all 8 motors
  //but fails when you shut off motors. If we were to implement that filter we would
  //have to filter the aileron and elevator signals. I don't really want to do that.
  //The filter below is a complimentary filter for the IMU signals. It's possible
  //we could change this to a Tustin filter but for now I suggest we simply increase the
  //Complimentary filter gain to remove some of the IMU noise.
  //float s = 0.35; //Same from Bumblebee
  //float s = 0.4; //Changed on 6/24/2019
  //float s = 0.45; //Changed on 07-02-19. Still getting a lot of noise. We think it is from the motor vibration. Looking into putting a NOX??? Filter to filter out motor vibrations.
  //Float value got moved as an input so we can change it easier.
  //Note that s = 0 is totally unfiltered and s = 1 is overfiltering
  //printf(" gx,gy,gz A = %lf %lf %lf ",gx,gy,gz);
  double s = FilterConstant; //This is set externally or initialized to zero
  gx_filtered = gx_filtered*s + (1-s)*gy;
  gy_filtered = gy_filtered*s + (1-s)*gx;
  gz_filtered = gz_filtered*s + (1-s)*(-gz);
  roll_rate = gx_filtered; //I got rid of RAD2DEG
  pitch_rate = gy_filtered; //roll, pitch yaw is in degrees
  yaw_rate = gz_filtered; //p q r is in rad/s
}

void IMU::printALL() {
  printEuler();
  printRates();
}

void IMU::printEuler() {
  printstdoutdbl(roll);
  printstdout(" ");
  printstdoutdbl(pitch);
  printstdout(" ");
  printstdoutdbl(yaw);
  printstdout(" ");
}

void IMU::printRates() {
  printstdoutdbl(roll_rate);
  printstdout(" ");
  printstdoutdbl(pitch_rate);
  printstdout(" ");
  printstdoutdbl(yaw_rate);
  printstdout(" "); 
}
