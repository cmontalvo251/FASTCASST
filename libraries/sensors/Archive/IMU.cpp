#include "IMU.h"

///Constructor
IMU::IMU() {
}

void IMU::init(int sensor_type){
  #ifndef DESKTOP
  if (sensor_type == 0) {
    printf("Selected: MPU9250\n");
    mpulsm = new MPU9250();
    TEMP_SCALE = 1.0;
  }
  if (sensor_type == 1) {
    printf("Selected: LSM9DS1\n");
    mpulsm = new LSM9DS1();
    TEMP_SCALE = 10.0;
  }

  if (!mpulsm->probe()) {
    printf("Sensor not enabled. Exiting prematurely \n");
    return;
  } else {
    printf("Sensor enabled properly \n");
    printf("Running initialization procedure.....\n");
  }
  mpulsm->initialize();
  printf("Beginning Gyro calibration...\n");
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  int N = 100;
  for(int i = 0; i<N; i++)
    {
      mpulsm->update();
      mpulsm->read_gyroscope(&gx, &gy, &gz);
      offset[0] += gx;
      offset[1] += gy;
      offset[2] += gz;
      usleep(10000);
      //printf("Counter = %d \n",i);
    }
  offset[0]/=N;
  offset[1]/=N;
  offset[2]/=N;
  printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
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
  mpulsm->update();
  mpulsm->read_accelerometer(&ax, &ay, &az);
  mpulsm->read_gyroscope(&gx, &gy, &gz);

  ///SUBSTRACT OFFSETS HERE
  gx-=offset[0];
  gy-=offset[1];
  gz-=offset[2];
  
  //printf(" gx,gy,gz B = %lf %lf %lf ",gx,gy,gz);
  temperature = mpulsm->read_temperature()/TEMP_SCALE;

  //ahrs.update(ax,ay,az,gx,gy,gz,mx,my,mz,elapsedTime);
  ahrs.updateNOMAG(ax,ay,az,gx,gy,gz,elapsedTime);
  ahrs.getEuler(&pitch,&roll,&yaw);
  #endif
  //Call the filter
  filter();
}

void IMU::filter() {
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
  gx_filtered = gx_filtered*s + (1-s)*gy*RAD2DEG;
  gy_filtered = gy_filtered*s + (1-s)*gx*RAD2DEG;
  gz_filtered = gz_filtered*s + (1-s)*(-gz)*RAD2DEG;
  roll_rate = gx_filtered; //NOTICE THAT I PUT RAD2DEG UP THERE
  pitch_rate = gy_filtered; //THIS IS SO RPY IS IN DEG AND PQR
  yaw_rate = gz_filtered; //IS IN DEG/S
}

void IMU::printALL() {
  printEuler();
  printRates();
}

void IMU::printEuler() {
  printf("%lf %lf %lf ",roll,pitch,yaw);
}

void IMU::printRates() {
  printf("%lf %lf %lf ",roll_rate,pitch_rate,yaw_rate);
}
