#include "Airspeed.h"

Airspeed::Airspeed(){
}

void Airspeed::setup(int* channelin,int numADCsIN,float T_avgIN) {
  analog_converter = new ADC_Navio2();
  analog_converter->initialize();
  numADCs = numADCsIN;
  T_avg = T_avgIN;
  a_inf = sqrt(gamma*R*T_avg);
  B = 2.0/7.0;
  for (int i = 0;i<=numADCs;i++) {
    channel[i] = channelin[i];
    ADCVEC[i].zeros(NAVERAGES,1,"ADCVEC");
    ctr[i] = 1;
  }
  printf("Starting ADC...\n");
}

void Airspeed::ComputeWindSpeed(int i){
  //printf("ADC Signal = %lf ",adc_in);
  float WSPressure = (adc_result[i] - V_avg[i])/Pascals;
  //printf("WSPressure = %lf ",WSPressure);
  //printf("Temperature = %lf ",T_avg);
  float WS = WSPressure+1.0;
  float K = 5.0*((pow(WS,B))-1.0);
  if(K < 0.0){
    K = 0.0;
  }
  windspeed[i] = a_inf*sqrt(K);
}

void Airspeed::FilterWindSpeed(int i) {
  float s = 0.7; //After flight test on 07-12-19, decided to crank up windspeed filter. Windspeed in general is still garbage. We decided to go with speed from GPS by converting to cartesian coordinates.
  //float s = 0.45; //Using wind tunnel to test windspeed filter on 07-05-19. Will test three separate filter constants and plot to see which is best. Set wind speed in wind tunnel to 32 Hz for each experiment. Decided to use this filter constant. Will run wind tunnel test again except this time record windspeed as wind tunnel spins up. Will use 32 Hz again. Repeated experiment and plotted data. Seems we do not need a filter on the windspeed, however the flight data begs to differ. Will try moving the pitot tube to a different position on the wings to see if data is better for the flight test on 07-11-19. 
  //float s = 0.4; //Using wind tunnel to test windspeed filter on 07-05-19. Wind tunnel set to 32 Hz. Decided to use 0.45.
  //float s = 0.35; //Using wind tunnel to test windspeed filter on 07-05-19. Wind tunnel set to 32 Hz. Decided to use 0.45
  windspeed_filtered[i] = windspeed_filtered[i]*s + (1-s)*windspeed[i];
  //printf("windspeed_filtered = %lf ",windspeed_filtered);
  //printf("windspeed = %lf \n",windspeed);
}

void Airspeed::readADC(int i) {
  //printf("Polling ADC \n");
  adc_result[i] = analog_converter->read(channel[i])/1000.0;
  //printf("Polled ADC \n");
}

void Airspeed::computeADCAverages() {
  for (int i = 0;i<numADCs;i++) {
    //Compute the average
    V_avg[i] = ADCVEC[i].mean();
    //Since we have the average now. Let's print the windspeed to stdout.
    readADC(i);
    ComputeWindSpeed(i);
    printf("V_avg = %lf Current Windspeed = %lf, channel = %d \n",V_avg[i],windspeed[i],channel[i]);
    //ADCVEC[i].disp();
    //sleep(10);
  }
}

//This is the most fundamental adc reading routine
void Airspeed::poll(int fileopen) {
  //No matter what, if the datalogging file is open or not
  //or rather the aircraft is armed or not. We must read the pitot probe
  windspeed_average = 0;
  for (int i = 0;i<numADCs;i++) {
    //printf("Channel = %d , ADC = %lf ",channel[i],adc_result[i]);
    readADC(i);
    //After this we need to check if we are armed or not.
    if (fileopen) {
      //We are armed and must convert the adc to windspeed and filter the result
      ComputeWindSpeed(i);
      FilterWindSpeed(i);
      windspeed_average += windspeed_filtered[i];
      //printf("%d = %lf ",idx,pitots.windspeed_filtered[idx]);
      //printf(" Average WindSpeed = %lf GPS Speed = %lf \n",windspeed_average,satellites.speed);
    } else {
      //We are not armed in which case we must simply save the adc value into the matrix
      ADCVEC[i].set(ctr[i],1,adc_result[i]);
      ctr[i]+=1;
      if (ctr[i] > ADCVEC[i].getRow()) {
	ctr[i] = 1;
	printf("Obtained enough ADC measurements \n");
      }
    }
  }
  windspeed_average /= (double)numADCs;
}

