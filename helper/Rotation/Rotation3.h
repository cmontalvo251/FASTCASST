#ifndef ROTATION3_H
#define ROTATION3_H

#include <MATLAB/MATLAB.h>
#include <math.h>
#include <Mathp/mathp.h>

using namespace std;

class Rotation3 {
 private:
  double phi,theta,psi;
  double q0,q1,q2,q3;
  MATLAB T1,T2,T3,temp_mat,q0123,ptp,temp_vec,temp_mat2,ptp_temp;
  void L1(double);
  void L2(double);
  void L3(double);
  MATLAB T321,T123;
 public:
  //Contructor
  Rotation3(); 
  void L321(MATLAB,int); //int is a type //0 = Euler Angles,1 = Quaternions
  void getptp(MATLAB);
  void rotateBody2Inertial(MATLAB,MATLAB);
  void rotateInertial2Body(MATLAB,MATLAB);
  void disp();
  void dispEuler();
  void dispQuat();
  void rotateInertia(MATLAB,MATLAB);
  void extractEuler(MATLAB);
  void extractQuaternions(MATLAB);
  MATLAB H;
};


#endif 



