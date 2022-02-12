#include "Rotation3.h"

Rotation3::Rotation3() {
  temp_vec.zeros(3,1,"temp_vec");
  ptp.zeros(3,1,"PTP");
  ptp_temp.zeros(3,1,"PTP_TEMP");
  q0123.zeros(4,1,"QUATERNIONS");
  T321.zeros(3,3,"T321");
  T123.zeros(3,3,"T123");
  temp_mat.zeros(3,3,"temp_mat");
  temp_mat2.zeros(3,3,"temp_mat");
  //Make individual matrices
  T1.zeros(3, 3, "T1"); //initialize them
  T2.zeros(3, 3, "T2");
  T3.zeros(3, 3, "T3");
  H.zeros(3,3,"H");
}

void Rotation3::rotateBody2Inertial(MATLAB outvec,MATLAB invec) {
  //This rotates from Body to Inertial
  //vecI = T321*vecB
  outvec.mult(T321,invec); //you need to do it in two step by making a temp_vec
}

void Rotation3::rotateInertial2Body(MATLAB outvec,MATLAB invec) {
  //This rotates from Inertial to Body
  //vecB = T123*vecI
  outvec.mult(T123,invec); //you need to do it in two step by making a temp_vec
}

void Rotation3::L321(MATLAB rot_params,int type) {
  if (type == 0) {
    //Euler angles
    ptp.overwrite(rot_params);
    q0123.euler2quat(ptp);
    phi = rot_params.get(1,1);
    theta = rot_params.get(2,1);
    psi = rot_params.get(3,1);
    L1(phi); //now set the 3x3s
    L2(theta);
    L3(psi);
    // printdouble(psi,"psi");
    //Now compute T321
    temp_mat.overwrite(T3); //temp_mat = T3
    T321.mult(temp_mat,T2); //T321 = T3*T2
    temp_mat.overwrite(T321); //temp_mat = T321 = T3*T2
    T321.mult(temp_mat,T1); //T321 = temp*T1 = T3*T2*T1
    //If you decide to use euler angles I assume you'd like to have the H matrix as well
    //Kptp = [1 sphi*ttheta cphi*ttheta;0 cphi -sphi;0 sphi/ctheta cphi/ctheta];
    H.mult_eq(0);
    H.set(1,1,1);
    H.set(1,2,sin(phi)*tan(theta));
    H.set(1,3,cos(phi)*tan(theta));
    H.set(2,2,cos(phi));
    H.set(2,3,-sin(phi));
    H.set(3,2,sin(phi)/cos(theta));
    H.set(3,3,cos(phi)/cos(theta));
  }
  else if (type == 1) {
    //Quaternions
    q0123.overwrite(rot_params);
    ptp.quat2euler(q0123);
    q0 = rot_params.get(1,1);
    q1 = rot_params.get(2,1);
    q2 = rot_params.get(3,1);
    q3 = rot_params.get(4,1);
    //Now set T321 using q0,q1,q2,q3
    T321.set(1,1,SQUARE(q0)+SQUARE(q1)-SQUARE(q2)-SQUARE(q3));
    T321.set(1,2,2*(q1*q2-q0*q3));
    T321.set(1,3,2*(q0*q2+q1*q3));
    T321.set(2,1,2*(q1*q2+q0*q3));
    T321.set(2,2,(SQUARE(q0)-SQUARE(q1)+SQUARE(q2)-SQUARE(q3)));
    T321.set(2,3,2*(q2*q3-q0*q1));
    T321.set(3,1,2*(q1*q3-q0*q2));
    T321.set(3,2,2*(q0*q1+q2*q3));
    T321.set(3,3,(SQUARE(q0)-SQUARE(q1)-SQUARE(q2)+SQUARE(q3)));
  } else if (type == 2) {
    //printf("Screw Rotation \n");
    //Screw rotation
    //Make sure we have a valid entry
    if (rot_params.norm() < 1e-10) {
      return;
    } else {
      rot_params.normalize();
      double x = rot_params.get(1,1);
      double y = rot_params.get(2,1);
      double z = rot_params.get(3,1);
      double psi = atan2(y,x);
      double theta = atan2(z,sqrt(x*x + y*y));
      double phi = 0;
      ptp.set(1,1,phi);
      ptp.set(2,1,theta);
      ptp.set(3,1,psi);
      L321(ptp,0); //0 for Euler
    }
  }
  T123.overwrite(T321);
  T123.transpose();
}

void Rotation3::rotateInertia(MATLAB II,MATLAB IB) {
  //This will rotate IB in the body frame to II which is the inertia in the inertial frame
  //In matlab we have
  //v(inertial) = R * v(body)
  //II = R*IB*R'
  //in C++ we have
  //v(inertial) = T321* v(body)
  //Thus we need
  //II = T321*IB*T123
  temp_mat.mult(IB,T123);
  II.mult(T321,temp_mat);
}

void Rotation3::dispEuler() {
  ptp.disp();
}

void Rotation3::dispQuat() {
  q0123.disp();
}

void Rotation3::disp() {
  T1.disp();
  T2.disp();
  T3.disp();
  T321.disp();
}

void Rotation3::L1(double ang)
{
	T1.set(1, 1, 1);
	T1.set(1, 2, 0);
	T1.set(1, 3, 0);
	T1.set(2, 1, 0);
	T1.set(2, 2, cos(ang));
	T1.set(2, 3, -sin(ang));
	T1.set(3, 1, 0);
	T1.set(3, 2, sin(ang));
	T1.set(3, 3, cos(ang));
}
void Rotation3::L2(double ang)
{
	T2.set(1, 1, cos(ang));
	T2.set(1, 2, 0);
	T2.set(1, 3, sin(ang));
	T2.set(2, 1, 0);
	T2.set(2, 2, 1);
	T2.set(2, 3, 0);
	T2.set(3, 1, -sin(ang));
	T2.set(3, 2, 0);
	T2.set(3, 3, cos(ang));
}
void Rotation3::L3(double ang)
{
	T3.set(1, 1, cos(ang));
	T3.set(1, 2, -sin(ang));
	T3.set(1, 3, 0);
	T3.set(2, 1, sin(ang));
	T3.set(2, 2, cos(ang));
	T3.set(2, 3, 0);
	T3.set(3, 1, 0);
	T3.set(3, 2, 0);
	T3.set(3, 3, 1);
}

void Rotation3::getptp(MATLAB ptpout) {
  ptpout.overwrite(ptp);
}

void Rotation3::extractEuler(MATLAB TBI) {
  //Given a TBI matrix extract euler angles
  double theta = -asin(TBI.get(1,3));
  double sphi  = TBI.get(2,3)/cos(theta);
  double cphi = TBI.get(3,3)/cos(theta);
  double phi = atan2(sphi,cphi);
  double spsi = TBI.get(1,2)/cos(theta);
  double cpsi = TBI.get(1,1)/cos(theta);
  double psi = atan2(spsi,cpsi);  

  ptp.set(1,1,phi);
  ptp.set(2,1,theta);
  ptp.set(3,1,psi);
}

void Rotation3::extractQuaternions(MATLAB TBI) {
  //Given a TBI matrix extract the quaternions.
  //This code below was originally written in MATLAB
  //Then ported over to Python.
  //Finally this code has been transcribed to C++.
  //many of the comments and syntax will look wierd due to the number of
  //conversions
  //#%%%Assuming T is a 3x3 matrix, extract the quaternion vector (q0,q1,q2,q3)
  //#%%%assuming a 3-2-1 transformation sequence
  //#%%%Let T be defined such that v(body) = T v(inertial)
  //#%%In Mark Costello's notation, T would TBI
  double alfa = TBI.get(1,2) + TBI.get(2,1);
  double bata = TBI.get(3,1) + TBI.get(1,3);
  double gama = TBI.get(3,2) + TBI.get(2,3);
  double q1squared = alfa*bata/(4.0*gama);

  //#%%%Solutions split into two different solutions here
  double q1a = sqrt(q1squared);
  double q1b = -sqrt(q1squared);
  double q2a = alfa/(4.0*q1a);
  double q2b = alfa/(4.0*q1b);
  double q3a = bata/(4.0*q1a);
  double q3b = bata/(4.0*q1b);

  //#%%%These are the same so just pick one
  //#%q0asquared = TBI(1,1) + q2a^2 + q3a^2 - q1a^2
  //#%q0bsquared = TBI(1,1) + q2b^2 + q3b^2 - q1b^2
  double q0squared = TBI.get(1,1) + q2a*q2a + q3a*q3a - q1a*q1a;

  //#%%%Solution However still splits into 4 possible solutions
  //#You can get around this though by enforcing q0a to be positive
  //#See this article here %%http://planning.cs.uiuc.edu/node151.html
  double q0a = sqrt(q0squared);
  //#q0b = -sqrt(q0squared);

  //#%%%Here are my 2 possible solutions
  //q0123aa = np.asarray([q0a,q1a,q2a,q3a]);
  //q0123ab = np.asarray([q0a,q1b,q2b,q3b]);
    
  //#%According to this website there are multiple solutions 
  //#%that yield the same euler angles. 
  //#%http://planning.cs.uiuc.edu/node151.html
  //#%So what we want to do is compute the euler angles from the matrix
  extractEuler(TBI); //this saves the euler angles into a vector called ptp
  //#ptp = [phi,theta,psi];
  //#%Then check and see which have the same euler angles
  //#because you restricted q0 to be positive above though.
  //#This will only result in one unique solution from the 2.
  //#Thus you can break out of this loop as soon as you find it.
  //quats = [q0123aa,q0123ab]
  //for q0123j in quats:
  //ptpj = quat2euler(q0123j)
  //val = abs(sum(ptpj-ptp))
  //if val < 1e-10:
  //return q0123j
  //Since we only have to do this twice we will just do this manually
  //in C++
  q0123.set(1,1,q0a);
  q0123.set(2,1,q1a);
  q0123.set(3,1,q2a);
  q0123.set(4,1,q3a);
  ptp_temp.quat2euler(q0123);
  if (abs(ptp_temp.diffsum(ptp)) < 1e-10) {
    printf("AA worked \n");
    return;
  } else {
    printf("AB selected \n");
    q0123.set(1,1,q0a);
    q0123.set(2,1,q1b);
    q0123.set(3,1,q2b);
    q0123.set(4,1,q3b);
    return;
  }
}
