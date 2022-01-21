#ifndef MATLAB_H
#define MATLAB_H

#ifdef __linux__ || __APPLE
#include <cmath> //only for g++
#include <iostream> //only for g++
#include <string> //only for g++
#endif

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <Mathp/mathp.h>
#include <fstream>
#include "nrutils.h"
#define NRANSI
#define NR_END 1
#define FREE_ARG char*

class MATLAB {
 private:
  int row_,col_,idiag_,isquare_,ivec_,init_;
  double **data_;
  char *name_;
  void init(int,int,char*);
  void mult_init(MATLAB,double,char*);
  void mult_init(MATLAB,MATLAB,char*);
  void plus_init(MATLAB,MATLAB,char*);
  void plus_init(MATLAB,double,char*);
  void minus_init(MATLAB,MATLAB,char*);
  void minus_init(MATLAB,double,char*);
  void cross_init(MATLAB,MATLAB,char*);
 public:
  double pythag(double a, double b);
  double* dvector(long nl, long nh); 
  void nrerror(char error_text[]);
  void svdcmp(double **a, int m, int n, double w[], double **v);
  void free_dvector(double *v, long nl, long nh);
  void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch);
  double** dmatrix(long nrl, long nrh, long ncl, long nch) ;
  void matrix_multiply(double** M1, double** M2, double** Res, int dim11, int dim12, int dim22) ;
  void zeros(int,int,char*);
  void ones(int,int,char*);
  void transpose();
  void transpose_not_square(MATLAB);
  //Inverses for diagonal matrices and 2x2
  void inv(MATLAB,char*); //DO NOT DELETE KEPT FOR BACKWARDS COMPATIBILITY
  //Inverses for diagonal matrices and up to 3x3
  void inverse(); //DO NOT DELETE KEEP FOR BACKWARDS COMPATIBILITY
  //Inverses for all size matrices
  void matrix_inverse(MATLAB A, int DIM) ;
  void set(int,int,double);
  void vecset(int thistart,int thisend,MATLAB,int thatstart);
  void matrixset(int,MATLAB);
  double get(int,int);
  void linspace(double,double,int,char*);
  int found_nans();

  void GaussLagrange(MATLAB,char*); //This routine will compute inv(RN'*RN)*RN'

  void parallel_axis_theorem(MATLAB,double);

  void normalize();

  int checkSizeCompatibility(MATLAB,char*);

  double mean();

  double diffsum(MATLAB);
  
  void plus(MATLAB A,MATLAB B); //this = A + B
  void plus_eq(MATLAB A); // this = this + A
  void plus_eq(double v); // this = this + v (scalar)
  void plus_eq1(int,int,double); //this(int,int) = this(int,int) + double
  void plus_mult_eq(MATLAB,double); //This is this = this + MATLAB*double
  void minus(MATLAB,MATLAB);
  void minus_eq(MATLAB);
  void minus_eq(double);
  //void mult_eq(MATLAB); This won't work because if x = 3x1 and A = 5x3
  //then res = A*x is 5x3.3x1 = 5x1 so you have to dynamically create anoth
  //so we have to be careful here because a 3x3 times a 3x1 is still a 3x1
  //also a 3x3 times a 3x3 is a 3x3. I think we'd like to have something be
  //MATLAB A,B,C; then A.zeros,B.zeros,C.zeros
  //finally A.mult(B,C); without the init part
  void mult(MATLAB,MATLAB);
  void mult(MATLAB,double);
  void mult_eq(double);
  void mult_eq1(int,int,double); //Only multiplies 1 element of the vector

  void copy_init(MATLAB,char*);
  void overwrite(MATLAB); 
  void overwrite(MATLAB,char*);

  double norm();
  double max_abs();

  void cross(MATLAB,MATLAB);
  void cross_eq(MATLAB);
  void plus_cross_eq(MATLAB,MATLAB); //this = this + a x b

  double dot(MATLAB);

  void list();

  void skew(MATLAB);

  void size();

  double sum();
  double abssum();

  void euler2quat(MATLAB);
  void quat2euler(MATLAB);

  void diag(MATLAB,char*);
  void diag(MATLAB);
  void eye(int,char*);
  void disp();
  void dmatrixprint(double**,int,char*);
  int getRow();
  int getCol();
  char* getName();

  void vecfprintf(FILE*);
  void vecfprintfln(FILE*);

  void dlmread(char*,MATLAB*,char*);
  void dlmread(char*);

  double interp2(MATLAB x,MATLAB y,double xstar,double ystar,int debug);
  double interp(MATLAB t,double tstar,int debug);
  int find(MATLAB,double);

  //Constructor
  MATLAB();

};


#endif

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
