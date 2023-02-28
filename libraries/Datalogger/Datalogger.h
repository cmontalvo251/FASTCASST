#ifndef DATALOGGER_H //HEADER GUARD - It makes sure that you only import this header file once
#define DATALOGGER_H

#ifdef ARDUINO
#include "MATLAB.h"
#include <SD.h>
#else
#include <MATLAB/MATLAB.h>
#include <iostream>
#endif

void printstdout(char* msg);
void printstdoutint(int);

class Datalogger {
 private:
  #ifdef ARDUINO
  File outfile;
  #else
  FILE* outfile;
  #endif
  int number = 0;
  int total_length = 0;
  char filename[256];
  int headerctr = 0;
  int logctr = 1;
 public:
  void init(char* directory,int length);
  void findfile(char* directory);
  void print(MATLAB);
  void printvar(double);
  void writecomma();
  void writenewline();
  void printint(int);
  void print();
  void println(MATLAB);
  void println();
  void printarray(int[],int);
  void printarrayln(int[],int);
  void printchar(char*);
  void closefile();
  void flush();
  void openfile();
  void reopen(char*);
  void printc(char);
  void printheaders();
  void appendheader(char*);
  void appendheaders(char** headers,int length);
  void append(MATLAB matrix);
  int ImportFile(char* filename,MATLAB* data,char* name,int length);
  //Contructor
  Datalogger();
  int IsHeader = 0;
  int filetype = 1;
  int echo = 1;
  MATLAB logvars;
  char** logheader;
  void setLogVars(int);
};

#endif


