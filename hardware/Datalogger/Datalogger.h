#ifndef DATALOGGER_H //HEADER GUARD - It makes sure that you only import this header file once
#define DATALOGGER_H

#include <MATLAB/MATLAB.h>
#include <iostream>

class Datalogger {
 private:
  FILE* outfile;
  int number = 0;
  int length = 0;
  char filename[256];
  int headerctr = 0;
  int logctr = 1;
 public:
  void init(char* directory,int length);
  void findfile(char* directory);
  void print(MATLAB);
  void print();
  void println(MATLAB);
  void println();
  void printchar(char*);
  void close();
  void flush();
  void open();
  void printheaders();
  void appendheaders(char** headers,int length);
  void append(MATLAB matrix);
  int ImportFile(char* filename,MATLAB* data,char* name,int length);
  //Contructor
  Datalogger();
  int IsHeader = 0;
  int filetype = 1;
  MATLAB logvars;
  char** logheader;
  void setLogVars(int);
};





#endif


