#include "Datalogger.h"

void printstdout(char* msg) {
  #ifdef ARDUINO
  Serial.print(msg);
  #else
  printf("%s ",msg);
  #endif
}

void printstdoutint(int var) {
    #ifdef ARDUINO
    Serial.print(var);
    Serial.print(" ");
    #else
    printf("%d ",var);
    #endif
}

void printstdoutdbl(double var,int decimals) {
  #ifdef ARDUINO
  if (decimals > 0) {
    Serial.print(var,decimals);
  } else {
    Serial.print(var);
  }
  Serial.print(" ");
  #else
  printf("%lf ",var);
  #endif
}

void printstdoutdbl(double var) {
    printstdoutdbl(var,0);
}

//Constructor
Datalogger::Datalogger() {
}

void Datalogger::findfile(char* directory) {
  int found = 0;
  //char filename[256];
  FILE* fileout;
  while (!found) {

    ////////////CREATE FILENAME///////////////////
    if (filetype == 0) {
      sprintf(filename,"%s%d%s",directory,number,".txt");
    } else {
      sprintf(filename,"%s%d%s",directory,number,".csv");
    }

    //////NOTIFY USER/////////////////////
    if (echo) {
      printstdout("Attempting to check for file: ");
      printstdout(filename);
      printstdout("\n");
    }
    
    /////////CHECK TO SEE IF FILE EXISTS///////
    #ifdef ARDUINO
    int sdexist = SD.exists(filename); 
    found = 1-sdexist; //If SD.exists throws a 0 it means the file doesn't exist. But our found 
    //flag needs to have the indication of hey we found a file that doesn't exist. Make sense? 
    Serial.print("SD Exist = ");
    Serial.print(sdexist);
    Serial.print(" File found flag = ");
    Serial.println(found);
    #else
    fileout = fopen(filename,"r");
    if (!fileout) {
      //File doesn't exist so set found to 1
      found = 1;
    } else {
      //Otherwise close the file we just opened since it exists
      fclose(fileout);
    }    
    #endif

    ////NOTIFY USER/////////////
    if (found) {
      if (echo) {
      printstdout("File does not exist. Exiting loop \n");
    }
    } else {
      if (echo) {
      printstdout("File exists. Skipping and adding 1 \n");
    }
      number+=1; //Number is global in header file
    }

  } //END OF WHILE LOOP

  ///NOTIFY USER///
  if (echo) {
  printstdout("File found for writing = ");
  printstdout(filename);
  printstdout("\n");
  }
}

void Datalogger::openfile() {
  if (echo) {
    printf("Attempting to open: %s \n",filename);
  }
  #ifdef ARDUINO
  outfile = SD.open(filename,FILE_WRITE);
  #else
  outfile = fopen(filename,"wb");
  #endif
  //printf("Attempting to open %s \n",filename);
  if (echo) {
  if (!outfile) {
     printf("File not opened properly = %s \n",filename);
   } else {
     printf("File %s opened successfully \n",filename);
   }
   }
}

void Datalogger::init(char* directory,int num) {
  #ifdef ARDUINO
  pinMode(44,OUTPUT);
  if (!SD.begin(44)) { 
    Serial.println("SD Card Arduino initialization failed!");
    exit(1);
  }
  #endif
  //Set echo to 1
  echo = 1;
  //Find a file in the desired directory
  findfile(directory);
  //open said file
  openfile();
  //Also set echo to 0
  echo = 0;
  //Set size of variables
  setLogVars(num);  
}

void Datalogger::reopen(char* directory) {
  //close currently open file
  closefile();
  //find a new file
  findfile(directory);
  //Open file
  openfile();
}

void Datalogger::setLogVars(int num) {
  logvars.zeros(num,1,"Vars to Log");
  logheader = (char**)malloc(num*sizeof(char*));
  total_length = num;
}

void Datalogger::appendheader(char* header) {
  logheader[headerctr] = header;
  headerctr++;
  //printf("Appending Header = %s Header Counter = %d \n",header,headerctr);
}

void Datalogger::appendheaders(char **headers,int length){
  //printf("Appending Headers: ");
  for (int i = 0;i<length;i++) {
    logheader[headerctr] = headers[i];
    headerctr++;
    //printf("%s Header Counter = %d \n",headers[i],headerctr);
  }
  //printf("Counter = %d \n",headerctr);
}

void Datalogger::printheaders() {
  headerctr = 0;
  printstdout("Logging Headers to Data File \n");
  for (int i = 0;i<total_length;i++) {
    printchar(logheader[i]);
    printstdout(logheader[i]);
    printstdout("\n");
    if (i < total_length - 1) {
      writecomma();
    }
  }
  writenewline();
  IsHeader = 1;
  flush();
}

void Datalogger::append(MATLAB in) {
  for (int i = 1;i<=in.getRow();i++) {
    logvars.set(logctr,1,in.get(i,1));
    logctr++;
  }
}

//Print functions
void Datalogger::printvar(double var,int decimals) {
  #ifdef ARDUINO
  if (decimals > 0) { 
    outfile.print(var,decimals);
  } else {
    outfile.print(var);
  }
  #else
  fprintf(outfile,"%lf",var);
  #endif
  flush();
}
void Datalogger::printvar(double var) {
  printvar(var,0);
}

void Datalogger::printint(int var) {
  #ifdef ARDUINO
  outfile.print(var);
  #else
  fprintf(outfile,"%d",var);
  #endif
  flush();
}

void Datalogger::print(MATLAB out) {
  logctr = 1;
  out.vecfprintf(outfile);
  flush();
}

void Datalogger::print() {
  logctr = 1;
  logvars.vecfprintf(outfile);
  flush();
}

void Datalogger::writenewline() {
  #ifdef ARDUINO
  outfile.print("\n");
  #else
  fprintf(outfile,"%s ","\n");
  #endif
}


void Datalogger::writecomma() {
  #ifdef ARDUINO
  outfile.print(",");
  #else
  fprintf(outfile,",");
  #endif
}

void Datalogger::println(MATLAB out) {
  logctr = 1;
  out.vecfprintfln(outfile);
  flush();
}

void Datalogger::println() {
  logctr = 1;
  logvars.vecfprintfln(outfile);
  flush();
}

void Datalogger::printarray(int array[],int num) {
  for (int i = 0;i<num;i++) {
    printint(array[i]);
    if (i != num-1) {
      writecomma();
    }
  }
  flush();
}

void Datalogger::printarrayln(int array[],int num) {
  printarray(array,num);
  writenewline();
  flush();
}

//Print char*
void Datalogger::printchar(char* msg) {
  #ifdef ARDUINO
  outfile.print(msg);
  #else
  fprintf(outfile,"%s ",msg);
  #endif
  flush();
}

//Print a single character
void Datalogger::printc(char c) {
  #ifdef ARDUINO
  outfile.print(c);
  #else
  fprintf(outfile,"%c",c);
  #endif
  flush();
}

//Close function
void Datalogger::closefile() {
  flush();
  if (echo) {
    printf("Closing File \n");
  }
  #ifdef ARDUINO
  outfile.close();
  #else
  fclose(outfile);
  #endif
  if (echo) {
    printf("File closed \n");
  }
}

//Flush function
void Datalogger::flush() {
  #ifdef ARDUINO
  outfile.flush();
  #else
  fflush(outfile);
  #endif
}

#ifndef ARDUINO //you can't import files on Arduino
int Datalogger::ImportFile(char* filename,MATLAB* data,char* name,int length) {
  //cout << "Reading " << filename << endl;
  //Gonna open this file the old school way
  FILE *file;
  file = fopen(filename,"r");
  char dummy[256];
  if (file) {
    if (length == -99) {
      length = 0;
      while (!feof(file)) {
	fgets(dummy,256,file);
	//cout << dummy << endl;
	length++;
      }
      fclose(file);
    }
    //cout << "File length is " << length << endl;
    //Now that we know how big the file is we need to make a MATLAB vector the same size
    data->zeros(length,1,name); //Because we passed a pointer we have to use the -> instead of .		//data.disp();
    //Then we open the file with FSTREAM and throw it in there. Done. Boom.
    fstream datafile;
    datafile.open(filename);
    if (datafile.is_open()) {
      string input;
      for (int idx = 0;idx<length;idx++) {
	      getline(datafile,input);
      	data->set(idx+1,1,atof(input.c_str()));
      }
      //For debug purposes let's make sure we read everything correctly. File I/O in C++
      //is always hit or miss for me.
      //data->disp();
    } else {
      cout << "Something went wrong in FSTREAM. Maybe the file wasn't closed properly?" << endl;
      return 0;
    }
  } else {
    cout << "File not found = " << filename << endl;
    return 0;
  }
  //This code will automatically generate a MATLAB vector based on how many rows are in
  //the file. Probably need to do an FSEEK or something and then do a ZEROS call to a MATLAB
  //Array.
  //If everything checks out we just return 1
  return 1;
}
#endif
