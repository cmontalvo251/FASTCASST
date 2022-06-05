#include "Datalogger.h"

//Constructor
Datalogger::Datalogger() {
}

void Datalogger::findfile(char* directory) {
  int found = 0;
  //char filename[256];
  FILE* fileout;
  while (!found) {
    if (filetype == 0) {
      sprintf(filename,"%s%d%s",directory,number,".txt");
    } else {
      sprintf(filename,"%s%d%s",directory,number,".csv");
    }
    if (echo) {
      printf("%s%s \n","Attempting to check for file: ",filename);
    }
    fileout = fopen(filename,"r");
    if (!fileout) {
      found = 1;
    } else {
      fclose(fileout);
      if (echo) {
        printf("File exists. Skipping \n");
      }
    }
    number+=1; //Number is global in header file
  }
  if (echo) {
    printf("File found for writing = %s \n",filename);
  }
  //return number
}

void Datalogger::open() {
  if (echo) {
    printf("Attempting to open: %s \n",filename);
  }
  outfile = fopen(filename,"wb");
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
  //Find a file in the desired directory
  findfile(directory);
  //open said file
  open();
  //Set size of variables
  setLogVars(num);  
}

void Datalogger::reopen(char* directory) {
  //close currently open file
  close();
  //find a new file
  findfile(directory);
  //Open file
  open();
}

void Datalogger::setLogVars(int num) {
  logvars.zeros(num,1,"Vars to Log");
  logheader = (char**)malloc(num*sizeof(char*));
  total_length = num;
}

void Datalogger::appendheader(char* header) {
  printf("Appending Header = %s \n",header);
  logheader[headerctr] = header;
  headerctr++;
}

void Datalogger::appendheaders(char **headers,int length){
  printf("Appending Headers \n");
  for (int i = 0;i<length;i++) {
    printf("%s \n",headers[i]);
    logheader[headerctr] = headers[i];
    headerctr++;
  }
}

void Datalogger::printheaders() {
  headerctr = 0;
  for (int i = 0;i<total_length;i++) {
    fprintf(outfile,"%s ",logheader[i]);
    //printf("%s ",logheader[i]);
    if (i < total_length - 1) {
      fprintf(outfile,"%s",",");
    }
  }
  fprintf(outfile,"%s ","\n");
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
void Datalogger::printvar(double var) {
  fprintf(outfile,"%lf,",var);
  flush();
}

void Datalogger::print(MATLAB out) {
  logctr = 1;
  out.vecfprintf(outfile);
  fprintf(outfile,",");
  flush();
}

void Datalogger::print() {
  logctr = 1;
  logvars.vecfprintf(outfile);
  fprintf(outfile,",");
  flush();
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
    fprintf(outfile,"%d",array[i]);
    if (i != num-1) {
      fprintf(outfile,",");
    }
  }
  flush();
}

void Datalogger::printarrayln(int array[],int num) {
  printarray(array,num);
  fprintf(outfile,"\n");
  flush();
}

//Print char*
void Datalogger::printchar(char* msg) {
  fprintf(outfile,"%s ",msg);
  flush();
}

//Print a single character
void Datalogger::printc(char c) {
  fprintf(outfile,"%c",c);
  flush();
}

//Close function
void Datalogger::close() {
  flush();
  if (echo) {
    printf("Closing File \n");
  }
  fclose(outfile);
  if (echo) {
    printf("File closed \n");
  }
}

//Flush function
void Datalogger::flush() {
  fflush(outfile);
}

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

