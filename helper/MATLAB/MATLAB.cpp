#include "MATLAB.h"

//Constructor
MATLAB::MATLAB() {
  //Do this just to make sure you don't screw somthing up
  row_ = 0;
  col_ = 0;
  name_ = "Not Initialized. You did something very bad";
  init_ = 0;
}

int MATLAB::find(MATLAB vec,double vi) {
  //This function assumes vec is in ascending order and that vi is in b/t min(vec) and max(vec)
  int out = 1;
  while ((vec.get(out,1) <= vi) && (out <= vec.row_)) {
    out+=1;
  }
  if (out > row_) {
    cout << "find() assumes vi is in b/t min(vec) and max(vec)" << endl;
    exit(1);
  }
  return out;
}

double MATLAB::interp(MATLAB T,double tstar,int debug) {
  //I'm going to assume alot here to get rid of a lot of error checking
  //For starters X and Y are in ascending order

  double tmax = T.get(T.row_,1);
  double tmin = T.get(1,1);
  double trange = tmax-tmin;
  
  //Check for out of bounds on T
  if (tstar > tmax) {
    return get(row_,1);
  }
  if (tstar < tmin) {
    return get(1,1);
  }

  //Find the position of xstar within X
  int tr = find(T,tstar);
  int tl = tr-1;

  //%%Interpolate between X points */
  double outUpper = get(tr,1);
  double outLower = get(tl,1);
  double tslope = (outUpper-outLower)/(T.get(tr,1)-T.get(tl,1));
  double out = tslope*(tstar-T.get(tl,1))+outLower;

  if (debug) {
    cout << tstar << " " << tmin << " " << tmax << "  " << trange << endl;
    cout << tr << " " << tl << endl;
    cout << tslope << " " << out << endl;
    disp();
  }

  return out;
}

double MATLAB::interp2(MATLAB X,MATLAB Y,double xstar,double ystar,int WRAP) {
  //I'm going to assume alot here to get rid of a lot of error checking
  //For starters X and Y are in ascending order

  double xmax = X.get(X.row_,1);
  double xmin = X.get(1,1);
  double ymax = Y.get(Y.row_,1);
  double ymin = Y.get(1,1);
  double xrange = xmax-xmin;
  double yrange = ymax-ymin;

  int yr,yl,xl,xr;
  
  //Check for out of bounds on X
  if (WRAP) {
    while (xstar > xmax) {
      xstar -= xrange;
    }
    while (xstar < xmin) {
      xstar += xrange;
    }
  }

  if (xstar >= xmax) {
      xstar = xmax;
      xr = row_;
  } else if (xstar <= xmin) {
    xstar = xmin;
    xr = 2;
  } else {
    //Find the position of xstar within X
    xr = find(X,xstar);
  }
  xl = xr-1;
  
  //Check for out of bounds on Y
  if (WRAP) {
    while (ystar >= ymax) {
      ystar -= yrange;
    }
    while (ystar <= ymin) {
      ystar += yrange;
    }
  }

  if (ystar >= ymax) {
      ystar = ymax;
      yr = row_;
  } else if (ystar <= ymin) {
    ystar = ymin;
    yr = 2;
  } else {
    //Find the position of xstar within X
    yr = find(Y,ystar);
  }
  yl = yr-1;

  int debug = 0;
  // if (xstar > 7118) {
  //   debug = 1;
  // }
  if (debug) {
    cout << xstar << " " << xmin << " " << xmax << "  " << xrange << endl;
    cout << xr << " " << xl << endl;
    cout << ystar << " " << ymin << " " << ymax << "  " << yrange << endl;
    cout << yr << " " << yl << endl;
  }
  
  //We start with 4 points
  double four[4];
  four[0] = get(xr,yr);
  four[1] = get(xl,yr);
  four[2] = get(xl,yl);
  four[3] = get(xr,yl);

  //Interpolate b/t points 2 and 1 as well as 3 and 0
  double slope21 = (four[1]-four[2])/(Y.get(yr,1)-Y.get(yl,1));
  double slope30 = (four[0]-four[3])/(Y.get(yr,1)-Y.get(yl,1));
  double outLower = slope21*(ystar-Y.get(yl,1))+four[2];
  double outUpper = slope30*(ystar-Y.get(yl,1))+four[3];

  //%%Interpolate between X points */
  double xslope = (outUpper-outLower)/(X.get(xr,1)-X.get(xl,1));
  double out = xslope*(xstar-X.get(xl,1))+outLower;

  if (debug) {
    for (int idx = 0;idx<4;idx++) {
      cout << idx << " " << four[idx] << " ";
    }
    cout << endl;
    cout << slope21 << " " << slope30 << " " << outLower << " " << outUpper << endl;
    cout << xslope << " " << out << endl;
    disp();
    X.disp();
    Y.disp();
  }

  return out;
}

//Overloaded function. Use this when you already
//have the matrix allocated
void MATLAB::dlmread(char* filename) {
  cout << "Reading " << filename << endl;
  //Gonna open this file the old school way
  FILE *file;
  file = fopen(filename,"r");
  double val = 0;
  if (file) {
    for (int idx = 1;idx<=row_;idx++) {
      for (int jdx = 1;jdx<=col_;jdx++) {
	fscanf(file,"%lf ",&val);
	set(idx,jdx,val);
      }
    }
  } else {
    printf("!!!!!!!!!!! File: %s not found !!!!!!!!!!!! \n",filename);
    exit(1);
  }
  fclose(file);
}


//I don't think this function below is correct but I'm going to keep it to make sure I do
//not break old code
void MATLAB::dlmread(char* filename,MATLAB* data,char* name) {
  cout << "Reading " << filename << endl;
  //Gonna open this file the old school way
  FILE *file;
  file = fopen(filename,"r");
  int length=0;
  char dummy[256];
  if (file) {
    while (!feof(file)) {
      fgets(dummy,256,file);
      //cout << dummy << endl;
      length++;
    }
    fclose(file);
    //cout << "File length is " << length << endl;
    //Now that we know how big the file is we need to make a MATLAB vector the same size
    data->zeros(length,1,name); //Because we passed a pointer we have to use the -> instead of .
    //data.disp();
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
    }
  } else {
    cout << "File not found = " << filename << endl;
  }
}

void MATLAB::vecfprintf(FILE* outfile) {
  for (int idx = 0;idx<row_;idx++) {
    fprintf(outfile,"%lf",get(idx+1,1));
    if (idx<row_-1) {
      fprintf(outfile,"%s",",");
    }
  }
}

void MATLAB::vecfprintfln(FILE* outfile) {
  //Print the contents like normal
  vecfprintf(outfile);
  //only different is we print a newline at the end
  fprintf(outfile,"\n");
}

void MATLAB::dmatrixprint(double **A,int DIM,char* name) {
  printf("%s%s%s",name," = ","\n");
  int i;
  int j;
  for(i=1;i<=DIM;i++)
    {
      for(j=1;j<=DIM;j++)
	{
	  printf("%.8e%s",A[i][j]," ");
	}
      printf("%s","\n");
    }
  printf("%s","\n");
}

void MATLAB::list() {
  int i;
  int j;
  for(i=0;i<row_;i++)
    {
      for(j=0;j<col_;j++)
      {
        printf("%.8e%s",data_[i][j]," ");
      }
    }
}

void MATLAB::disp() {
  printf("%s%s%s",name_," = ","\n");
  int i;
  int j;
  for(i=0;i<row_;i++)
    {
      for(j=0;j<col_;j++)
	{
	  printf("%.8e%s",data_[i][j]," ");
	}
      printf("%s","\n");
    }
  printf("%s","\n");
}

int MATLAB::found_nans() {
  for (int idx = 1;idx <= row_;idx++) {
    for (int jdx = 1;jdx <= col_;jdx++) {
      if (isnan(get(idx,jdx))) {
	disp();
	  return 1;
	}
    }
  }
  return 0;
}

double MATLAB::diffsum(MATLAB B) {
  double out = 0;
  //Need to make sure the matrices are the same size
  if ((B.row_ != row_)) {
    printf("Error in diff sum. %s is not the same size as %s \n",name_,B.name_);
  }
  if ((B.col_ != col_)) {
    printf("Error in diff sum. %s is not the same size as %s \n",name_,B.name_);
  }
  int len = B.row_;
  int axis = 1;
  if (B.col_ > B.row_) {
    len = B.col_;
    axis = 2;
  }
  for (int idx = 1;idx <= len;idx++) {
    if (axis == 1) {
      out += (B.get(idx,1) - get(idx,1));
    } else {
      out += (B.get(1,idx) - get(1,idx));
    }
  }
  return out;
}


void MATLAB::quat2euler(MATLAB q0123) {
  if ((q0123.row_ != 4) || (q0123.col_ != 1)) {
      printf("Error in quat2euler. %s is not a 4x1 vector \n",q0123.name_);
      return;
    }
  if ((row_ != 3) || (col_ != 1)) {
    printf("Error in quat2euler. %s is not a 3x1 vector \n",name_);
    return;
  }
  double q0 = q0123.get(1,1);
  double q1 = q0123.get(2,1);
  double q2 = q0123.get(3,1);
  double q3 = q0123.get(4,1);
  double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  set(1,1,atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1 + q2*q2)));
  double val = 2*(q0*q2-q3*q1);
  if (abs(val) > 1.0) {
    set(2,1,SIGN(1.0,val)*PI/2.0);
    printf("NORM = %lf \n",norm);
    q0123.disp();
    printf("val = %lf asin(val) = %lf \n",val,asin(val));
    printf("INVALID QUATERNION CONVERSION IN QUAT2EULER in MATLAB.cpp \n");
    printf("MOST LIKELY PITCH ANGLE IS 90 deg \n");
    //exit(1);
  } else {
   set(2,1,asin(val));
  }
  set(3,1,atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2 + q3*q3)));
  //disp();
}

void MATLAB::euler2quat(MATLAB ptp) {
  if ((ptp.row_ != 3) || (ptp.col_ != 1)) {
      printf("Error in euler2quat. %s is not a 3x1 vector \n",ptp.name_);
      return;
    }
  if ((row_ != 4) || (col_ != 1)) {
      printf("Error in euler2quat. %s is not a 4x1 vector \n",name_);
      return;
    }
  double phi = ptp.get(1,1);
  double theta = ptp.get(2,1);
  double psi = ptp.get(3,1);
  set(1,1,cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2));
  set(2,1,sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2));
  set(3,1,cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2));
  set(4,1,cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2));
}

double MATLAB::get(int row,int col) {
  //Error checking
  if ((row > row_) || (col > col_)) {
      printf("Error in get() command. Request Out of bounds - var = %s \n",name_);
      printf("Requested Row and Column -> row = %d, col = %d \n",row,col);
      printf("Size of Matrix = %d %d \n",row_,col_);
      return 0;
    }
  if ((row == 0) || (col == 0)) {
    printf("Error in get() command. Requested zero - var = %s \n",name_);
    printf("Returning Zero as output");
    return 0;
  }
  return data_[row-1][col-1];
}


void MATLAB::set(int row,int col,double var) {
  //Error checking
  if ((row > row_) || (col > col_)) {
    printf("Matrix out of bounds in set() command - var = %s \n",name_);
    return;
  }
  if ((row <= 0) || (col <= 0)) {
    printf("Set commands use MATLAB notation and start at 1 not 0 - var = %s \n",name_);
    return;
  }
  //If the statement above doesn't break go ahead and set the command
  //The minus 1 is there to make the conversion from MATLAB to C++
  data_[row-1][col-1] = var;  
}

//Overloaded overwrite to initialize variable at the same time
void MATLAB::overwrite(MATLAB A,char name[]) {
  zeros(A.row_,A.col_,name);
  for (int idx = 1;idx<=row_;idx++) {
    for (int jdx = 1;jdx<=col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx));
    }
  }
}

int MATLAB::checkSizeCompatibility(MATLAB A,char* functionname) {
  if ((A.row_ != row_) || (A.col_ != col_)) {
    printf("Error in %s. Matrices are not the same size. vars = %s,%s \n",functionname,A.name_,name_);
    A.size();
    size();
    return 0;
  } else {
    return 1;
  }
}

void MATLAB::overwrite(MATLAB A) {
  //Make sure A and this are the same size
  if (!checkSizeCompatibility(A,"overwrite")) {
    return;
  }
  for (int idx = 1;idx<=row_;idx++) {
    for (int jdx = 1;jdx<=col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx));
    }
  }
}

double MATLAB::mean() {
  int length = 0;
  if (row_ > col_) {
    length = row_;
  } else {
    length = col_;
  }
  return sum()/length;
}

double MATLAB::sum()
{
  double ans = 0;
  int i;
  for(i = 1;i<=row_;i++)
    {
      ans = ans + get(i,1);
    }

  return ans;

}

double MATLAB::abssum()
{
  double ans = 0;
  int i;
  for(i = 1;i<=row_;i++)
    {
      ans = ans + abs(get(i,1));
    }

  return ans;

}


void MATLAB::eye(int row,char name[]) {
  zeros(row,row,name);
  for (int idx = 1;idx<=row_;idx++)
    {
      set(idx,idx,1.0);
    }
}  

void MATLAB::diag(MATLAB vec,char name[]) {
  //Create a matrix that is the same size as this
  zeros(vec.row_,vec.row_,name);
  //idiag_ = 1; //This is a diagonal matrix now -- this is a terrible idea.
  //What if the matrix is no longer diag after this?
  for (int idx = 0;idx<row_;idx++) {
    data_[idx][idx] = vec.data_[idx][0];
  }
}

void MATLAB::diag(MATLAB vec) {
  //Create a matrix that is the same size as this
  //zeros(vec.row_,vec.row_,name);
  if (init_ != 1) {
    printf("Error in diag. I don't think you've initialized this matrix yet, var = %s \n",name_);
    exit(1);
  }
  for (int idx = 0;idx<row_;idx++) {
    data_[idx][idx] = vec.data_[idx][0];
  }
}

int MATLAB::getRow() {
  return row_;
}

int MATLAB::getCol() {
  return col_;
}

char* MATLAB::getName() {
  if (init_) {
    return name_;
  }
  else {
    return 0;
  }
}

void MATLAB::transpose() {
  if (!isquare_) {
    printf("Error in transpose. Matrix is not square. Suggest running transpose_not_square() var = %s \n",name_);
    return;
  }
  double temp;
  for (int idx=1;idx<=row_;idx++) {
    for (int jdx=idx;jdx<=col_;jdx++) {
      temp = get(idx,jdx);
      set(idx,jdx,get(jdx,idx));
      set(jdx,idx,temp);
    }
  }
}

void MATLAB::transpose_not_square(MATLAB A) {
  //Make sure that row and columns match up
  if ((row_ != A.col_) || (col_ != A.row_)) {
    printf("Error in transpose_not_square. Matrices were not initialized properly vars = %s and %s \n",name_,A.name_);
  }
  for (int idx=1;idx<=A.row_;idx++) { //loop through the rows of A and set it to the columns of this
    for (int jdx=1;jdx<=A.col_;jdx++) { //loop through the columns of A and set it to the rows of this
      //Grab the current cell and put it in the opposite cell
      set(jdx,idx,A.get(idx,jdx));
    }
  }
}

void MATLAB::init(int row,int col,char name[]) {
  if (init_) {
    printf("Error in init routine. You already initialized this variable = %s \n",name_);
    return;
  }
  init_ = 1;
  idiag_ = 0;
  name_ = name;
  row_ = row;
  col_ = col;
  if ((row_ == 1) || (col_ == 1)) {ivec_ = 1;} else {ivec_ = 0;}
  if (row_ == col_) { isquare_ = 1;} else { isquare_ = 0;}
  data_ = (double**)malloc(row_*sizeof(double*));
  int i;
  for (i = 0;i<row_;i++)
    {
      data_[i] = (double*)malloc(col_*sizeof(double));
    }
}

void MATLAB::ones(int row,int col,char name[]) {
  init(row,col,name);
  int i,j;
  for(i = 0;i < row_;i++)
    {
      for(j=0;j<col_;j++)
	{
	  data_[i][j] = 1;
	}
    }
}

void MATLAB::zeros(int row,int col,char name[]) {
  init(row,col,name);
  int i,j;
  for(i = 0;i < row_;i++)
    {
      for(j=0;j<col_;j++)
	{
	  data_[i][j] = 0;
	}
    }
}

void MATLAB::linspace(double start,double end,int length,char name[]) {
  double inc = (end-start)/(double)(length-1);
  zeros(length,1,name);
  if (end < start) { 
    printf("Error in linspace. End is not greater than start - var = %s\n",name);
    return;
  }
  for (int idx = 1;idx<=length;idx++){
    set(idx,1,start);
    start += inc;
  }
}

void MATLAB::plus_init(MATLAB A,double b,char name[]) {
  zeros(A.row_,A.col_,name);
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)+b);
    }
  }
  
}

void MATLAB::minus_init(MATLAB A,double b,char name[]) {
  zeros(A.row_,A.col_,name);
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)-b);
    }
  }
  
}

void MATLAB::mult_init(MATLAB A,MATLAB B,char name[]) {
  int mm,nn,ll;
  //Make sure inner dimensions match
  if (A.col_ != B.row_) {
    printf("Error in mult. Inner matrix dimensions don't match. var = %s,%s \n",A.name_,B.name_);
    return;
  }
  zeros(A.row_,B.col_,name);
  for(mm = 0;mm<A.row_;mm++)    {
      for(nn = 0;nn<B.col_;nn++)	{
	  for(ll = 0;ll<A.col_;ll++)	    {
	      data_[mm][nn] = data_[mm][nn] + A.data_[mm][ll]*B.data_[ll][nn];
	    }
	}
    }
}

// This doesn't work at the moment. Where do you put the temporary data?
// void MATLAB::mult_eq(MATLAB A) {
//   int mm,nn,ll;
//   //Make sure inner dimensions match
//   if (col_ != A.row_) {
//     printf("Error in mult. Inner matrix dimensions don't match. var = %s,%s \n",name_,A.name_);
//     size();
//     A.size();
//     return;
//   }
//   //Make sure this has been initialized
//   if (!init_) {
//     printf("Error in mult. You tried to multiply two matrices without initializing [this], vars = %s,%s \n",name_,A.name_);
//     return;
//   }

//   //Check outer dimensions
//   if ((col_ != A.col_) || (row_ != A.row_)) {
//     printf("Error in mult. Outer dimensions don't match, vars = %s,%s,%s \n",name_,name_,A.name_);
//     size();
//     A.size();
//     return;
//   }
//   for(mm = 0;mm<A.row_;mm++)    {
//       for(nn = 0;nn<B.col_;nn++)	{
// 	data_[mm][nn] = 0;
// 	  for(ll = 0;ll<A.col_;ll++)	    {
// 	      data_[mm][nn] = data_[mm][nn] + A.data_[mm][ll]*B.data_[ll][nn];
// 	    }
// 	}
//     }
// }

void MATLAB::mult(MATLAB A,MATLAB B) {
  int mm,nn,ll;
  //Make sure inner dimensions match
  if (A.col_ != B.row_) {
    printf("Error in mult. Inner matrix dimensions don't match. var = %s,%s \n",A.name_,B.name_);
    A.size();
    B.size();
    size();
    return;
  }
  //Make sure this has been initialized
  if (!init_) {
    printf("Error in mult. You tried to multiply two matrices without initializing [this], vars = %s,%s \n",A.name_,B.name_);
    return;
  }

  //Check outer dimensions
  if ((col_ != B.col_) || (row_ != A.row_)) {
    printf("Error in mult. Outer dimensions don't match, vars = %s,%s,%s \n",name_,A.name_,B.name_);
    A.size();
    B.size();
    size();
    return;
  }
  for(mm = 0;mm<A.row_;mm++)    {
      for(nn = 0;nn<B.col_;nn++)	{
	data_[mm][nn] = 0;
	  for(ll = 0;ll<A.col_;ll++)	    {
	      data_[mm][nn] = data_[mm][nn] + A.data_[mm][ll]*B.data_[ll][nn];
	    }
	}
    }
}

void MATLAB::mult(MATLAB A,double var) {
  //Make sure this has been initialized
  if (!init_) {
    printf("Error in mult. You tried to multiply a matrix by a variable without initializing [this], vars = %s \n",A.name_);
    return;
    }
  //Check outer dimensions
  if ((col_ != A.col_) || (row_ != A.row_)) {
    printf("Error in mult. Outer dimensions don't match, vars = %s,%s \n",name_,A.name_);
    A.size();
    size();
    return;
  }
  //First copy matrix A into [this]
  overwrite(A);
  //Then perform a mult_eq with [this]
  mult_eq(var);  
}

void MATLAB::mult_init(MATLAB A,double var,char name[]) {
  int mm,nn;
  zeros(A.row_,A.col_,name);
  for(mm = 1;mm<=A.row_;mm++)    {
      for(nn = 1;nn<=A.col_;nn++)	{
	  set(mm,nn,var*A.get(mm,nn));
	}
    }
}

void MATLAB::mult_eq1(int row,int col,double var) {
  if (row > row_) {
    printf("Error in mult_eq1. Input row is greater than row_. Matrix = %s \n",name_);
  }
  if (col > col_) {
    printf("Error in mult_eq1. Input col is greater than col_. Matrix = %s \n",name_);
  }
  double current_val = get(row,col);
  set(row,col,current_val*var);
}

void MATLAB::mult_eq(double var) //this = this*var
{
  if (!init_) {
    printf("Error in mult_eq. You tried to multiply a matrix without initializing it \n");
  }
  int mm,nn;
  for(mm = 1;mm<=row_;mm++)
    {
      for(nn = 1;nn<=col_;nn++)
	{
	  set(mm,nn,var*get(mm,nn));
	}
    }
}

void MATLAB::plus_init(MATLAB A,MATLAB B,char name[]) {
  //Check and make sure the size of A and B are the same
  if ((A.row_ != B.row_) || (A.col_ != B.col_)) {
    printf("Error in plus_init. Matrices must be the same size vars = %s,%s \n",A.name_,B.name_);
    return;
  }
  zeros(A.row_,A.col_,name);
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)+B.get(idx,jdx));
    }
  }
  
}

void MATLAB::plus(MATLAB A,MATLAB B) {
  //Check and make sure the size of A and B are the same
  if ((A.row_ != B.row_) || (A.col_ != B.col_)) {
    printf("Error in plus. Matrices must be the same size vars = %s,%s \n",A.name_,B.name_);
    A.disp();
    B.disp();
    return;
  }
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)+B.get(idx,jdx));
    }
  }
  
}

double MATLAB::dot(MATLAB B) {
  //Check and make sure the size of A and B are the same
  double answer = 0;
  if ((row_ != B.row_) || (col_ != B.col_)) {
    printf("Error in dot. Matrices must be the same size vars = %s,%s \n",name_,B.name_);
    return 0;
  }
  for (int idx = 1;idx<=row_;idx++) {
    answer += get(idx,1)*B.get(idx,1);
  }
  return answer;
}

void MATLAB::minus(MATLAB A,MATLAB B) {
  //Check and make sure the size of A and B are the same
  if ((A.row_ != B.row_) || (A.col_ != B.col_)) {
    printf("Error in minus. Matrices must be the same size vars = %s,%s \n",A.name_,B.name_);
    return;
  }
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)-B.get(idx,jdx));
    }
  }
  
}

void MATLAB::minus_init(MATLAB A,MATLAB B,char name[]) {
  //Check and make sure the size of A and B are the same
  if ((A.row_ != B.row_) || (A.col_ != B.col_)) {
    printf("Error in minus. Matrices must be the same size vars = %s,%s \n",A.name_,B.name_);
    return;
  }
  zeros(A.row_,A.col_,name);
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)-B.get(idx,jdx));
    }
  }
  
}

//Adds 1 variable to row,col
void MATLAB::plus_eq1(int row,int col,double var) {
  if (row > row_) {
    printf("Error in mult_eq1. Input row is greater than row_. Matrix = %s \n",name_);
  }
  if (col > col_) {
    printf("Error in mult_eq1. Input col is greater than col_. Matrix = %s \n",name_);
  }
  double current_val = get(row,col);
  set(row,col,current_val+var);
}

void MATLAB::plus_eq(MATLAB A) {
  //Check and make sure the size of A and this are the same
  if ((A.row_ != row_) || (A.col_ != col_)) {
    printf("Error in plus_eq. Matrices must be the same size vars = %s,%s \n",A.name_,name_);
    A.disp();
    disp();
    return;
  }
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx)+get(idx,jdx));
    }
  }
  
}

void MATLAB::plus_mult_eq(MATLAB A,double var) {
  //Check and make sure the size of A and this are the same
  if ((A.row_ != row_) || (A.col_ != col_)) {
    printf("Error in plus_mult_eq. Matrices must be the same size vars = %s,%s \n",A.name_,name_);
    A.disp();
    disp();
    return;
  }
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,var*A.get(idx,jdx)+get(idx,jdx));
    }
  }
}

void MATLAB::minus_eq(MATLAB A) {
  //Check and make sure the size of A and this are the same
  if ((A.row_ != row_) || (A.col_ != col_)) {
    printf("Error in minus_eq. Matrices must be the same size vars = %s,%s \n",A.name_,name_);
    A.disp();
    disp();
    return;
  }
  for (int idx = 1;idx<=A.row_;idx++) {
    for (int jdx = 1;jdx<=A.col_;jdx++) {
      set(idx,jdx,get(idx,jdx)-A.get(idx,jdx));
    }
  }
  
}

void MATLAB::plus_eq(double a) {
  for (int idx = 1;idx<=row_;idx++) {
    for (int jdx = 1;jdx<=col_;jdx++) {
      set(idx,jdx,a+get(idx,jdx));
    }
  }  
}

void MATLAB::minus_eq(double a) {
  for (int idx = 1;idx<=row_;idx++) {
    for (int jdx = 1;jdx<=col_;jdx++) {
      set(idx,jdx,get(idx,jdx)-a);
    }
  } 
}

void MATLAB::inverse() {
  //First check to make sure the matrix is square
  if (row_ != col_) {
    printf("Error in inverse(). Matrix must be square var = %s \n",name_);
    return;
  }
  //Check and see if this is a diagonal matrix
  if (idiag_ == 1) {
    //Invert the diagonal
    for (int idx = 0;idx<row_;idx++) {
      data_[idx][idx] = 1.0/data_[idx][idx];
    }
    return;
  }
  else {
    //In this case the matrix is not diagonal
      //Check and see if matrix is a 2x2
      if (row_ == 2) {
	//!Compute 2x2 inverse
	double detA;
	detA = data_[0][0]*data_[1][1] - data_[1][0]*data_[0][1];
	if (fabs(detA) > 1e-25) {
	  double temp00 = data_[0][0];
	  data_[0][0] = data_[1][1]/detA;
	  data_[1][1] = temp00/detA;
	  data_[0][1] = -data_[0][1]/detA;
	  data_[1][0] = -data_[1][0]/detA;
	}
	else
	  {
	    printf("Error in inverse(). Matrix is singular var = %s \n",name_);
	    return;
	  }
      } else if (row_ == 3) {
	//Compute 3x3 Inverse which has a closed form solution
	//Code below was computed based on using syms in MATLAB and running inv()
	double a = data_[0][0];
	double b = data_[0][1];
	double c = data_[0][2];
	double d = data_[1][0];
	double e = data_[1][1];
	double f = data_[1][2];
	double g = data_[2][0];
	double h = data_[2][1];
	double ii = data_[2][2];
	double deti = (a*f*h - b*f*g - c*d*h + c*e*g - a*e*ii + b*d*ii);
	if (fabs(deti) > 1e-25) {
	  data_[0][0] = (f*h - e*ii)/deti;
	  data_[1][0] = -(f*g - d*ii)/deti;
	  data_[2][0] = -(d*h - e*g)/deti;
	  data_[1][0] = -(c*h - b*ii)/deti;
	  data_[1][1] = (c*g - a*ii)/deti;
	  data_[1][2] = (a*h - b*g)/deti;
	  data_[2][0] = -(b*f - c*e)/deti;
	  data_[2][1] = (a*f - c*d)/deti;
	  data_[2][2] = -(a*e - b*d)/deti;
	}
	else {
	  printf("Error in inverse(). Matrix is singular var = %s \n",name_);
	  return;
	}
      }
      else {
	printf("Error in inverse(). Matrix bigger than a 3x3. var = %s \n",name_);
	printf("You need to run matrix_inverse() instead. \n");
	return;
      }
    }
}

double MATLAB::norm() {
  if (ivec_) {
    int idx,len;
    double ans = 0;
    if (row_ == 1) {
      len = col_;
      for(idx=1;idx<=len;idx++)	{
	  ans = ans + SQUARE(get(1,idx));
	}
    } 
    else {
      len=row_;
      for(idx=1;idx<=len;idx++)	{
	ans = ans + SQUARE(get(idx,1));
      }
    }
    return(sqrt(ans));
  }
  else {
    printf("Error in Norm. Matrix is not a vector. var = %s \n",name_);
    return 0;
  }
}

void MATLAB::normalize() {
  double norm_val = norm();
  if (abs(norm_val) > 1e-20) {
    mult_eq(1/norm_val);
  } else {
    mult_eq(0);
  }
}

void MATLAB::cross_init(MATLAB a,MATLAB b,char name[]) {
  if ((a.row_ == 3) && (a.col_ == 1) && (b.row_ == 3) && (b.col_ == 1)) {
    double a1 = a.get(1,1);
    double a2 = a.get(2,1);
    double a3 = a.get(3,1);
    double b1 = b.get(1,1);
    double b2 = b.get(2,1);
    double b3 = b.get(3,1);
    init(3,1,name);
    set(1,1,-a3*b2 + a2*b3);
    set(2,1,a3*b1 - a1*b3);
    set(3,1,-a2*b1 + a1*b2);
    } 
  else {
    printf("Error in cross product. Vectors are not 3x1. var = %s,%s \n",a.name_,b.name_);
  }
}

void MATLAB::cross(MATLAB a,MATLAB b) {
  if (!init_) {
    printf("Error in cross(). [this] not initialized \n");
    return;
  }
  if ((a.row_ == 3) && (a.col_ == 1) && (b.row_ == 3) && (b.col_ == 1)) {
    double a1 = a.get(1,1);
    double a2 = a.get(2,1);
    double a3 = a.get(3,1);
    double b1 = b.get(1,1);
    double b2 = b.get(2,1);
    double b3 = b.get(3,1);
    set(1,1,-a3*b2 + a2*b3);
    set(2,1,a3*b1 - a1*b3);
    set(3,1,-a2*b1 + a1*b2);
    } 
  else {
    printf("Error in cross product. Vectors are not 3x1. var = %s,%s \n",a.name_,b.name_);
  }
}

void MATLAB::plus_cross_eq(MATLAB a,MATLAB b) {
  if (!init_) {
    printf("Error in plus_cross_eq(). [this] not initialized \n");
    return;
  }
  if ((a.row_ == 3) && (a.col_ == 1) && (b.row_ == 3) && (b.col_ == 1)) {
    double a1 = a.get(1,1);
    double a2 = a.get(2,1);
    double a3 = a.get(3,1);
    double b1 = b.get(1,1);
    double b2 = b.get(2,1);
    double b3 = b.get(3,1);
    set(1,1,get(1,1)-a3*b2 + a2*b3);
    set(2,1,get(2,1)+a3*b1 - a1*b3);
    set(3,1,get(3,1)-a2*b1 + a1*b2);
    } 
  else {
    printf("Error in cross product. Vectors are not 3x1. var = %s,%s \n",a.name_,b.name_);
  }
}

void MATLAB::matrixset(int col,MATLAB A) {
  //This will put a vector into a matrix in column "col"
  if (A.row_ != row_) {
    printf("Error in matrixset(). %s does not have the same number of rows as %s",A.name_,name_);
    return;
  }
  for (int idx = 1;idx<=A.row_;idx++) {
    set(idx,col,A.get(idx,1));
  }
}

void MATLAB::vecset(int in_start,int in_end,MATLAB A,int A_start) {
  for (int idx = in_start;idx<=in_end;idx++) {
    set(idx,1,A.get(A_start,1));
    A_start++;
  }
}

void MATLAB::cross_eq(MATLAB a) { //this = this x a
  if ((a.row_ == 3) && (a.col_ == 1) && (row_ == 3) && (col_ == 1)) {
    double a1 = get(1,1);
    double a2 = get(2,1);
    double a3 = get(3,1);
    double b1 = a.get(1,1);
    double b2 = a.get(2,1);
    double b3 = a.get(3,1);
    set(1,1,-a3*b2 + a2*b3);
    set(2,1,a3*b1 - a1*b3);
    set(3,1,-a2*b1 + a1*b2);
    } 
  else {
    printf("Error in cross product. Vectors are not 3x1. var = %s,%s \n",name_,a.name_);
  }
}


void MATLAB::skew(MATLAB a) {
  mult_eq(0);
  if ((a.row_ == 3) && (a.col_ == 1) && (row_ == 3) && (col_ == 3)) {
    set(1,2,-a.get(3,1));
    set(1,3,a.get(2,1));
    set(2,1,a.get(3,1));
    set(2,3,-a.get(1,1));
    set(3,1,-a.get(2,1));
    set(3,2,a.get(1,1));
  }
  else {
    printf("Error in skew. Vectors are not 3x1. var = %s,%s \n",name_,a.name_);
    return;
  }
}

//Return the maximum absolute value and preserve the sign
double MATLAB::max_abs() {
  double val = 0;
  for (int a = 1;a<=row_;a++) {
    for (int b = 1;b<=col_;b++) {
      if (abs(get(a,b)) >= abs(val)) {
	  val = get(a,b);
	}
    }
  }
  return val;
}

//Compute the least squares matrix
void MATLAB::GaussLagrange(MATLAB A,char name[]) {
  //Solution is always a transpose of the input matrix
  zeros(A.col_,A.row_,name);
  
  //When you optimize the set of controls you have two different scenarios. If the matrix is square just return the inverse
  if (A.row_ == A.col_) {
    overwrite(A); //First copy A into output matrix
    inverse(); //then invert
    return;
  }
  //If the number of row is bigger than the number of columns it means you have more equations then unknowns
  //You must use Least Squares regression
  if (A.row_ > A.col_) {
    //Computing inv(A'*A)*A'
    printf("Error in Gauss. Have not coded least squares regression yet nor has it been tested \n");
    printf("I suggest contacting the current developer of this code to implement this. \n");
    return;
  }
  //If you have the opposite scenario where the number of columns is greater than rows it means you have
  //more unknowns than equations. You can't use least squares because multiple solutions exist. The easiest
  //way to solve this is to use the Lagrangian and maximize the -negative of the solution. That is the solution will
  //attempt to kick out an answer with all zeros unless it does not satisfy the contraint Ax = b. The full lagrangian looks like this
  // L = -x'*x + lambda'*(b-A*x)
  // This leads to the solutions x = A'*inv(A*A')*b 
  if (A.row_ < A.col_) {
    //Ok first things first. We need a copy of the matrix but transposed
    MATLAB Atranspose;
    Atranspose.zeros(A.col_,A.row_,"Transposed");
    //A.disp();
    Atranspose.transpose_not_square(A);
    //Atranspose.disp();
    //Then we need to do A*A'
    MATLAB ATA;
    ATA.zeros(A.row_,A.row_,"A*A'");
    ATA.mult(A,Atranspose);
    //ATA.disp();
    //Then here we need to invert the matrix
    ATA.inverse();
    //ATA.disp();
    //Now we need to multiply A'*ATA
    mult(Atranspose,ATA);
    
    //Atranspose.size();
    //size();
    //ATA.size();
    //disp();
    //Atranspose.disp();
    //ATA.disp();
    return;
  }
  
}

void MATLAB::size() {
  printf("Matrix = %s, Rows = %d , Columns = %d \n",name_,row_,col_);
}

void MATLAB::parallel_axis_theorem(MATLAB r,double mass) {
  //So what do we need?
  MATLAB rskew,rskewT,Ishift;
  rskew.zeros(3,3,"rskew");
  rskew.skew(r);
  //Then transpose the matrix
  rskewT.zeros(3,3,"rskewT");
  rskewT.overwrite(rskew);
  rskewT.transpose();
  //Then multiply the two matrices together
  Ishift.mult_init(rskew,rskewT,"Parrallel Axis shift Matrix");
  //Multiply Ishift by mass
  Ishift.mult_eq(mass);
  //Then add everything together
  plus_eq(Ishift);
}

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner

/////////////////////////????DO NOT EDIT

////ARCHIVED FUNCTIONS FOR BACKWARDS COMPATIBILITY ---- DO NOT EDIT
void MATLAB::inv(MATLAB A,char name[])
{
  //First check to make sure the matrix is square
  if (A.row_ != A.col_) {
    printf("Error in inv(). Matrix must be square var = %s \n",A.name_);
    return;
  }
  zeros(A.row_,A.col_,name);
  //Check and see if A is a diagonal matrix
  if (A.idiag_ == 1) {
    //Invert the diagonal
    for (int idx = 0;idx<row_;idx++) {
      data_[idx][idx] = 1.0/A.data_[idx][idx];
    }
  }
  else
    {
      //Check and see if matrix is a 2x2
      if (row_ == 2) {
	//!Compute 2x2 inverse
	double detA;
	detA = A.data_[0][0]*A.data_[1][1] - A.data_[1][0]*A.data_[0][1];
	if (detA != 0) {
	  data_[0][0] = A.data_[1][1]/detA;
	  data_[1][1] = A.data_[0][0]/detA;
	  data_[0][1] = -A.data_[0][1]/detA;
	  data_[1][0] = -A.data_[1][0]/detA;
	}
	else
	  {
	    printf("Error in inv(). Matrix is singular var = %s \n",A.name_);
	    return;
	  }
      }
      else {
	printf("Error in inv(). Matrix bigger than a 2x2. var = %s \n",A.name_);
	printf("A new function called inverse() has been created. I suggest using that function \n");
	return;
      }
    }
}

void MATLAB::copy_init(MATLAB A,char name[]) {
  zeros(A.row_,A.col_,name);
  for (int idx = 1;idx<=row_;idx++) {
    for (int jdx = 1;jdx<=col_;jdx++) {
      set(idx,jdx,A.get(idx,jdx));
    }
  }
}

void MATLAB::matrix_multiply(double** M1, double** M2, double** Res, int dim11, int dim12, int dim22){
  // given M1 of dimension (dim11,dim12) and M2 of dimension (dim21,dim22) multiply to obtain a matrix Res of dimension (dim11,dim22)
  int i = 0, j = 0 ;
  int a = 0, b = 0 ;
  // initialize all of Res to zero
  for(i = 1; i <= dim11; i++){
    for(j = 1; j <= dim22; j++){
      Res[i][j] = 0.0 ;
    }
  }
  for(i = 1; i <= dim11; i++){
    for(j = 1; j <= dim22; j++){
			
      for(a = 1; a <= dim12; a++){
					
	Res[i][j] = Res[i][j] + M1[i][a]*M2[a][j] ;

      }
    }
  }
  return ;
}

////////////////////////////////////////////////////////////////
//
// Return magnitude of 2D vector (a,b)
//
////////////////////////////////////////////////////////////////
double MATLAB::pythag(double a, double b){

	double absa, absb ;
	absa = fabs(a) ;
	absb = fabs(b) ;
	if(absa > absb) return absa*sqrt(1.0 + DSQR(absb/absa)) ;
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0 + DSQR(absa/absb))) ;
}

////////////////////////////////////////////////////////////////
//
// Free memory created using dvector()
//
////////////////////////////////////////////////////////////////
void MATLAB::free_dvector(double *v, long nl, long nh)
/* free a double vector allocated with dvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void MATLAB::nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}

////////////////////////////////////////////////////////////////
//
// Return a vector with indices nl -> nh
//
////////////////////////////////////////////////////////////////
double* MATLAB::dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl+NR_END;
}

////////////////////////////////////////////////////////////////
//
// Return a matrix with indices nrl -> nrh, ncl -> nch
//
////////////////////////////////////////////////////////////////
double** MATLAB::dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	double **m;

	/* allocate pointers to rows */
	m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

////////////////////////////////////////////////////////////////
//
// Invert a square matrix A of dimension DIM, output INV
//
// AMATLAB = matrix to invert (in)
// Output written to self.
// DIM = number of rows or number of columns of AMATLAB
//
////////////////////////////////////////////////////////////////
void MATLAB::matrix_inverse(MATLAB AMATLAB, int DIM){

  if (!checkSizeCompatibility(AMATLAB,"matrix_inverse")) {
    return;
  }

  if (AMATLAB.row_ <= 3) {
    //Run the other version of inverse
    //First copy AMATLAB into self
    overwrite(AMATLAB);
    inverse();
    return;
  }

  //Allocate matrices for inverse
  double **A, **INV;
  A = dmatrix(1,DIM,1,DIM);
  INV = dmatrix(1,DIM,1,DIM);

  //Put AMATLAB in to A
  for (int i = 1;i<=DIM;i++){
    for (int j = 1;j<=DIM;j++){
      A[i][j] = AMATLAB.get(i,j);
    }
  }

  // A and AI must have been created with NR util's dmatrix function!
  int i, j ;
  double **a, **v, *w, **ut;
  
  a = dmatrix(1,DIM,1,DIM) ;
  v = dmatrix(1,DIM,1,DIM) ;
  ut = dmatrix(1,DIM,1,DIM) ;
  w = dvector(1,DIM); 
  
  // Copy A into a
  for(i = 1; i <= DIM; i++){
    for(j = 1; j<= DIM; j++){
      a[i][j] = A[i][j] ;
    }
  }

  //dmatrixprint(a,DIM,"a");
  
  // Perform SVD
  svdcmp(a, DIM, DIM, w, v);

  // Transpose U matrix
  for(i = 1; i <=DIM; i++){
    for (j = 1; j <=DIM; j++){
      ut[i][j] = a[j][i] ;
    }
  }
  // Check singular values and invert them
  for(i = 1; i <= DIM; i++){
    if(w[i] < 1e-6){
      //printf("\nWARNING: Matrix may be singular.\n");
      //w[i] = 0.0 ;
      printf("Error in matrix_inverse(). \n");
      printf("Matrix too close to singular.\n");
      printf("Singular value = %lf \n",w[i]);
      return;
    }
    else{
      w[i] = 1.0/w[i] ;
    }
  }
  
  // Build SIGMAt
  double **Sigmat = dmatrix(1,DIM,1,DIM) ;
  for(i = 1; i <=DIM; i++){
    for (j = 1; j <=DIM; j++){
      if(i != j){
	Sigmat[i][j] = 0.0 ;
      }else{
	Sigmat[i][j] = w[i] ;
      }
    }
  }

  // Multiply Sigma*ut
  double **R1 = dmatrix(1,DIM,1,DIM) ;
  matrix_multiply(Sigmat,ut,R1,DIM,DIM,DIM) ;
  
  // Find inverse
  matrix_multiply(v,R1,INV,DIM,DIM,DIM);

  // Free matrices
  free_dmatrix(a,1,DIM,1,DIM) ;
  free_dmatrix(v,1,DIM,1,DIM) ;
  free_dmatrix(ut,1,DIM,1,DIM) ;
  free_dvector(w,1,DIM) ;
  free_dmatrix(Sigmat,1,DIM,1,DIM) ;
  free_dmatrix(R1,1,DIM,1,DIM);

  //Once inverse is computed. Plase INV into self
  for (int i = 1;i<=DIM;i++) {
    for (int j = 1;j<=DIM;j++) {
      set(i,j,INV[i][j]);
    }
  }
}

void MATLAB::svdcmp(double **a, int m, int n, double w[], double **v) {
  int flag,i,its,j,jj,k,l,nm;
  double anorm,c,f,g,h,s,scale,x,y,z,*rv1;
  rv1=dvector(1,n);
  g=scale=anorm=0.0;
  for (i=1;i<=n;i++) {
    l=i+1;
    rv1[i]=scale*g;
    g=s=scale=0.0;
    if (i <= m) {
      for (k=i;k<=m;k++) scale += fabs(a[k][i]);
      if (scale) {
	for (k=i;k<=m;k++) {
	  a[k][i] /= scale;
	  s += a[k][i]*a[k][i];
	}
	f=a[i][i];
	g = -SIGN(sqrt(s),f);
	h=f*g-s;
	a[i][i]=f-g;
	for (j=l;j<=n;j++) {
	  for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
	  f=s/h;
	  for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
	}
	for (k=i;k<=m;k++) a[k][i] *= scale;
      }
    }
    w[i]=scale *g;
    g=s=scale=0.0;
    if (i <= m && i != n) {
      for (k=l;k<=n;k++) scale += fabs(a[i][k]);
      if (scale) {
	for (k=l;k<=n;k++) {
	  a[i][k] /= scale;
	  s += a[i][k]*a[i][k];
	}
	f=a[i][l];
	g = -SIGN(sqrt(s),f);
	h=f*g-s;
	a[i][l]=f-g;
	for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
	for (j=l;j<=m;j++) {
	  for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
	  for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
	}
	for (k=l;k<=n;k++) a[i][k] *= scale;
      }
    }
    anorm=FMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
  }
  for (i=n;i>=1;i--) {
    if (i < n) {
      if (g) {
	for (j=l;j<=n;j++)
	  v[j][i]=(a[i][j]/a[i][l])/g;
	for (j=l;j<=n;j++) {
	  for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
	  for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
	}
      }
      for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
    }
    v[i][i]=1.0;
    g=rv1[i];
    l=i;
  }
  for (i=IMIN(m,n);i>=1;i--) {
    l=i+1;
    g=w[i];
    for (j=l;j<=n;j++) a[i][j]=0.0;
    if (g) {
      g=1.0/g;
      for (j=l;j<=n;j++) {
	for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
	f=(s/a[i][i])*g;
	for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
      }
      for (j=i;j<=m;j++) a[j][i] *= g;
    } else for (j=i;j<=m;j++) a[j][i]=0.0;
    ++a[i][i];
  }
  for (k=n;k>=1;k--) {
    for (its=1;its<=30;its++) {
      flag=1;
      for (l=k;l>=1;l--) {
	nm=l-1;
	if ((double)(fabs(rv1[l])+anorm) == anorm) {
	  flag=0;
	  break;
	}
	if ((double)(fabs(w[nm])+anorm) == anorm) break;
      }
      if (flag) {
	c=0.0;
	s=1.0;
	for (i=l;i<=k;i++) {
	  f=s*rv1[i];
	  rv1[i]=c*rv1[i];
	  if ((double)(fabs(f)+anorm) == anorm) break;
	  g=w[i];
	  h=pythag(f,g);
	  w[i]=h;
	  h=1.0/h;
	  c=g*h;
	  s = -f*h;
	  for (j=1;j<=m;j++) {
	    y=a[j][nm];
	    z=a[j][i];
	    a[j][nm]=y*c+z*s;
	    a[j][i]=z*c-y*s;
	  }
	}
      }
      z=w[k];
      if (l == k) {
	if (z < 0.0) {
	  w[k] = -z;
	  for (j=1;j<=n;j++) v[j][k] = -v[j][k];
	}
	break;
      }
      if (its == 30) nrerror("no convergence in 30 svdcmp iterations");
      x=w[l];
      nm=k-1;
      y=w[nm];
      g=rv1[nm];
      h=rv1[k];
      f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
      g=pythag(f,1.0);
      f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
      c=s=1.0;
      for (j=l;j<=nm;j++) {
	i=j+1;
	g=rv1[i];
	y=w[i];
	h=s*g;
	g=c*g;
	z=pythag(f,h);
	rv1[j]=z;
	c=f/z;
	s=h/z;
	f=x*c+g*s;
	g = g*c-x*s;
	h=y*s;
	y *= c;
	for (jj=1;jj<=n;jj++) {
	  x=v[jj][j];
	  z=v[jj][i];
	  v[jj][j]=x*c+z*s;
	  v[jj][i]=z*c-x*s;
	}
	z=pythag(f,h);
	w[j]=z;
	if (z) {
	  z=1.0/z;
	  c=f*z;
	  s=h*z;
	}
	f=c*g+s*y;
	x=c*y-s*g;
	for (jj=1;jj<=m;jj++) {
	  y=a[jj][j];
	  z=a[jj][i];
	  a[jj][j]=y*c+z*s;
	  a[jj][i]=z*c-y*s;
	}
      }
      rv1[l]=0.0;
      rv1[k]=f;
      w[k]=x;
    }
  }
  free_dvector(rv1,1,n);
}

void MATLAB::free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by dmatrix() */
{
  free((FREE_ARG) (m[nrl]+ncl-NR_END));
  free((FREE_ARG) (m+nrl-NR_END));
}
