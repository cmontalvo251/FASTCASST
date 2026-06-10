///////////////////////////////////////////////////////////////////////////////////
///                                                                             ///
///       Mult-Magnetic-Aircraft Connection Simulation Tool - (MMACS)           ///
///                                                                             ///
///       written by: Carlos Montalvo - Fall 2011                               ///
///       Last Update: Spring 2016                                              ///
///                                                                             ///
///       a.) Soft Contact Model written by: Eric Beyer                         ///
///       b.) Magnet Force Simulation derived from: Jean-Paul Yonnet            ///
///       c.) Horseshoe Vortex Interaction: Anderson                            ///
///       d.) Hard Contact Model: Simple Rotational Spring Mass Damper System   ///
///       e.) Aerodynamic Model: Point Model = Jung PhD Thesis Georgia Tech     ///
///       f.) Aerodynamic Model: Panel Model = NACA Airfoils + Equivalence      ///
///       from Archive_MATLAB_Routines/Approximations                           ///
///       g.) Renderer: OpenGL - uses opengl.cpp, mathp.cpp, and MATLAB.cpp     ///
///       To use Renderer set RUN_VISUALIZER to true as a #define or -D         ///
///                                                                             ///
///////////////////////////////////////////////////////////////////////////////////

#include <string.h>
#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#ifdef RUN_VISUALIZER
#include "timer.h"
#include "opengl.h"
#include <boost/thread.hpp>
using namespace boost;
#endif
#ifdef JOYSTICK
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"
#define NAME_LENGTH 80
#endif
#include <fstream>
#include <unistd.h>

/******************Routines*************/

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static float maxarg1, maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ?\
        (maxarg1) : (maxarg2))
static int iminarg1, iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ?\
        (iminarg1) : (iminarg2))
static double dsqrarg;
#define DSQR(a) ((dsqrarg=(a)) == 0.0 ? 0.0 : dsqrarg*dsqrarg)

using namespace std;
time_t rawtime;
struct tm * ptm;

//////////////Functions/////////////////////////////////////

void inv2(double **invA,double **A){
  //!Compute 2x2 inverse
  double detA;
  detA = A[0][0]*A[1][1] - A[1][0]*A[0][1];
  if (detA != 0){
    invA[0][0] = A[1][1]/detA;
    invA[1][1] = A[0][0]/detA;
    invA[0][1] = -A[0][1]/detA;
    invA[1][0] = -A[1][0]/detA;
  }
  else
    {
      printf("Matrix is singular \ n");
    }
}

void matrixdisp(double **mat, int row, int col, char name[]) {
	printf("%s%s%s", name, " = ", "\n");
	int i;
	int j;
	for (i = 0; i < row; i++) {
		for (j = 0; j < col; j++) {
			printf("%lf%s", mat[i][j], " ");
		}
		printf("%s", "\n");
	}
	printf("%s", "\n");
}

void Get_Time() {
	time(&rawtime);
	ptm = gmtime(&rawtime);
}

void Print_Elapsed_Time() {
	int hourElapsed, minElapsed, secElapsed, day, dayElapsed;
	int ii, jj, seed, hour, min, start_sec, end_sec, iwl = 0;

	printf("--------------------------------------\n");
	char Months[12][4] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul",
			"Aug", "Sep", "Oct", "Nov", "Dec" };
	printf("Simulation Start Time:   %2d of %s %2d:%02d:%02d\n", ptm->tm_mday,
			Months[ptm->tm_mon], (ptm->tm_hour - 4) % 24, ptm->tm_min,
			ptm->tm_sec);
	day = (ptm->tm_mday);
	hour = (ptm->tm_hour - 4) % 24;
	min = ptm->tm_min;
	start_sec = ptm->tm_sec + min * 60 + hour * 3600 + day * 24 * 3600;
	time(&rawtime);
	ptm = gmtime(&rawtime);
	printf("Completion Time:  %2d of %s %2d:%02d:%02d\n", ptm->tm_mday,
			Months[ptm->tm_mon], (ptm->tm_hour - 4) % 24, ptm->tm_min,
			ptm->tm_sec);
	day = (ptm->tm_mday);
	hour = (ptm->tm_hour - 4) % 24;
	min = (ptm->tm_min);
	end_sec = ptm->tm_sec + min * 60 + hour * 3600 + day * 24 * 3600;
	dayElapsed = (end_sec - start_sec) / (3600 * 24);
	hourElapsed = (end_sec - start_sec - dayElapsed * 3600 * 24) / 3600;
	minElapsed = (end_sec - start_sec - hourElapsed * 3600
			- dayElapsed * 3600 * 24) / 60;
	secElapsed = (end_sec - start_sec - hourElapsed * 3600 - minElapsed * 60
			- dayElapsed * 3600 * 24);
	printf("Elapsed Time : %02dd:%02d:%02d:%02d\n", dayElapsed, hourElapsed,
			minElapsed, secElapsed);
	printf("--------------------------------------\n");
}

/**************************************  Globals  ************************************/
#define NSTATE 15				/* Number of 6DOF States */
#define NVERTS 34				/* Number of Vertices */
#define NVRTST 12				/* Number of States Per Vertex */
#define N NBODIES*NSTATE+NVRTST*NVERTS	/* Number of EOM */
///Globals for SERVO DELAY
#define TDELAY 0 //Number of control updates to wait before control is sent to aircraft/
#define PI 3.14159265358979		/* Define PI */
//#define GRAVITY 9.81000			/* Define Gravity */
//#define NOMINALWINGSPAN 2.04
//#define nchord 1
//#define PSO 1000
#define NBODIES 100 //Max number of bodies

//MPC variables
int HP, PMPC, MMPC, ND, P, M;
double **YCOMMANDS, **KMPC, **KCA, **UD, **KCASTATE, **YKCA, **YKCAD,
		**MPCSTATE, timestepD = 0.01;
double NOMINALMPC[12], **AMPC, **BMPC, **DMPC, **tempA, **CADMPC, **APOW,
		**ADMPC, **KCAD, **CD, LL, DD;
//Vortex Method variables
int WINGPANELS;
double **r_cgp_B, dxpanel, l0, dypanel, **E_cgp, **uvw_induced_B, **TBP,
		*Gammai, *wij_I, **rp_I, **uvw_induced_backup, **uvw1aircraft;
double **r_cgA_B, **r_cgB_B, f0[2], **vortexA_I, **vortexB_I, **VP_noInduced_B,
		**VP_P, **VP_B, **VP_I;
double **a, **vmi, *w, **ut, **INV, **Sigmat, **R1, *Gammaiprev, *Gammai1;
int NPANELS, COLDSTART;
///Transmission Delay Globals
int FCSUPDATE, CONTROLON, CONTROLTYPE;
double DEOLD[NBODIES][TDELAY + 1], DAOLD[NBODIES][TDELAY + 1],
		DROLD[NBODIES][TDELAY + 1], DTOLD[NBODIES][TDELAY + 1];
//Sensor Error Globals
double GPSNOISETYPE, PQRNOISETYPE, HNOISETYPE, ACCELNOISETYPE, VISNAVNOISETYPE,
		VISNAVMASTER;
double taupos, GPSupdate, N01[3], xbias[3], xnoise[3], GPSnext = 0, VisNav[6];
double ACCELupdate, ACCELnext = 0, ACCELUVWDOT[3][NBODIES],
  ACCELUVW[3][NBODIES], GPSUVW[3][NBODIES],C_m_0_T,Lift_Total,Drag_Total;
double GPSXYZ[6][NBODIES], GYROupdate, GYROnext = 0, GYROPQR[3][NBODIES];
double posBias[3], posNoise[3], velNoise[3], epsilonP, tauvel;
double epsilonV, xdotnoise[3], xdotbias[3], pqrdrift[3], HPTP[3][NBODIES];
double sigmanoise[3], sigmaturnon[3], sigmadrift[3], sigmah[3], biash[3];
double accelsigmanoise[3], accelsigmaturnon[3], accelsigmadrift[3];
double accelXYcross, accelYZcross, accelXZcross, accelmisalign[3];
double AccelScaleFactor[3], Orthogonalityaccel[3][3], Tgyroaccel[3][3];
double ScaleFactor[3], Orthogonality[3][3], XYcross, YZcross, XZcross;
double Tgyro[3][3], misalign1, misalign2, misalign3, pqrbias[3], pqrnoise[3];
double accelnoise[3], accelbias[3];
double sigmaturnonIO[3], XYcrossIO, YZcrossIO, XZcrossIO, misalign1IO,
		misalign2IO;
double misalign3IO, ScaleFactorIO[3], accelsigmaturnonIO[3],
		AccelScaleFactorIO[3];
double accelmisalignIO[3], accelXYcrossIO, accelYZcrossIO, accelXZcrossIO;
//General
int num_runs, MANUAL, UVWTURB[3], UVWTURBTOTAL, PQRTURB[3], TORQUEMODEL,
		VORTEXMODEL = 0, INTERACT, TIBCONSTANT, VORTEXCONTROLLER, ICONSTANT,
		WGUSTCONTROLLER;
int SMARTSIMULATING, CONTROLLER, MAPPING, INTCONTROLS, USESENSORS, IADAPT,
		PCONTROL, AEROMODEL, VCOUNTER, VSKIP, SOI, irun, LATERAL = 1, NEXT = 0;
int VERROR = 0, VERRORMAX = 1, MULTIBODY, MODE, FIXED, iwl = 0, prevWL = 0, WL =
		0;
double mass[NBODIES], SLCG, BLCG, WLCG, Iner[NBODIES][3][3],InerInv[NBODIES][3][3], WEIGHT, II[3][3],massMETA=0;
double **WIND, **WINDB, T0, WINDMAX = -1, **WINDGUST, MASSRATIO, INITIAL[N],
		LOOP = 0, elevG[NBODIES], GMIX[2 * NBODIES][2];
double **xkadapt, **wk, **xk1adapt, **Axk, **Buk, **uk, Nadapt[13], **D2C, **D,
  XOFFSET,YOFFSET,PSIOFFSET,elevGx[NBODIES], numconnection = 0;
double TCURRENT, TFINAL, TINITIAL, MAGNETSCALE, TCONTACT, TFILTER,
  ICONSTANTSCALE[3], *uOLD, *vOLD, *wOLD,GRAVITY;
int m, record_incr, reco, randcalls = 0;
int IGRAVITY, IBODYAERO, IBLADE, ICONTACT, IMAGNET;
int IMONTECARLO = 0, NUMMC, ICRASH, MCRUN = 0, CHECKCRASH;
int IDEBUG = 0, IWRF, GUSTS[3];
int IVERTS = NVERTS, ITURB;
double TIMESTEP, IWRFSCALE[3], Time[4], TURBLEVEL[3], IWINDSCALE = 0, AZIMUTH,
		TURBLENGTH[3];
double KV = 0.25, KP = 0.1, KD = 0.01, MASSMAGNET, DENMAGNET, FREQ[3], G1, G2,
		G3; //0.1
		/*System Variables */
double TBI[NBODIES][3][3], TIB[NBODIES][3][3], State[N], StateDot[N], StateE[N],
		StateEDot[N], MIND = 100, MAXF = 0, TINTB[NBODIES][3][3],
		TBINT[NBODIES][3][3], ContactList[4][NVERTS],
		PrevContactList[4][NVERTS];
/*Aerodynamic Variables*/
double C_L_0, C_D_0, C_m_0, C_D_u, C_L_alpha, C_D_alpha, C_m_alpha;
double C_m_alpha_dot, C_m_u, C_L_q, C_m_q, C_L_de, C_m_de, C_x_delThrust;
double C_y_beta, C_l_beta, C_n_beta, C_l_p, C_n_p, C_l_r, C_n_r, C_l_da, C_n_da;
double C_L_0t, C_L_alphat, C_D_0t, C_D_alphat, C_L_0v, C_L_alphav, C_D_0v,
  C_D_alphav,C_L_0f;
double C_L_qf, C_D_alphaf, C_y_betaf, C_y_pf, C_y_rf, C_l_betaf, C_l_pf, C_l_rf,
  C_m_alphaf, C_m_qf,C_L_alphaf,C_D_0f,C_m_0f;
double C_L_0i, C_L_alphai, C_D_0i, C_D_alphai, C_m_i;
double C_n_pf, C_n_betaf, C_n_rf, C_n_pf_alpha;
double C_y_dr, C_l_dr, C_n_dr, C_y_p, C_y_r, Sarea[NBODIES], wspan[NBODIES], c,
		Vtrim, mainwx, htailx, vtailx, vtailz;
double SH[NBODIES], SV[NBODIES], tailspan;
/*Control Variables */
int INDOOR = 0,AUTOPILOT=1,PRESS=0;
double TPATH, THOLD = 1, THOLDPRIME, TY, PSIPATH = 1, PSIHOLD = 0, PSIY = 0,
		WAYPOINT = 0, WAYPOINTOFFSET = 1, YDELT = 0, PHIDELT = 0, STARTY = 0,
		ZDELT = 0, KPZ;
double RADIUS = 400, ARCLENGTHPARENT = 10, ARCLENGTH = 10, SCOORD = 0,
		rpcPSI1OLD = 0, rpcPsi[3], rpcB[3], rpcInt[3];
double XYZYOUT[NBODIES][3], XYZUIN[NBODIES][3][2], PTPYOUT[NBODIES][3][2],
  PTPUIN[NBODIES][3][3];
double DE[NBODIES], DA[NBODIES], DELTHRUST[NBODIES], DR[NBODIES], XDELT = 0;
double DEU[NBODIES], DAU[NBODIES], DELTHRUSTU[NBODIES], DRU[NBODIES];
double ZCOMMAND[NBODIES], XCOMMAND[NBODIES], YCOMMAND[NBODIES];
double PHICOMMAND[NBODIES], THETACOMMAND[NBODIES], PSICOMMAND[NBODIES];
double PHIFILTERED[NBODIES], PHIFILTOLD[NBODIES], PHICOMOLD[NBODIES];
double UCOMMAND[NBODIES], VCOMMAND[NBODIES], WCOMMAND[NBODIES],
  PCOMMAND[NBODIES],zMETA;
double QCOMMAND[NBODIES], RCOMMAND[NBODIES], MSLOPE, BINTERCEPT, MAXTHRUST;
double PMAX,QMAX,KPPSI,NUM_COL=0,NUM_ROW=0,**I_b_6DOF,Ixx_6DOF,Iyy_6DOF,Izz_6DOF,xyzAC[2][NBODIES],RCG2AC[2][NBODIES],**GMETA;
double WNQ,WNP,**FMETA,**Gamma_META,**UFL,**Gt,**GGt,**GGtINV,**GGtINVUFL,**METACONTROLS,**Inv_b_6DOF;
/* Contact Variables */
double K[2][2], C[2][2], mu, dp, rbubble;
double rcg_vertex[3][NVERTS], tConnect, rVertex[3][NVERTS];
double rcg1_plane[3]; //vector from a/c1 to plane
double rcg1_vertex[3][NVERTS]; //vector from cg1 to vertex
int Hit = 0, numContacts, GOODCONTACT = 0, CONNECTION = 0, MAGCONSTANT = 0,
		connectiontype;
double obj[6];
double xobjL, yobjL, zobjL, xobjR, yobjR, zobjR, aobj, bobj, cobj;
double thetal, thetar;
double pconnection = 0, numtotal = 0, PERCENT[100], WLMAT[100];
/* Magnet Variables */
double J, rcg_magnet[3], rcg1_magnet[3], msize[3], msizeI[3], rmag[3],
		MAGDISTANCE = 100, Fconn;
int Nx, Ny, Nz;
//CONNECTION VARIABLES
double rB_Joint[100][2][3], KL[3], KR[3], CL[3], CR[3];
int CONNECTMAT[100][2], NUMCONNECTIONS, METABODIES;
//Aerodynamics
int ALTHOLD[NBODIES];
double F_A[NBODIES][3], M_A[NBODIES][3],LIFT_A[NBODIES],DRAG_A[NBODIES],ALFA_TOTAL;
//Force Variables
double F_M_B[NBODIES][3], M_C[NBODIES][3], F_C[NBODIES][3], F_M1_B1[3],
		F_M1_B[3], T_M1_B1[3], T_M_B[NBODIES][3];
//WRF Variables
//char* PATH="/Users/carlos/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/";
//char* PATH="/home/carlos/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/";
//char* PATH="/home/cmontalvo/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/";
//char* PATH="/media/sf_C_DRIVE/Root/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/";
char PATH[256];
char* temp = (char*) malloc(256);
int *markX, *markY, *markZ, markT, parameters[5];
int *markXT, *markYT, *markZT;
int bounds, boundflag, pathlength, dx, dy, ztop, boundsT, boundflagTURB;
double xshift = 0, yshift = 0, *xshiftT, *yshiftT;
char* U0name = (char*) malloc(256);
char* Udtname = (char*) malloc(256);
char* V0name = (char*) malloc(256);
char* Vdtname = (char*) malloc(256);
char* W0name = (char*) malloc(256);
char* Wdtname = (char*) malloc(256);
char* UTurbname = (char*) malloc(256);
char* VTurbname = (char*) malloc(256);
char* WTurbname = (char*) malloc(256);
/* File Names */
char inTimeFile[256], inBodyPropFile[256], inBodyAeroFile[256], inErrFile[256];
char inSpDampFile[256], inFlagFile[256], inSaveFile[256], inMagFile[256],
  inLogFile[256], inForceFile[256], inMCFile[256], inMCOUTFile[256],
  CONNFILE[256];
char inControlFile[256], inATMFile[256], inMPCFile[256], inConnFile[256],
  inControlOUTFile[256], inWindFile[256], inBLENDFileOUT[256],
  inNoiseOUT[256],inCoeffOUT[256],inSweepsLocation[256];
char inFuselageLocation[256],FilesIn[256],inFuse[256];
FILE *outfile = NULL;
FILE *lodfile = NULL;
FILE *ldfile = NULL;
FILE *noisefile = NULL;
FILE *adaptfile = NULL;
FILE *logfile = NULL;
FILE *propfile = NULL;
FILE *contactfile = NULL;
FILE *controlfile = NULL;
FILE *forcefile = NULL;
FILE *commandfile = NULL;
FILE *afile = NULL;
FILE *mcfile = NULL;
FILE *mcoutfile = NULL;
FILE *blendfile = NULL;
FILE *windoutfile = NULL;
FILE *linearfile = NULL;
FILE *nominalfile = NULL;
FILE *fmetafile = NULL;
int BODIES;

#if RUN_VISUALIZER
VISOBJECT statetime; //The opengl visualizer object
#endif

//Load Extra ToolBoxes
#define NR_END 1
#define FREE_ARG char*

void file_error(char file[],char type[]){
  printf("The following text file been specified incorrectly in %s \n",FilesIn);
  printf("FILE = %s TYPE = %s \n",file,type);
  printf("Please close the command window and change the path name\n");
  exit(1);
}

double randNormal() {
	//random normal
	double GaussNum = 0.0;
	int NumInSum = 10;
	for (int i = 0; i < NumInSum; i++) {
		GaussNum += ((double) rand() / (double) RAND_MAX - 0.5);
	}
	GaussNum = GaussNum * sqrt((double) 12 / (double) NumInSum);
	return GaussNum;

}

double randUniform() {
	//outputs a value between 0 and 1
	double out = 0.0;
	out = ((double) (rand() % 100) / (double) 50) - 1;
	return out;
}

void veceq(double*out, double*in, int row) {
	int ii;
	for (ii = 0; ii < row; ii++) {
		out[ii] = in[ii];
	}
}

double** matrixallocatedbl(int row, int col) {
	double **matout;
	matout = (double**) malloc(row * sizeof(double*));
	int i;
	for (i = 0; i < row; i++) {
		matout[i] = (double*) malloc(col * sizeof(double));
	}
	return matout;
}

double** eye(int row, int col) {
	double **mat;
	mat = matrixallocatedbl(row, col);
	int ihat, jhat;
	for (ihat = 0; ihat < col; ihat++) {
		for (jhat = 0; jhat < row; jhat++) {
			mat[ihat][jhat] = 0;
			if (ihat == jhat) {
				mat[ihat][jhat] = 1;
			}
		}
	}
	return mat;
}

void mateq(double**out, double**in, int row, int col) {
	int ii, jj;
	for (ii = 0; ii < row; ii++) {
		for (jj = 0; jj < col; jj++) {
			out[ii][jj] = in[ii][jj];
		}
	}
}

void mateqii(double out[][3][3], double in[][3][3], int row, int col,
		int body) {
	int ii, jj;
	for (ii = 0; ii < row; ii++) {
		for (jj = 0; jj < col; jj++) {
			out[body][ii][jj] = in[body][ii][jj];
		}
	}
}

double** matzeros(int row, int col) {
	double **mat;
	mat = matrixallocatedbl(row, col);
	int i, j;
	for (i = 0; i < row; i++) {
		for (j = 0; j < col; j++) {
			mat[i][j] = 0;
		}
	}
	return mat;
}

double* vecallocatedbl(int row) {
	int ii;
	double *vecout;
	vecout = (double*) malloc(row * sizeof(double));
	for (ii = 0; ii < row; ii++) {
		vecout[ii] = 0;
	}
	return vecout;
}

void normalize(double vhat[3], double v[3]) {
	vhat[0] = 0;
	vhat[1] = 0;
	vhat[2] = 0;
	double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (norm != 0) {
		vhat[0] = v[0] / norm;
		vhat[1] = v[1] / norm;
		vhat[2] = v[2] / norm;
	}
}

void vecminus(double C[], double A[], double B[], int row) {
	for (int ii = 0; ii < row; ii++) {
		C[ii] = A[ii] - B[ii];
	}
}

double dotproduct(double A[], double B[], int row) {
	//printf("A = %f %f %f \n",A[0],A[1],A[2]);
	//printf("B = %f %f %f \n",B[0],B[1],B[2]);
	double out = 0;
	double aii, bii, cii;
	int ii;
	for (ii = 0; ii < row; ii++) {
		aii = (double) A[ii];
		bii = (double) B[ii];
		cii = aii * bii;
		out += cii;
	}
	return out;
}

void cross(double z[3], double x[3], double y[3]) {
	z[0] = -x[2] * y[1] + x[1] * y[2];
	z[1] = x[2] * y[0] - x[0] * y[2];
	z[2] = -x[1] * y[0] + x[0] * y[1];
}

void GSchmidt(double Rout[3][3], double p1[3], double p2[3], double p3[3]) {
	Rout[0][0] = p1[0];
	Rout[0][1] = p1[1];
	Rout[0][2] = p1[2];
	Rout[1][0] = p2[0];
	Rout[1][1] = p2[1];
	Rout[1][2] = p2[2];
	Rout[2][0] = p3[0];
	Rout[2][1] = p3[1];
	Rout[2][2] = p3[2];
}

int find(double invec[], int row, double value) {
	int idx, counter;
	idx = 0;
	counter = row;
	while (idx < row) {
		if ((invec[idx] > value)) {
			counter = idx - 1;
			idx = row + 1;
		}
		idx++;
	}
	return counter;
}

double sign(double in)
{
  if (in < 0)
    {
      return -1;
    }
  if (in > 0)
    {
      return 1;
    }
  if (in == 0)
    {
      return 0;
    }

}

double sat(double S, double BL, int flag) {
	double y, out;
	if (flag) {
		y = S / BL;
	} else {
		y = S;
	}
	if (fabs(y) <= 1) {
		out = y;
	} else {
		out = copysign(1.0, y);
	}
	return out;

}

int* vecallocateint(int row) {
	int ii;
	int *vecout;
	vecout = (int*) malloc(row * sizeof(int));
	for (ii = 0; ii < row; ii++) {
		vecout[ii] = 0;
	}
	return vecout;
}

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	fprintf(stderr, "Numerical Recipes run-time error...\n");
	fprintf(stderr, "%s\n", error_text);
	//fprintf(stderr,"...now exiting to system...\n");
	//exit(1);

}

void matmult(double** mat, double** a, int rowa, int cola, double** b, int rowb,int colb) {
	//mat = a*b
	int mm, nn, ll;
	//zero out out matrix
	for (mm = 0; mm < rowa; mm++) {
		for (nn = 0; nn < colb; nn++) {
			mat[mm][nn] = 0;
		}
	}
	//double** mat = matzeros(rowa,colb);
	for (mm = 0; mm < rowa; mm++) {
		for (nn = 0; nn < colb; nn++) {
			for (ll = 0; ll < cola; ll++) {
				mat[mm][nn] = mat[mm][nn] + a[mm][ll] * b[ll][nn];
			}
		}
	}
	//return mat;
}

void matplusminus(double** outmat, double** a, double** b, double factor,int row, int col) {
	int i, j;
	//double**outmat = matzeros(row,col);
	for (i = 0; i < row; i++) {
		for (j = 0; j < col; j++) {
			outmat[i][j] = a[i][j] + factor * b[i][j];
		}
	}
	//return(outmat);

}

void free_dvector(double *v, long nl, long nh) {
	/* free a double vector allocated with dvector() */

	free((FREE_ARG) (v + nl - NR_END));
}

void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch) {
	/* free a double matrix allocated by dmatrix() */

	free((FREE_ARG) (m[nrl] + ncl - NR_END));
	free((FREE_ARG) (m + nrl - NR_END));
}

double **dmatrix(long nrl, long nrh, long ncl, long nch) {
	/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */

	long i, nrow = nrh - nrl + 1, ncol = nch - ncl + 1;
	double **m;

	/* allocate pointers to rows */
	m = (double **) malloc((size_t) ((nrow + NR_END) * sizeof(double*)));
	if (!m)
		nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl] = (double *) malloc(
			(size_t) ((nrow * ncol + NR_END) * sizeof(double)));
	if (!m[nrl])
		nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for (i = nrl + 1; i <= nrh; i++)
		m[i] = m[i - 1] + ncol;

	/* return pointer to array of pointers to rows */
	return m;

}

double *dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;

	v = (double *) malloc((size_t) ((nh - nl + 1 + NR_END) * sizeof(double)));
	if (!v)
		nrerror("allocation failure in dvector()");
	return v - nl + NR_END;

}

double pythag(double a, double b) {
	double absa, absb;
	absa = fabs(a);
	absb = fabs(b);
	if (absa > absb)
		return absa * sqrt(1.0 + DSQR(absb/absa));
	else
		return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + DSQR(absa/absb)));

}

void transpose_pre(double **outmat,double **inmat,int row,int col) {
  int i, j;
  for (i = 0; i < row; i++) {
    for (j = 0; j < col; j++) {
      outmat[j][i] = inmat[i][j];
    }
  }
}


double** transpose(double **inmat, int row, int col) {
	double **outmat;
	outmat = matrixallocatedbl(col, row);
	int i, j;
	for (i = 0; i < row; i++) {
		for (j = 0; j < col; j++) {
			outmat[j][i] = inmat[i][j];
		}
	}
	return outmat;

}

void svdcmp(double **a, int m, int n, double w[], double **v) {
	int flag, i, its, j, jj, k, l, nm;
	double anorm, c, f, g, h, s, scale, x, y, z, *rv1;

	rv1 = dvector(1, n);
	g = scale = anorm = 0.0;
	for (i = 1; i <= n; i++) {
		l = i + 1;
		rv1[i] = scale * g;
		g = s = scale = 0.0;
		if (i <= m) {
			for (k = i; k <= m; k++)
				scale += fabs(a[k][i]);
			if (scale) {
				for (k = i; k <= m; k++) {
					a[k][i] /= scale;
					s += a[k][i] * a[k][i];
				}
				f = a[i][i];
				g = -SIGN(sqrt(s),f);
				h = f * g - s;
				a[i][i] = f - g;
				for (j = l; j <= n; j++) {
					for (s = 0.0, k = i; k <= m; k++)
						s += a[k][i] * a[k][j];
					f = s / h;
					for (k = i; k <= m; k++)
						a[k][j] += f * a[k][i];
				}
				for (k = i; k <= m; k++)
					a[k][i] *= scale;
			}
		}
		w[i] = scale * g;
		g = s = scale = 0.0;
		if (i <= m && i != n) {
			for (k = l; k <= n; k++)
				scale += fabs(a[i][k]);
			if (scale) {
				for (k = l; k <= n; k++) {
					a[i][k] /= scale;
					s += a[i][k] * a[i][k];
				}
				f = a[i][l];
				g = -SIGN(sqrt(s),f);
				h = f * g - s;
				a[i][l] = f - g;
				for (k = l; k <= n; k++)
					rv1[k] = a[i][k] / h;
				for (j = l; j <= m; j++) {
					for (s = 0.0, k = l; k <= n; k++)
						s += a[j][k] * a[i][k];
					for (k = l; k <= n; k++)
						a[j][k] += s * rv1[k];
				}
				for (k = l; k <= n; k++)
					a[i][k] *= scale;
			}
		}
		anorm = FMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
	}
	for (i = n; i >= 1; i--) {
		if (i < n) {
			if (g) {
				for (j = l; j <= n; j++)
					v[j][i] = (a[i][j] / a[i][l]) / g;
				for (j = l; j <= n; j++) {
					for (s = 0.0, k = l; k <= n; k++)
						s += a[i][k] * v[k][j];
					for (k = l; k <= n; k++)
						v[k][j] += s * v[k][i];
				}
			}
			for (j = l; j <= n; j++)
				v[i][j] = v[j][i] = 0.0;
		}
		v[i][i] = 1.0;
		g = rv1[i];
		l = i;
	}
	for (i = IMIN(m,n); i >= 1; i--) {
		l = i + 1;
		g = w[i];
		for (j = l; j <= n; j++)
			a[i][j] = 0.0;
		if (g) {
			g = 1.0 / g;
			for (j = l; j <= n; j++) {
				for (s = 0.0, k = l; k <= m; k++)
					s += a[k][i] * a[k][j];
				f = (s / a[i][i]) * g;
				for (k = i; k <= m; k++)
					a[k][j] += f * a[k][i];
			}
			for (j = i; j <= m; j++)
				a[j][i] *= g;
		} else
			for (j = i; j <= m; j++)
				a[j][i] = 0.0;
		++a[i][i];
	}
	for (k = n; k >= 1; k--) {
		for (its = 1; its <= 30; its++) {
			flag = 1;
			for (l = k; l >= 1; l--) {
				nm = l - 1;
				if ((double) (fabs(rv1[l]) + anorm) == anorm) {
					flag = 0;
					break;
				}
				if ((double) (fabs(w[nm]) + anorm) == anorm)
					break;
			}
			if (flag) {
				c = 0.0;
				s = 1.0;
				for (i = l; i <= k; i++) {
					f = s * rv1[i];
					rv1[i] = c * rv1[i];
					if ((double) (fabs(f) + anorm) == anorm)
						break;
					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g * h;
					s = -f * h;
					for (j = 1; j <= m; j++) {
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y * c + z * s;
						a[j][i] = z * c - y * s;
					}
				}
			}
			z = w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j = 1; j <= n; j++)
						v[j][k] = -v[j][k];
				}
				break;
			}
			//if (its == 30) nrerror("no convergence in 30 svdcmp iterations");
			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
			g = pythag(f, 1.0);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g,f))) - h)) / x;
			c = s = 1.0;
			for (j = l; j <= nm; j++) {
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;
				for (jj = 1; jj <= n; jj++) {
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x * c + z * s;
					v[jj][i] = z * c - x * s;
				}
				z = pythag(f, h);
				w[j] = z;
				if (z) {
					z = 1.0 / z;
					c = f * z;
					s = h * z;
				}
				f = c * g + s * y;
				x = c * y - s * g;
				for (jj = 1; jj <= m; jj++) {
					y = a[jj][j];
					z = a[jj][i];
					a[jj][j] = y * c + z * s;
					a[jj][i] = z * c - y * s;
				}
			}
			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
	free_dvector(rv1, 1, n);
}

void matrix_multiply(double** M1, double** M2, double** Res, int dim11,
		int dim12, int dim22) {
	// given M1 of dimension (dim11,dim12) and M2 of dimension (dim21,dim22) multiply to obtain a matrix Res of dimension (dim11,dim22)

	int i = 0, j = 0;
	int a = 0, b = 0;

	// initialize all of Res to zero
	for (i = 1; i <= dim11; i++) {
		for (j = 1; j <= dim22; j++) {
			Res[i][j] = 0.0;
		}
	}

	for (i = 1; i <= dim11; i++) {
		for (j = 1; j <= dim22; j++) {
			for (a = 1; a <= dim12; a++) {
				Res[i][j] = Res[i][j] + M1[i][a] * M2[a][j];
			}
		}
	}

	return;
}

void matrix_inverse(double **INMAT, double**INVERSE, int DIM) {
	// A and AI must have been created with NR util's dmatrix function!

	int i, j, ii, jj;
	double **a, **vmi, *w, **ut, **INV;

	a = dmatrix(1, DIM, 1, DIM);
	vmi = dmatrix(1, DIM, 1, DIM);
	ut = dmatrix(1, DIM, 1, DIM);
	w = dvector(1, DIM);
	INV = dmatrix(1, DIM, 1, DIM);

	// Copy A into a
	for (i = 1; i <= DIM; i++) {
		for (j = 1; j <= DIM; j++) {
			a[i][j] = INMAT[i][j];
		}
	}

	// Perform SVD
	svdcmp(a, DIM, DIM, w, vmi);

	// Transpose U matrix
	for (i = 1; i <= DIM; i++) {
		for (j = 1; j <= DIM; j++) {
			ut[i][j] = a[j][i];
		}
	}

	// Check singular values and invert them
	for (i = 1; i <= DIM; i++) {
		if (w[i] < 1e-6) {
			//printf("\nWARNING: Matrix may be singular.\n");
			w[i] = 0.0;
			//printf("Matrix too close to singular.\n");
			//exit(1);
		} else {
			w[i] = 1.0 / w[i];
		}
	}

	// Build SIGMAt
	double **Sigmat = dmatrix(1, DIM, 1, DIM);
	for (i = 1; i <= DIM; i++) {
		for (j = 1; j <= DIM; j++) {
			if (i != j) {
				Sigmat[i][j] = 0.0;
			} else {
				Sigmat[i][j] = w[i];
			}
		}
	}

	// Multiply Sigma*ut
	double **R1 = dmatrix(1, DIM, 1, DIM);
	matrix_multiply(Sigmat, ut, R1, DIM, DIM, DIM);

	// Find inverse
	matrix_multiply(vmi, R1, INV, DIM, DIM, DIM);

	//Copy INV into INVERSE with indices that start with 0
	for (i = 1; i <= DIM; i++) {
		for (j = 1; j <= DIM; j++) {
			INVERSE[i][j] = INV[i][j];
		}
	}

	// Free matrices
	free_dmatrix(a, 1, DIM, 1, DIM);
	free_dmatrix(vmi, 1, DIM, 1, DIM);
	free_dmatrix(ut, 1, DIM, 1, DIM);
	free_dvector(w, 1, DIM);
	free_dmatrix(Sigmat, 1, DIM, 1, DIM);
	free_dmatrix(R1, 1, DIM, 1, DIM);
	free_dmatrix(INV, 1, DIM, 1, DIM);

}

//Density Routine
double Density()
{
  if (GRAVITY < 10)
    {
      //return(1.225);
      return(1.220281);
    }
  else
    {
      return(0.002378);
    }
}

//Defines for WRF Wind Environment
#define dim 40
#define dimT 500
#define tlength 601
double xcoord[dim], ycoord[dim], zcoord[dim], terrain[dim][dim],
		tcoord[tlength];
double U0[dim][dim][dim], Udt[dim][dim][dim], V0[dim][dim][dim],
		Vdt[dim][dim][dim];
double W0[dim][dim][dim], Wdt[dim][dim][dim];
double xcoordT[dimT], ycoordT[dimT];
double UTurb[dimT][dimT], VTurb[dimT][dimT], WTurb[dimT][dimT];

void uvwconstant(double x, int panel, int body) {
	int stepX, stepY, stepZ, stepT, extrapX, extrapY, extrapZ, extrapT;
	int tinterp, x1, x2, y1, y2, z1, z2, tt, ii;
	double uvw[3][2], xpts2[2], ypts2[2], zpts2[2], zpts1, xpts1, ypts1;
	double u8[8], v8[8], w8[8], u4[4], v4[4], w4[4], uslope, vslope, wslope;
	double u2[2], v2[2], w2[2], u, v, w, tpts[2], Lu, Lv, Lw, sigw, sigu, sigv;
	double ugo, vgo, wgo, alfaf;

	WIND[0][panel] = WIND[0][panel]
			+ 2.3 * ICONSTANTSCALE[0] * cos(FREQ[0] * x);
	WIND[1][panel] = WIND[1][panel]
			+ 2.3 * ICONSTANTSCALE[1] * cos(FREQ[1] * x);
	WIND[2][panel] = WIND[2][panel]
			+ 2.3 * ICONSTANTSCALE[2] * cos(FREQ[2] * x);

	//Rotate to body frame
	WINDB[0][panel] = TBI[body][0][0] * WIND[0][panel]
			+ TBI[body][0][1] * WIND[1][panel]
			+ TBI[body][0][2] * WIND[2][panel];
	WINDB[1][panel] = TBI[body][1][0] * WIND[0][panel]
			+ TBI[body][1][1] * WIND[1][panel]
			+ TBI[body][1][2] * WIND[2][panel];
	WINDB[2][panel] = TBI[body][2][0] * WIND[0][panel]
			+ TBI[body][2][1] * WIND[1][panel]
			+ TBI[body][2][2] * WIND[2][panel];

}

void Turbulence3D(double xstar, double ystar, int panel, int body) {
	int stepX, stepY, stepZ, stepT, extrapX, extrapY, extrapZ, extrapT;
	int tinterp, x1, x2, y1, y2, z1, z2, tt, ii, panelX, panelY, panelZ;
	double uvw[3][2], xpts2[2], ypts2[2], zpts2[2], zpts1, xpts1, ypts1;
	double u8[8], v8[8], w8[8], u4[4], v4[4], w4[4], uslope, vslope, wslope;
	double u2[2], v2[2], w2[2], u, v, w, tpts[2], Lu, Lv, Lw, sigw, sigu, sigv;
	double ugo, vgo, wgo, alfaf, pgo, qgo, rgo, tstar;

	//This is the 3D full field Dryden Turbulence model that was
	//generated using L = 5000 ft. wu = 10 wl = 10 N = 100
	//This is high altitude turbulence thus sigu=sigv=sigw

	///USE RANDOM XOFFSET AND YOFFSET to recreate xstar and ystar
	xstar = xstar*cos(PSIOFFSET) + ystar*sin(PSIOFFSET);
	ystar = -xstar*sin(PSIOFFSET) + ystar*cos(PSIOFFSET);

	//Add in Dryden Gust model
	//Use 2D interpolation
	stepX = 1;
	stepY = 1;
	stepZ = 1;
	stepT = 1;
	extrapX = 0;
	extrapY = 0;
	extrapZ = 0;
	extrapT = 0;
	xstar = xstar + xshiftT[panel];
	ystar = ystar + yshiftT[panel];
	tstar = TCURRENT;

	panelX = markXT[panel];
	panelY = markYT[panel];
	panelZ = markZT[panel];

	//%%Check X */
	if (panelX == dimT - 1) {
		panelX--;
	}
	if ((xstar >= xcoordT[panelX]) && (xstar <= xcoordT[panelX + 1])) {
		/*Your in between the markers so keep going */
	} else {
		//%Find panelX
		if (xstar > xcoordT[dimT - 1]) {
			//%use endpt
			panelX = dimT - 1;
			stepX = -1;
			extrapX = 1;
			xshiftT[panel] = -1.9 * xcoordT[dimT - 1] + xshiftT[panel];
		} else if (xstar < xcoordT[0]) {
			//%use starpt */
			panelX = 0;
			stepX = 1;
			extrapX = 1;
			xshiftT[panel] = 1.9 * xcoordT[dimT - 1] + xshiftT[panel];
		} else {
			panelX = find(xcoordT, dimT, xstar);
			if (panelX == dimT - 1) {
				panelX--;
			} else if (panelX <= -1) {
				panelX = 0;
			}
		}
	}

	/* %%Check Y */
	if (panelY == dimT - 1) {
		panelY--;
	}
	if ((ystar >= ycoordT[panelY]) && (ystar <= ycoordT[panelY + 1])) {
		//Your in between the markers so keep going */
	} else {
		//%Find panelY */
		if (ystar > ycoordT[dimT - 1]) {
			//use endpt */
			panelY = dimT - 1;
			stepY = -1;
			extrapY = 1;
			yshiftT[panel] = -1.9 * ycoordT[dimT - 1] + yshiftT[panel];
		} else if (ystar < ycoordT[0]) {
			panelY = 0;
			stepY = 1;
			extrapY = 1;
			yshiftT[panel] = 1.9 * ycoordT[dimT - 1] + yshiftT[panel];
		} else {
			panelY = find(ycoordT, dimT, ystar);
			if (panelY == dimT - 1) {
				panelY--;
			} else if (panelY <= -1) {
				panelY = 0;
			}
		}
	}

	//We start with 4 points
	x1 = panelX;
	x2 = panelX + stepX;
	y1 = panelY;
	y2 = panelY + stepY;
	z1 = panelZ;
	z2 = panelZ + stepZ;
	xpts2[0] = xcoordT[panelX];
	xpts2[1] = xcoordT[panelX + stepX];
	ypts2[0] = ycoordT[panelY];
	ypts2[1] = ycoordT[panelY + stepY];

	u4[0] = UTurb[y1][x1];
	u4[1] = UTurb[y1][x2];
	u4[2] = UTurb[y2][x2];
	u4[3] = UTurb[y2][x1];
	v4[0] = VTurb[y1][x1];
	v4[1] = VTurb[y1][x2];
	v4[2] = VTurb[y2][x2];
	v4[3] = VTurb[y2][x1];
	w4[0] = WTurb[y1][x1];
	w4[1] = WTurb[y1][x2];
	w4[2] = WTurb[y2][x2];
	w4[3] = WTurb[y2][x1];

	if (extrapY) {
		//%%You don't need to interpolate on y */
		ypts1 = ypts2[0];
		u2[0] = u4[0];
		u2[1] = u4[1];
		v2[0] = v4[0];
		v2[1] = v4[1];
		w2[0] = w4[0];
		w2[1] = w4[1];
		boundsT = 1;
	} else {
		//%%Interpolate between Y points(interpolate pts 1-2 and 3-4) */
		//%%Pts 1,4 : 2,3 */
		int cord1[2] = { 0, 1 };
		int cord2[2] = { 3, 2 };
		for (ii = 0; ii < 2; ii++) {
			uslope = (u4[cord2[ii]] - u4[cord1[ii]]) / (ypts2[1] - ypts2[0]);
			vslope = (v4[cord2[ii]] - v4[cord1[ii]]) / (ypts2[1] - ypts2[0]);
			wslope = (w4[cord2[ii]] - w4[cord1[ii]]) / (ypts2[1] - ypts2[0]);
			u2[ii] = uslope * (ystar - ypts2[0]) + u4[cord1[ii]];
			v2[ii] = vslope * (ystar - ypts2[0]) + v4[cord1[ii]];
			w2[ii] = wslope * (ystar - ypts2[0]) + w4[cord1[ii]];
		}
		ypts1 = ystar;
	}

	//%%%%interpX%%%%%%%%%%%% */
	if (extrapX) {
		//%%You don't need to interpolate on x */
		xpts1 = xpts2[0];
		u = u2[0];
		v = v2[0];
		w = w2[0];
		boundsT = 1;
	} else {
		//%%Interpolate between X points */
		uslope = (u2[1] - u2[0]) / (xpts2[1] - xpts2[0]);
		vslope = (v2[1] - v2[0]) / (xpts2[1] - xpts2[0]);
		wslope = (w2[1] - w2[0]) / (xpts2[1] - xpts2[0]);
		u = uslope * (xstar - xpts2[0]) + u2[0];
		v = vslope * (xstar - xpts2[0]) + v2[0];
		w = wslope * (xstar - xpts2[0]) + w2[0];
		xpts1 = xstar;
	}

	//Multiply by scale
	//printf("u v w %lf %lf %lf \n",u,v,w);
	WIND[0][panel] = WIND[0][panel] + TURBLEVEL[0] * u * 4.0;
	WIND[1][panel] = WIND[1][panel] + TURBLEVEL[1] * v * 4.0;
	WIND[2][panel] = WIND[2][panel] + TURBLEVEL[2] * w * 4.0;

	//Rotate to body frame
	WINDB[0][panel] = TBI[body][0][0] * WIND[0][panel]
			+ TBI[body][0][1] * WIND[1][panel]
			+ TBI[body][0][2] * WIND[2][panel];
	WINDB[1][panel] = TBI[body][1][0] * WIND[0][panel]
			+ TBI[body][1][1] * WIND[1][panel]
			+ TBI[body][1][2] * WIND[2][panel];
	WINDB[2][panel] = TBI[body][2][0] * WIND[0][panel]
			+ TBI[body][2][1] * WIND[1][panel]
			+ TBI[body][2][2] * WIND[2][panel];

	if (boundsT && boundflagTURB) {
		printf("Turbulence Boundary hit @ T = %lf XY = %lf , %lf \n", tstar,
				xstar, ystar);
		boundflagTURB = 0;
		COLDSTART = 1;
	}
	//Reset markers
	markXT[panel] = panelX;
	markYT[panel] = panelY;
	markZT[panel] = panelZ;

}

void uvwwindStatic(double xstar, double ystar, double zstar, double tstar,
		int panel, int body) {
	int stepX, stepY, stepZ, stepT, extrapX, extrapY, extrapZ, extrapT;
	int tinterp, x1, x2, y1, y2, z1, z2, tt, ii, panelX, panelY, panelZ;
	double uvw[3][2], xpts2[2], ypts2[2], zpts2[2], zpts1, xpts1, ypts1;
	double u8[8], v8[8], w8[8], u4[4], v4[4], w4[4], uslope, vslope, wslope;
	double u2[2], v2[2], w2[2], u, v, w, tpts[2], Lu, Lv, Lw, sigw, sigu, sigv;
	double ugo, vgo, wgo;

	/*   %%This function will take in x,y,z(m),t(sec) and location and  */
	/*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
	/*   %so many globals must be defined. location is a string that */
	/*   %contains the location of the data to be interpolated. */

	/*   %Note when you plug in zstar this code will compute height above */
	/*   %ground. So make sure you give this code the absolute height and */
	/*   %this code will compute height above ground. */

	xstar = xstar*cos(PSIOFFSET) + ystar*sin(PSIOFFSET);
	ystar = -xstar*sin(PSIOFFSET) + ystar*cos(PSIOFFSET);

	stepX = 1;
	stepY = 1;
	stepZ = 1;
	stepT = 1;
	extrapX = 0;
	extrapY = 0;
	extrapZ = 0;
	extrapT = 0;
	zstar = fabs(zstar);
	xstar = xstar + xshift;
	ystar = ystar + yshift;

	panelX = markX[panel];
	panelY = markY[panel];
	panelZ = markZ[panel];

	//%%Check X */
	if (panelX == dim - 1) {
		panelX--;
	}
	if ((xstar >= xcoord[panelX]) && (xstar <= xcoord[panelX + 1])) {
		/*Your in between the markers so keep going */
	} else {
		//%Find panelX
		if (xstar > xcoord[dim - 1]) {
			//%use endpt
			panelX = dim - 1;
			stepX = -1;
			extrapX = 1;
			xshift = -1.9 * xcoord[dim - 1] + xshift;
		} else if (xstar < xcoord[0]) {
			//%use starpt */
			panelX = 0;
			stepX = 1;
			extrapX = 1;
			xshift = 1.9 * xcoord[dim - 1] + xshift;
		} else {
			panelX = find(xcoord, dim, xstar);
			if (panelX == dim - 1) {
				panelX--;
			} else if (panelX <= -1) {
				panelX = 0;
			}
		}
	}

	/* %%Check Y */
	if (panelY == dim - 1) {
		panelY--;
	}
	if ((ystar >= ycoord[panelY]) && (ystar <= ycoord[panelY + 1])) {
		//Your in between the markers so keep going */
	} else {
		//%Find panelY */
		if (ystar > ycoord[dim - 1]) {
			//use endpt */
			panelY = dim - 1;
			stepY = -1;
			extrapY = 1;
			yshift = -1.9 * ycoord[dim - 1] + yshift;
		} else if (ystar < ycoord[0]) {
			panelY = 0;
			stepY = 1;
			extrapY = 1;
			yshift = 1.9 * ycoord[dim - 1] + yshift;
		} else {
			panelY = find(ycoord, dim, ystar);
			if (panelY == dim - 1) {
				panelY--;
			} else if (panelY <= -1) {
				panelY = 0;
			}
		}
	}

	if ((zstar >= zcoord[panelZ]) && (zstar <= zcoord[panelZ + 1])) {
		//%%Your in between the markers so keep going */
	} else {
		//%Find panelZ
		if (zstar > zcoord[dim - 1]) {
			//%use endpt */
			panelZ = dim - 1;
			stepZ = -1;
			extrapZ = 1;
		} else if (zstar < zcoord[0]) {
			panelZ = 0;
			stepZ = 1;
			extrapZ = 1;
		} else {
			panelZ = find(zcoord, dim, zstar);
			if (panelZ == dim - 1) {
				panelZ--;
			} else if (panelZ <= -1) {
				panelZ = 0;
			}
		}
	}

	//%Interpolate Spatially */
	//%%To start we have 8 discrete point (8 corners of a cube) */
	xpts2[0] = xcoord[panelX];
	xpts2[1] = xcoord[panelX + stepX];
	ypts2[0] = ycoord[panelY];
	ypts2[1] = ycoord[panelY + stepY];
	zpts2[0] = zcoord[panelZ];
	zpts2[1] = zcoord[panelZ + stepZ];
	x1 = panelX;
	x2 = panelX + stepX;
	y1 = panelY;
	y2 = (panelY + stepY);
	z1 = panelZ;
	z2 = panelZ + stepZ;

	u8[0] = U0[y1][x1][z1];
	u8[1] = U0[y1][x2][z1];
	u8[2] = U0[y2][x2][z1];
	u8[3] = U0[y2][x1][z1];
	u8[4] = U0[y1][x1][z2];
	u8[5] = U0[y1][x2][z2];
	u8[6] = U0[y2][x2][z2];
	u8[7] = U0[y2][x1][z2];
	v8[0] = V0[y1][x1][z1];
	v8[1] = V0[y1][x2][z1];
	v8[2] = V0[y2][x2][z1];
	v8[3] = V0[y2][x1][z1];
	v8[4] = V0[y1][x1][z2];
	v8[5] = V0[y1][x2][z2];
	v8[6] = V0[y2][x2][z2];
	v8[7] = V0[y2][x1][z2];
	w8[0] = W0[y1][x1][z1];
	w8[1] = W0[y1][x2][z1];
	w8[2] = W0[y2][x2][z1];
	w8[3] = W0[y2][x1][z1];
	w8[4] = W0[y1][x1][z2];
	w8[5] = W0[y1][x2][z2];
	w8[6] = W0[y2][x2][z2];
	w8[7] = W0[y2][x1][z2];

	//%%%%%interpZ%%%%%%%%%%%% */

	if (extrapZ) {
		//%%You don't need to interpolate on z and you can just use */
		//%%the values at panelZ or z1 */
		zpts1 = zpts2[0];
		u4[0] = u8[0];
		u4[1] = u8[1];
		u4[2] = u8[2];
		u4[3] = u8[3];
		v4[0] = v8[0];
		v4[1] = v8[1];
		v4[2] = v8[2];
		v4[3] = v8[3];
		w4[0] = w8[0];
		w4[1] = w8[1];
		w4[2] = w8[2];
		w4[3] = w8[3];
		bounds = 1;
	} else {
		//%%Interpolate Between Z points(interpolate pts 1-4 and 5-8) */
		//%Pts 1,5 : 2,6 : 3,7 : 4,8 */
		int coord1[4] = { 0, 1, 2, 3 };
		int coord2[4] = { 4, 5, 6, 7 };
		for (ii = 0; ii < 4; ii++) {
			uslope = (u8[coord2[ii]] - u8[coord1[ii]]) / (zpts2[1] - zpts2[0]);
			vslope = (v8[coord2[ii]] - v8[coord1[ii]]) / (zpts2[1] - zpts2[0]);
			wslope = (w8[coord2[ii]] - w8[coord1[ii]]) / (zpts2[1] - zpts2[0]);
			u4[ii] = uslope * (zstar - zpts2[0]) + u8[coord1[ii]];
			v4[ii] = vslope * (zstar - zpts2[0]) + v8[coord1[ii]];
			w4[ii] = wslope * (zstar - zpts2[0]) + w8[coord1[ii]];
		}
		zpts1 = zstar;
	}

	//%%%%%interpY%%%%%%%%%%% */

	if (extrapY) {
		//%%You don't need to interpolate on y */
		ypts1 = ypts2[0];
		u2[0] = u4[0];
		u2[1] = u4[1];
		v2[0] = v4[0];
		v2[1] = v4[1];
		w2[0] = w4[0];
		w2[1] = w4[1];
		bounds = 1;
	} else {
		//%%Interpolate between Y points(interpolate pts 1-2 and 3-4) */
		//%%Pts 1,4 : 2,3 */
		int cord1[2] = { 0, 1 };
		int cord2[2] = { 3, 2 };
		for (ii = 0; ii < 2; ii++) {
			uslope = (u4[cord2[ii]] - u4[cord1[ii]]) / (ypts2[1] - ypts2[0]);
			vslope = (v4[cord2[ii]] - v4[cord1[ii]]) / (ypts2[1] - ypts2[0]);
			wslope = (w4[cord2[ii]] - w4[cord1[ii]]) / (ypts2[1] - ypts2[0]);
			u2[ii] = uslope * (ystar - ypts2[0]) + u4[cord1[ii]];
			v2[ii] = vslope * (ystar - ypts2[0]) + v4[cord1[ii]];
			w2[ii] = wslope * (ystar - ypts2[0]) + w4[cord1[ii]];
		}
		ypts1 = ystar;
	}

	//%%%%interpX%%%%%%%%%%%% */
	if (extrapX) {
		//%%You don't need to interpolate on x */
		xpts1 = xpts2[0];
		u = u2[0];
		v = v2[0];
		w = w2[0];
		bounds = 1;
	} else {
		//%%Interpolate between X points */
		uslope = (u2[1] - u2[0]) / (xpts2[1] - xpts2[0]);
		vslope = (v2[1] - v2[0]) / (xpts2[1] - xpts2[0]);
		wslope = (w2[1] - w2[0]) / (xpts2[1] - xpts2[0]);
		u = uslope * (xstar - xpts2[0]) + u2[0];
		v = vslope * (xstar - xpts2[0]) + v2[0];
		w = wslope * (xstar - xpts2[0]) + w2[0];
		xpts1 = xstar;
	}

	//Store Wind in Inertial Frame
	WIND[0][panel] = (0.01 * u * IWRFSCALE[0] + 0.99 * uOLD[panel]);
	WIND[1][panel] = (0.01 * v * IWRFSCALE[1] + 0.99 * vOLD[panel]);
	WIND[2][panel] = (0.01 * w * IWRFSCALE[2] + 0.99 * wOLD[panel]);
	uOLD[panel] = WIND[0][panel];
	vOLD[panel] = WIND[1][panel];
	wOLD[panel] = WIND[2][panel];
	//printf("u v w %lf %lf %lf \n",WIND[0][panel],WIND[1][panel],WIND[2][panel]);
	//Rotate to Body Frame
	WINDB[0][panel] = TBI[body][0][0] * WIND[0][panel]
			+ TBI[body][0][1] * WIND[1][panel]
			+ TBI[body][0][2] * WIND[2][panel];
	WINDB[1][panel] = TBI[body][1][0] * WIND[0][panel]
			+ TBI[body][1][1] * WIND[1][panel]
			+ TBI[body][1][2] * WIND[2][panel];
	WINDB[2][panel] = TBI[body][2][0] * WIND[0][panel]
			+ TBI[body][2][1] * WIND[1][panel]
			+ TBI[body][2][2] * WIND[2][panel];

	if (bounds && boundflag) {
		printf("You went out of bounds at T = %lf XYZ = %lf , %lf , %lf \n",
				tstar, xstar, ystar, zstar);
		boundflag = 0;
	}
	//Reset markers
	markX[panel] = panelX;
	markY[panel] = panelY;
	markZ[panel] = panelZ;

}

void importwind(double outmat[dim][dim][dim], char* file) {
	FILE* fid = NULL;
	int ii, jj, kk, nii, njj;
	double inmat[dim][dim][dim];
	fid = fopen(file, "r");
	if (fid == NULL) {
		printf("Wind Data File defined incorrectly \n");
		printf("%s \n", file);
		exit(1);
	}
	for (jj = 0; jj < dim; jj++) {
		for (ii = 0; ii < dim; ii++) {
			for (kk = 0; kk < dim; kk++) {
				fscanf(fid, "%lf", &inmat[ii][kk][jj]);
				outmat[ii][kk][jj] = inmat[ii][kk][jj];
			}
		}
	}
	fclose(fid);
}

void importTurb(double outmat[dimT][dimT], char* file) {
	FILE* fid = NULL;
	int ii, jj, kk, nii, njj;
	double inmat[dimT][dimT];
	//printf("Importing File = %s \n",file);
	fid = fopen(file, "r");
	if (fid == NULL) {
		printf("Turbulence Data File defined incorrectly \n");
		printf("%s \n", file);
		exit(1);
	}
	for (ii = 0; ii < dimT; ii++) {
		for (jj = 0; jj < dimT; jj++) {
			fscanf(fid, "%lf ", &inmat[ii][jj]);
			outmat[ii][jj] = inmat[ii][jj];
		}
	}
	fclose(fid);
}

void UVWTURBSTARTUP() {
	FILE *infile = NULL;
	int ii, jj;

	printf("Dryden Initialization \n");

	xshiftT = vecallocatedbl(NPANELS * NBODIES);
	yshiftT = vecallocatedbl(NPANELS * NBODIES);

	/* %%%%%%%%%%%Define Data Location%%%%%%%%%%%%%% */

	/* %%%%%%%%%%%Initialize Variables%%%%%%%%%%%%% */

	//Allocate memory for WRF interpolation
	markXT = vecallocateint(NPANELS * NBODIES);
	markYT = vecallocateint(NPANELS * NBODIES);
	markZT = vecallocateint(NPANELS * NBODIES);
	boundsT = 0;
	boundflagTURB = 1;

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/* %%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% */

	double dxT = 1;
	double dyT = 1;
	printf("Grid Size = %f %s \n", dxT, "m");

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/* %%%%%%%%%%%Create xcoord and ycoord%%%%%%% */
	for (ii = 0; ii < dimT; ii++) {
		xcoordT[ii] = -dxT * (dimT - 1) / 2 + ii * dxT;
		ycoordT[ii] = -dyT * (dimT - 1) / 2 + ii * dyT;
	}

	/* %%%%%%%Import Initial UVW matrices%%%%%%%%% */

	sprintf(temp, "%s", "Uturb.txt");
	strcpy(UTurbname, PATH);
	strcat(UTurbname, temp);
	sprintf(temp, "%s", "Vturb.txt");
	strcpy(VTurbname, PATH);
	strcat(VTurbname, temp);
	sprintf(temp, "%s", "Wturb.txt");
	strcpy(WTurbname, PATH);
	strcat(WTurbname, temp);

	importTurb(UTurb, UTurbname);
	importTurb(VTurb, VTurbname);
	importTurb(WTurb, WTurbname);

	printf("Turbulence Initialization Complete \n");

}

void UVWSTARTUP() {
	FILE *infile = NULL;
	int ii, jj;

	/* %%%%%%%%%%%Define Data Location%%%%%%%%%%%%%% */

	printf("Using Wind File = %s \n", PATH);

	/* %%%%%%%%%%%Initialize Variables%%%%%%%%%%%%% */

	//Allocate memory for WRF interpolation
	markX = vecallocateint(NPANELS * NBODIES);
	markY = vecallocateint(NPANELS * NBODIES);
	markZ = vecallocateint(NPANELS * NBODIES);
	markT = 0;
	bounds = 0;
	boundflag = 1;

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/* %%%%%%%%Import Extra Parameters File%%%%%%%%% */

	pathlength = strlen(PATH);
	char* ParametersFile = "Parameters.txt";
	char* inParameters = (char*) malloc(pathlength + strlen(ParametersFile));
	strcpy(inParameters, PATH);
	strcat(inParameters, ParametersFile);
	infile = fopen(inParameters, "r");
	//infile = fopen("/media/sf_C_DRIVE/Root/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/Parameters.txt","r");
	if (infile == NULL) {
		printf("Parameters.txt File defined incorrectly \n");
		printf("%s \n", inParameters);
		exit(1);
	}
	for (ii = 0; ii < 5; ii++) {
		fscanf(infile, "%d,", &parameters[ii]);
	}
	fclose(infile);

	char* ZcoordFile = "Zcoord.txt";
	char* inZcoord = (char*) malloc(pathlength + strlen(ZcoordFile));
	strcpy(inZcoord, PATH);
	strcat(inZcoord, ZcoordFile);
	infile = fopen(inZcoord, "r");
	if (infile == NULL) {
		printf("Zcoord.txt File defined incorrectly \n");
		printf("%s \n", inZcoord);
		exit(1);
	}
	for (ii = 0; ii < parameters[4]; ii++) {
		fscanf(infile, "%lf,", &zcoord[ii]);
	}
	fclose(infile);

	char* TimesFile = "SampleTimes.txt";
	char* inTimes = (char*) malloc(pathlength + strlen(TimesFile));
	strcpy(inTimes, PATH);
	strcat(inTimes, TimesFile);
	infile = fopen(inTimes, "r");
	//infile = fopen("/media/6ECCF5E3CCF5A58D_/Root/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/SampleTimes.txt","r");
	if (infile == NULL) {
		printf("SampleTimes.txt File defined incorrectly \n");
		printf("%s \n", inTimes);
		exit(1);
	}
	for (ii = 0; ii < 601; ii++) {
		fscanf(infile, "%lf,", &tcoord[ii]);
	}
	fclose(infile);

	char* HeightFile = "THeight.txt";
	char* inHeight = (char*) malloc(pathlength + strlen(HeightFile));
	strcpy(inHeight, PATH);
	strcat(inHeight, HeightFile);
	infile = fopen(inHeight, "r");
	if (infile == NULL) {
		printf("THeight.txt File defined incorrectly \n");
		printf("%s \n", inHeight);
		exit(1);
	}
	for (ii = 0; ii < dim; ii++) {
		for (jj = 0; jj < dim; jj++) {
			fscanf(infile, "%lf ", &terrain[ii][jj]);
		}
	}
	fclose(infile);

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/* %%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% */

	dx = parameters[0];
	dy = parameters[1];
	ztop = parameters[2];
	printf("Grid Size = %d %s \n", dx, "m");
	printf("Maximum Height = %d %s \n", ztop, "m");

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/* %%%%%%%%%%%Create xcoord and ycoord%%%%%%% */

	for (ii = 0; ii < dim; ii++) {
		xcoord[ii] = -(double) dx * (dim - 1) / 2 + ii * dx;
		ycoord[ii] = -(double) dy * (dim - 1) / 2 + ii * dy;
	}

	/* %%%%%%%Import Initial UVW matrices%%%%%%%%% */

	sprintf(temp, "%s%d%s", "U", (int) tcoord[0], ".txt");
	strcpy(U0name, PATH);
	strcat(U0name, temp);
	sprintf(temp, "%s%d%s", "U", (int) tcoord[1], ".txt");
	strcpy(Udtname, PATH);
	strcat(Udtname, temp);
	sprintf(temp, "%s%d%s", "V", (int) tcoord[0], ".txt");
	strcpy(V0name, PATH);
	strcat(V0name, temp);
	sprintf(temp, "%s%d%s", "V", (int) tcoord[1], ".txt");
	strcpy(Vdtname, PATH);
	strcat(Vdtname, temp);
	sprintf(temp, "%s%d%s", "W", (int) tcoord[0], ".txt");
	strcpy(W0name, PATH);
	strcat(W0name, temp);
	sprintf(temp, "%s%d%s", "W", (int) tcoord[1], ".txt");
	strcpy(Wdtname, PATH);
	strcat(Wdtname, temp);

	importwind(U0, U0name);
	importwind(Udt, Udtname);
	importwind(V0, V0name);
	importwind(Vdt, Vdtname);
	importwind(W0, W0name);
	importwind(Wdt, Wdtname);

	printf("UVW Initialization Complete \n");
}

void PanelModel(double Coeff[3], double ub, double vb, double wb) {
	//!Compute Angle of attack
	double alpha = 0;
	if (fabs(ub) > 0) {
		alpha = atan2(wb, ub);
	}
	Coeff[0] = C_L_0i + C_L_alphai * alpha;
	Coeff[1] = C_D_0i + C_D_alphai * alpha * alpha;
	Coeff[2] = C_m_i;
}

double* Horseshoe(double rC_I[3], double rA_I[3], double rB_I[3],
		double VP_I[3], double Gamma) {
	double coeff = Gamma / (4 * PI);
	double v1[3], rAB_I[3], p2[3], dot, p1[3], p1prime[3], p3[3], RGS[3][3],
			p2dot[3];
	double Vab_I[3], Vainf_I[3], Vbinf_I[3], rBC_I[3], rAC_H[3], rBC_H[3],
			rAC_I[3];

	normalize(v1, VP_I);

	//AB Contribution
	//%%%%%%%%%%%%AB CONTRIBUTION%%%%%%%%
	vecminus(rAB_I, rB_I, rA_I, 3);
	normalize(p2, rAB_I);
	double AB = sqrt(
			rAB_I[0] * rAB_I[0] + rAB_I[1] * rAB_I[1] + rAB_I[2] * rAB_I[2]);
	vecminus(rAC_I, rC_I, rA_I, 3);
	dot = dotproduct(rAC_I, p2, 3);
	p2dot[0] = dot * p2[0];
	p2dot[1] = dot * p2[1];
	p2dot[2] = dot * p2[2];
	vecminus(p1prime, rAC_I, p2dot, 3);
	normalize(p1, p1prime);
	cross(p3, p1, p2);
	GSchmidt(RGS, p1, p2, p3);
	rAC_H[0] = RGS[0][0] * rAC_I[0] + RGS[0][1] * rAC_I[1]
			+ RGS[0][2] * rAC_I[2];
	rAC_H[1] = RGS[1][0] * rAC_I[0] + RGS[1][1] * rAC_I[1]
			+ RGS[1][2] * rAC_I[2];
	rAC_H[2] = RGS[2][0] * rAC_I[0] + RGS[2][1] * rAC_I[1]
			+ RGS[2][2] * rAC_I[2];
	double rAC = sqrt(
			rAC_H[0] * rAC_H[0] + rAC_H[1] * rAC_H[1] + rAC_H[2] * rAC_H[2]);
	double rAC2 = sqrt(
			rAC_H[0] * rAC_H[0] + (-AB + rAC_H[1]) * (-AB + rAC_H[1])
					+ rAC_H[2] * rAC_H[2]);
	//rAC_H[2] should be zero
	double theta1, theta2, wab_H;

	//Assume Bound Vorticy provides zero contribution
	wab_H = -coeff / (rAC_H[0]) * (rAC_H[1] / rAC - (-AB + rAC_H[1]) / rAC2);
	//wab_H = 0;

	double Vab_H[3];
	Vab_H[0] = 0;
	Vab_H[1] = 0;
	Vab_H[2] = wab_H;
	Vab_I[0] = RGS[0][0] * Vab_H[0] + RGS[1][0] * Vab_H[1]
			+ RGS[2][0] * Vab_H[2];
	Vab_I[1] = RGS[0][1] * Vab_H[0] + RGS[1][1] * Vab_H[1]
			+ RGS[2][1] * Vab_H[2];
	Vab_I[2] = RGS[0][2] * Vab_H[0] + RGS[1][2] * Vab_H[1]
			+ RGS[2][2] * Vab_H[2];

	//%%%%%%%%%%%%Ainf CONTRIBUTION%%%%%%

	p2[0] = -v1[0];
	p2[1] = -v1[1];
	p2[2] = -v1[2];
	dot = dotproduct(rAC_I, p2, 3);
	p2dot[0] = dot * p2[0];
	p2dot[1] = dot * p2[1];
	p2dot[2] = dot * p2[2];
	vecminus(p1prime, rAC_I, p2dot, 3);
	normalize(p1, p1prime);
	cross(p3, p1, p2);
	GSchmidt(RGS, p1, p2, p3);
	rAC_H[0] = RGS[0][0] * rAC_I[0] + RGS[0][1] * rAC_I[1]
			+ RGS[0][2] * rAC_I[2];
	rAC_H[1] = RGS[1][0] * rAC_I[0] + RGS[1][1] * rAC_I[1]
			+ RGS[1][2] * rAC_I[2];
	rAC_H[2] = RGS[2][0] * rAC_I[0] + RGS[2][1] * rAC_I[1]
			+ RGS[2][2] * rAC_I[2];
	rAC = sqrt(rAC_H[0] * rAC_H[0] + rAC_H[1] * rAC_H[1] + rAC_H[2] * rAC_H[2]);

	wab_H = (coeff / rAC_H[0]) * (rAC_H[1] / rAC + 1);

	Vab_H[0] = 0;
	Vab_H[1] = 0;
	Vab_H[2] = wab_H;
	Vainf_I[0] = RGS[0][0] * Vab_H[0] + RGS[1][0] * Vab_H[1]
			+ RGS[2][0] * Vab_H[2];
	Vainf_I[1] = RGS[0][1] * Vab_H[0] + RGS[1][1] * Vab_H[1]
			+ RGS[2][1] * Vab_H[2];
	Vainf_I[2] = RGS[0][2] * Vab_H[0] + RGS[1][2] * Vab_H[1]
			+ RGS[2][2] * Vab_H[2];
	//%%%%%%%%%%%Binf CONTRIBUTION%%%%%%%
	vecminus(rBC_I, rC_I, rB_I, 3);
	dot = dotproduct(rBC_I, p2, 3);
	p2dot[0] = dot * p2[0];
	p2dot[1] = dot * p2[1];
	p2dot[2] = dot * p2[2];
	vecminus(p1prime, rBC_I, p2dot, 3);
	normalize(p1, p1prime);
	cross(p3, p1, p2);
	GSchmidt(RGS, p1, p2, p3);
	rBC_H[0] = RGS[0][0] * rBC_I[0] + RGS[0][1] * rBC_I[1]
			+ RGS[0][2] * rBC_I[2];
	rBC_H[1] = RGS[1][0] * rBC_I[0] + RGS[1][1] * rBC_I[1]
			+ RGS[1][2] * rBC_I[2];
	rBC_H[2] = RGS[2][0] * rBC_I[0] + RGS[2][1] * rBC_I[1]
			+ RGS[2][2] * rBC_I[2];
	double rBC = sqrt(
			rBC_H[0] * rBC_H[0] + rBC_H[1] * rBC_H[1] + rBC_H[2] * rBC_H[2]);

	wab_H = -(coeff / rBC_H[0]) * (rBC_H[1] / rBC + 1);

	Vab_H[0] = 0;
	Vab_H[1] = 0;
	Vab_H[2] = wab_H;
	Vbinf_I[0] = RGS[0][0] * Vab_H[0] + RGS[1][0] * Vab_H[1]
			+ RGS[2][0] * Vab_H[2];
	Vbinf_I[1] = RGS[0][1] * Vab_H[0] + RGS[1][1] * Vab_H[1]
			+ RGS[2][1] * Vab_H[2];
	Vbinf_I[2] = RGS[0][2] * Vab_H[0] + RGS[1][2] * Vab_H[1]
			+ RGS[2][2] * Vab_H[2];

	//%%Sum
	wij_I[0] = Vab_I[0] + Vainf_I[0] + Vbinf_I[0];
	wij_I[1] = Vab_I[1] + Vainf_I[1] + Vbinf_I[1];
	wij_I[2] = Vab_I[2] + Vainf_I[2] + Vbinf_I[2];

	//I'm pretty sure that wij_I[1] = 0
	//REVISIT REVISIT REVISIT
	// wij_I[1] = 0;

	return wij_I;
}

void LiftingLine(int numbody) {
	int iterate, iternum, ii, loc, body, jj, locjj, locii, mm, nn, itermax;
	double rho, c_bar, Spanel, cpanel, Vinf, Coeff[3], CL, S;
	double Axyz[3], Bxyz[3], uvwVP[3], Pxyz[3];

	//Reset induced velocity everytime
	for (ii = 0; ii < NPANELS * numbody; ii++) {
		Gammaiprev[ii] = 0;
		//assume initial condition is -Vinf*alfa
		uvw_induced_B[0][ii] = 0;
		uvw_induced_B[1][ii] = 0;
		uvw_induced_B[2][ii] = (double) 0.1509;
	}

	//Save a backup
	for (ii = 0; ii < NPANELS * numbody; ii++) {
		uvw_induced_backup[0][ii] = uvw_induced_B[0][ii];
		uvw_induced_backup[1][ii] = uvw_induced_B[1][ii];
		uvw_induced_backup[2][ii] = uvw_induced_B[2][ii];
	}

	////////ITERATE ON PANELS///////
	iterate = 1;
	iternum = 1;
	itermax = 10000;
	if (FIXED >= itermax)
	  {
	    itermax = FIXED+100;
	  }
	int itermin = 5;
	if (COLDSTART) {
		itermin = 10;
		COLDSTART = 0;
	}
	S = Sarea[0];
	c_bar = c;
	Spanel = S / WINGPANELS;
	cpanel = c_bar;
	double damping = 0.2;
	while (iterate) {
		for (ii = 0; ii < numbody; ii++) {
			for (jj = 0; jj < WINGPANELS; jj++) {
				loc = NPANELS * ii + jj;
				body = ii;
				//%%%%%%ADD IN THE CURRENT SOLUTION OF INDUCED VELOCITY%%%%%%%%%%
				//VP_B(:,ii) = VP_noInduced_B(:,ii) + uvw_induced_B(:,ii);
				VP_B[0][loc] = VP_noInduced_B[0][loc] - uvw_induced_B[0][loc];
				VP_B[1][loc] = VP_noInduced_B[1][loc] - uvw_induced_B[1][loc];
				VP_B[2][loc] = VP_noInduced_B[2][loc] - uvw_induced_B[2][loc];
				//VP_I(:,ii) = TIB*VP_B(:,ii);
				if (TIBCONSTANT) {
					VP_I[0][loc] = TINTB[body][0][0] * VP_B[0][loc]
							+ TINTB[body][0][1] * VP_B[1][loc]
							+ TINTB[body][0][2] * VP_B[2][loc];
					VP_I[1][loc] = TINTB[body][1][0] * VP_B[0][loc]
							+ TINTB[body][1][1] * VP_B[1][loc]
							+ TINTB[body][1][2] * VP_B[2][loc];
					VP_I[2][loc] = TINTB[body][2][0] * VP_B[0][loc]
							+ TINTB[body][2][1] * VP_B[1][loc]
							+ TINTB[body][2][2] * VP_B[2][loc];
				} else {
					VP_I[0][loc] = TIB[body][0][0] * VP_B[0][loc]
							+ TIB[body][0][1] * VP_B[1][loc]
							+ TIB[body][0][2] * VP_B[2][loc];
					VP_I[1][loc] = TIB[body][1][0] * VP_B[0][loc]
							+ TIB[body][1][1] * VP_B[1][loc]
							+ TIB[body][1][2] * VP_B[2][loc];
					VP_I[2][loc] = TIB[body][2][0] * VP_B[0][loc]
							+ TIB[body][2][1] * VP_B[1][loc]
							+ TIB[body][2][2] * VP_B[2][loc];
				}
				//VP_P(:,ii) = TPB(:,:,ii)*VP_B(:,ii);
				VP_P[0][loc] = VP_B[0][loc];
				VP_P[1][loc] = VP_B[1][loc];
				VP_P[2][loc] = VP_B[2][loc];
				//Vinf = norm(VP_P(:,ii));
				Vinf = sqrt(
						pow(VP_P[0][loc], 2) + pow(VP_P[1][loc], 2)
								+ pow(VP_P[2][loc], 2));
				//%%%%%Compute CL, CD at each panel%%%%%
				PanelModel(Coeff, VP_P[0][loc], VP_P[1][loc], VP_P[2][loc]);
				CL = Coeff[0];
				//%SectionLpanel = 1/2*rho*Vinf^2*cpanel*CL;
				//%%%%%%Compute Gamma at each panel%%%%%%
				//%Gammai = SectionLpanel/(rho*Vinf)
				Gammai[loc] = Vinf * cpanel * CL / 2;
				//Damp iterations
				Gammai[loc] = (1 - damping) * Gammaiprev[loc]
						+ damping * Gammai[loc];
			} //For loop around wing panels
		} //For loop around bodies

		double prevnorm = 0, norm = 0;
		for (ii = 0; ii < NPANELS * numbody; ii++) {
		  prevnorm = prevnorm + Gammaiprev[ii] * Gammaiprev[ii];
		  norm = norm + Gammai[ii] * Gammai[ii];
		}
		norm = sqrt(norm);
		prevnorm = sqrt(prevnorm);

		

		  //Test for Convergence
		if (FIXED > 0) {
			if (iternum >= FIXED) {
				iterate = 0;
				// printf("arg = %lf \n",fabs(norm-prevnorm));
			}
		} else {
			if ((fabs(norm - prevnorm) < 1e-10) && (iternum > itermin)) {
				iterate = 0;
				// printf("iternum = %d \n",iternum);
				// printf("arg = %lf \n",fabs(norm-prevnorm));
			}
		}
		veceq(Gammaiprev, Gammai, NPANELS * numbody);
		//%%%%%%Recompute w_induced%%%%%%%%%%%
		//%%This is a double sum around all the panels
		for (ii = 0; ii < numbody; ii++) {
			for (jj = 0; jj < WINGPANELS; jj++) {
				locii = NPANELS * ii + jj;
				//%locii = control point
				//uvw_induced_B(:,loc) = [0;0;0];
				uvw_induced_B[0][locii] = 0;
				uvw_induced_B[1][locii] = 0;
				uvw_induced_B[2][locii] = 0;
				for (mm = 0; mm < numbody; mm++) {
					for (nn = 0; nn < WINGPANELS; nn++) {
						locjj = NPANELS * mm + nn;
						//%%locjj = current horshoe vortex
						//%%vortexA_I = x,y,z coordinates of A point of vortex
						//%%vortexB_I = x,y,z coordinates of B point of vortex
						///%%VP_I = u,v,w coordinates of panel in inertial space
						Axyz[0] = vortexA_I[0][locjj];
						Axyz[1] = vortexA_I[1][locjj];
						Axyz[2] = vortexA_I[2][locjj];
						Bxyz[0] = vortexB_I[0][locjj];
						Bxyz[1] = vortexB_I[1][locjj];
						Bxyz[2] = vortexB_I[2][locjj];
						uvwVP[0] = VP_I[0][locjj];
						uvwVP[1] = VP_I[1][locjj];
						uvwVP[2] = VP_I[2][locjj];
						Pxyz[0] = rp_I[0][locii];
						Pxyz[1] = rp_I[1][locii];
						Pxyz[2] = rp_I[2][locii];
						wij_I = Horseshoe(Pxyz, Axyz, Bxyz, uvwVP,
								Gammai[locjj]);
						//uvw_induced_B(:,ii) = uvw_induced_B(:,ii) + TBI*wij_I;
						//Assume phi,theta,psi=phi0,theta0,psi0
						if (TIBCONSTANT) {
							uvw_induced_B[0][locii] = uvw_induced_B[0][locii]
									+ TBINT[ii][0][0] * wij_I[0]
									+ TBINT[ii][0][1] * wij_I[1]
									+ TBINT[ii][0][2] * wij_I[2];
							uvw_induced_B[1][locii] = uvw_induced_B[1][locii]
									+ TBINT[ii][1][0] * wij_I[0]
									+ TBINT[ii][1][1] * wij_I[1]
									+ TBINT[ii][1][2] * wij_I[2];
							uvw_induced_B[2][locii] = uvw_induced_B[2][locii]
									+ TBINT[ii][2][0] * wij_I[0]
									+ TBINT[ii][2][1] * wij_I[1]
									+ TBINT[ii][2][2] * wij_I[2];
						} else {
							uvw_induced_B[0][locii] = uvw_induced_B[0][locii]
									+ TBI[ii][0][0] * wij_I[0]
									+ TBI[ii][0][1] * wij_I[1]
									+ TBI[ii][0][2] * wij_I[2];
							uvw_induced_B[1][locii] = uvw_induced_B[1][locii]
									+ TBI[ii][1][0] * wij_I[0]
									+ TBI[ii][1][1] * wij_I[1]
									+ TBI[ii][1][2] * wij_I[2];
							uvw_induced_B[2][locii] = uvw_induced_B[2][locii]
									+ TBI[ii][2][0] * wij_I[0]
									+ TBI[ii][2][1] * wij_I[1]
									+ TBI[ii][2][2] * wij_I[2];
						}
					}
				}
			}
		} // end for loop on computing uvw_induced
		iternum++;
		// printf("iternum = %d \n",iternum);
		if (iternum == itermax) {
			if (VERROR < VERRORMAX) {
				VERROR++;
				TFINAL = 0;
				COLDSTART = 1;
				printf("Lifting Line Not Converged \n");
			}
			//Reset uvw_induced_B to backup value
			for (ii = 0; ii < NPANELS * numbody; ii++) {
				uvw_induced_B[0][ii] = uvw_induced_backup[0][ii];
				uvw_induced_B[1][ii] = uvw_induced_backup[1][ii];
				uvw_induced_B[2][ii] = uvw_induced_backup[2][ii];
				//Reset Gammaiprev
				Gammaiprev[ii] = 0;
			}
			iterate = 0;
		}
	} //while iterate

	// printf("Converged \n");
	// printf("iternum %d \n",iternum);
	// printf("uvw_induced_B = \n");
	// for (ii = 0;ii<NPANELS*BODIES;ii++)
	//   {
	//     printf("%f %f %f \n",uvw_induced_B[0][ii],uvw_induced_B[1][ii],uvw_induced_B[2][ii]);
	//   }
	// PAUSE();

}

void LiftingLineNoInteract(int numbody) {
	int iterate, iternum, ii, loc, body, jj, locjj, locii, mm, nn, itermax;
	double rho, c_bar, Spanel, cpanel, Vinf, Coeff[3], CL, S;
	double Axyz[3], Bxyz[3], uvwVP[3], Pxyz[3];

	//Reset induced velocity everytime
	for (ii = 0; ii < NPANELS * numbody; ii++) {
		Gammaiprev[ii] = 0;
		//assume initial condition is -Vinf*alfa
		uvw_induced_B[0][ii] = 0;
		uvw_induced_B[1][ii] = 0;
		uvw_induced_B[2][ii] = (double) 0.1509;
	}

	//Save a backup
	for (ii = 0; ii < NPANELS * numbody; ii++) {
		uvw_induced_backup[0][ii] = uvw_induced_B[0][ii];
		uvw_induced_backup[1][ii] = uvw_induced_B[1][ii];
		uvw_induced_backup[2][ii] = uvw_induced_B[2][ii];
	}

	////////ITERATE ON PANELS///////
	iterate = 1;
	iternum = 1;
	itermax = 1000;
	int itermin = 5;
	if (COLDSTART) {
		itermin = 10;
		COLDSTART = 0;
	}
	S = Sarea[0];
	c_bar = c;
	Spanel = S / WINGPANELS;
	cpanel = c_bar;
	double damping = 0.2;
	while (iterate) {
		for (ii = 0; ii < numbody; ii++) {
			for (jj = 0; jj < WINGPANELS; jj++) {
				loc = NPANELS * ii + jj;
				body = ii;
				//%%%%%%ADD IN THE CURRENT SOLUTION OF INDUCED VELOCITY%%%%%%%%%%
				//VP_B(:,ii) = VP_noInduced_B(:,ii) + uvw_induced_B(:,ii);
				VP_B[0][loc] = VP_noInduced_B[0][loc] - uvw_induced_B[0][loc];
				VP_B[1][loc] = VP_noInduced_B[1][loc] - uvw_induced_B[1][loc];
				VP_B[2][loc] = VP_noInduced_B[2][loc] - uvw_induced_B[2][loc];
				//VP_I(:,ii) = TIB*VP_B(:,ii);
				if (TIBCONSTANT) {
					VP_I[0][loc] = TINTB[body][0][0] * VP_B[0][loc]
							+ TINTB[body][0][1] * VP_B[1][loc]
							+ TINTB[body][0][2] * VP_B[2][loc];
					VP_I[1][loc] = TINTB[body][1][0] * VP_B[0][loc]
							+ TINTB[body][1][1] * VP_B[1][loc]
							+ TINTB[body][1][2] * VP_B[2][loc];
					VP_I[2][loc] = TINTB[body][2][0] * VP_B[0][loc]
							+ TINTB[body][2][1] * VP_B[1][loc]
							+ TINTB[body][2][2] * VP_B[2][loc];
				} else {
					VP_I[0][loc] = TIB[body][0][0] * VP_B[0][loc]
							+ TIB[body][0][1] * VP_B[1][loc]
							+ TIB[body][0][2] * VP_B[2][loc];
					VP_I[1][loc] = TIB[body][1][0] * VP_B[0][loc]
							+ TIB[body][1][1] * VP_B[1][loc]
							+ TIB[body][1][2] * VP_B[2][loc];
					VP_I[2][loc] = TIB[body][2][0] * VP_B[0][loc]
							+ TIB[body][2][1] * VP_B[1][loc]
							+ TIB[body][2][2] * VP_B[2][loc];
				}
				//VP_P(:,ii) = TPB(:,:,ii)*VP_B(:,ii); //Assume TPB = I
				VP_P[0][loc] = VP_B[0][loc];
				VP_P[1][loc] = VP_B[1][loc];
				VP_P[2][loc] = VP_B[2][loc];
				//Vinf = norm(VP_P(:,ii));
				Vinf = sqrt(
						pow(VP_P[0][loc], 2) + pow(VP_P[1][loc], 2)
								+ pow(VP_P[2][loc], 2));
				//%%%%%Compute CL, CD at each panel%%%%%
				PanelModel(Coeff, VP_P[0][loc], VP_P[1][loc], VP_P[2][loc]);
				CL = Coeff[0];
				//%SectionLpanel = 1/2*rho*Vinf^2*cpanel*CL;
				//%%%%%%Compute Gamma at each panel%%%%%%
				//%Gammai = SectionLpanel/(rho*Vinf)
				Gammai[loc] = Vinf * cpanel * CL / 2;
				//Damp iterations
				Gammai[loc] = (1 - damping) * Gammaiprev[loc]
						+ damping * Gammai[loc];
			} //For loop around wing panels
		} //For loop around bodies

		  //Test for Convergence
		if (FIXED > 0) {
			if (iternum >= FIXED) {
				iterate = 0;
			}
		} else {
			double prevnorm = 0, norm = 0;
			for (ii = 0; ii < NPANELS * numbody; ii++) {
				prevnorm = prevnorm + Gammaiprev[ii] * Gammaiprev[ii];
				norm = norm + Gammai[ii] * Gammai[ii];
			}
			norm = sqrt(norm);
			prevnorm = sqrt(prevnorm);
			if ((fabs(norm - prevnorm) < 1e-5) && (iternum > itermin)) {
				iterate = 0;
			}
		}
		veceq(Gammaiprev, Gammai, NPANELS * numbody);
		//%%%%%%Recompute w_induced%%%%%%%%%%%
		//%%This is a double sum around all the panels
		for (ii = 0; ii < numbody; ii++) {
			for (jj = 0; jj < WINGPANELS; jj++) {
				locii = NPANELS * ii + jj;
				//%locii = control point
				//uvw_induced_B(:,loc) = [0;0;0];
				uvw_induced_B[0][locii] = 0;
				uvw_induced_B[1][locii] = 0;
				uvw_induced_B[2][locii] = 0;
				mm = ii; //Only compute downwash on same aircraft
				for (nn = 0; nn < WINGPANELS; nn++) {
					locjj = NPANELS * mm + nn;
					//%%locjj = current horshoe vortex
					//%%vortexA_I = x,y,z coordinates of A point of vortex
					//%%vortexB_I = x,y,z coordinates of B point of vortex
					///%%VP_I = u,v,w coordinates of panel in inertial space
					Axyz[0] = vortexA_I[0][locjj];
					Axyz[1] = vortexA_I[1][locjj];
					Axyz[2] = vortexA_I[2][locjj];
					Bxyz[0] = vortexB_I[0][locjj];
					Bxyz[1] = vortexB_I[1][locjj];
					Bxyz[2] = vortexB_I[2][locjj];
					uvwVP[0] = VP_I[0][locjj];
					uvwVP[1] = VP_I[1][locjj];
					uvwVP[2] = VP_I[2][locjj];
					Pxyz[0] = rp_I[0][locii];
					Pxyz[1] = rp_I[1][locii];
					Pxyz[2] = rp_I[2][locii];
					wij_I = Horseshoe(Pxyz, Axyz, Bxyz, uvwVP, Gammai[locjj]);
					//uvw_induced_B(:,ii) = uvw_induced_B(:,ii) + TBI*wij_I;
					//Assume phi,theta,psi=phi0,theta0,psi0
					if (TIBCONSTANT) {
						uvw_induced_B[0][locii] = uvw_induced_B[0][locii]
								+ TBINT[ii][0][0] * wij_I[0]
								+ TBINT[ii][0][1] * wij_I[1]
								+ TBINT[ii][0][2] * wij_I[2];
						uvw_induced_B[1][locii] = uvw_induced_B[1][locii]
								+ TBINT[ii][1][0] * wij_I[0]
								+ TBINT[ii][1][1] * wij_I[1]
								+ TBINT[ii][1][2] * wij_I[2];
						uvw_induced_B[2][locii] = uvw_induced_B[2][locii]
								+ TBINT[ii][2][0] * wij_I[0]
								+ TBINT[ii][2][1] * wij_I[1]
								+ TBINT[ii][2][2] * wij_I[2];
					} else {
						uvw_induced_B[0][locii] = uvw_induced_B[0][locii]
								+ TBI[ii][0][0] * wij_I[0]
								+ TBI[ii][0][1] * wij_I[1]
								+ TBI[ii][0][2] * wij_I[2];
						uvw_induced_B[1][locii] = uvw_induced_B[1][locii]
								+ TBI[ii][1][0] * wij_I[0]
								+ TBI[ii][1][1] * wij_I[1]
								+ TBI[ii][1][2] * wij_I[2];
						uvw_induced_B[2][locii] = uvw_induced_B[2][locii]
								+ TBI[ii][2][0] * wij_I[0]
								+ TBI[ii][2][1] * wij_I[1]
								+ TBI[ii][2][2] * wij_I[2];
					}
				}
			}
		} // end for loop on computing uvw_induced
		iternum++;
		if (iternum == itermax) {
			if (VERROR < VERRORMAX) {
				VERROR++;
				TFINAL = 0;
				COLDSTART = 1;
				printf("Lifing Line Not Converged \n");
			}
			//Reset uvw_induced_B to backup value
			for (ii = 0; ii < NPANELS * numbody; ii++) {
				uvw_induced_B[0][ii] = uvw_induced_backup[0][ii];
				uvw_induced_B[1][ii] = uvw_induced_backup[1][ii];
				uvw_induced_B[2][ii] = uvw_induced_backup[2][ii];
				//Reset Gammaiprev
				Gammaiprev[ii] = 0;
			}
			iterate = 0;
		}
	} //while iterate

	// printf("Converged \n");
	// printf("iternum %d \n",iternum);
	// printf("uvw_induced_B = \n");
	// for (ii = 0;ii<NPANELS*BODIES;ii++)
	//   {
	//     printf("%f %f %f \n",uvw_induced_B[0][ii],uvw_induced_B[1][ii],uvw_induced_B[2][ii]);
	//   }
	// PAUSE();

}

void PanelForces(double BodyAeroLoads[], int body) {
	int ii, panel, jj;
	double x, y, ub, vb, wb, rho, rho0, Vstall, Torque, d, f, VortexCoeff[4],
			wd;
	double vel, drag, zerror, u, v, w, p, q, r, eff, factor2, C_L_alpha1,
			C_L_alpha2;
	double rxaero, ryaero, rzaero, Qa, rpm, Vthrust, omega, alfa, Q, Qt, deG[3];
	double alpha, beta, C_L, C_D, C_lmoment, C_m, C_n, C_l_vortex, factor1;
	double C_x, C_y, C_z, Lift, Drag, SOS, C_x_delV, rpmthr, rpmvel, drVortex,
			deGust;
	double C_L_alpha_alpha, Thrust, z, ailf, ruddf, thrustf, elevf, daVortex;
	double XFORCEi, YFORCEi, ZFORCEi, LMOMENTi, MMOMENTi, NMOMENTi;
	double xpanelB, ypanelB, zpanelB, uI, vI, wI;

	for (ii = 0; ii < 6; ii++) {
		BodyAeroLoads[ii] = 0.00;
	}

 	//printf("Computing AeroForces \n");

	ALFA_TOTAL = 0;

	//Unwrap State of CG
	x = State[body * NSTATE];
	y = State[body * NSTATE + 1];
	z = State[body * NSTATE + 2];
	u = State[body * NSTATE + 6];
	v = State[body * NSTATE + 7];
	w = State[body * NSTATE + 8];
	p = State[body * NSTATE + 9];
	q = State[body * NSTATE + 10];
	r = State[body * NSTATE + 11];

	/* Air Density, Speed of Sound, and Mach Number as a Function of Altitude */
	rho = Density();
	
	//What we are going to do is loop through all the panels on the 
	//main wing.
	//We are then going to add the effect of the horizontal tail
	//and vertical tails plus the fuselage
	for (ii = 0; ii < WINGPANELS + 3; ii++) {

	  //	  printf("ii = %d \n",ii);
		panel = body * NPANELS + ii;
		//Add in effect of Lifting Line model
		uI = 0;
		vI = 0;
		wI = 0;
		if (VORTEXMODEL) {
			if (ii < WINGPANELS) {
				uI = uvw_induced_B[0][panel];
				vI = uvw_induced_B[1][panel];
				wI = uvw_induced_B[2][panel];
			}
		}
		//Location of panel in body frame
		xpanelB = r_cgp_B[0][panel];
		ypanelB = r_cgp_B[1][panel];
		zpanelB = r_cgp_B[2][panel];
		//Add in induced velocity component
		VP_B[0][panel] = VP_noInduced_B[0][panel] - uI;
		VP_B[1][panel] = VP_noInduced_B[1][panel] - vI;
		VP_B[2][panel] = VP_noInduced_B[2][panel] - wI;
		//Rotate to panel frame (Assume Identity)
		ub = VP_B[0][panel];
		vb = VP_B[1][panel];
		wb = VP_B[2][panel];

		vel = sqrt(pow(ub, 2.0000) + pow(vb, 2.0000) + pow(wb, 2.0000));
		Qa = 0.5 * rho * pow(vel, 2.00);

		//Non-Dimensionalize p,q,r
		double phat = wspan[body] * p / (2 * vel);
		double qhat = c * q / (2 * vel);
		double rhat = wspan[body] * r / (2 * vel);

		//!Compute Angle of attack
		alpha = 0;
		if (fabs(ub) > 0) {
			alpha = atan2(wb, ub);
		}

		ALFA_TOTAL = alpha + ALFA_TOTAL;

		// if (ii < WINGPANELS) //DONT EVER DO THIS AGAIN
		//   {
		//     alpha = alpha + 0.004424;
		//   }

		// printf("Panel = %d \n",panel);
		// printf("Alpha = %lf \n",alpha);

		//!Slidelslip
		beta = 0;
		if (fabs(vel) > 0) {
			beta = asin(vb / vel);
		}

		//This is where things start to differ depending on the panel
		if (ii == WINGPANELS) {
		  //printf("Horizontal tail \n");
			C_L = C_L_0t + C_L_alphat * alpha;
			C_D = C_D_0t + C_D_alphat * alpha * alpha;
			Lift = Qa * SH[body] * C_L;
			Drag = Qa * SH[body] * C_D;
			LIFT_A[body] = LIFT_A[body] + Lift;
			DRAG_A[body] = DRAG_A[body] + Drag;
			XFORCEi = Lift * sin(alpha) - Drag * cos(alpha);
			YFORCEi = 0;
			ZFORCEi = -Lift;
			LMOMENTi = 0;
			MMOMENTi = 0;
			NMOMENTi = 0;
		} else if (ii == WINGPANELS + 1) {
		  //printf("Vertical Tail \n");
			zpanelB = zpanelB;
			xpanelB = xpanelB;
			beta = 0;
			if (fabs(vel) > 0) {
				beta = asin(vb / vel);
			}
			C_L = C_L_0v + C_L_alphav * beta;
			C_D = C_D_0v + C_D_alphav * beta * beta;
			Lift = Qa * SV[body] * C_L;
			Drag = Qa * SV[body] * C_D;
			LIFT_A[body] = LIFT_A[body] + 0;
			DRAG_A[body] = DRAG_A[body] + Drag;
			XFORCEi = -Drag;
			YFORCEi = -Lift;
			ZFORCEi = 0;
			LMOMENTi = 0;
			MMOMENTi = -zpanelB * XFORCEi + xpanelB * ZFORCEi;
			NMOMENTi = 0;
		} else if (ii == WINGPANELS + 2) {
		  //		  printf("Fuselage \n");
			C_L = C_L_0f + C_L_alphaf*alpha + C_L_qf * qhat + C_L_de * DE[body];
			C_D = C_D_0f + C_D_alphaf*alpha*alpha + C_D_u * ub;
			C_x = 0;
			// printf("C_y_pf phat %lf %lf \n",C_y_pf,phat);
			C_y = C_y_betaf * beta + C_y_dr * DR[body] + C_y_pf * phat + C_y_rf * rhat;
			C_z = 0;
			Lift = Qa * Sarea[body] * C_L;
			Drag = Qa * Sarea[body] * C_D;
			LIFT_A[body] = LIFT_A[body] + Lift;
			DRAG_A[body] = DRAG_A[body] + Drag;
			//Thrust Information
			T0 = 0.2882 + 4.8;
			if (Vtrim == 25) {
			  T0 = T0 + 0.28 * C_x_delThrust;
			} else if (Vtrim == 15) {
			  T0 = T0 - 0.14 * C_x_delThrust;
			} else if (Vtrim == 18) {
			  T0 = T0 - 0.066 * C_x_delThrust;
			} else if (Vtrim == 28) {
			  T0 = T0 + 0.48 * C_x_delThrust;
			} else if (Vtrim != 20) {
			  T0 = T0 + (0.0477 * (Vtrim - 20) + 0.1) * C_x_delThrust;
			}
			if (MODE == 3)
			  {
			    T0 = 0;
			  }
			Thrust = T0 + C_x_delThrust * DELTHRUST[body];
			XFORCEi = Lift * sin(alpha) - Drag * cos(alpha)
					+ Qa * Sarea[body] * C_x + Thrust;
			YFORCEi = Qa * Sarea[body] * C_y;
			// printf("YFORCEi = %lf \n",YFORCEi);
			ZFORCEi = -Lift * cos(alpha) - Drag * sin(alpha) + C_z;
			C_lmoment = C_l_rf*rhat + C_l_betaf * beta + C_l_pf * phat + C_l_da * DA[body] + C_l_dr * DR[body];
			C_m = C_m_0f + C_m_alphaf * alpha + C_m_qf * qhat + C_m_de * DE[body] + C_m_u * ub;
			C_n = C_n_betaf * beta + C_n_pf* phat + C_n_rf * rhat + C_n_da * DA[body] + C_n_dr * DR[body];
			LMOMENTi = C_lmoment * Qa * Sarea[body] * wspan[body]; 
			MMOMENTi = C_m * Qa * Sarea[body] * c;
			NMOMENTi = C_n * Qa * Sarea[body] * wspan[body];
			//			printf("Fuselage Forces \n");
		} else {
		  //printf("Main Wing \n");
			C_L = C_L_0i + C_L_alphai * alpha;
			C_D = C_D_0i + C_D_alphai * alpha * alpha;
			Lift = Qa * (Sarea[body] / WINGPANELS) * C_L;
			Drag = Qa * (Sarea[body] / WINGPANELS) * C_D;
			LIFT_A[body] = LIFT_A[body] + Lift;
			DRAG_A[body] = DRAG_A[body] + Drag;
			XFORCEi = Lift * sin(alpha) - Drag * cos(alpha);
			YFORCEi = 0;
			ZFORCEi = -Lift * cos(alpha) - Drag * sin(alpha);
			LMOMENTi = 0;
			MMOMENTi = Qa * (Sarea[body] / WINGPANELS) * c * C_m_i;
			NMOMENTi = 0;
		}

		// printf("CL = %lf \n",C_L);
		// printf("CD = %lf \n",C_D);
		
		BodyAeroLoads[0] = BodyAeroLoads[0] + XFORCEi;
		BodyAeroLoads[1] = BodyAeroLoads[1] + YFORCEi;
		BodyAeroLoads[2] = BodyAeroLoads[2] + ZFORCEi;

		//To compute the moments we take the contributions from the main wing the fuselage and the cross terms
		BodyAeroLoads[3] = BodyAeroLoads[3] + -zpanelB * YFORCEi
				+ ypanelB * ZFORCEi + LMOMENTi;
		BodyAeroLoads[4] = BodyAeroLoads[4] + zpanelB * XFORCEi
				- xpanelB * ZFORCEi + MMOMENTi;
		BodyAeroLoads[5] = BodyAeroLoads[5] + -ypanelB * XFORCEi
				+ xpanelB * YFORCEi + NMOMENTi;

	}

	//Compute Total Lift and Drag for L/D Analysis
	if (MODE == 3) {
	  // printf("Alpha = %lf \n",alpha);
		LL = BodyAeroLoads[0] * sin(alpha) - BodyAeroLoads[2] * cos(alpha);
		DD = -BodyAeroLoads[0] * cos(alpha) - BodyAeroLoads[2] * sin(alpha);
	}

}

void AeroForces(double BodyAeroLoads[], int body) {
	int ii;
	double x, y, ub, vb, wb, rho, rho0, Vstall, Torque, d, f, VortexCoeff[4],
			wd;
	double vel, drag, zerror, u, v, w, p, q, r, eff, factor2, C_L_alpha1,
			C_L_alpha2;
	double rxaero, ryaero, rzaero, Qa, rpm, Vthrust, omega, alfa, Q, Qt, deG[3];
	double alpha, beta, C_L, C_D, C_lmoment, C_m, C_n, C_l_vortex, factor1;
	double C_x, C_y, C_z, Lift, Drag, SOS, C_x_delV, rpmthr, rpmvel, drVortex,
			deGust;
	double C_L_alpha_alpha, Thrust, z, ailf, ruddf, thrustf, elevf, daVortex;

	//Unwrap State
	x = State[body * NSTATE];
	y = State[body * NSTATE + 1];
	z = State[body * NSTATE + 2];
	u = State[body * NSTATE + 6];
	v = State[body * NSTATE + 7];
	w = State[body * NSTATE + 8];
	p = State[body * NSTATE + 9];
	q = State[body * NSTATE + 10];
	r = State[body * NSTATE + 11];

	// v = 1;
	// w = -1;
	// p = 1;
	// q = -1;
	// r = 0.3;

	//Zero out Winds
	WIND[0][body] = 0;
	WIND[1][body] = 0;
	WIND[2][body] = 0;
	//Get Velocity of Winds
	if (IWRF)
		uvwwindStatic(x, y, z, TCURRENT, body, body);
	if (ITURB)
		Turbulence3D(x, y, body, body);
	if (ICONSTANT)
		uvwconstant(x, body, body);

	/* Velocity of the A/C cg with respect to the Atmosphere (in B frame) */
	ub = u - WINDB[0][body];
	vb = v - WINDB[1][body];
	wb = w - WINDB[2][body];
	vel = sqrt(pow(ub, 2.0000) + pow(vb, 2.0000) + pow(wb, 2.0000));

	//Non-Dimensionalize p,q,r
	double phat = wspan[body] * p / (2 * vel);
	double qhat = c * q / (2 * vel);
	double rhat = wspan[body] * r / (2 * vel);

	/* Air Density */
	rho = Density();

	//Dynamic Pressure
	Qa = 0.5 * rho * pow(vel, 2.00);

	//Thrust Information
	T0 = 0.2882 + 4.8;
	if (Vtrim == 25) {
		T0 = T0 + 0.28 * C_x_delThrust;
	} else if (Vtrim == 15) {
		T0 = T0 - 0.14 * C_x_delThrust;
	} else if (Vtrim == 18) {
		T0 = T0 - 0.066 * C_x_delThrust;
	} else if (Vtrim == 28) {
		T0 = T0 + 0.48 * C_x_delThrust;
	} else if (Vtrim != 20) {
		T0 = T0 + (0.0477 * (Vtrim - 20) + 0.1) * C_x_delThrust;
	}

	//!Compute Angle of attack
	alpha = 0;
	if (fabs(ub) > 0) {
		alpha = atan2(wb, ub);
	}

	//!Slidelslip
	beta = 0;
	if (fabs(vel) > 0) {
		beta = asin(vb / vel);
	}

	//REVISIT REVISIT REVISIT
	//Hardcode Control surfaces
	// DELTHRUST[body] = 0.09241;
	// DE[body] = 0.00773;
	// if (MODE == 4)
	//   {
	//     DELTHRUST[body] = 0.0;
	//     DE[body] = 0.0;
	//     DR[body] = 0.0;
	//     DA[body] = 0.0;
	//   }
	// DELTHRUST[body] = 0.0;
	// DE[body] = 0.0;
	// DR[body] = 0.0;
	// DA[body] = 0.0;
	
	//Lift and Drag
	C_L = C_L_0 + C_L_alpha * alpha + C_L_q * qhat + C_L_de * DE[body];
	C_D = C_D_0 + C_D_alpha * (pow(alpha, 2)) + C_D_u * ub;
	Lift = C_L * Qa * Sarea[body];
	Drag = C_D * Qa * Sarea[body];
	
	LIFT_A[body] = Lift;
	DRAG_A[body] = Drag;

	//XYZ Forces
	C_x = 0;
	C_y = C_y_beta * beta + C_y_dr * DR[body] + C_y_p * phat + C_y_r * rhat;
	C_z = 0;

	// if (MODE == 1)
	//   {
	//     T0 = 0;
	//   }

	if (MODE == 3)
	  {
	    T0 = 0;
	  }
	
	//printf("T0 = %lf \n",T0);

	Thrust = T0 + C_x_delThrust * DELTHRUST[body];

	//Moments
	// printf("DA = %lf \n",DA[body]);
	// printf("DR = %lf \n",DR[body]);
	// printf("DE = %lf \n",DE[body]);
	// printf("DELTHRUST = %lf \n",DELTHRUST[body]);
	C_lmoment = C_l_beta * beta + C_l_p * phat + C_l_r * rhat + C_l_da * DA[body] + C_l_dr * DR[body];
	C_m = C_m_0 + C_m_alpha * alpha + C_m_q * qhat + C_m_de * DE[body]
			+ C_m_u * ub;
	C_n = C_n_p * phat + C_n_beta * beta + C_n_r * rhat + C_n_da * DA[body]
			+ C_n_dr * DR[body];

	BodyAeroLoads[0] = Lift * sin(alpha) - Drag * cos(alpha)
			+ C_x * Qa * Sarea[body] + Thrust;
	BodyAeroLoads[1] = C_y * Qa * Sarea[body];
	BodyAeroLoads[2] = -Lift * cos(alpha) - Drag * sin(alpha)
			+ C_z * Qa * Sarea[body];

	BodyAeroLoads[3] = C_lmoment * Qa * Sarea[body] * wspan[body]; 
	BodyAeroLoads[4] = C_m * Qa * Sarea[body] * c; 
	BodyAeroLoads[5] = C_n * Qa * Sarea[body] * wspan[body];
        
        //printf("%lf \n",(BodyAeroLoads[4]/(Qa*Sarea[body]*c)-C_m_0)/qhat);
	//printf("%lf \n",(BodyAeroLoads[4]/(0.5*rho*pow(Vtrim,2)*Sarea[body]*c)-C_m_0)/qhat);
	// q = 1e-8;
	// qhat = c*q/(2*Vtrim);
	// printf("%lf \n",(BodyAeroLoads[4]/(0.5*rho*pow(Vtrim,2)*Sarea[0]*c)-C_m_0_T)/qhat);
	// printf("%lf \n",(BodyAeroLoads[4]/(0.5*rho*pow(Vtrim,2)*Sarea[0]*c)-C_m_0)/qhat);

	// printf("%0.20f \n",C_m_0-0.0598);
	// printf("%0.20f \n",C_m_0_T-0.0598);

	if (MODE == 3) {
	  // printf("Alpha = %lf \n",alpha);
		LL = BodyAeroLoads[0] * sin(alpha) - BodyAeroLoads[2] * cos(alpha);
		DD = -BodyAeroLoads[0] * cos(alpha) - BodyAeroLoads[2] * sin(alpha);
	}

	// PAUSE();

	// printf("Coefficients = Cl,Cm,Cn = %lf,%lf,%lf \n");

	// printf("rho = %lf \n",rho);

	// printf("Controls \n");
	// printf("Controls(DT,DE,DA,DR) = %lf %lf %lf %lf \n", DELTHRUST[ii],DE[ii], DA[ii], DR[ii]);

	// printf("State Vector \n");
	// for (ii = 0;ii<11;ii++){
	//   printf("%lf \n",State[body*NSTATE+ii]);
	// }

	// printf("BodyAeroLoads\n");
	// for (ii = 0;ii<=5;ii++)
	//   {
	//     printf("%lf \n",BodyAeroLoads[ii]);
	//   }
	// PAUSE();
}

void ComputeFG_w_VORTEX(double LMii[],int body,double Ixx,double Iyy) {
  int ii, panel, jj;
  double x, y, ub, vb, wb, rho, rho0, Vstall, Torque, d, f, VortexCoeff[4],
    wd;
  double vel, drag, zerror, u, v, w, p, q, r, eff, factor2, C_L_alpha1,
    C_L_alpha2;
  double rxaero, ryaero, rzaero, Qa, rpm, Vthrust, omega, alfa, Q, Qt, deG[3];
  double alpha, beta, C_L, C_D, C_lmoment, C_m, C_n, C_l_vortex, factor1;
  double C_x, C_y, C_z, Lift, Drag, SOS, C_x_delV, rpmthr, rpmvel, drVortex,
    deGust;
  double C_L_alpha_alpha, Thrust, z, ailf, ruddf, thrustf, elevf, daVortex;
  double XFORCEi, YFORCEi, ZFORCEi, LMOMENTi, MMOMENTi, NMOMENTi;
  double xpanelB, ypanelB, zpanelB, uI, vI, wI;
  double BodyAeroLoads[6];

  for (ii = 0; ii < 6; ii++) {
    BodyAeroLoads[ii] = 0.00;
  }

  //Unwrap State of CG
  x = State[body * NSTATE];
  y = State[body * NSTATE + 1];
  z = State[body * NSTATE + 2];
  u = State[body * NSTATE + 6];
  v = State[body * NSTATE + 7];
  w = State[body * NSTATE + 8];
  p = State[body * NSTATE + 9];
  q = State[body * NSTATE + 10];
  r = State[body * NSTATE + 11];

  /* Air Density, Speed of Sound, and Mach Number as a Function of Altitude */
  rho = Density();

  //What we are going to do is loop through all the panels on the 
  //main wing.
  //We are then going to add the effect of the horizontal tail
  //and vertical tails plus the fuselage
  for (ii = 0; ii < WINGPANELS + 3; ii++) {
    panel = body * NPANELS + ii;
    //Add in effect of Lifting Line model
    uI = 0;
    vI = 0;
    wI = 0;
    if (VORTEXMODEL) {
      if (ii < WINGPANELS) {
	uI = uvw_induced_B[0][panel];
	vI = uvw_induced_B[1][panel];
	wI = uvw_induced_B[2][panel];
      }
    }
    //Location of panel in body frame
    xpanelB = r_cgp_B[0][panel];
    ypanelB = r_cgp_B[1][panel];
    zpanelB = r_cgp_B[2][panel];
    //Add in induced velocity component
    VP_B[0][panel] = VP_noInduced_B[0][panel] - uI;
    VP_B[1][panel] = VP_noInduced_B[1][panel] - vI;
    VP_B[2][panel] = VP_noInduced_B[2][panel] - wI;
    //Rotate to panel frame (Assume Identity)
    ub = VP_B[0][panel];
    vb = VP_B[1][panel];
    wb = VP_B[2][panel];

    vel = sqrt(pow(ub, 2.0000) + pow(vb, 2.0000) + pow(wb, 2.0000));
    Qa = 0.5 * rho * pow(vel, 2.00);

    //Non-Dimensionalize p,q,r
    double phat = wspan[body] * p / (2 * vel);
    double qhat = c * q / (2 * vel);
    double rhat = wspan[body] * r / (2 * vel);

    //!Compute Angle of attack
    alpha = 0;
    if (fabs(ub) > 0) {
      alpha = atan2(wb, ub);
    }

    //!Slidelslip
    beta = 0;
    if (fabs(vel) > 0) {
      beta = asin(vb / vel);
    }

    //This is where things start to differ depending on the panel
    if (ii == WINGPANELS) {
      //Horizontal tail
      C_L = C_L_0t + C_L_alphat * alpha;
      C_D = C_D_0t + C_D_alphat * alpha * alpha;
      Lift = Qa * SH[body] * C_L;
      Drag = Qa * SH[body] * C_D;
      XFORCEi = Lift * sin(alpha) - Drag * cos(alpha);
      YFORCEi = 0;
      ZFORCEi = -Lift;
      LMOMENTi = 0;
      MMOMENTi = 0;
      NMOMENTi = 0;
    } else if (ii == WINGPANELS + 1) {
      //Vertical Tail
      zpanelB = zpanelB;
      xpanelB = xpanelB;
      beta = 0;
      if (fabs(vel) > 0) {
	beta = asin(vb / vel);
      }
      C_L = C_L_0v + C_L_alphav * beta;
      C_D = C_D_0v + C_D_alphav * beta * beta;
      Lift = Qa * SV[body] * C_L;
      Drag = Qa * SV[body] * C_D;
      XFORCEi = -Drag;
      YFORCEi = -Lift;
      ZFORCEi = 0;
      LMOMENTi = 0;
      MMOMENTi = -zpanelB * XFORCEi + xpanelB * ZFORCEi;
      NMOMENTi = 0;
    } else if (ii == WINGPANELS + 2) {
      //Fuselage
      C_L = C_L_0f + C_L_alphaf*alpha + C_L_qf * qhat + C_L_de * DE[body];
      C_D = C_D_0f + C_D_alphaf*alpha*alpha + C_D_u * ub;
      C_x = 0;
      C_y = C_y_betaf * beta + C_y_dr * DR[body] + C_y_pf * phat
	+ C_y_rf * rhat;
      C_z = 0;
      Lift = Qa * Sarea[body] * C_L;
      Drag = Qa * Sarea[body] * C_D;
      //cout << "T0 = " << T0 << endl;
      //Thrust Information
      T0 = 0.2882 + 4.8;
      if (Vtrim == 25) {
	T0 = T0 + 0.28 * C_x_delThrust;
      } else if (Vtrim == 15) {
	T0 = T0 - 0.14 * C_x_delThrust;
      } else if (Vtrim == 18) {
	T0 = T0 - 0.066 * C_x_delThrust;
      } else if (Vtrim == 28) {
	T0 = T0 + 0.48 * C_x_delThrust;
      } else if (Vtrim != 20) {
	T0 = T0 + (0.0477 * (Vtrim - 20) + 0.1) * C_x_delThrust;
      }
      Thrust = T0 + C_x_delThrust * DELTHRUST[body];
      XFORCEi = Lift * sin(alpha) - Drag * cos(alpha)
	+ Qa * Sarea[body] * C_x + Thrust;
      YFORCEi = Qa * Sarea[body] * C_y;
      ZFORCEi = -Lift * cos(alpha) - Drag * sin(alpha) + C_z;
      C_lmoment = C_l_rf*rhat + C_l_betaf * beta + C_l_pf * phat + C_l_da * DA[body] + C_l_dr * DR[body];
      C_m = C_m_0f + C_m_alphaf * alpha + C_m_qf * qhat + C_m_de * DE[body] + C_m_u * ub;
      C_n = C_n_betaf * beta + C_n_pf*phat + C_n_rf * rhat + C_n_da * DA[body] + C_n_dr * DR[body];
      LMOMENTi = Qa * Sarea[body] * wspan[body] * C_lmoment;
      MMOMENTi = C_m * Qa * Sarea[body] * c;
      NMOMENTi = C_n * Qa * Sarea[body] * wspan[body];
    } else {
      //Main Wing
      C_L = C_L_0i + C_L_alphai * alpha;
      C_D = C_D_0i + C_D_alphai * alpha * alpha;
      Lift = Qa * (Sarea[body] / WINGPANELS) * C_L;
      Drag = Qa * (Sarea[body] / WINGPANELS) * C_D;
      XFORCEi = Lift * sin(alpha) - Drag * cos(alpha);
      YFORCEi = 0;
      ZFORCEi = -Lift * cos(alpha) - Drag * sin(alpha);
      LMOMENTi = 0;
      MMOMENTi = Qa * (Sarea[body] / WINGPANELS) * c * C_m_i;
      NMOMENTi = 0;
    }
    BodyAeroLoads[0] = BodyAeroLoads[0] + XFORCEi;
    BodyAeroLoads[1] = BodyAeroLoads[1] + YFORCEi;
    BodyAeroLoads[2] = BodyAeroLoads[2] + ZFORCEi;

    //To compute the moments we take the contributions from the main wing the fuselage and the cross terms
    BodyAeroLoads[3] = BodyAeroLoads[3] + -zpanelB * YFORCEi
      + ypanelB * ZFORCEi + LMOMENTi;
    BodyAeroLoads[4] = BodyAeroLoads[4] + zpanelB * XFORCEi
      - xpanelB * ZFORCEi + MMOMENTi;
    BodyAeroLoads[5] = BodyAeroLoads[5] + -ypanelB * XFORCEi
      + xpanelB * YFORCEi + NMOMENTi;
  }
  
  ///REVISIT REVISIT REVISIT
  //LMii[0] = BodyAeroLoads[3] + BodyAeroLoads[2]*RCG2AC[1][body];
  //LMii[1] = BodyAeroLoads[4] - BodyAeroLoads[2]*RCG2AC[0][body];

  //printf("Term1 = %lf , Term 2 = %lf \n",BodyAeroLoads[3],BodyAeroLoads[2]*RCG2AC[1][body]);
  
  LMii[0] = BodyAeroLoads[3];
  LMii[1] = BodyAeroLoads[3] + BodyAeroLoads[2]*RCG2AC[1][body];
  
  //Compute the G matrix
  double G11 = (Qa * Sarea[body] * wspan[body] * C_l_da); //aileron to phi
  double G12 = (-Qa * Sarea[body] * RCG2AC[1][body] * cos(alpha) * C_L_de); //elevator to phi
  double G21 = 0; //aileron to theta
  double G22 = Qa * Sarea[body] * (c * C_m_de + cos(alpha) * RCG2AC[0][body] * C_L_de); //elevator to theta
  
  G12 = 0; //Artificially decouple the elevator on phi
  
  GMETA[0][0+(body)*2] = (1/Ixx) * G11;
  GMETA[0][1+(body)*2] = (1/Ixx) * G12;
  GMETA[1][0+(body)*2] = (1/Iyy) * G21;
  GMETA[1][1+(body)*2] = (1/Iyy) * G22;
    
  ///Note [phic;thetac] = [G11 G12 ; G21 G22] * [da ; de]
  //Thus:
  //G11 is the effect of the aileron on phi
  //G12 is the effect of the elevator on phi
  //G21 is the effect of the aileron on theta
  //G22 is the effect of the elevator on theta
  
  //matrixdisp(GMETA,2,2*METABODIES,"GMETA");

}

void ComputeFG(double LMii[],int body,double Ixx,double Iyy) {
  int ii;
  double x, y, ub, vb, wb, rho, rho0, Vstall, Torque, d, f, VortexCoeff[4],
    wd;
  double vel, drag, zerror, u, v, w, p, q, r, eff, factor2, C_L_alpha1,
    C_L_alpha2;
  double rxaero, ryaero, rzaero, Qa, rpm, Vthrust, omega, alfa, Q, Qt, deG[3];
  double alpha, beta, C_L, C_D, C_lmoment, C_m, C_n, C_l_vortex, factor1;
  double C_x, C_y, C_z, Lift, Drag, SOS, C_x_delV, rpmthr, rpmvel, drVortex,
    deGust;
  double C_L_alpha_alpha, Thrust, z, ailf, ruddf, thrustf, elevf, daVortex;
  
  //Unwrap State
  x = State[body * NSTATE];
  y = State[body * NSTATE + 1];
  z = State[body * NSTATE + 2];
  u = State[body * NSTATE + 6];
  v = State[body * NSTATE + 7];
  w = State[body * NSTATE + 8];
  p = State[body * NSTATE + 9];
  q = State[body * NSTATE + 10];
  r = State[body * NSTATE + 11];

  //Zero out Winds
  WIND[0][body] = 0;
  WIND[1][body] = 0;
  WIND[2][body] = 0;
  /* Velocity of the A/C cg with respect to the Atmosphere(assume we don't know the atm)*/
  ub = u;
  vb = v;
  wb = w;
  vel = sqrt(pow(ub, 2.0000) + pow(vb, 2.0000) + pow(wb, 2.0000));
  
  //Non-Dimensionalize p,q,r
  double phat = wspan[body] * p / (2 * vel);
  double qhat = c * q / (2 * vel);
  double rhat = wspan[body] * r / (2 * vel);
  
  /* Air Density */
  rho = Density();

  //Dynamic Pressure
  Qa = 0.5 * rho * pow(vel, 2.00);
  
  //Thrust Information
  T0 = 0.2882 + 4.8;
  if (Vtrim == 25) {
    T0 = T0 + 0.28 * C_x_delThrust;
  } else if (Vtrim == 15) {
    T0 = T0 - 0.14 * C_x_delThrust;
  } else if (Vtrim == 18) {
    T0 = T0 - 0.066 * C_x_delThrust;
  } else if (Vtrim == 28) {
    T0 = T0 + 0.48 * C_x_delThrust;
  } else if (Vtrim != 20) {
    T0 = T0 + (0.0477 * (Vtrim - 20) + 0.1) * C_x_delThrust;
  }
  
  //!Compute Angle of attack
  alpha = 0;
  if (fabs(ub) > 0) {
    alpha = atan2(wb, ub);
  }

  //!Slidelslip
  beta = 0;
  if (fabs(vel) > 0) {
    beta = asin(vb / vel);
  }
  
  //Lift and Drag with no control
  C_L = C_L_0 + C_L_alpha * alpha + C_L_q * qhat;
  C_D = C_D_0 + C_D_alpha * (pow(alpha, 2)) + C_D_u * ub;
  Lift = C_L * Qa * Sarea[body];
  Drag = C_D * Qa * Sarea[body];

  //XYZ Forces with no control
  C_x = 0;
  C_y = C_y_beta * beta + C_y_p * phat + C_y_r * rhat;
  C_z = 0;
  Thrust = T0;
  
  //Moments with nocontrol
  C_lmoment = C_l_beta * beta + C_l_p * phat + C_l_r * rhat;
  C_m = C_m_0 + C_m_alpha * alpha + C_m_q * qhat  + C_m_u * ub;
  C_n = C_n_p * phat + C_n_beta * beta + C_n_r * rhat;

  double Z0 = -Lift * cos(alpha) - Drag * sin(alpha)  + C_z * Qa * Sarea[body];

  // LMii[0] = C_lmoment * Qa * Sarea[body] * wspan[body] + Z0*RCG2AC[1][body]; 
  // LMii[1] = C_m * Qa * Sarea[body] * c - Z0*RCG2AC[0][body];

  //LMii[0] = C_lmoment * Qa * Sarea[body] * wspan[body];
  //LMii[1] = 0;

  // LMii[0]  = (C_l_beta * beta + C_l_p * phat + C_l_r * rhat) * Qa * Sarea[body] * wspan[body];
  // LMii[1] = 0;

  // LMii[0]  = (C_l_p * phat) * Qa * Sarea[body] * wspan[body];
  // LMii[1] = 0;

  LMii[0]  =  0.25 * rho * vel * Sarea[body] * wspan[body] * wspan[body] * C_l_p * p;
  LMii[1] = 0;
  
  //Compute the G matrix
  double G11 = (Qa * Sarea[body] * wspan[body] * C_l_da); //aileron to phi
  double G12 = (-Qa * Sarea[body] * RCG2AC[1][body] * cos(alpha) * C_L_de); //elevator to phi
  double G21 = 0; //aileron to theta
  double G22 = Qa * Sarea[body] * (c * C_m_de + cos(alpha) * RCG2AC[0][body] * C_L_de); //elevator to theta

  G12 = 0; //Artificially decouple the elevator on phi

  GMETA[0][0+(body)*2] = (1/Ixx) * G11;
  GMETA[0][1+(body)*2] = (1/Ixx) * G12;
  GMETA[1][0+(body)*2] = (1/Iyy) * G21;
  GMETA[1][1+(body)*2] = (1/Iyy) * G22;

  ///Note [phic;thetac] = [G11 G12 ; G21 G22] * [da ; de]
  //Thus:
  //G11 is the effect of the aileron on phi
  //G12 is the effect of the elevator on phi
  //G21 is the effect of the aileron on theta
  //G22 is the effect of the elevator on theta

  //matrixdisp(GMETA,2,2*METABODIES,"GMETA");

}

void VelocityPanels(int body) {
	int ii, panel, jj;
	double xpanelI, ypanelI, zpanelI, xpanelB, ypanelB, zpanelB;
	double x, y, z, u, v, w, p, q, r, xAB, yAB, zAB, xBB, yBB, zBB;

	//Unwrap state vector
	x = State[body * NSTATE];
	y = State[body * NSTATE + 1];
	z = State[body * NSTATE + 2];
	u = State[body * NSTATE + 6];
	v = State[body * NSTATE + 7];
	w = State[body * NSTATE + 8];
	p = State[body * NSTATE + 9];
	q = State[body * NSTATE + 10];
	r = State[body * NSTATE + 11];
	for (ii = 0; ii < WINGPANELS + 3; ii++) {
		//Basically at every location on the body we will have to compute
		//the location of every panel in inertial space so we will essentially
		//compute rpanelIi = rcg + TIB*rpanelBi;
		panel = body * NPANELS + ii;
		xpanelB = r_cgp_B[0][panel];
		ypanelB = r_cgp_B[1][panel];
		zpanelB = r_cgp_B[2][panel];
		//Assume phi,theta,psi = phi0,theta0,psi0
		if (TIBCONSTANT) {
			rp_I[0][panel] = x + TINTB[body][0][0] * xpanelB
					+ TINTB[body][0][1] * ypanelB + TINTB[body][0][2] * zpanelB;
			rp_I[1][panel] = y + TINTB[body][1][0] * xpanelB
					+ TINTB[body][1][1] * ypanelB + TINTB[body][1][2] * zpanelB;
			rp_I[2][panel] = z + TINTB[body][2][0] * xpanelB
					+ TINTB[body][2][1] * ypanelB + TINTB[body][2][2] * zpanelB;
		} else {
			rp_I[0][panel] = x + TIB[body][0][0] * xpanelB
					+ TIB[body][0][1] * ypanelB + TIB[body][0][2] * zpanelB;
			rp_I[1][panel] = y + TIB[body][1][0] * xpanelB
					+ TIB[body][1][1] * ypanelB + TIB[body][1][2] * zpanelB;
			rp_I[2][panel] = z + TIB[body][2][0] * xpanelB
					+ TIB[body][2][1] * ypanelB + TIB[body][2][2] * zpanelB;
		}
		//compute location of pt A in inertial space
		xAB = r_cgA_B[0][panel];
		yAB = r_cgA_B[1][panel];
		zAB = r_cgA_B[2][panel];
		if (TIBCONSTANT) {
			vortexA_I[0][panel] = x + TINTB[body][0][0] * xAB
					+ TINTB[body][0][1] * yAB + TINTB[body][0][2] * zAB;
			vortexA_I[1][panel] = y + TINTB[body][1][0] * xAB
					+ TINTB[body][1][1] * yAB + TINTB[body][1][2] * zAB;
			vortexA_I[2][panel] = z + TINTB[body][2][0] * xAB
					+ TINTB[body][2][1] * yAB + TINTB[body][2][2] * zAB;
		} else {
			vortexA_I[0][panel] = x + TIB[body][0][0] * xAB
					+ TIB[body][0][1] * yAB + TIB[body][0][2] * zAB;
			vortexA_I[1][panel] = y + TIB[body][1][0] * xAB
					+ TIB[body][1][1] * yAB + TIB[body][1][2] * zAB;
			vortexA_I[2][panel] = z + TIB[body][2][0] * xAB
					+ TIB[body][2][1] * yAB + TIB[body][2][2] * zAB;
		}
		//compute location of pt B in inertial space
		xBB = r_cgB_B[0][panel];
		yBB = r_cgB_B[1][panel];
		zBB = r_cgB_B[2][panel];
		if (TIBCONSTANT) {
			vortexB_I[0][panel] = x + TINTB[body][0][0] * xBB
					+ TINTB[body][0][1] * yBB + TINTB[body][0][2] * zBB;
			vortexB_I[1][panel] = y + TINTB[body][1][0] * xBB
					+ TINTB[body][1][1] * yBB + TINTB[body][1][2] * zBB;
			vortexB_I[2][panel] = z + TINTB[body][2][0] * xBB
					+ TINTB[body][2][1] * yBB + TINTB[body][2][2] * zBB;
		} else {
			vortexB_I[0][panel] = x + TIB[body][0][0] * xBB
					+ TIB[body][0][1] * yBB + TIB[body][0][2] * zBB;
			vortexB_I[1][panel] = y + TIB[body][1][0] * xBB
					+ TIB[body][1][1] * yBB + TIB[body][1][2] * zBB;
			vortexB_I[2][panel] = z + TIB[body][2][0] * xBB
					+ TIB[body][2][1] * yBB + TIB[body][2][2] * zBB;
		}
		//Zero wind at given panel
		WIND[0][panel] = 0;
		WIND[1][panel] = 0;
		WIND[2][panel] = 0;
		//Compute Velocity at each panel
		if (IWRF)
			uvwwindStatic(rp_I[0][panel], rp_I[1][panel], rp_I[2][panel],TCURRENT, panel, body);
		if (ITURB)
			Turbulence3D(rp_I[0][panel], rp_I[1][panel], panel, body);
		if (ICONSTANT)
			uvwconstant(rp_I[0][panel], panel, body);
		/* Velocity of the Box cg with respect to the Atmosphere (in B frame) */
		VP_noInduced_B[0][panel] = u - r * ypanelB + q * zpanelB
				- WINDB[0][panel];
		VP_noInduced_B[1][panel] = v + r * xpanelB - p * zpanelB
				- WINDB[1][panel];
		VP_noInduced_B[2][panel] = w - q * xpanelB + p * ypanelB
				- WINDB[2][panel];
//                printf("VP_noInduced_B = %0.20f %0.20f %0.20f \n",VP_noInduced_B[0][panel],VP_noInduced_B[1][panel],VP_noInduced_B[2][panel]);
	}
//        PAUSE();
}

void AeroSplit() 
{

  SH[0] = tailspan * 0.295;
  SV[0] = SH[0] / 2;
  int ii;
  for (ii = 1; ii < NBODIES; ii++) 
    {
      SH[ii] = tailspan * 0.295;
      SV[ii] = SH[ii] / 2;
    }
  //Now we must use the equivalence from Panels.m to create coefficients for everything.
  //%%Main Wing
  //We are actually going to make a fitting procedure for these coefficients
  //L_total = L_main_wing + L_tail
  //1/2*rho*V^2*S*C_L = 1/2*rho*V^2*S*C_L_main + 1/2*rho*V^2*SH*C_L_tail
  //C_L = C_L_main + SH/S*C_L_tail
  //Let C_L_main = C_L_tail = C_L_i
  //C_L = (1+SH/S)*C_L_i
  //C_L/(1+SH/S) = C_L_i
  C_L_0i = C_L_0 / (1 + SH[0] / (Sarea[0]));
  C_D_0i = C_D_0 / (1 + SH[0] / (Sarea[0]));
  C_D_alphai = C_D_alpha / (1 + SH[0] / Sarea[0]); //I think this is the problem
  C_L_alphai = 1.008*C_L_alpha;
  C_m_i = 1.05*C_m_0;
  //%%%Horizontal tail
  C_L_0t = 0;
  C_D_0t = C_D_0i;
  C_L_alphat = C_L_alphai;
  C_D_alphat = C_D_alphai;
  //%%Vertical Tail
  C_L_0v = 0;
  C_D_0v = C_D_0t;
  C_L_alphav = C_L_alphat;
  C_D_alphav = 0;

  //%%Fuselage
  //These are our initial guesses
  C_m_0f = 0.0;
  C_L_0f = 0.018;
  C_D_0f = 0.00;
  C_L_alphaf = 1.225;
  C_D_alphaf = 0.8;
  C_m_alphaf = C_m_alpha - SH[0] * htailx * C_L_alphat / (Sarea[0] * c);
  C_m_qf = C_m_q + (SH[0] / Sarea[0]) * htailx * htailx * 2 * C_L_alphat/ (c * c);
  C_L_qf = C_L_q + 2 * (SH[0] / Sarea[0]) * (htailx / c) * C_L_alphat;
  C_l_pf = 0.54;
  C_n_pf = -0.6816;
  C_y_pf = 0.9922;
  C_l_betaf = -11 * C_l_beta;
  C_n_betaf = -0.4815;
  C_y_betaf = 0.510; 
  C_y_rf = -0.922;
  C_n_rf = 0.655;
  C_l_rf = -0.6577;

  ///BUT THESE ARE WHAT WE ARE FITTING THEM TO BE
  FILE* coefffile;
  sprintf(temp, "%s%s%d%s",inFuse,"Fuselage", (int) WINGPANELS, ".Coeff");
  coefffile = fopen(temp,"r");
  if (coefffile == NULL)
    {
      printf("Fuselage Location Inputted Incorrectly or Fuselage file not named properly \n");
      //Give the user the benefit of the doubt here and run AeroFit
      printf("It is possible that the file has never been created yet \n");
      if ((BODIES == 1) && (AEROMODEL))
	{
	  printf("Setting all coefficients to zero for now because you're going to fit them in another step \n");
	  C_m_0f = 0.0;
	  C_L_0f = 0.0; 
	  C_D_0f = 0.0;
	  C_L_alphaf = 0.0; 
	  C_D_alphaf = 0.0; 
	  C_m_alphaf = 0.0; 
	  C_m_qf = 0.0; 
	  C_L_qf = 0.0; 
	  C_l_pf = 0.0; 
	  C_n_pf = 0.0; 
	  C_y_pf = 0.0; 
	  C_l_betaf = 0.0; 
	  C_n_betaf = 0.0; 
	  C_y_betaf = 0.0; 
	  C_y_rf = 0.0; 
	  C_n_rf = 0.0; 
	  C_l_rf = 0.0; 
	}
      else {
	printf("Unable to run AeroFit because BODIES does not equal 1. Set the number of bodies to 1 in the %s file and try again \n",CONNFILE);
	exit(1);
      }
    }
  else {
    fscanf(coefffile,"%lf \n",&C_m_0f);
    fscanf(coefffile,"%lf \n",&C_L_0f);
    fscanf(coefffile,"%lf \n",&C_D_0f);
    fscanf(coefffile,"%lf \n",&C_L_alphaf);
    fscanf(coefffile,"%lf \n",&C_D_alphaf);
    fscanf(coefffile,"%lf \n",&C_m_alphaf);
    fscanf(coefffile,"%lf \n",&C_m_qf);
    fscanf(coefffile,"%lf \n",&C_L_qf);
    fscanf(coefffile,"%lf \n",&C_l_pf);
    fscanf(coefffile,"%lf \n",&C_n_pf);
    fscanf(coefffile,"%lf \n",&C_y_pf);
    fscanf(coefffile,"%lf \n",&C_l_betaf);
    fscanf(coefffile,"%lf \n",&C_n_betaf);
    fscanf(coefffile,"%lf \n",&C_y_betaf);
    fscanf(coefffile,"%lf \n",&C_y_rf);
    fscanf(coefffile,"%lf \n",&C_n_rf);
    fscanf(coefffile,"%lf \n",&C_l_rf);
    fclose(coefffile);
  }

}

void SetupPanels() {
	int repeater, loc, ii, jj;
	double ycoord,delta;
	for (ii = 0; ii < NBODIES; ii++) {
            //First define the main wing panels in the body frame
            delta = wspan[ii]/(double)WINGPANELS;
	    loc = NPANELS * ii;
	    //Let's assume that the number of panels is odd
	    r_cgA_B[0][loc] = mainwx + c / 2;
	    r_cgA_B[1][loc] = -delta/2;
	    r_cgA_B[2][loc] = 0;
	    //Control Point
	    r_cgp_B[0][loc] = mainwx; //x coordinate of main wing
	    r_cgp_B[1][loc] = 0;
	    r_cgp_B[2][loc] = 0;
	    //B point of Horshoe
	    r_cgB_B[0][loc] = mainwx + c / 2;
	    r_cgB_B[1][loc] = delta/2;
	    r_cgB_B[2][loc] = 0;
	    //Rotation of control point
	    E_cgp[0][loc] = 0;
	    E_cgp[1][loc] = 0;
	    E_cgp[2][loc] = 0;
	    for (jj = 0; jj < (WINGPANELS-1)/2; jj++) 
	      {
		loc = NPANELS * ii + jj*2 + 1;
		ycoord = 0 + delta*(jj+1);
		//A Point of Horshoe
		r_cgA_B[0][loc] = mainwx + c / 2;
		r_cgA_B[1][loc] = ycoord - delta/2;
		r_cgA_B[2][loc] = 0;
		//Control Point
		r_cgp_B[0][loc] = mainwx; //x coordinate of main wing
		r_cgp_B[1][loc] = ycoord;
		r_cgp_B[2][loc] = 0;
		//B point of Horshoe
		r_cgB_B[0][loc] = mainwx + c / 2;
		r_cgB_B[1][loc] = ycoord + delta/2;
		r_cgB_B[2][loc] = 0;
		//Rotation of control point
		E_cgp[0][loc] = 0;
		E_cgp[1][loc] = 0;
		E_cgp[2][loc] = 0;
		
		loc = NPANELS * ii + jj*2 + 2;
		//A Point of Horshoe
		r_cgA_B[0][loc] = mainwx + c / 2;
		r_cgA_B[1][loc] = -ycoord - delta/2;
		r_cgA_B[2][loc] = 0;
		//Control Point
		r_cgp_B[0][loc] = mainwx; //x coordinate of main wing
		r_cgp_B[1][loc] = -ycoord;
		r_cgp_B[2][loc] = 0;
		//B point of Horshoe
		r_cgB_B[0][loc] = mainwx + c / 2;
		r_cgB_B[1][loc] = -ycoord + delta/2;
		r_cgB_B[2][loc] = 0;
		//Rotation of control point
		E_cgp[0][loc] = 0;
		E_cgp[1][loc] = 0;
		E_cgp[2][loc] = 0;
	      }
		//Then we define the horizontal tail surface
		r_cgp_B[0][NPANELS * ii + WINGPANELS] = htailx;
		r_cgp_B[1][NPANELS * ii + WINGPANELS] = 0;
		r_cgp_B[2][NPANELS * ii + WINGPANELS] = 0;
		//Then we define the vertical tail surface
		r_cgp_B[0][NPANELS * ii + WINGPANELS + 1] = vtailx;
		r_cgp_B[1][NPANELS * ii + WINGPANELS + 1] = 0;
		r_cgp_B[2][NPANELS * ii + WINGPANELS + 1] = vtailz;
		//Finally we define the fuselage
		r_cgp_B[0][NPANELS * ii + WINGPANELS + 2] = 0; //Fuselage acts at CG
		r_cgp_B[1][NPANELS * ii + WINGPANELS + 2] = 0;
		r_cgp_B[2][NPANELS * ii + WINGPANELS + 2] = 0;
	}

	// printf("A,P,B \n");
	// for (ii = 0;ii<NPANELS*BODIES;ii++)
	//   {
	//     printf("%0.20f %0.20f %0.20f \n",r_cgA_B[1][ii],r_cgp_B[1][ii],r_cgB_B[1][ii]);
	//   }
	// PAUSE();

}

void AllocatePanels() {
  //printf("Allocating Panels \n");
	if (WINGPANELS < 2) {
		WINGPANELS = 2;
	};
	NPANELS = (WINGPANELS + 3);
	/////////GENERATE PANEL LOCATIONS//////////
	r_cgp_B = matrixallocatedbl(3, NPANELS * NBODIES);
	r_cgA_B = matrixallocatedbl(3, NPANELS * NBODIES);
	r_cgB_B = matrixallocatedbl(3, NPANELS * NBODIES);
	E_cgp = matrixallocatedbl(3, NPANELS * NBODIES);
	rp_I = matrixallocatedbl(3, NPANELS * NBODIES);
	vortexA_I = matrixallocatedbl(3, NPANELS * NBODIES);
	vortexB_I = matrixallocatedbl(3, NPANELS * NBODIES);
	Gammai = vecallocatedbl(NPANELS * NBODIES);
	Gammaiprev = vecallocatedbl(NPANELS * NBODIES);
	Gammai1 = vecallocatedbl(NPANELS * NBODIES);
	wij_I = vecallocatedbl(3);
	VP_noInduced_B = matrixallocatedbl(3, NPANELS * NBODIES);
	VP_P = matzeros(3, NPANELS * NBODIES);
	VP_B = matzeros(3, NPANELS * NBODIES);
	VP_I = matzeros(3, NPANELS * NBODIES);
	///DEFINE SOME REFERENCE FRAMES//////
	TBP = eye(3, 3);
	uvw_induced_B = matrixallocatedbl(3, NPANELS * NBODIES);
	uvw1aircraft = matrixallocatedbl(3, NPANELS * NBODIES);
	uvw_induced_backup = matrixallocatedbl(3, NPANELS * NBODIES);

}

double radius(double NormalB[], double phi, double theta) {
	int index, zdir;
	double pos_neg;
	double p, atheta, s, limit;

	//Initialize NormalB Vector
	NormalB[0] = 0.0000;
	NormalB[1] = 0.0000;
	NormalB[2] = 0.0000;
	index = 0;
	pos_neg = 1.00;
	zdir = 1.00;

	atheta = fabs(theta);
	s = aobj;
	//Map theta to 1st quadrant
	if (atheta > thetar) {
		theta = PI - atheta;
		//Normal vector along -x direction or +-z direction
		index = 0;
		pos_neg = -1.00;
	} else if (atheta > thetal) {
		s = bobj;
		theta = PI / 2 - atheta;
		//Normal vector along +-y
		index = 1;
		pos_neg = copysign(1.0, theta);
	}
	//Find phi limit
	limit = atan2(s, cobj * cos(theta));
	//Map phi to upper hemisphere
	if (phi > PI / 2) {
		phi = PI - phi;
		//zdirection is now negative
		zdir = -1;
	}
	if (phi >= limit) {
		phi = phi - PI / 2;
	} else if (phi < limit) {
		theta = 0;
		s = cobj;
		index = 2;
		pos_neg = zdir;
	}
	p = (s / 2) / (cos(theta) * cos(phi));

	NormalB[index] = pos_neg;

	return p;

}

void GetNormal(double NormalB[], double xp, double yp, double zp) {

	double r, phi, theta, p;

	//Map input vector to spherical coordinates
	r = sqrt(xp * xp + yp * yp + zp * zp);
	if (!r)
		r = 1;
	phi = acos(zp / r);
	theta = atan2(yp, xp);

	p = radius(NormalB, phi, theta);

}

void ComputeContactForce() {
	int ii, jj;
	double s1n[3][NVERTS], s1t[3][NVERTS], s2n[3][NVERTS], s2t[3][NVERTS];
	double NormalB[3], rplane_vertex[3][NVERTS], rcg_vertex_I[3], rInter[3],
			rcg1_vertex_I[3];
	double delun[3][NVERTS], delut[3][NVERTS], delwt[3][NVERTS], Normal[3];
	double uB[3], uC[3], u1B[3], u1C[3], delu[3];
	double deluDotNormal, norm2Normal, c_star, commands[3];
	double bn[3], bt[3], norm_bn, norm_bt, lambda;
	double F_CI[NBODIES][3], M_CI[NBODIES][3], Fn[3], Ft[3], M_Cf[3], M_Cf1[3];
	double CoeffDamper1[4], CoeffDamper2[4], x[2], y[2], z[2], u[2], v[2], w[2],
			p[2], q[2], r[2];
	double s1ndot[3][NVERTS], s1tdot[3][NVERTS], s2ndot[3][NVERTS],
			s2tdot[3][NVERTS];

	//Unwrap State vector
	for (int body = 0; body < BODIES; body++) {
		x[body] = State[body * NSTATE];
		y[body] = State[body * NSTATE + 1];
		z[body] = State[body * NSTATE + 2];
		u[body] = State[body * NSTATE + 6];
		v[body] = State[body * NSTATE + 7];
		w[body] = State[body * NSTATE + 8];
		p[body] = State[body * NSTATE + 9];
		q[body] = State[body * NSTATE + 10];
		r[body] = State[body * NSTATE + 11];
		//Zero out Contact Force
		F_CI[body][0] = 0.000000000;
		F_CI[body][1] = 0.000000000;
		F_CI[body][2] = 0.000000000;
		M_CI[body][0] = 0.000000000;
		M_CI[body][1] = 0.000000000;
		M_CI[body][2] = 0.000000000;
	}

	//Initialize Contact Vector
	for (ii = 0; ii < IVERTS; ii++) {
		for (jj = 0; jj < 3; jj++) {
			s1n[jj][ii] = State[NVRTST * ii + NBODIES * NSTATE + jj];
			s1t[jj][ii] = State[NVRTST * ii + NBODIES * NSTATE + jj + 3];
			s2n[jj][ii] = State[NVRTST * ii + NBODIES * NSTATE + jj + 6];
			s2t[jj][ii] = State[NVRTST * ii + NBODIES * NSTATE + jj + 9];
		}
	}

	/* Position Vector from Point P (plane) to V (vertex) (in B1 Frame) */
	for (ii = 0; ii < IVERTS; ii++) {
		//rB - x,y,z
		//rB1 - x1,y1,z1
		//rcg1_plane - xc,yc,zc - vector from cg of a/c1 to center of plane
		//rplane_vertex = TBI1*(TIB*rcg_vertex + rB - rB1) - rcg1_plane
		rInter[0] = (TIB[0][0][0] * rcg_vertex[0][ii]
				+ TIB[0][0][1] * rcg_vertex[1][ii]
				+ TIB[0][0][2] * rcg_vertex[2][ii]
				+ (State[0] - State[NSTATE + 0]));
		rInter[1] = (TIB[0][1][0] * rcg_vertex[0][ii]
				+ TIB[0][1][1] * rcg_vertex[1][ii]
				+ TIB[0][1][2] * rcg_vertex[2][ii]
				+ (State[1] - State[NSTATE + 1]));
		rInter[2] = (TIB[0][2][0] * rcg_vertex[0][ii]
				+ TIB[0][2][1] * rcg_vertex[1][ii]
				+ TIB[0][2][2] * rcg_vertex[2][ii]
				+ (State[2] - State[NSTATE + 2]));
		rplane_vertex[0][ii] = (TBI[1][0][0] * rInter[0]
				+ TBI[1][0][1] * rInter[1] + TBI[1][0][2] * rInter[2])
				- rcg1_plane[0];
		rplane_vertex[1][ii] = (TBI[1][1][0] * rInter[0]
				+ TBI[1][1][1] * rInter[1] + TBI[1][1][2] * rInter[2])
				- rcg1_plane[1];
		rplane_vertex[2][ii] = (TBI[1][2][0] * rInter[0]
				+ TBI[1][2][1] * rInter[1] + TBI[1][2][2] * rInter[2])
				- rcg1_plane[2];
		//rcg1_vertex
		rcg1_vertex[0][ii] = rcg1_plane[0] + rplane_vertex[0][ii];
		rcg1_vertex[1][ii] = rcg1_plane[1] + rplane_vertex[1][ii];
		rcg1_vertex[2][ii] = rcg1_plane[2] + rplane_vertex[2][ii];
		// rcg1_vertex[0][ii] = rcg_vertex[0][ii];
		// rcg1_vertex[1][ii] = -rcg_vertex[1][ii];
		// rcg1_vertex[2][ii] = rcg_vertex[2][ii];
		//ContactList[0][ii]: 0.0=NoContact, 1.0=Contact, Changed in Runge-Kutta
		ContactList[1][ii] = rplane_vertex[0][ii];
		ContactList[2][ii] = rplane_vertex[1][ii];
		ContactList[3][ii] = rplane_vertex[2][ii];
	}

	for (ii = 0; ii < IVERTS; ii++) {
		/* Initialize delun, delut, delwt to Zero (in I Frame) */
		delun[0][ii] = 0.00000000;
		delun[1][ii] = 0.00000000;
		delun[2][ii] = 0.00000000;
		delut[0][ii] = 0.00000000;
		delut[1][ii] = 0.00000000;
		delut[2][ii] = 0.00000000;
		delwt[0][ii] = 0.00000000;
		delwt[1][ii] = 0.00000000;
		delwt[2][ii] = 0.00000000;
		Fn[0] = 0;
		Fn[1] = 0;
		Fn[2] = 0;
		Ft[0] = 0;
		Ft[1] = 0;
		Ft[2] = 0;

		if (ContactList[0][ii] == 1.0) {
			/* The Plane Normal (in B1 Frame) plane is attached to a/c1 */
			GetNormal(NormalB, ContactList[1][ii], ContactList[2][ii],
					ContactList[3][ii]);

			/* Rotate NormalB vector to inertial reference frame from B1 frame*/
			Normal[0] = TIB[1][0][0] * NormalB[0] + TIB[1][0][1] * NormalB[1]
					+ TIB[1][0][2] * NormalB[2];
			Normal[1] = TIB[1][1][0] * NormalB[0] + TIB[1][1][1] * NormalB[1]
					+ TIB[1][1][2] * NormalB[2];
			Normal[2] = TIB[1][2][0] * NormalB[0] + TIB[1][2][1] * NormalB[1]
					+ TIB[1][2][2] * NormalB[2];

			/* u contact in B Frame */
			uB[0] = u[0] - r[0] * (rcg_vertex[1][ii])
					+ q[0] * (rcg_vertex[2][ii]);
			uB[1] = v[0] + r[0] * (rcg_vertex[0][ii])
					- p[0] * (rcg_vertex[2][ii]);
			uB[2] = w[0] - q[0] * (rcg_vertex[0][ii])
					+ p[0] * (rcg_vertex[1][ii]);
			/* u contact in I Frame */
			uC[0] = TIB[0][0][0] * uB[0] + TIB[0][0][1] * uB[1]
					+ TIB[0][0][2] * uB[2];
			uC[1] = TIB[0][1][0] * uB[0] + TIB[0][1][1] * uB[1]
					+ TIB[0][1][2] * uB[2];
			uC[2] = TIB[0][2][0] * uB[0] + TIB[0][2][1] * uB[1]
					+ TIB[0][2][2] * uB[2];
			/* u1 contact in B Frame */
			u1B[0] = u[1] - r[1] * (rcg1_vertex[1][ii])
					+ q[1] * (rcg1_vertex[2][ii]);
			u1B[1] = v[1] + r[1] * (rcg1_vertex[0][ii])
					- p[1] * (rcg1_vertex[2][ii]);
			u1B[2] = w[1] - q[1] * (rcg1_vertex[0][ii])
					+ p[1] * (rcg1_vertex[1][ii]);
			/* u1 contact in I Frame */
			u1C[0] = TIB[1][0][0] * u1B[0] + TIB[1][0][1] * u1B[1]
					+ TIB[1][0][2] * u1B[2];
			u1C[1] = TIB[1][1][0] * u1B[0] + TIB[1][1][1] * u1B[1]
					+ TIB[1][1][2] * u1B[2];
			u1C[2] = TIB[1][2][0] * u1B[0] + TIB[1][2][1] * u1B[1]
					+ TIB[1][2][2] * u1B[2];
			/* delu in I Frame */
			delu[0] = uC[0] - u1C[0];
			delu[1] = uC[1] - u1C[1];
			delu[2] = uC[2] - u1C[2];
			/* delun in I Frame */
			deluDotNormal = delu[0] * Normal[0] + delu[1] * Normal[1]
					+ delu[2] * Normal[2];
			norm2Normal = pow(Normal[0], 2.0000) + pow(Normal[1], 2.0000)
					+ pow(Normal[2], 2.0000);
			if (norm2Normal == 0) {
				norm2Normal = 1;
			}
			delun[0][ii] = (deluDotNormal / norm2Normal) * Normal[0];
			delun[1][ii] = (deluDotNormal / norm2Normal) * Normal[1];
			delun[2][ii] = (deluDotNormal / norm2Normal) * Normal[2];
			/* delut in I Frame */
			delut[0][ii] = delu[0] - delun[0][ii];
			delut[1][ii] = delu[1] - delun[1][ii];
			delut[2][ii] = delu[2] - delun[2][ii];
			/* c_star */
			c_star = C[0][1] * C[1][1] / (C[0][1] + C[1][1]);
			/* bn in I Frame */
			bn[0] = (1 / (C[0][0] + C[1][0]))
					* (C[1][0] * K[0][0] * s1n[0][ii]
							- C[0][0] * K[1][0] * s2n[0][ii]
							+ C[0][0] * C[1][0] * delun[0][ii]);
			bn[1] = (1 / (C[0][0] + C[1][0]))
					* (C[1][0] * K[0][0] * s1n[1][ii]
							- C[0][0] * K[1][0] * s2n[1][ii]
							+ C[0][0] * C[1][0] * delun[1][ii]);
			bn[2] = (1 / (C[0][0] + C[1][0]))
					* (C[1][0] * K[0][0] * s1n[2][ii]
							- C[0][0] * K[1][0] * s2n[2][ii]
							+ C[0][0] * C[1][0] * delun[2][ii]);
			/* bt in I Frame */
			bt[0] = (1 / (C[0][1] + C[1][1]))
					* (C[1][1] * K[0][1] * s1t[0][ii]
							- C[0][1] * K[1][1] * s2t[0][ii]
							+ C[0][1] * C[1][1] * delut[0][ii]);
			bt[1] = (1 / (C[0][1] + C[1][1]))
					* (C[1][1] * K[0][1] * s1t[1][ii]
							- C[0][1] * K[1][1] * s2t[1][ii]
							+ C[0][1] * C[1][1] * delut[1][ii]);
			bt[2] = (1 / (C[0][1] + C[1][1]))
					* (C[1][1] * K[0][1] * s1t[2][ii]
							- C[0][1] * K[1][1] * s2t[2][ii]
							+ C[0][1] * C[1][1] * delut[2][ii]);
			/* Fn in I Frame */
			Fn[0] = -bn[0];
			Fn[1] = -bn[1];
			Fn[2] = -bn[2];
			/* Ft in I Frame */
			norm_bn = sqrt(
					pow(bn[0], 2.0000) + pow(bn[1], 2.0000)
							+ pow(bn[2], 2.0000));
			norm_bt = sqrt(
					pow(bt[0], 2.0000) + pow(bt[1], 2.0000)
							+ pow(bt[2], 2.0000));
			//printf("norm_bt = %lf\n",norm_bt);
			if (norm_bt <= mu * norm_bn) {
				/* State of Stick */
				lambda = 0.0000;
				delwt[0][ii] = 0.0000; // (in I Frame)
				delwt[1][ii] = 0.0000;
				delwt[2][ii] = 0.0000;
				Ft[0] = -bt[0]; // (in I Frame)
				Ft[1] = -bt[1];
				Ft[2] = -bt[2];
			} else if (norm_bt > mu * norm_bn) {
				lambda = (norm_bt - mu * norm_bn) / (c_star * mu * norm_bn);
				delwt[0][ii] = (lambda * bt[0]) / (1 + lambda * c_star); // (in I Frame)
				delwt[1][ii] = (lambda * bt[1]) / (1 + lambda * c_star);
				delwt[2][ii] = (lambda * bt[2]) / (1 + lambda * c_star);
				Ft[0] = -bt[0] / (1 + lambda * c_star); // (in I Frame)
				Ft[1] = -bt[1] / (1 + lambda * c_star);
				Ft[2] = -bt[2] / (1 + lambda * c_star);
			} else {
				printf("Calculated Contacts Wrong \n");
				fprintf(logfile, "Calculated Contacts Wrong run # = %d \n",
						irun);
				TFINAL = 0;
				Fn[0] = 0;
				Fn[1] = 0;
				Fn[2] = 0;
				Ft[0] = 0;
				Ft[1] = 0;
				Ft[2] = 0;
			}

			/* Contact Force in I Frame for a/c0*/
			F_CI[0][0] = F_CI[0][0] + Fn[0] + Ft[0]; // (in I Frame)
			F_CI[0][1] = F_CI[0][1] + Fn[1] + Ft[1];
			F_CI[0][2] = F_CI[0][2] + Fn[2] + Ft[2];

			/* Contact Moment in I Frame of a/c0*/
			rcg_vertex_I[0] = TIB[0][0][0] * (rcg_vertex[0][ii])
					+ TIB[0][0][1] * (rcg_vertex[1][ii])
					+ TIB[0][0][2] * (rcg_vertex[2][ii]);
			rcg_vertex_I[1] = TIB[0][1][0] * (rcg_vertex[0][ii])
					+ TIB[0][1][1] * (rcg_vertex[1][ii])
					+ TIB[0][1][2] * (rcg_vertex[2][ii]);
			rcg_vertex_I[2] = TIB[0][2][0] * (rcg_vertex[0][ii])
					+ TIB[0][2][1] * (rcg_vertex[1][ii])
					+ TIB[0][2][2] * (rcg_vertex[2][ii]);
			M_Cf[0] = -rcg_vertex_I[2] * (Fn[1] + Ft[1])
					+ rcg_vertex_I[1] * (Fn[2] + Ft[2]);
			M_Cf[1] = rcg_vertex_I[2] * (Fn[0] + Ft[0])
					- rcg_vertex_I[0] * (Fn[2] + Ft[2]);
			M_Cf[2] = -rcg_vertex_I[1] * (Fn[0] + Ft[0])
					+ rcg_vertex_I[0] * (Fn[1] + Ft[1]);
			M_CI[0][0] = M_CI[0][0] + M_Cf[0]; // (in I Frame)
			M_CI[0][1] = M_CI[0][1] + M_Cf[1];
			M_CI[0][2] = M_CI[0][2] + M_Cf[2];
			/* Contact Moment in I frame of a/c1 */
			rcg1_vertex_I[0] = TIB[1][0][0] * (rcg1_vertex[0][ii])
					+ TIB[1][0][1] * (rcg1_vertex[1][ii])
					+ TIB[1][0][2] * (rcg1_vertex[2][ii]);
			rcg1_vertex_I[1] = TIB[1][1][0] * (rcg1_vertex[0][ii])
					+ TIB[1][1][1] * (rcg1_vertex[1][ii])
					+ TIB[1][1][2] * (rcg1_vertex[2][ii]);
			rcg1_vertex_I[2] = TIB[1][2][0] * (rcg1_vertex[0][ii])
					+ TIB[1][2][1] * (rcg1_vertex[1][ii])
					+ TIB[1][2][2] * (rcg1_vertex[2][ii]);
			/* Contact Moment in I Frame */
			M_Cf1[0] = -rcg1_vertex_I[2] * (-Fn[1] - Ft[1])
					+ rcg1_vertex_I[1] * (-Fn[2] - Ft[2]);
			M_Cf1[1] = rcg1_vertex_I[2] * (-Fn[0] - Ft[0])
					- rcg1_vertex_I[0] * (-Fn[2] - Ft[2]);
			M_Cf1[2] = -rcg1_vertex_I[1] * (-Fn[0] - Ft[0])
					+ rcg1_vertex_I[0] * (-Fn[1] - Ft[1]);
			M_CI[1][0] = M_CI[1][0] + M_Cf1[0]; // (in I Frame)
			M_CI[1][1] = M_CI[1][1] + M_Cf1[1];
			M_CI[1][2] = M_CI[1][2] + M_Cf1[2];
		}
	}

	/* Contact Forces and Moments (in B Frame) */
	/* Contact Force in I Frame of a/c1*/
	F_CI[1][0] = -F_CI[0][0];
	F_CI[1][1] = -F_CI[0][1];
	F_CI[1][2] = -F_CI[0][2];

	/* Rotate Forces back to Aircraft Frame*/
	for (ii = 0; ii < 2; ii++) {
		F_C[ii][0] = (TBI[ii][0][0] * F_CI[ii][0] + TBI[ii][0][1] * F_CI[ii][1]
				+ TBI[ii][0][2] * F_CI[ii][2]);
		F_C[ii][1] = (TBI[ii][1][0] * F_CI[ii][0] + TBI[ii][1][1] * F_CI[ii][1]
				+ TBI[ii][1][2] * F_CI[ii][2]);
		F_C[ii][2] = (TBI[ii][2][0] * F_CI[ii][0] + TBI[ii][2][1] * F_CI[ii][1]
				+ TBI[ii][2][2] * F_CI[ii][2]);
		M_C[ii][0] = (TBI[ii][0][0] * M_CI[ii][0] + TBI[ii][0][1] * M_CI[ii][1]
				+ TBI[ii][0][2] * M_CI[ii][2]);
		M_C[ii][1] = (TBI[ii][1][0] * M_CI[ii][0] + TBI[ii][1][1] * M_CI[ii][1]
				+ TBI[ii][1][2] * M_CI[ii][2]);
		M_C[ii][2] = (TBI[ii][2][0] * M_CI[ii][0] + TBI[ii][2][1] * M_CI[ii][1]
				+ TBI[ii][2][2] * M_CI[ii][2]);
		//Magnet Check
		// if (MAGCONSTANT)
		// 	{
		M_C[ii][0] = 0;
		M_C[ii][1] = 0;
		M_C[ii][2] = 0;
		// }
	}

	/* Contact Model Derivatives */
	CoeffDamper1[0] = C[1][0] / (C[0][0] + C[1][0]);
	CoeffDamper2[0] = 1 / (C[0][0] + C[1][0]);
	CoeffDamper1[1] = C[1][1] / (C[0][1] + C[1][1]);
	CoeffDamper2[1] = 1 / (C[0][1] + C[1][1]);
	CoeffDamper1[2] = -C[0][0] / (C[0][0] + C[1][0]);
	CoeffDamper2[2] = 1 / (C[0][0] + C[1][0]);
	CoeffDamper1[3] = -C[0][1] / (C[0][1] + C[1][1]);
	CoeffDamper2[3] = 1 / (C[0][1] + C[1][1]);
	for (ii = 0; ii < IVERTS; ii++) {
		s1ndot[0][ii] = CoeffDamper1[0] * delun[0][ii]
				- CoeffDamper2[0]
						* (K[0][0] * s1n[0][ii] + K[1][0] * s2n[0][ii]);
		s1ndot[1][ii] = CoeffDamper1[0] * delun[1][ii]
				- CoeffDamper2[0]
						* (K[0][0] * s1n[1][ii] + K[1][0] * s2n[1][ii]);
		s1ndot[2][ii] = CoeffDamper1[0] * delun[2][ii]
				- CoeffDamper2[0]
						* (K[0][0] * s1n[2][ii] + K[1][0] * s2n[2][ii]);
		s1tdot[0][ii] = CoeffDamper1[1] * (delut[0][ii] - delwt[0][ii])
				- CoeffDamper2[1]
						* (K[0][1] * s1t[0][ii] + K[1][1] * s2t[0][ii]);
		s1tdot[1][ii] = CoeffDamper1[1] * (delut[1][ii] - delwt[1][ii])
				- CoeffDamper2[1]
						* (K[0][1] * s1t[1][ii] + K[1][1] * s2t[1][ii]);
		s1tdot[2][ii] = CoeffDamper1[1] * (delut[2][ii] - delwt[2][ii])
				- CoeffDamper2[1]
						* (K[0][1] * s1t[2][ii] + K[1][1] * s2t[2][ii]);
		s2ndot[0][ii] = CoeffDamper1[2] * delun[0][ii]
				- CoeffDamper2[2]
						* (K[0][0] * s1n[0][ii] + K[1][0] * s2n[0][ii]);
		s2ndot[1][ii] = CoeffDamper1[2] * delun[1][ii]
				- CoeffDamper2[2]
						* (K[0][0] * s1n[1][ii] + K[1][0] * s2n[1][ii]);
		s2ndot[2][ii] = CoeffDamper1[2] * delun[2][ii]
				- CoeffDamper2[2]
						* (K[0][0] * s1n[2][ii] + K[1][0] * s2n[2][ii]);
		s2tdot[0][ii] = CoeffDamper1[3] * (delut[0][ii] - delwt[0][ii])
				- CoeffDamper2[3]
						* (K[0][1] * s1t[0][ii] + K[1][1] * s2t[0][ii]);
		s2tdot[1][ii] = CoeffDamper1[3] * (delut[1][ii] - delwt[1][ii])
				- CoeffDamper2[3]
						* (K[0][1] * s1t[1][ii] + K[1][1] * s2t[1][ii]);
		s2tdot[2][ii] = CoeffDamper1[3] * (delut[2][ii] - delwt[2][ii])
				- CoeffDamper2[3]
						* (K[0][1] * s1t[2][ii] + K[1][1] * s2t[2][ii]);
	}

	for (ii = 0; ii < IVERTS; ii++) {
		for (jj = 0; jj < 3; jj++) {
			StateDot[NVRTST * ii + NBODIES * NSTATE + jj] = s1ndot[jj][ii];
			StateDot[NVRTST * ii + NBODIES * NSTATE + jj + 3] = s1tdot[jj][ii];
			StateDot[NVRTST * ii + NBODIES * NSTATE + jj + 6] = s2ndot[jj][ii];
			StateDot[NVRTST * ii + NBODIES * NSTATE + jj + 9] = s2tdot[jj][ii];
		}
	}

}

int ContactAnalysis(double xp, double yp, double zp) {
	int contact;
	double h;
	double r, theta, phi, pstar, NormalB[3];

	contact = 0;

	//Map input vector to spherical coordinates
	r = sqrt(xp * xp + yp * yp + zp * zp);
	phi = acos(zp / r);
	theta = atan2(yp, xp);

	//Get pstar = radius of body at given phi and theta
	pstar = radius(NormalB, phi, theta);

	if (pstar > r) {
		contact = 1;
	}

	return contact;

}

double InterpolateTime(int ii, double t, double prevTime) {
	double l1, l2, l3, prevx, prevy, prevz, x, y, z, lmag, prevmag, mag;
	double qmag, p, lambda, q1, q2, q3, phi, h, step, tstar, theta, NormalB[3],
			pi, pi1;
	double ix, iy, iz, ox, oy, oz, maxiter, distance, ratio;

	prevx = PrevContactList[1][ii];
	prevy = PrevContactList[2][ii];
	prevz = PrevContactList[3][ii];

	prevmag = sqrt(prevx * prevx + prevy * prevy + prevz * prevz);

	x = ContactList[1][ii];
	y = ContactList[2][ii];
	z = ContactList[3][ii];

	mag = sqrt(x * x + y * y + z * z);
	if (!mag)
		mag = 1;

	//Compute pi and pi1
	phi = acos(prevz / prevmag);
	theta = atan2(prevy, prevx);
	pi = radius(NormalB, phi, theta);
	phi = acos(z / mag);
	theta = atan2(y, x);
	pi1 = radius(NormalB, phi, theta);

	if ((pi > prevmag) && (pi1 < mag)) {
		//Contact Break
		ix = prevx;
		iy = prevy;
		iz = prevz;
		ox = x;
		oy = y;
		oz = z;
	} else if ((pi < prevmag) && (pi1 > mag)) {
		//New Contact
		ix = x;
		iy = y;
		iz = z;
		ox = prevx;
		oy = prevy;
		oz = prevz;
	}

	//Set maxiterations and ratio
	maxiter = 100;
	ratio = 0.5;
	for (ii = 0; ii < maxiter; ii++) {
		//%Compute distance between iR and oR (inside and outside points)
		distance = sqrt(
				pow((ix - ox), 2) + pow((iy - oy), 2) + pow((iz - oz), 2));
		l1 = (ox - ix) / distance;
		l2 = (oy - iy) / distance;
		l3 = (oz - iz) / distance;
		step = distance * ratio;
		//%%Step towards outside point by golden ratio
		q1 = ix + step * l1;
		q2 = iy + step * l2;
		q3 = iz + step * l3;
		qmag = sqrt(q1 * q1 + q2 * q2 + q3 * q3);
		phi = acos(q3 / qmag);
		theta = atan2(q2, q1);
		p = radius(NormalB, phi, theta);
		//%%now figure out if you are inside or outside
		if (p > qmag) {
			//%%you are still inside
			ix = q1;
			iy = q2;
			iz = q3;
		} else {
			//%you are now outside
			ox = q1;
			oy = q2;
			oz = q3;
		}
	}

	//Now interpolate to find tstar = time at impact
	tstar = prevTime + (p - prevmag) * (t - prevTime) / (mag - prevmag);

	return tstar;

}

void NewContacts(double PrevTime, double PrevState[]) {
	int ii, contact, jj;
	double xp, yp, zp, tt[NVERTS], UpdateContact[NVERTS];
	double BigTime = TFINAL + 1.000;
	/***************************** Start Contact Analysis *****************************/
	/* Change Contact List */
	for (ii = 0; ii < IVERTS; ii++) {
		contact = 0;
		xp = ContactList[1][ii];
		yp = ContactList[2][ii];
		zp = ContactList[3][ii];
		//Need to express contact point in plane
		contact = ContactAnalysis(xp, yp, zp);
		if (contact == 1) {
			ContactList[0][ii] = 1.0;
		} else {
			ContactList[0][ii] = 0.0;
		}
	}
	/* Check to See if New Contact Occurred */
	int numChange = 0;
	for (ii = 0; ii < IVERTS; ii++) {
		if (PrevContactList[0][ii] != ContactList[0][ii])
			numChange = numChange + 1; // Increment numChange
	}

	/* Backup Time/States, Reassign ContactList, and Record New States if Required */
	if (numChange > 0) {
		double SmallTime = BigTime;
		int index = 0;
		/* Initialize SmallTime and tt[ii] to Large Values */
		for (ii = 0; ii < IVERTS; ii++)
			tt[ii] = BigTime;
		for (ii = 0; ii < IVERTS; ii++) {
			if (PrevContactList[0][ii] != ContactList[0][ii]) {
				/* Interpolate to Find Time at which the Change Occurred */
				tt[ii] = InterpolateTime(ii, TCURRENT, PrevTime);
			}
			/* Find the Time at Which the First Contact Change Occurred */
			if (tt[ii] <= SmallTime) {
				SmallTime = tt[ii];
				index = ii;
			}
		}

		if (SmallTime == BigTime) {
			printf("Interpolation Not Computed Correctly \n");
			fprintf(logfile,
					"Interpolation Not Computed Correctly run # = %d \n", irun);
			TCURRENT = TFINAL + 100;
		}

		/* Find the Contact that First Changed or Changed Together */
		for (ii = 0; ii < IVERTS; ii++)
			UpdateContact[ii] = 0;
		for (ii = 0; ii < IVERTS; ii++) {
			if (tt[ii] == SmallTime)
				UpdateContact[ii] = 1;
			else
				UpdateContact[ii] = 0;
		}

		/* Backup Time and States */
		for (ii = 0; ii < NBODIES * NSTATE; ii++) {
			State[ii] = ((State[ii] - PrevState[ii]) * (tt[index] - PrevTime))
					/ (TCURRENT - PrevTime) + PrevState[ii];
		}
		/* Update Spring Lengths */ // Might Need to Back up these also
		for (ii = 0; ii < IVERTS; ii++) {
			if (UpdateContact[ii] == 1) {
				for (jj = 0; jj < 3; jj++) {
					State[NVRTST * ii + NBODIES * NSTATE + jj] = 0.0000;
					State[NVRTST * ii + NBODIES * NSTATE + jj + 3] = 0.0000;
					State[NVRTST * ii + NBODIES * NSTATE + jj + 6] = 0.0000;
					State[NVRTST * ii + NBODIES * NSTATE + jj + 9] = 0.0000;
				}
			}
		}

		//TCURRENT CHECK
		TCURRENT = tt[index];
		// if (100*TCURRENT/TFINAL < NEXT)
		// 	{
		// 	  NEXT = 100*TCURRENT/(TFINAL);
		// 	}

		/* Update the Contact List */
		numContacts = 0;
		double x, y, z, px, py, pz;
		for (ii = 0; ii < IVERTS; ii++) {
			if ((UpdateContact[ii] == 1)) {
				x = ContactList[1][ii];
				y = ContactList[2][ii];
				z = ContactList[3][ii];
				px = PrevContactList[1][ii];
				py = PrevContactList[2][ii];
				pz = PrevContactList[3][ii];

				/* New Contact */
				contact = ContactAnalysis(x, y, z);
				if (contact == 1) {
					//There is now contact
					contact = ContactAnalysis(px, py, pz);
					if (contact == 0) {
						//There used to be contact
						ContactList[0][ii] = 1.0;
					}
				}
				/* Broken Contact */
				else if (contact == 0)
				//there is no contact
						{
					contact = ContactAnalysis(px, py, pz);
					if (contact == 1)
					//there used to be contact
							{
						ContactList[0][ii] = 0.0;
					}
				}
			} else {
				ContactList[0][ii] = PrevContactList[0][ii];
				ContactList[1][ii] = PrevContactList[1][ii];
				ContactList[2][ii] = PrevContactList[2][ii];
				ContactList[3][ii] = PrevContactList[3][ii];
			}
			if (ContactList[0][ii] == 1.0)
				numContacts = numContacts + 1;
		}
	}

	if (numContacts == 0) {
		TCONTACT = TCURRENT;
	}

}

void ModelPredictiveControl(double MPControls[3], int body, double dzs) {
	int ii, jj, kk;
	double x = StateE[NSTATE * body];
	double y = StateE[NSTATE * body + 1];
	NOMINALMPC[0] = x;
	double x0, y0, z0, phi0, theta0, psi0, u0, v0, w0, p0, q0, r0;
	x0 = NOMINALMPC[0];
	y0 = NOMINALMPC[1];
	z0 = NOMINALMPC[2];
	phi0 = NOMINALMPC[3];
	theta0 = NOMINALMPC[4];
	psi0 = NOMINALMPC[5];
	u0 = NOMINALMPC[6];
	v0 = NOMINALMPC[7];
	w0 = NOMINALMPC[8];
	p0 = NOMINALMPC[9];
	q0 = NOMINALMPC[10];
	r0 = NOMINALMPC[11];

	///CONTROLLER TESTS/////////

	//PSICOMMAND[body] = 0;

	double phicoord0, phicoord1, thetacoord0, thetacoord1;
	double qcoord, pcoord, ucoord1, ucoord0, psicoord, p1, p2, zcoord0, zcoord1;
	double duii, dpii, dqii, xcoord0, xcoord1, ycoord0, ycoord1;
	double xcMPC1, xcMPC0, xcMPCs, ucMPC;
	///Generate Waypoint
	double xs = x + 10 * cos(PSICOMMAND[body]);
	double ys = y + 10 * sin(PSICOMMAND[body]);

	///////////?CONTROLLER TESTS////////

	//ys = 11;

	///////////////////////////////////

	///Create first values
	xcoord0 = x;
	ycoord0 = y;
	double z = StateE[body * NSTATE + 2];
	zcoord0 = z;
	double phi = StateE[body * NSTATE + 3];
	double theta = StateE[body * NSTATE + 4];
	double psi = StateE[body * NSTATE + 5];
	phicoord0 = phi;
	thetacoord0 = theta;
	double u = StateE[body * NSTATE + 6];
	ucoord0 = u;
	ucMPC = u;
	xcMPC0 = 0;
	p1 = 0;
	for (ii = 0; ii < HP; ii++) {
		SCOORD = 0 + (double) (ii + 1) / HP;
		xcoord1 = x + SCOORD * (xs - x);
		ycoord1 = y + SCOORD * (ys - y);
		zcoord1 = z + dzs;
		p2 = sqrt(
				pow(xcoord1 - x, 2) + pow(ycoord1 - y, 2)
						+ pow(zcoord1 - z, 2));
		thetacoord1 = -atan2(zcoord1 - zcoord0, p2 - p1);
		psicoord = atan2(ycoord1 - ycoord0, xcoord1 - xcoord0);
		if (fabs(psi - psicoord) > PI) {
			int numloops = 0;
			psicoord = psicoord - 4 * PI;
			while ((fabs(psi - psicoord) > PI) && (numloops < 3)) {
				numloops++;
				psicoord = psicoord + 2 * PI;
			}
		}

		////////////????CONTROLLER TESTS////////////

		//psicoord = 0.1;

		//////////////////////////////////////////

		phicoord1 = -0.1 * (psi - psicoord);
		if (phicoord1 > 45 * PI / 180) {
			phicoord1 = copysign(1, phicoord1) * 45 * PI / 180;
		}

		//////////////CONTROLLER TESTS/////////////

		phicoord1 = 5 * PI / 180;

		////////////////////////////////////////

		//Now use phicoord and thetacoord to generate p and q
		pcoord = (phicoord1 - phicoord0) / timestepD;
		qcoord = (thetacoord1 - thetacoord0) / timestepD;

		//Compute ucoord
		ucoord1 = Vtrim + 4.5 * (UCOMMAND[body] - u);
		//ucoord1 = u0;
		//Reset 1s and 0s
		p1 = p2;
		phicoord0 = phicoord1;
		thetacoord0 = thetacoord1;
		xcoord0 = xcoord1;
		ycoord0 = ycoord1;
		zcoord0 = zcoord1;
		ucoord0 = ucoord1;
		xcMPC0 = xcMPC1;
		///Substract nominal values to get deltas
		duii = ucoord1 - u0;
		dpii = pcoord - p0;
		dqii = qcoord - q0;
		YCOMMANDS[ii * PMPC][0] = duii;
		YCOMMANDS[ii * PMPC + 1][0] = dpii;
		YCOMMANDS[ii * PMPC + 2][0] = dqii;
	}
	//Generate MPCSTATE
	for (ii = 0; ii < 13; ii++) {
		if (ii == 3) {
			MPCSTATE[ii][0] = u0;
			ii = ii + 1;
		}
		if (ii > 2) {
			MPCSTATE[ii][0] = StateE[body * NSTATE + ii - 1]
					- NOMINALMPC[ii - 1];
		} else {
			MPCSTATE[ii][0] = StateE[body * NSTATE + ii] - NOMINALMPC[ii];
		}
	}
	MPCSTATE[0][0] = 0;
	// matrixdisp(MPCSTATE,13,1,"MPCSTATE");
	//U = K*(YCOMMAND-KCA*mpcstate);
	matmult(KCASTATE, KCA, PMPC * HP, 13, MPCSTATE, 13, 1);
	matplusminus(YKCA, YCOMMANDS, KCASTATE, -1.00, PMPC * HP, 1);
	matmult(UD, KMPC, MMPC * HP, PMPC * HP, YKCA, PMPC * HP, 1);
	// matrixdisp(KCASTATE,PMPC*HP,1,"KCASTATE");
	// matrixdisp(YKCA,PMPC*HP,1,"YKCA");
	// matrixdisp(UD,MMPC*HP,1,"UD");
	// PAUSE();
	double xint = StateE[body * NSTATE + 13];
	double zint = StateE[body * NSTATE + 12];
	//da
	MPControls[1] = UD[1][0];

	//de
	//Before aero update
	//MPControls[2] = UD[0][0] + (double)-0.3 - 0.6*zint; //Nominal control
	MPControls[2] = UD[0][0] + (double) -0.08 - 0.0 * zint; //Nominal control

	//dthrust
	//Before aero update
	//MPControls[0] = UD[2][0] + 0.4*xint; //0.04
	MPControls[0] = UD[2][0] + 0.02 * xint; //0.04
}

void SlidingMode(double SMControls[3], double deltax, int body) {
	//Alright so we have phicommand,thetacommand,and ucommand

	//////////////////GAINS///////////////////////////////////

	double lambdaphi = 1; //Reaching
	double lambdaz = 0.1;
	double etaphi = 30; //Switching 
	double etaz = 10;
	///For ucommand
	double lambda = 10;
	double eta = 10;

	////////////////DEFINE SLIDING SURFACES/////////////////

	PCOMMAND[body] = 0;
	QCOMMAND[body] = 0;

	double phi = StateE[body * NSTATE + 3];
	double theta = StateE[body * NSTATE + 4];
	double p = StateE[body * NSTATE + 9];
	double q = StateE[body * NSTATE + 10];
	double r = StateE[body * NSTATE + 11];

	double SurfacePHI = (p - PCOMMAND[body])
			+ lambdaphi * (phi - PHIFILTERED[body]);
	double SurfaceZ = (q - QCOMMAND[body])
			+ lambdaz * (theta - THETACOMMAND[body]);
	double BLPHI = 0.1;
	double BLZ = 0.001;

	///////////////MODEL PARAMETERS//////////////////////////

	double rho = Density();
	double u = StateE[body * NSTATE + 6];
	double v = StateE[body * NSTATE + 7];
	double w = StateE[body * NSTATE + 8];
	double vel = sqrt(pow(u, 2.0000) + pow(v, 2.0000) + pow(w, 2.0000));
	double Qa = 0.5 * rho * pow(vel, 2.00);
	double alpha = 0;
	if (fabs(u) > 0) {
		alpha = atan2(w, u);
	}
	//!Slidelslip
	double beta = 0;
	if (fabs(vel) > 0) {
		beta = asin(v / vel);
	}
	double phat = wspan[body] * p / (2 * vel);
	double qhat = c * q / (2 * vel);
	double rhat = wspan[body] * r / (2 * vel);

	double C_l = C_l_beta * beta + C_l_p * phat + C_l_r * rhat
			+ C_l_dr * (DR[body]);
	double C_m = C_m_0 + C_m_alpha * alpha + C_m_q * qhat + +C_m_u * u;
	double L0 = C_l * Qa * Sarea[body] * c;
	double M0 = C_m * Qa * Sarea[body] * c;
	double FPQR[2];
	FPQR[0] = InerInv[body][0][0]
			* (L0 + q * r * Iner[body][1][1] - r * q * Iner[body][2][2]);
	FPQR[1] = InerInv[body][1][1]
			* (M0 - p * r * Iner[body][0][0] + r * p * Iner[body][2][2]);
	double IGPQR[2][2];
	IGPQR[0][1] = 0;
	IGPQR[1][0] = 0;
	IGPQR[0][0] = Iner[body][0][0] / (Qa * Sarea[body] * c * C_l_da);
	IGPQR[1][1] = Iner[body][1][1] / (Qa * Sarea[body] * c * C_m_de);
	double zint = StateE[body * NSTATE + 12];

	////////////////AILERON AND ELEVATOR COMMAND//////////////

	//da
	SMControls[1] = IGPQR[0][0]
			* (-FPQR[0] - lambdaphi * (p - PCOMMAND[body])
					- etaphi * sat(SurfacePHI, BLPHI, 0));
	//de 
	SMControls[2] = IGPQR[1][1]
			* (-FPQR[1] - lambdaz * (q - QCOMMAND[body])
					- etaz * sat(SurfaceZ, BLZ, 1)) - 0.003 * zint;

	////////THRUST CHANNEL//////

	double de = SMControls[2];
	double C_L = C_L_0 + C_L_alpha * alpha + C_L_q * qhat + C_L_de * de;
	double C_D = C_D_0 + C_D_alpha * (pow(alpha, 2)) + C_D_u * u;
	double Lift = C_L * Qa * Sarea[body];
	double Drag = C_D * Qa * Sarea[body];
	double X0 = Lift * sin(alpha) - Drag * cos(alpha) + T0;
	double FU = X0 / mass[body] + r * v - q * w;
	double SurfaceU = (u - UCOMMAND[body]) + lambda * deltax;
	double xint = StateE[body * NSTATE + 13];
	//dthrust
	SMControls[0] = (mass[body] / C_x_delThrust)
			* (-lambda * (u - UCOMMAND[body]) - eta * sat(SurfaceU, 0, 0))
			+ 0.03 * xint;

}

void FeedbackLinearization(double FLControls[3], int body) {
	//Alright so we have phicommand,thetacommand,and ucommand
	//Pick natural frequency and damping of theta and q
	double zeta, wn, kpq, kpt;
	// zeta = 10;
	// wn = 23;
	// kpq = 2*zeta*wn;
	// kpt = DSQR(wn)/kpq;
	kpq = 100;
	kpt = 4;
	//Now for phi and p
	zeta = 2;
	wn = 15;
	double kpp = 2 * zeta * wn;
	double kpphi = DSQR(wn) / kpp;

	////////////PHI COMMAND/////////

	//PHIFILTERED[body] = 5.8*PI/180;
	double gptp[3];
	double phi = StateE[body * NSTATE + 3];
	gptp[0] = -kpphi * (phi - PHIFILTERED[body]);

	///////////THETA COMMAND////////

	//THETACOMMAND[body] = 5*PI/180;
	double sphi = sin(phi);
	double cphi = cos(phi);
	double p = StateE[body * NSTATE + 9];
	double q = StateE[body * NSTATE + 10];
	double r = StateE[body * NSTATE + 11];
	double thetadot = cphi * q - sphi * r;
	double theta = StateE[body * NSTATE + 4];
	gptp[1] = -kpt * (theta - THETACOMMAND[body]) - 1.0 * thetadot;

	///////////PSI COMMAND//////////

	//gptp[2] = -1*(psi-PSICOMMAND[body]);
	gptp[2] = 0;

	//////////P AND Q COMMAND/////////

	double ctheta = cos(theta);
	double stheta = sin(theta);
	PCOMMAND[body] = -stheta * gptp[2] + gptp[0];
	QCOMMAND[body] = ctheta * sphi * gptp[2] + cphi * gptp[1];
	//PCOMMAND[body] = 0.1;
	//QCOMMAND[body] = 0.01;

	//////////GAMMA GPQR and U///////////

	double gpqr[3];
	gpqr[0] = -kpp * (p - PCOMMAND[body]);
	gpqr[1] = -kpq * (q - QCOMMAND[body]);
	double u = StateE[body * NSTATE + 6];
	double gu = -10 * (u - UCOMMAND[body]);

	///////////////////////////////

	double rho = Density();
	double v = StateE[body * NSTATE + 7];
	double w = StateE[body * NSTATE + 8];
	double vel = sqrt(pow(u, 2.0000) + pow(v, 2.0000) + pow(w, 2.0000));
	double Qa = 0.5 * rho * pow(vel, 2.00);
	double alpha = 0;
	if (fabs(u) > 0) {
		alpha = atan2(w, u);
	}
	//!Slidelslip
	double beta = 0;
	if (fabs(vel) > 0) {
		beta = asin(v / vel);
	}
	double phat = wspan[body] * p / (2 * vel);
	double qhat = c * q / (2 * vel);
	double rhat = wspan[body] * r / (2 * vel);

	double C_l = C_l_beta * beta + C_l_p * phat + C_l_r * rhat
			+ C_l_dr * (DR[body]);
	double C_m = C_m_0 + C_m_alpha * alpha + C_m_q * qhat + +C_m_u * u;
	double L0 = C_l * Qa * Sarea[body] * c;
	double M0 = C_m * Qa * Sarea[body] * c;
	double FPQR[2];
	FPQR[0] = InerInv[body][0][0]
			* (L0 + q * r * Iner[body][1][1] - r * q * Iner[body][2][2]);
	FPQR[1] = InerInv[body][1][1]
			* (M0 - p * r * Iner[body][0][0] + r * p * Iner[body][2][2]);
	double IGPQR[2][2];
	IGPQR[0][1] = 0;
	IGPQR[1][0] = 0;
	IGPQR[0][0] = Iner[body][0][0] / (Qa * Sarea[body] * c * C_l_da);
	IGPQR[1][1] = Iner[body][1][1] / (Qa * Sarea[body] * c * C_m_de);

	////////////////AILERON AND ELEVATOR COMMAND//////////////

	//da
	FLControls[1] = IGPQR[0][0] * (gpqr[0] - FPQR[0])
			+ IGPQR[0][1] * (gpqr[1] - FPQR[1]);
	double zint = StateE[body * NSTATE + 12];
	//de
	FLControls[2] = IGPQR[1][0] * (gpqr[0] - FPQR[0])
			+ IGPQR[1][1] * (gpqr[1] - FPQR[1]) - 0.0 * zint;

	////////THRUST CHANNEL//////

	double de = FLControls[2];
	double C_L = C_L_0 + C_L_alpha * alpha + C_L_q * qhat + C_L_de * de;
	double C_D = C_D_0 + C_D_alpha * (pow(alpha, 2)) + C_D_u * u;
	double Lift = C_L * Qa * Sarea[body];
	double Drag = C_D * Qa * Sarea[body];
	double X0 = Lift * sin(alpha) - Drag * cos(alpha) + T0;
	double FU = X0 / mass[body] + r * v - q * w;
	double xint = StateE[body * NSTATE + 13];
	//dthrust
	FLControls[0] = (mass[body] / C_x_delThrust) * (gu - FU) + 0.01 * xint;

}

void ControlMETATRIM() {
  int child, parent, ii, SIMPLE, jj, counter, flightdirection, dummy, body,
    idx;
  double x, y, z, phi, theta, psi, u, v, w, p, q, r, mwingpsi[3], xyz[3],
    xyzdot[3];
  double xp, yp, zp, xdotp, ydotp, zdotp, psip, rcg2wt[3], mwing[3],
    mdotwing[3];
  double rpc[3], nrpc, vpc[3], nvpc, vpcPsi[3], normC, normW, pqr[3], delXYZ,
    kpphi;
  double zerror, zdot, xdot, ydot, rtotarget[2], velnorm[2], xdt, ydt, away1,
    away2;
  double phidot, thetadot, psidot, psic, delpsi, zint, psiint, xint, yint,
    DAy, delw, ptp[3];
  double invC[2][2], c0, c1, c2, c3, d0, d1, d2, d3, ds, s, xs, ys, xdots,
    ydots, rc, vcircle[2];
  double signYc, up, psimpc, tmpc, length, upath, psipathcommand,
    vpcpathPsi[2], vpcpath[2];
  double ycommand, xcommand, cwing[3], diff, psiALT, rcpath[2],
    planedirection, pathpsi, kpp;
  double rcpathPsi[2], phipathcommand, xprime, yprime, lbar[2], normAC, gamma,
    delxz;
  double mwingoffset[3], rpcPsiOffset[2], rpcoffset[3], thrustEngage, breakpt,
    lowerbound, upperbound;
  double a, b, delxp, delyp, delzp, delxdotp, delydotp, delzdotp, delx, ZFLAG,
    breakptupper, zs;
  double breakptlower, distupper, distlower, xvar, slope, nofly, deltax,
    deltay, phiint, psiflight;
  double rpcoffsetB[3], da, de, dthrust, depath, dapath, dtpath, vpcB[3],
    DELU, cflight, sflight;
  double gptp[3], gu, cphi, sphi, ctheta, stheta, cpsi, spsi, gpqr[3],
    FPQR[2], GPQR[2][2], IGPQR[2][2];
  double rho, vel, Qa, alpha, beta, L0, M0, X0, C_L, C_D, C_l, C_m, Lift,
    Drag, FU, DELTHETA, zeta, wn, kpq, kpt;
  double lambdaphi, lambdaz, etaphi, etaz, SurfaceU, SurfacePHI, SurfaceZ,
    BLZ, BLPHI, lambda, eta;
  double xMETA = 0, yMETA = 0, xMETAdot = 0, yMETAdot = 0,
    zMETAdot = 0 ,thetaMETA = 0,rMETA = 0,qMETA = 0,pMETA = 0,phiMETA = 0,phidotMETA = 0,thetadotMETA = 0,zintMETA = 0;
  double psiMETA = 0,yintMETA = 0,uMETA = 0,vMETA = 0;

  for (ii = 0;ii<METABODIES;ii++)
    {
      z = State[ii * NSTATE + 2];
      theta = State[ii * NSTATE + 4];
      zdot = StateDot[ii * NSTATE + 2];
      u = State[ii * NSTATE + 6];

      ////CONTROLLER INDEPENDENT
      DRU[ii] = 0;
      DELTHRUSTU[ii] = 0.0 * (UCOMMAND[0] - u);
      
      //Wingtip to Wingtip
      //SimplifiedContactModel(0);
      //Tip to tail
      //SimplifiedContactModel(1);
      if (connectiontype == 0) //Wing tip to Wing tip
      {
        DEU[ii] = -0.3*(0.1*(z-ZCOMMAND[0])+0.15*zdot); //W2W Num_ac = 1, AEROMODEL = 1
        DELTHRUSTU[ii] = 0.5 * (UCOMMAND[0] - u); //W2W Num_ac = 1, AEROMODEL = 1          
      }
      else if (connectiontype == 1) //Tip to tail
      {
        DEU[ii] = -0.05*(z-ZCOMMAND[0])-0.25*zdot; //T2T , AEROMODEL = 1
        DELTHRUSTU[ii] = 0.1 * (UCOMMAND[0] - u); //T2T , AEROMODEL = 1
      }
      if (connectiontype == 2) //Lattice 
      {
        DEU[ii] = -0.05*(z-ZCOMMAND[0])-0.25*zdot; //T2T , AEROMODEL = 1
        DELTHRUSTU[ii] = 0.1 * (UCOMMAND[0] - u); //T2T , AEROMODEL = 1
      }

      // DEU[ii] = -0.05*(z+200)-0.25*zdot; //T2T , AEROMODEL = 0
      // DELTHRUSTU[ii] = 0.0 * (UCOMMAND[0] - u); //T2T , AEROMODEL = 0

      // DEU[ii] = -0.3*(0.1*(z+200)+0.15*zdot); //W2W , AEROMODEL = 0
      // DELTHRUSTU[ii] = 0.5 * (UCOMMAND[0] - u); //W2W , AEROMODEL = 0

      ////AND AILERON///
      if (fabs(DAU[ii]) > 30 * PI / 180) {
	DAU[ii] = copysign(1, DAU[ii]) * 30 * PI / 180;
      }
      ///AND ELEVATOR//
      if (fabs(DEU[ii]) > 30 * PI / 180) {
	DEU[ii] = copysign(1, DEU[ii]) * 30 * PI / 180;
      }
      ////Check for almost zero controls
      if (fabs(DAU[ii]) < 1e-8) {
	DAU[ii] = 0;
      }
      if (fabs(DEU[ii]) < 1e-8) {
	DEU[ii] = 0;
      }
      
    }

}

void ControlMETA() {
  int child, parent, ii, SIMPLE, jj, counter, flightdirection, dummy, body,
    idx;
  double x, y, z, phi, theta, psi, u, v, w, p, q, r, mwingpsi[3], xyz[3],
    xyzdot[3];
  double xp, yp, zp, xdotp, ydotp, zdotp, psip, rcg2wt[3], mwing[3],
    mdotwing[3];
  double rpc[3], nrpc, vpc[3], nvpc, vpcPsi[3], normC, normW, pqr[3], delXYZ,
    kpphi;
  double zerror, zdot, xdot, ydot, rtotarget[2], velnorm[2], xdt, ydt, away1,
    away2;
  double phidot, thetadot, psidot, psic, delpsi, zint, psiint, xint, yint,
    DAy, delw, ptp[3];
  double invC[2][2], c0, c1, c2, c3, d0, d1, d2, d3, ds, s, xs, ys, xdots,
    ydots, rc, vcircle[2];
  double signYc, up, psimpc, tmpc, length, upath, psipathcommand,
    vpcpathPsi[2], vpcpath[2];
  double ycommand, xcommand, cwing[3], diff, psiALT, rcpath[2],
    planedirection, pathpsi, kpp;
  double rcpathPsi[2], phipathcommand, xprime, yprime, lbar[2], normAC, gamma,
    delxz;
  double mwingoffset[3], rpcPsiOffset[2], rpcoffset[3], thrustEngage, breakpt,
    lowerbound, upperbound;
  double a, b, delxp, delyp, delzp, delxdotp, delydotp, delzdotp, delx, ZFLAG,
    breakptupper, zs;
  double breakptlower, distupper, distlower, xvar, slope, nofly, deltax,
    deltay, phiint, psiflight;
  double rpcoffsetB[3], da, de, dthrust, depath, dapath, dtpath, vpcB[3],
    DELU, cflight, sflight;
  double gptp[3], gu, cphi, sphi, ctheta, stheta, cpsi, spsi, gpqr[3],
    FPQR[2], GPQR[2][2], IGPQR[2][2];
  double rho, vel, Qa, alpha, beta, L0, M0, X0, C_L, C_D, C_l, C_m, Lift,
    Drag, FU, DELTHETA, zeta, wn, kpq, kpt;
  double lambdaphi, lambdaz, etaphi, etaz, SurfaceU, SurfacePHI, SurfaceZ,
    BLZ, BLPHI, lambda, eta;
  double xMETA = 0, yMETA = 0, xMETAdot = 0, yMETAdot = 0,
    zMETAdot = 0 ,thetaMETA = 0,rMETA = 0,qMETA = 0,pMETA = 0,phiMETA = 0,phidotMETA = 0,thetadotMETA = 0,zintMETA = 0;
  double psiMETA = 0,yintMETA = 0,uMETA = 0,vMETA = 0,wMETA = 0;
  double k1,k2,k1phi,kpe,kde,ku,zMETA = 0,phic,k1theta=0,thetac;

  for (ii = 0; ii < METABODIES; ii++) {
    xMETA = xMETA + (1 / (double) METABODIES) * State[ii * NSTATE];
    yMETA = yMETA + (1 / (double) METABODIES) * State[ii * NSTATE + 1];
    zMETA = zMETA + (1 / (double) METABODIES) * State[ii * NSTATE + 2];
    xMETAdot = xMETAdot + (1 / (double) METABODIES) * StateDot[ii * NSTATE];
    yMETAdot = yMETAdot + (1 / (double) METABODIES) * StateDot[ii * NSTATE + 1];
    zMETAdot = zMETAdot + (1 / (double) METABODIES) * StateDot[ii * NSTATE + 2];
  }

  //COMMANDS
  MSLOPE = tan(10*PI/180);
  if (TCURRENT == 0) {
    BINTERCEPT = yMETA + 0;
  }
  #ifndef JOYSTICK
  ZCOMMAND[0] = -205;
  #endif
  UCOMMAND[0] = Vtrim;

  //Compute Heading command
  lbar[0] = cos(MSLOPE);
  lbar[1] = sin(MSLOPE);
  planedirection = lbar[0] * (xMETA - 0) + lbar[1] * (yMETA - BINTERCEPT);
  normAC = sqrt(xMETA * xMETA + yMETA * yMETA);
  flightdirection = copysign(1, xMETAdot * lbar[0] + yMETAdot * lbar[1]);
  xs = 0 + (planedirection + flightdirection * ARCLENGTHPARENT) * lbar[0];
  ys = BINTERCEPT + (planedirection + flightdirection * ARCLENGTHPARENT) * lbar[1];
  rcpath[0] = xs - xMETA;
  rcpath[1] = ys - yMETA;
  rcpathPsi[0] = cos(psiMETA)*rcpath[0] + sin(psiMETA)*rcpath[1];
  rcpathPsi[1] = -sin(psiMETA)*rcpath[0] + cos(psiMETA)*rcpath[1];
  vpcpathPsi[0] = cos(MSLOPE)*xMETAdot + sin(MSLOPE)*yMETAdot;
  vpcpathPsi[1] = -sin(MSLOPE)*xMETAdot + cos(MSLOPE)*yMETAdot;
    
  delpsi = -1*rcpathPsi[1] + 7*vpcpathPsi[1];

  //Elevator and thrust Gains

  if (connectiontype == 1 || connectiontype == 2)//Tip to tail or lattice)
    {
      double b0 = -0.29;
      double b1 = 0.0842857142857156;
      double b2 = -0.0357142857142859;

      kpe = -0.05*NUM_ROW;
      kde = b0 + b1*NUM_ROW + b2*DSQR(NUM_ROW);

      k1theta = 1.0;
      ku = 0.1;
    }
  else if (connectiontype == 0)//wing tip to wing tip
    {
      kpe = -0.1;
      kde = -0.15;
      k1theta = 0.3;
      ku = 0.5;
    }

  //Aileron Gains
  if (connectiontype == 1) //tip to tail
    {
      k1 = 95.5724006081462;
      k2 = 1.08114195381555;
      k1phi = -0.170958696146073;
    }
  else if (connectiontype == 0 || connectiontype == 2) //wing tip to wing tip or lattice
    {
      double a1 = 0.0918601064332432;
      double a2 = 147.188993820914;
      double a3 = 8.31219884504991;
      k1 = a1 + a2/DSQR(NUM_COL) + a3/(pow(NUM_COL,3));
      k2 = 3;
      k1phi = -0.015;
    }


  ///Loop through all bodies and use own error signal
  for (ii = 0;ii<BODIES;ii++)
    {
      //Replace all METAS with actual values
      zMETA = State[ii * NSTATE + 2];
      phiMETA = State[ii * NSTATE + 3];
      thetaMETA = State[ii * NSTATE + 4];
      uMETA = State[ii * NSTATE + 6];
      zMETAdot = StateDot[ii * NSTATE + 2];
      phidotMETA = StateDot[ii * NSTATE + 3];
      thetadotMETA = State[ii * NSTATE + 4];

      ///AILERON
      phic = k1phi*delpsi;
      
      ///Phi command saturation
      if (fabs(phic) > 15*PI/180){
	phic = sign(phic)*15*PI/180;
      }

      // if (ii == 0)
      // 	{
      // 	  phic = -20*PI/180;
      // 	}
      // else
      // 	{
      // 	  phic = 20*PI/180;
      // 	}

      DAU[ii] = k1*(phiMETA-phic) + k2*phidotMETA;

      ////RUDDER
      DRU[ii] = 0;

      //ELEVATOR

      thetac = -(kpe*(zMETA-ZCOMMAND[0])+kde*(zMETAdot));

      //Theta command saturation
      if (fabs(thetac) > 30*PI/180)
      	{
      	  thetac = sign(thetac)*30*PI/180;
      	}
      //printf("Thetac = %lf \n",thetac);

      DEU[ii] = k1theta*(thetaMETA-thetac);

      //THRUST
      DELTHRUST[ii] = ku*(UCOMMAND[0]-uMETA);

      //SATURATION
      if (fabs(DAU[ii]) > 30 * PI / 180) {
	DAU[ii] = copysign(1, DAU[ii]) * 30 * PI / 180;
      }
      if (fabs(DEU[ii]) > 30 * PI / 180) {
	DEU[ii] = copysign(1, DEU[ii]) * 30 * PI / 180;
      }

      //ZEROS
      if (fabs(DAU[ii]) < 1e-8) {
	DAU[ii] = 0;
      }
      if (fabs(DEU[ii]) < 1e-8) {
	DEU[ii] = 0;
      }
    }
}

void Control(int body) {
	int child, parent, ii, SIMPLE, jj, counter, flightdirection, dummy;
	double x, y, z, phi, theta, psi, u, v, w, p, q, r, mwingpsi[3], xyz[3],
			xyzdot[3];
	double xp, yp, zp, xdotp, ydotp, zdotp, psip, rcg2wt[3], mwing[3],
			mdotwing[3];
	double rpc[3], nrpc, vpc[3], nvpc, vpcPsi[3], normC, normW, pqr[3], delXYZ,
			kpphi;
	double zerror, zdot, xdot, ydot, rtotarget[2], velnorm[2], xdt, ydt, away1,
			away2;
	double phidot, thetadot, psidot, psic, delpsi, zint, psiint, xint, yint,
			DAy, delw, ptp[3];
	double invC[2][2], c0, c1, c2, c3, d0, d1, d2, d3, ds, s, xs, ys, xdots,
			ydots, rc, vcircle[2];
	double signYc, up, psimpc, tmpc, length, upath, psipathcommand,
			vpcpathPsi[2], vpcpath[2];
	double ycommand, xcommand, cwing[3], diff, psiALT, rcpath[2],
			planedirection, pathpsi, kpp;
	double rcpathPsi[2], phipathcommand, xprime, yprime, lbar[2], normAC, gamma,
			delxz;
	double mwingoffset[3], rpcPsiOffset[2], rpcoffset[3], thrustEngage, breakpt,
			lowerbound, upperbound;
	double a, b, delxp, delyp, delzp, delxdotp, delydotp, delzdotp, delx, ZFLAG,
			breakptupper, zs;
	double breakptlower, distupper, distlower, xvar, slope, nofly, deltax,
			deltay, phiint, psiflight;
	double rpcoffsetB[3], da, de, dthrust, depath, dapath, dtpath, vpcB[3],
			DELU, cflight, sflight;
	double gptp[3], gu, cphi, sphi, ctheta, stheta, cpsi, spsi, gpqr[3],
			FPQR[2], GPQR[2][2], IGPQR[2][2];
	double rho, vel, Qa, alpha, beta, L0, M0, X0, C_L, C_D, C_l, C_m, Lift,
			Drag, FU, DELTHETA, zeta, wn, kpq, kpt;
	double lambdaphi, lambdaz, etaphi, etaz, SurfaceU, SurfacePHI, SurfaceZ,
			BLZ, BLPHI, lambda, eta;

	/* Initialize Controls */
	DEU[body] = 0;
	DAU[body] = 0;
	DELTHRUSTU[body] = 0;
	DRU[body] = 0;

	//Unwrap State Vector
	x = StateE[body * NSTATE];
	y = StateE[body * NSTATE + 1];
	z = StateE[body * NSTATE + 2];
	phi = StateE[body * NSTATE + 3];
	theta = StateE[body * NSTATE + 4];
	psi = StateE[body * NSTATE + 5];
	sphi = sin(phi);
	cphi = cos(phi);
	ctheta = cos(theta);
	stheta = sin(theta);
	cpsi = cos(psi);
	spsi = sin(psi);
	u = StateE[body * NSTATE + 6];
	v = StateE[body * NSTATE + 7];
	w = StateE[body * NSTATE + 8];
	p = StateE[body * NSTATE + 9];
	q = StateE[body * NSTATE + 10];
	r = StateE[body * NSTATE + 11];
	zint = StateE[body * NSTATE + 12];
	xint = StateE[body * NSTATE + 13];
	yint = StateE[body * NSTATE + 14];
	//Unwrap Derivatives
	xdot = StateEDot[body * NSTATE];
	ydot = StateEDot[body * NSTATE + 1];
	zdot = StateEDot[body * NSTATE + 2];
	phidot = StateEDot[body * NSTATE + 3];
	thetadot = StateEDot[body * NSTATE + 4];
	psidot = StateEDot[body * NSTATE + 5];

	//PARENT CONTROLLER
	if (body == 0) {
		//////PID CONTROLLER FOR PARENT/////
		/////////////Speed Controller////////////

		DELTHRUSTU[body] = 2 * (UCOMMAND[0] - u);

		////////////Altitude Hold and Rudder///////////////////

		zerror = z - ZCOMMAND[0];
		if (TCURRENT > 0.1) {
			DELTHETA = 0.1 * zerror + 0.15 * zdot;
			THETACOMMAND[body] = theta + DELTHETA;
			DEU[body] = -0.3 * (THETACOMMAND[body] - theta);
			DRU[body] = -KV * v;
		} else {
			DEU[body] = -5 * PI / 180;
			DELTHRUSTU[body] = 0;
			DRU[body] = 0;
		}

		/////////////AILERON COMMAND///////////
		//Figure out waypoint for Parent aircraft
		BINTERCEPT = INITIAL[body * NSTATE + 1];
		///////////CONTROLLER TESTS////

		//BINTERCEPT = 1;

		///////////////////////
		lbar[0] = cos(MSLOPE);
		lbar[1] = sin(MSLOPE);
		planedirection = lbar[0] * (x - 0) + lbar[1] * (y - BINTERCEPT);
		normAC = sqrt(x * x + y * y);
		flightdirection = copysign(1, xdot * lbar[0] + ydot * lbar[1]);
		xs = 0 + (planedirection + flightdirection * ARCLENGTHPARENT) * lbar[0];
		ys = BINTERCEPT
				+ (planedirection + flightdirection * ARCLENGTHPARENT)
						* lbar[1];
		rcpath[0] = xs - x;
		rcpath[1] = ys - y;
		//Convert to Body Frame
		rcpathPsi[0] = cos(psi) * rcpath[0] + sin(psi) * rcpath[1];
		rcpathPsi[1] = -sin(psi) * rcpath[0] + cos(psi) * rcpath[1];
		vpcpathPsi[0] = cos(MSLOPE) * xdot + sin(MSLOPE) * ydot;
		vpcpathPsi[1] = -sin(MSLOPE) * xdot + cos(MSLOPE) * ydot;
		delpsi = -0.01 * rcpathPsi[1] + 0.07 * vpcpathPsi[1];
		PSICOMMAND[body] = psi - delpsi;

		/////////CONTROLLER TEST////////////////
		// if (TCURRENT > 5)
		// 	{
		// 	  PSICOMMAND[body] = 5*PI/180;
		// 	  delpsi = psi - PSICOMMAND[body];
		// 	}
		// else
		// 	{
		// 	  PSICOMMAND[body] = 0*PI/180;
		// 	  delpsi = psi - PSICOMMAND[body];
		// 	}
		/////////////////////////////////

		phipathcommand = -0.6 * delpsi - 0 * psidot;
		if (fabs(phipathcommand) > 45 * PI / 180) {
			phipathcommand = copysign(1, phipathcommand) * 45 * PI / 180;
		}
		//////?CONTROLLER TEST?//////
		// if (TCURRENT > 5)
		// 	{
		// 	  phipathcommand = 5*PI/180;
		// 	}
		// else
		// 	{
		// 	  phipathcommand = 0;
		// 	}
		///////////////////////////////
		//DAU[body] = 2.4*(phi-phipathcommand)+0.4*p; //1.2 and 0.1
		DAU[body] = 1.2 * (phi - phipathcommand) + 0.1 * p; //1.2 and 0.1
	}
	////////////CHILD CONTOLLER////////////
	if (body == 1) {
		//Easier Indices
		child = body;
		parent = body - 1;

		//VISNAV SENSOR MEASUREMENTS
		delxp = VisNav[0];
		delyp = VisNav[1];
		delzp = VisNav[2];
		delxdotp = VisNav[3];
		delydotp = VisNav[4];
		delzdotp = VisNav[5];

		//Location of Wingtip//
		rcg2wt[0] = 0;
		rcg2wt[1] = wspan[1] / 2;
		rcg2wt[2] = 0;

		//DEFINE LOCATION OF WINGS//
		cwing[0] = TIB[child][0][0] * rcg2wt[0]
				+ TIB[child][0][1] * (-rcg2wt[1])
				+ TIB[child][0][2] * rcg2wt[2];
		cwing[1] = TIB[child][1][0] * rcg2wt[0]
				+ TIB[child][1][1] * (-rcg2wt[1])
				+ TIB[child][1][2] * rcg2wt[2];
		cwing[2] = TIB[child][2][0] * rcg2wt[0]
				+ TIB[child][2][1] * (-rcg2wt[1])
				+ TIB[child][2][2] * rcg2wt[2];
		//OFFET LOCATION OF PARENT WING//
		delw = wspan[0] * 2;
		mwingoffset[0] = delxp + TIB[parent][0][0] * rcg2wt[0]
				+ TIB[parent][0][1] * (rcg2wt[1] + delw)
				+ TIB[parent][0][2] * rcg2wt[2];
		mwingoffset[1] = delyp + TIB[parent][1][0] * rcg2wt[0]
				+ TIB[parent][1][1] * (rcg2wt[1] + delw)
				+ TIB[parent][1][2] * rcg2wt[2];
		mwingoffset[2] = delzp + TIB[parent][2][0] * rcg2wt[0]
				+ TIB[parent][2][1] * (rcg2wt[1] + delw)
				+ TIB[parent][2][2] * rcg2wt[2];
		//LOCATION OF PARENT WING//
		mwing[0] = delxp + TIB[parent][0][0] * rcg2wt[0]
				+ TIB[parent][0][1] * (rcg2wt[1])
				+ TIB[parent][0][2] * rcg2wt[2];
		mwing[1] = delyp + TIB[parent][1][0] * rcg2wt[0]
				+ TIB[parent][1][1] * (rcg2wt[1])
				+ TIB[parent][1][2] * rcg2wt[2];
		mwing[2] = delzp + TIB[parent][2][0] * rcg2wt[0]
				+ TIB[parent][2][1] * (rcg2wt[1])
				+ TIB[parent][2][2] * rcg2wt[2];

		///CONTROLLER TEST///

		// mwing[2] = ZCOMMAND[0]-1;
		// cwing[2] = z;

		//DEFINE DELTA FROM PARENT WING TO CHILD WING
		rpc[0] = mwing[0] - cwing[0];
		rpc[1] = mwing[1] - cwing[1];
		rpc[2] = mwing[2] - cwing[2];

		//////////////////////PATH COMMAND///////////////////

		//equation derived in notes (May 24th)
		double x0, y0, psi0, x1, y1, psi1, xdot0, ydot0, xdot1, ydot1;
		x1 = x;
		y1 = y;
		psi1 = INITIAL[3]; //location of child //assume psi1 = psi0
		x0 = StateE[0];
		y0 = StateE[1];
		psi0 = INITIAL[3]; //location of parent

		//rcpathPsi[1] = (delw + y0*cos(psi0) - y1*cos(psi0) - x0*sin(psi0) + x1*sin(psi0))/cos(psi0 - psi1);
		//if psi1 = psi0 then cos(psi0-psi1) = 1
		rcpathPsi[1] = (1.47 * delw + y0 * cos(psi0) - y1 * cos(psi0)
				- x0 * sin(psi0) + x1 * sin(psi0));

		xdot1 = xdot;
		ydot1 = ydot; //velocity of child
		xdot0 = StateEDot[0];
		ydot0 = StateEDot[1]; //velocity of parent

		vpcpathPsi[1] = -sin(psi0) * (xdot0 - xdot1)
				+ cos(psi0) * (ydot0 - ydot1);

		delpsi = -0.02 * rcpathPsi[1] - 0.050 * vpcpathPsi[1];
		phipathcommand = -1 * delpsi;
		if (fabs(phipathcommand) > 45 * PI / 180) {
			phipathcommand = copysign(1, phipathcommand) * 45 * PI / 180;
		}
		PHIDELT = phi - phipathcommand;

		///AILERON GLOBAL TRACKING COMMAND
		dapath = 1.2 * (phi - phipathcommand) + 0.1 * p; //1.2 and 0.1

		///ELEVATOR GLOBAL TRACKING COMMAND
		zerror = -rpc[2];
		ZDELT = rpc[2];
		depath = -0.03 * zerror - 0.045 * zdot - 0.009 * zint;

		///THRUST GLOBAL TRACKING COMMAND
		UCOMMAND[body] = Vtrim;
		dtpath = 2.0 * (UCOMMAND[body] - u);

		//WINGTIP TRACKING CONTROLLER//
		//Define Relative vectors

		//Let psiflight be the flight path angle
		psiflight = atan2(ydot, xdot);
		cflight = cos(psiflight);
		sflight = sin(psiflight);
		rpcInt[0] = cflight * rpc[0] + sflight * (rpc[1]);
		rpcInt[1] = -sflight * rpc[0] + cflight * (rpc[1]);
		rpcInt[2] = rpc[2];

		//Rotate to account for theta
		rpcB[0] = ctheta * rpcInt[0] - stheta * rpcInt[2];
		rpcB[1] = rpcInt[1];
		rpcB[2] = rpcInt[2];

		//Compute the offset value
		rpcoffset[0] = mwingoffset[0] - cwing[0];
		rpcoffset[1] = mwingoffset[1] - cwing[1];
		rpcoffset[2] = mwingoffset[2] - cwing[2];

		//Rotate to heading angle frame
		rpcoffsetB[0] = cflight * rpcoffset[0] + sflight * (rpcoffset[1]);
		rpcoffsetB[1] = -sflight * rpcoffset[0] + cflight * (rpcoffset[1]);
		rpcoffsetB[2] = rpcoffset[2];

		//Define relative velocity vectors
		vpc[0] = delxdotp;
		vpc[1] = delydotp;
		vpc[2] = delzdotp;
		vpcB[0] = cflight * vpc[0] + sflight * (vpc[1]);
		vpcB[1] = -sflight * vpc[0] + cflight * (vpc[1]);
		vpcB[2] = vpc[2];

		//Create PID, FDBK LINEARIZATION, and SLIDING MODE MAPPING

		/////////////////UCOMMAND/////////////////////

		deltax = rpcB[0];
		if (deltax > 0.8)
			deltax = 0.8;

		DELU = 2.0 * deltax + 0.6 * vpcB[0] + 0.2 * xint;
		UCOMMAND[body] = u + DELU;

		////////////////THETACOMMAND/////////////////

		DELTHETA = -0.1 * rpcB[2] + 0.15 * zdot + 0.02 * zint;
		THETACOMMAND[body] = theta + DELTHETA;
		if (fabs(THETACOMMAND[body]) > 45 * PI / 180) {
			THETACOMMAND[body] = copysign(1, THETACOMMAND[body]) * 45 * PI
					/ 180;
		}

		///////////////PHICOMMAND///////////////////

		//delpsi = -0.005*(WAYPOINTOFFSET*rpcoffsetB[1]+WAYPOINT*rpcB[1]) - 0.050*vpcB[1] - 0.002*yint;
		//delpsi = G1*(WAYPOINTOFFSET*rpcoffsetB[1]+WAYPOINT*rpcB[1]) + G2*vpcB[1] - 0.0000*yint;
		delpsi = -0.01 * (WAYPOINTOFFSET * rpcoffsetB[1] + WAYPOINT * rpcB[1])
				- 0.06 * vpcB[1] - 0.00 * yint;
		PSICOMMAND[child] = psi - delpsi;

		//////////??CONTROLLER TEST?/////////////

		//PSICOMMAND[child] = 5*PI/180;

		///////////////////////////////////////

		delpsi = psi - PSICOMMAND[child];
		PHICOMMAND[body] = -0.6 * delpsi;
		//PHICOMMAND[body] = -G1*delpsi;
		//Limit Phicommand
		if (fabs(PHICOMMAND[body]) > 45 * PI / 180) {
			PHICOMMAND[body] = copysign(1, PHICOMMAND[body]) * 45 * PI / 180;
		}
		PHIFILTERED[body] = PHICOMMAND[body];

		switch (CONTROLTYPE) {
		case 0:
			//PID Controller
			//UCOMMAND[body] = 20;
			dthrust = 2.0 * (UCOMMAND[body] - u);
			PHIFILTERED[body] = 0;
			da = 1.2 * (phi - PHIFILTERED[body]) + 0.1 * p; //1.2 and 0.1
			de = -0.3 * (THETACOMMAND[body] - theta) - 0.01 * zint;
			break;
		case 1:
			///Feedback Linearization Controller
			double FLControls[3];
			FeedbackLinearization(FLControls, body);
			dthrust = FLControls[0];
			da = FLControls[1];
			de = FLControls[2];

			//dthrust = 2.0*(UCOMMAND[body]-u);
			//da = 1.2*(phi-PHIFILTERED[body])+0.1*p; //1.2 and 0.1
			//de = -0.3*(THETACOMMAND[body]-theta)-0.0*zint;

			break;
		case 2:
			///Sliding Mode Control
			double SMControls[3];
			SlidingMode(SMControls, deltax, body);
			dthrust = SMControls[0];
			da = SMControls[1];
			de = SMControls[2];
			break;
		case 3:
			//Model Predicice Control
			double MPControls[3];
			ModelPredictiveControl(MPControls, body, rpcB[2]);
			//dthrust = MPControls[0];
			da = MPControls[1];
			de = MPControls[2];

			//PID Controller
			dthrust = 2.0 * (UCOMMAND[body] - u);
			//PHIFILTERED[body] = 0;
			//da = 1.2*(phi-PHIFILTERED[body])+0.1*p; //1.2 and 0.1
			//de = -0.3*(THETACOMMAND[body]-theta)-0.0*zint;

			////////////////

			break;
		default:
			dthrust = 0;
			da = 0;
			de = 0;
			break;
		}

		////////////BLEND CONTROLS////////
		if (THOLD < 0.02) {
			THOLD = 0;
			//printf("THOLD = 0 \n");
		} else {
			breakpt = 1;
			lowerbound = 0.001;
			upperbound = 0.99;
			b = log(1 / lowerbound - 1);
			a = (log(upperbound) - b) / breakpt;
			int THOLDNEW = 1 / (1 + exp(a * fabs(rpcoffsetB[1]) + b));
			if (THOLDNEW < THOLD) {
				THOLD = THOLD + 0.0001 * (THOLDNEW - THOLD);
			}
		}
		if (THOLD < 0.1) {
			TY = (1 - THOLD);
			THOLDPRIME = THOLD;
		} else {
			TY = 0;
			THOLDPRIME = 1;
		}

		/////////////SAFETY CONE DEFINITIONS////////

		//Conservative

		breakptupper = 0.4;
		breakptlower = 0.2;
		distupper = 1;
		distlower = 0.1;
		nofly = 0;

		if (rpcB[0] > 0.8) {
			rpcB[0] = 0.8;
		}

		xvar = -rpcB[1];
		if (xvar > distupper) {
			breakpt = breakptupper;
		} else if (xvar < distlower) {
			breakpt = breakptlower;
		} else {
			slope = (breakptupper - breakptlower) / (distupper - distlower);
			breakpt = breakptlower + slope * (xvar - distlower);
		}

		lowerbound = 0.0001;
		upperbound = 0.99;
		b = log(1 / lowerbound - 1);
		a = (log(upperbound) - b) / breakpt;
		delxz = sqrt(pow(delzp, 2) + pow(rpcB[0], 2));
		//delxz = fabs(rpcB[0]);
		if (PSIPATH < 0.02) {
			PSIPATH = 0;
		} else {
			PSIPATH = 1 / (1 + exp(a * fabs(delxz) + b));
		}
		PSIY = 1 - PSIPATH;
		if (PSIY == 1) {
			///Compute waypoint

			//ONLY DO THIS THE FIRST TIME
			if (STARTY == 0) {
				WAYPOINT = 1 - 1 / (1 + exp(a * fabs(delxz) + b));
				if (WAYPOINT > 0.1) {
					STARTY = 1;
				}
			}

			if (WAYPOINT == 0) {
				if ((fabs(rpcoffsetB[1]) < 0.1) && (fabs(vpcB[1]) < 0.1)) {
					WAYPOINT = 1 - 1 / (1 + exp(a * fabs(delxz) + b));
				}
			} else {
				WAYPOINT = 1 - 1 / (1 + exp(a * fabs(delxz) + b));
			}

			///CHECK TO MAKE SURE YOU HAVEN'T OVERSHOT THE BOUNDARY
			if (rpcB[1] > -nofly) {
				WAYPOINT = 0;
			}

			//CHECK TO MAKE SURE THRUST IS ALMOST FULLY ON
			if (TY < 0.9) {
				WAYPOINT = 0;
			}
			WAYPOINTOFFSET = 1 - WAYPOINT;
		}

		YDELT = 0;

		///CONTROLLER TEST

		// THOLDPRIME = 1;
		// TY = 0;
		// PSIY = 0;
		// PSIPATH = 1;
		// WAYPOINT = 0;
		// WAYPOINTOFFSET = 1;

		///////////////////

		if (WAYPOINT > 0.8) {
			YDELT = rpcB[1];
		} else {
			YDELT = rpcoffsetB[1];
		}

		//////////////////BLEND CONTROLS///////////////

		DAU[body] = PSIPATH * dapath + PSIY * da;
		DAU[body] = da;
		// DEU[body] = PSIPATH*depath + PSIY*de;
		DEU[body] = de;
		DELTHRUSTU[body] = THOLDPRIME * dtpath + TY * dthrust;
		DRU[body] = -KV * v;

	}

	//CONTACT//
	if (GOODCONTACT) {
		zerror = z - ZCOMMAND[0];
		DELTHETA = 0.1 * zerror + 0.15 * zdot;
		THETACOMMAND[body] = theta + DELTHETA;
		DEU[body] = -0.3 * (THETACOMMAND[body] - theta);
		//DEU[body] = 0;
		DAU[body] = 1.2 * (phi) + 0.1 * p; //1.2 and 0.1
		//DAU[body] = 0;
		DELTHRUSTU[body] = 2 * (UCOMMAND[0] - u);
		//DELTHRUSTU[body] = 0;
		DRU[body] = 0;
	}

	if ((TY > 0.8)) {
		XDELT = rpcB[0];
	} else {
		XDELT = 0;
	}
	////////CHECK FOR SATURATION ON THRUST////////////
	if ((C_x_delThrust * DELTHRUSTU[body] + T0) > MAXTHRUST) {
		DELTHRUSTU[body] = (MAXTHRUST - T0) / C_x_delThrust;
		XDELT = 0;
	}
	////CHECK FOR SATURATION ON VELOCITY
	if (u > (UCOMMAND[0] + 4)) {
		DELTHRUSTU[body] = -T0 / C_x_delThrust;
	}
	////////CHECK FOR NEGATIVE THRUST/////////////
	if ((C_x_delThrust * DELTHRUSTU[body] + T0) < 0) //-0.5
			{
		DELTHRUSTU[body] = -T0 / C_x_delThrust; //-0.5
		XDELT = 0;
	}
	////AND AILERON///
	if (fabs(DAU[body]) > 30 * PI / 180) {
		DAU[body] = copysign(1, DAU[body]) * 30 * PI / 180;
	}
	///AND ELEVATOR//
	if (fabs(DEU[body]) > 30 * PI / 180) {
		DEU[body] = copysign(1, DEU[body]) * 30 * PI / 180;
	}
	///Add in check for small control deflections
	if (fabs(DEU[body]) < 1e-8) {
		DEU[body] = 0.00;
	}
	if (fabs(DAU[body]) < 1e-8) {
		DAU[body] = 0.00;
	}
	if (fabs(DRU[body]) < 1e-8) {
		DRU[body] = 0.00;
	}

}

void MPC_Setup() {
	int ii, jj, nn, ll, kk;
	FILE* fid = NULL;
	double dummy, **QD, **RD, **KCAB, **prodA, **Acont;
	double **tempC, **rowKCAB, **rowcolKCAB, **tempKCAB, **rowcolKCABB;
	double upenalty, ppenalty, qpenalty, dTpenalty, depenalty, dapenalty;
	double **KCABT, **QKCAB, **KCABTQKCAB, **KCABTQKCABR;

	ND = 13;
	P = 3;
	M = 3;
	HP = 10;
	PMPC = P;
	MMPC = M;

	//Read the MPC File
	//TimestepD = 0.01
	fid = fopen(inMPCFile, "r");
	if (fid == NULL) {
	  file_error(inMPCFile,"MPCFile");
	}
	//Read A Matrix
	AMPC = matrixallocatedbl(13, 13);
	for (ii = 0; ii < 13; ii++) {
		for (jj = 0; jj < 13; jj++) {
			fscanf(fid, "%lf ", &AMPC[ii][jj]);
		}
	}
	//Read B Matrix
	BMPC = matrixallocatedbl(13, 3);
	for (ii = 0; ii < 13; ii++) {
		for (jj = 0; jj < 3; jj++) {
			fscanf(fid, "%lf ", &BMPC[ii][jj]);
		}
	}
	//Read Nominal MPC Files
	for (ii = 0; ii < 12; ii++) {
		fscanf(fid, "%lf \n", &NOMINALMPC[ii]);
	}
	//Fix x and psi
	NOMINALMPC[0] = INITIAL[NSTATE]; //x
	NOMINALMPC[5] = INITIAL[NSTATE + 5]; //psi

	//Create C Matrix
	CD = matrixallocatedbl(P, ND);
	CD[0][7] = 1;
	CD[1][10] = 1;
	CD[2][11] = 1;

	//////////Create Q matrix///////////////

	QD = eye(P * HP, P * HP);
	upenalty = 15; //150 
	ppenalty = 10; //1
	//ppenalty = G2;
	qpenalty = 0.01; //2.5
	for (ii = 0; ii < P; ii++) {
		QD[ii * P][ii * P] = upenalty;
		QD[ii * P + 1][ii * P + 1] = ppenalty;
		QD[ii * P + 2][ii * P + 2] = qpenalty;
	}

	///////////R matrix////////////

	RD = eye(M * HP, M * HP);
	dTpenalty = 2.5;
	dapenalty = 120; //0.001
	//dapenalty = G1;
	depenalty = 20; //0.06
	for (ii = 0; ii < M; ii++) {
		RD[ii * M][ii * M] = depenalty;
		RD[ii * M + 1][ii * M + 1] = dapenalty;
		RD[ii * M + 2][ii * M + 2] = dTpenalty;
	}
	//KCA AND KCAB MATRICES
	KCA = matrixallocatedbl(HP * P, ND);
	KCAB = matrixallocatedbl(P * HP, M * HP);
	UD = matrixallocatedbl(HP * M, 1);
	KCASTATE = matrixallocatedbl(P * HP, 1);
	YKCA = matrixallocatedbl(P * HP, 1);
	YKCAD = matrixallocatedbl(P * HP, 1);
	MPCSTATE = matrixallocatedbl(13, 1);
	YCOMMANDS = matrixallocatedbl(P * HP, 1);
	prodA = matrixallocatedbl(ND, ND);
	tempA = matrixallocatedbl(ND, ND);
	tempC = matrixallocatedbl(P, ND);
	mateq(prodA, AMPC, ND, ND);
	rowKCAB = matrixallocatedbl(P, M * HP);
	rowcolKCAB = matrixallocatedbl(P, ND);
	tempKCAB = matrixallocatedbl(P, ND);
	rowcolKCABB = matrixallocatedbl(P, M);
	for (nn = 0; nn < HP; nn++) {
		//KCA
		if (nn > 0) {
			matmult(tempA, AMPC, ND, ND, prodA, ND, ND);
			mateq(prodA, tempA, ND, ND);
		}
		matmult(tempC, CD, P, ND, prodA, ND, ND);
		for (ii = 0; ii < P; ii++) {
			for (jj = 0; jj < ND; jj++) {
				KCA[P * nn + ii][jj] = tempC[ii][jj];
			}
		}
		//KCAB
		for (ii = 0; ii <= nn; ii++) {
			mateq(rowcolKCAB, CD, P, ND);
			for (jj = 0; jj <= nn - ii - 1; jj++) {
				matmult(tempKCAB, rowcolKCAB, P, ND, AMPC, ND, ND);
				mateq(rowcolKCAB, tempKCAB, P, ND);
			}
			matmult(rowcolKCABB, rowcolKCAB, P, ND, BMPC, ND, M);
			for (ll = 0; ll < P; ll++) {
				for (kk = 0; kk < M; kk++) {
					rowKCAB[ll][P * ii + kk] = rowcolKCABB[ll][kk];
				}
			}
		}
		for (ii = 0; ii < P; ii++) {
			for (jj = 0; jj < M * HP; jj++) {
				KCAB[P * nn + ii][jj] = rowKCAB[ii][jj];
			}
		}
	}
	//Compute K = inv(KCAB'*Q*KCAB+R)*KCAB'*Q;
	KCABT = transpose(KCAB, P * HP, M * HP);
	QKCAB = matrixallocatedbl(P * HP, M * HP);
	matmult(QKCAB, QD, P * HP, P * HP, KCAB, P * HP, M * HP);
	KCABTQKCAB = matrixallocatedbl(M * HP, M * HP);
	matmult(KCABTQKCAB, KCABT, M * HP, P * HP, QKCAB, P * HP, M * HP);
	KCABTQKCABR = matrixallocatedbl(M * HP, M * HP);
	matplusminus(KCABTQKCABR, KCABTQKCAB, RD, 1, M * HP, M * HP);
	double **INSIDE, **INVERSE;
	INSIDE = dmatrix(1, M * HP, 1, M * HP);
	INVERSE = dmatrix(1, M * HP, 1, M * HP);

	for (ii = 0; ii < M * HP; ii++) {
		for (jj = 0; jj < M * HP; jj++) {
			INSIDE[ii + 1][jj + 1] = KCABTQKCABR[ii][jj];
		}
	}

	matrix_inverse(INSIDE, INVERSE, M * HP);

	for (ii = 0; ii < M * HP; ii++) {
		for (jj = 0; jj < M * HP; jj++) {
			KCABTQKCABR[ii][jj] = INVERSE[ii + 1][jj + 1];
		}
	}

	//Compute K = invKCABTQKCABR*KCAB'*Q;
	double **KCABTQ;
	KCABTQ = matrixallocatedbl(M * HP, P * HP);
	matmult(KCABTQ, KCABT, M * HP, P * HP, QD, P * HP, P * HP);
	KMPC = matrixallocatedbl(M * HP, P * HP);
	matmult(KMPC, KCABTQKCABR, M * HP, M * HP, KCABTQ, M * HP, P * HP);

	//matrixdisp(KMPC,MMPC*HP,PMPC*HP,"K");

	printf("MPC Matrices Computed\n");

	free_dmatrix(INSIDE, 1, M * HP, 1, M * HP);
	free_dmatrix(INVERSE, 1, M * HP, 1, M * HP);

}

double dphix(double S, double T, double W, double R, double Mx0, double My0,
		double Mz0) {
	double delFx;

	delFx = Mx0 * (T - R) / (R * (pow(S, 2) + pow(W, 2))) - My0 / R
			+ Mz0 * T * W / (R * (pow(S, 2) + pow(W, 2)));
	return delFx;

}

double dphiy(double S, double T, double W, double R, double Mx0, double My0,
		double Mz0) {
	double delFy;

	delFy = -Mx0 / R + My0 * T * (S - R) / (R * (pow(T, 2) + pow(W, 2)))
			+ Mz0 * S * W / (R * (pow(T, 2) + pow(W, 2)));
	return delFy;

}

double dphiz(double S, double T, double W, double R, double Mx0, double My0,
		double Mz0) {
	double delFz;

	delFz = Mx0 * W * (T - R) / (R * (pow(T, 2) + pow(W, 2)))
			+ My0 * W * (S - R) / (R * (pow(T, 2) + pow(W, 2)))
			- Mz0 * S * T * (pow(R, 2) + pow(W, 2))
					/ (R * (pow(S, 2) + pow(W, 2)) * (pow(T, 2) + pow(W, 2)));
	return delFz;

}

double MagForce(double F_Magnet[3], double T_Magnet[3], double xM, double yM,
		double zM, int Type) {
	int ii, jj, kk, ll, pp, qq, uu, vv, ww, body;
	double Uij[2][2], Vkl[2][2], Wpq[2][2], a, a1, b, b1, c, c1, int1, int2,
			int3;
	double Fz, mu0, U, V, W, one, R, dFx, dFy, dFz, xI, yI, zI, Fx, Fy, Mx, My,
			Mz, d1, d2, d3;

	//Size of magnet
	a = msize[0] / 2;
	b = msize[1] / 2;
	c = msize[2] / 2;
	a1 = msize[0] / 2;
	b1 = msize[1] / 2;
	c1 = msize[2] / 2;

	//////ANALYTIC SOLUTION//////
	if (Type == 1) {
		//%%Compute U,V,W
		for (ii = 0; ii < 2; ii++) {
			for (jj = 0; jj < 2; jj++) {
				Uij[ii][jj] = xM - a * pow(-1, ii) + a1 * pow(-1, jj);
				Vkl[ii][jj] = yM - b * pow(-1, ii) + b1 * pow(-1, jj);
				Wpq[ii][jj] = zM - c * pow(-1, ii) + c1 * pow(-1, jj);
			}
		}

		//Compute Magnet Force
		Fx = 0;
		Fy = 0;
		Fz = 0;
		Mx = 0;
		My = 0;
		Mz = 0;
		for (ii = 0; ii < 2; ii++) {
			for (jj = 0; jj < 2; jj++) {
				for (kk = 0; kk < 2; kk++) {
					for (ll = 0; ll < 2; ll++) {
						for (pp = 0; pp < 2; pp++) {
							for (qq = 0; qq < 2; qq++) {
								U = Uij[ii][jj];
								V = Vkl[kk][ll];
								W = Wpq[pp][qq];
								R = sqrt(pow(U, 2) + pow(V, 2) + pow(W, 2));
								one = pow(-1,
										(double) (ii + jj + kk + ll + pp + qq));
								dFx = one
										* (0.5 * (pow(V, 2) - pow(W, 2))
												* log(R - U)
												+ U * V * log(R - V)
												+ V * W * atan2(U * V, R * W)
												+ 0.5 * R * U);
								dFy = one
										* (0.5 * (pow(U, 2) - pow(W, 2))
												* log(R - V)
												+ U * V * log(R - U)
												+ U * W * atan2(U * V, R * W)
												+ 0.5 * R * V);
								dFz = one
										* (-U * W * log(R - U)
												- V * W * log(R - V)
												+ U * V * atan2(U * V, R * W)
												- R * W);
								Fx = Fx + dFx;
								Fy = Fy + dFy;
								Fz = Fz + dFz;
								d1 = -U / 2 + a1 * pow(-1, (double) jj);
								d2 = -V / 2 + b1 * pow(-1, (double) ll);
								d3 = -W / 2 + c1 * pow(-1, (double) qq);
								Mx = Mx - d3 * dFy + d2 * dFz;
								My = My + d3 * dFx - d1 * dFz;
								Mz = Mz - d2 * dFx + d1 * dFy;
							}
						}
					}
				}
			}
		}
	}
	if (Type == 2) {
		double du, dv, dw, duvw, S, T, delFx, delFy, delFz;
		du = (2 * a) / (double) Nx;
		dv = (2 * b) / (double) Ny;
		dw = (2 * c) / (double) Nz;
		duvw = du * dv * dw;
		Fx = 0;
		Fy = 0;
		Fz = 0;
		//Numerical solution  !!Sum dF over entire volume of magnet2
		for (uu = 1; uu < Nx; uu++) {
			for (vv = 1; vv < Ny; vv++) {
				for (ww = 0; ww < Nz + 1; ww++) {
					//!compute dF as a function of Mag field generated by magnet0
					dFx = 0;
					dFy = 0;
					dFz = 0;
					for (ii = 0; ii < 2; ii++) {
						for (jj = 0; jj < 2; jj++) {
							for (kk = 0; kk < 2; kk++) {
								S = -a + xM + du * uu - a * pow(-1, ii);
								T = -b + yM + dv * vv - b * pow(-1, jj);
								W = -c + zM + dw * ww - c * pow(-1, kk);
								R = sqrt(pow(S, 2) + pow(T, 2) + pow(W, 2));
								one = pow(-1, (double) (ii + jj + kk));
								delFx = dphix(S, T, W, R, 0, 0, J);
								delFy = dphiy(S, T, W, R, 0, 0, J);
								delFz = dphiz(S, T, W, R, 0, 0, J);
								dFx = dFx + one * delFx;
								dFy = dFy + one * delFy;
								dFz = dFz + one * delFz;
							}
						}
					}
					//Add to F
					Fx = Fx + dFx * duvw;
					Fy = Fy + dFy * duvw;
					Fz = Fz + dFz * duvw;
				}
			}
		}
		//%%Compute U,V,W
		for (ii = 0; ii < 2; ii++) {
			for (jj = 0; jj < 2; jj++) {
				Uij[ii][jj] = xM - a * pow(-1, ii) + a1 * pow(-1, jj);
				Vkl[ii][jj] = yM - b * pow(-1, ii) + b1 * pow(-1, jj);
				Wpq[ii][jj] = zM - c * pow(-1, ii) + c1 * pow(-1, jj);
			}
		}

		//Compute Torque
		Mx = 0;
		My = 0;
		Mz = 0;
		for (ii = 0; ii < 2; ii++) {
			for (jj = 0; jj < 2; jj++) {
				for (kk = 0; kk < 2; kk++) {
					for (ll = 0; ll < 2; ll++) {
						for (pp = 0; pp < 2; pp++) {
							for (qq = 0; qq < 2; qq++) {
								U = Uij[ii][jj];
								V = Vkl[kk][ll];
								W = Wpq[pp][qq];
								R = sqrt(pow(U, 2) + pow(V, 2) + pow(W, 2));
								one = pow(-1,
										(double) (ii + jj + kk + ll + pp + qq));
								dFx = one
										* (0.5 * (pow(V, 2) - pow(W, 2))
												* log(R - U)
												+ U * V * log(R - V)
												+ V * W * atan2(U * V, R * W)
												+ 0.5 * R * U);
								dFy = one
										* (0.5 * (pow(U, 2) - pow(W, 2))
												* log(R - V)
												+ U * V * log(R - U)
												+ U * W * atan2(U * V, R * W)
												+ 0.5 * R * V);
								dFz = one
										* (-U * W * log(R - U)
												- V * W * log(R - V)
												+ U * V * atan2(U * V, R * W)
												- R * W);
								d1 = -U / 2 + a1 * pow(-1, (double) jj);
								d2 = -V / 2 + b1 * pow(-1, (double) ll);
								d3 = -W / 2 + c1 * pow(-1, (double) qq);
								Mx = Mx - d3 * dFy + d2 * dFz;
								My = My + d3 * dFx - d1 * dFz;
								Mz = Mz - d2 * dFx + d1 * dFy;
							}
						}
					}
				}
			}
		}
	}
	F_Magnet[0] = Fx;
	F_Magnet[1] = Fy;
	F_Magnet[2] = Fz;
	T_Magnet[0] = Mx;
	T_Magnet[1] = My;
	T_Magnet[2] = Mz;
	mu0 = 4 * PI / pow(10, 7); ////permeability of free space in T*m/A
	F_Magnet[0] = (J * J / (4 * PI * mu0)) * F_Magnet[0];
	F_Magnet[1] = (J * J / (4 * PI * mu0)) * F_Magnet[1];
	F_Magnet[2] = (J * J / (4 * PI * mu0)) * F_Magnet[2];
	T_Magnet[0] = (J * J / (4 * PI * mu0)) * T_Magnet[0];
	T_Magnet[1] = (J * J / (4 * PI * mu0)) * T_Magnet[1];
	T_Magnet[2] = (J * J / (4 * PI * mu0)) * T_Magnet[2];
	return (fabs(F_Magnet[2]));
}

void ComputeMagnetForce() {
	int Analytic, ii, jj, kk, ll, pp, qq, uu, vv, ww, body;
	double Uij[2][2], Vkl[2][2], Wpq[2][2], a, a1, bm, b1, c, c1, int1, int2,
			int3;
	double Fx, Fy, Fz, mu0, U, V, W, one, R, dFx, dFy, dFz, xI, yI, zI,
			F_M1_M[3];
	double x[NBODIES], y[NBODIES], z[NBODIES], phi[NBODIES], theta[NBODIES],
			psi[NBODIES];
	double u[NBODIES], v[NBODIES], w[NBODIES], p[NBODIES], q[NBODIES],
			r[NBODIES], phiF, thetaF, xB, yB, zB;
	double VcgAtm[3], Vel, rho, SOS, Mach, AngVel[3];
	double F_R[3], F_P[3], F_W[NBODIES][3], F_T[NBODIES][3], M_R[3], M_P[3];
	double xdot, ydot, zdot, phidot, thetadot, psidot, udot, vdot, wdot, pdot,
			qdot, rdot;
	double BodyAeroLoads[6], Normal[3], rcg1_magnetI[3], rcg_magnetI[3];
	double M_T[NBODIES][3], ttheta, psiint, ptpdot[3], xydot[2], zint, xint,
			yint;
	/* Contact Analysis Variables */
	double s1n[3][NVERTS], s1t[3][NVERTS], s2n[3][NVERTS], s2t[3][NVERTS];
	double NormalB[3], rplane_vertex[3][NVERTS], rcg_vertex_I[3], rInter[3],
			rcg1_vertex_I[3];
	double delun[3][NVERTS], delut[3][NVERTS], delwt[3][NVERTS];
	double uB[3], uC[3], u1B[3], u1C[3], delu[3];
	double deluDotNormal, norm2Normal, c_star, commands[3];
	double bn[3], bt[3], norm_bn, norm_bt, lambda;
	double F_CI[NBODIES][3], M_CI[NBODIES][3], Fn[3], Ft[3], M_Cf[3], M_Cf1[3];
	double CoeffDamper1[4], CoeffDamper2[4];
	double ctheta, stheta, spsi, cpsi, sphi, cphi, M_M[NBODIES][3], T_M1_I[3],
			T_M1_B1[3];
	double position[3], xyzdot[3], T_M1_M[3], T_M_I[3];
	double F_M_I[3], F_M1_I[3], Fmax, dist, F0, imag, d1, d2, d3, Mx, My, Mz,
			T_M1_B[3];
	double S, T, delFx, delFy, delFz, du, dv, dw, duvw, dctheta, dstheta, dsphi,
			dcphi, dspsi;
	double phim0, thetam0, psim0, phim1, thetam1, psim1, dphi, dtheta, dpsi,
			T01[3][3];
	double dcpsi, dttheta, Mx0, My0, Mz0, xp, yp, zp, crash, F_mag, d, xM, zM,
			yM;

	/*Calculate distance between magnets */
	rcg_magnetI[0] = TIB[0][0][0] * rcg_magnet[0] + TIB[0][0][1] * rcg_magnet[1]
			+ TIB[0][0][2] * rcg_magnet[2];
	rcg_magnetI[1] = TIB[0][1][0] * rcg_magnet[0] + TIB[0][1][1] * rcg_magnet[1]
			+ TIB[0][1][2] * rcg_magnet[2];
	rcg_magnetI[2] = TIB[0][2][0] * rcg_magnet[0] + TIB[0][2][1] * rcg_magnet[1]
			+ TIB[0][2][2] * rcg_magnet[2];
	rcg1_magnetI[0] = TIB[1][0][0] * rcg1_magnet[0]
			+ TIB[1][0][1] * rcg1_magnet[1] + TIB[1][0][2] * rcg1_magnet[2];
	rcg1_magnetI[1] = TIB[1][1][0] * rcg1_magnet[0]
			+ TIB[1][1][1] * rcg1_magnet[1] + TIB[1][1][2] * rcg1_magnet[2];
	rcg1_magnetI[2] = TIB[1][2][0] * rcg1_magnet[0]
			+ TIB[1][2][1] * rcg1_magnet[1] + TIB[1][2][2] * rcg1_magnet[2];
	xI = State[NSTATE + 0] + rcg1_magnetI[0] - State[0] - rcg_magnetI[0];
	yI = State[NSTATE + 1] + rcg1_magnetI[1] - State[1] - rcg_magnetI[1];
	zI = State[NSTATE + 2] + rcg1_magnetI[2] - State[2] - rcg_magnetI[2];
	//Compute Absolute Distance
	dist = sqrt(pow(xI, 2) + pow(yI, 2) + pow(zI, 2));
	MAGDISTANCE = dist;
	//Check to see if we need to run magnet solution
	imag = IMAGNET;
	if (dist > 0.5) {
		imag = 0;
	}
	// printf("dist %lf \n",dist);
	// PAUSE();
	if (imag) {
		//Rotate Inertial vectors to magnet frame
		//First rotate inertial frame to body0 frame
		xB = TBI[0][0][0] * xI + TBI[0][0][1] * yI + TBI[0][0][2] * zI;
		yB = TBI[0][1][0] * xI + TBI[0][1][1] * yI + TBI[0][1][2] * zI;
		zB = TBI[0][2][0] * xI + TBI[0][2][1] * yI + TBI[0][2][2] * zI;
		//assume that z-mag = y-plane
		//x-plane = x-mag and y-mag = -z-plane
		xM = xB;
		yM = -zB;
		zM = yB; //this is a vector from magnet 0 to magnet 1 in magnet0 frame
		if (zM <= msize[2] + 0.0001) {
			zM = msize[2] + 0.0001;
		}
		//Connection Check
		// if (TCURRENT > 60)
		// 	{
		// 	  printf("GOODCONTACT = %d \n",GOODCONTACT);
		// 	}
		d = msize[2] * 1.1;
		if ((numContacts > 0) && (GOODCONTACT == 0)) {
			if ((fabs(xM) < 0.1) && (fabs(yM) < 0.1)) {
				if (numContacts > 0) {
					GOODCONTACT = 1;
				}
			}
		}

		//Compute Magnet Force
		double F = MagForce(F_M1_M, T_M1_M, xM, yM, zM, imag);
		//Check to make sure that magnitude of force is less than max force
		F_mag = sqrt(pow(F_M1_M[0], 2) + pow(F_M1_M[1], 2) + pow(F_M1_M[2], 2));
		//printf("Fmag = %lf \n",F_mag);
		if (F_mag > Fconn) {
			F_M1_M[0] = Fconn * F_M1_M[0] / F_mag;
			F_M1_M[1] = Fconn * F_M1_M[1] / F_mag;
			F_M1_M[2] = Fconn * F_M1_M[2] / F_mag;
		}
		if (F_mag > MAXF) {
			MAXF = F_mag;
		}
		if (MAGCONSTANT == 1) {
			F_M1_M[2] = -Fconn;
		}

		//Now Rotate Forces to Body Frame
		//Note this equation computes the force on magnet1 in the magnet0 frame
		F_M1_B[0] = F_M1_M[0];
		F_M1_B[1] = F_M1_M[2]; //rotate from mag to plane0
		F_M1_B[2] = -F_M1_M[1];
		//Torques
		T_M1_B[0] = 0 * T_M1_M[0];
		T_M1_B[1] = 0 * T_M1_M[2]; //rotate from mag to plane0
		T_M1_B[2] = 0 * T_M1_M[1];

		//Now rotate body forces to inertial
		F_M1_I[0] = TIB[0][0][0] * F_M1_B[0] + TIB[0][0][1] * F_M1_B[1]
				+ TIB[0][0][2] * F_M1_B[2];
		F_M1_I[1] = TIB[0][1][0] * F_M1_B[0] + TIB[0][1][1] * F_M1_B[1]
				+ TIB[0][1][2] * F_M1_B[2];
		F_M1_I[2] = TIB[0][2][0] * F_M1_B[0] + TIB[0][2][1] * F_M1_B[1]
				+ TIB[0][2][2] * F_M1_B[2];

		T_M1_I[0] = (TIB[0][0][0] * T_M1_B[0] + TIB[0][0][1] * T_M1_B[1]
				+ TIB[0][0][2] * T_M1_B[2]);
		T_M1_I[1] = (TIB[0][1][0] * T_M1_B[0] + TIB[0][1][1] * T_M1_B[1]
				+ TIB[0][1][2] * T_M1_B[2]);
		T_M1_I[2] = (TIB[0][2][0] * T_M1_B[0] + TIB[0][2][1] * T_M1_B[1]
				+ TIB[0][2][2] * T_M1_B[2]);

		//Now copy force to body 0 in inertial frame
		F_M_I[0] = -F_M1_I[0];
		F_M_I[1] = -F_M1_I[1];
		F_M_I[2] = -F_M1_I[2];

		T_M_I[0] = T_M1_I[0];
		T_M_I[1] = T_M1_I[1];
		T_M_I[2] = T_M1_I[2];

		/* Rotate Forces to body 1 frame */
		F_M_B[1][0] = TBI[1][0][0] * F_M1_I[0] + TBI[1][0][1] * F_M1_I[1]
				+ TBI[1][0][2] * F_M1_I[2];
		F_M_B[1][1] = TBI[1][1][0] * F_M1_I[0] + TBI[1][1][1] * F_M1_I[1]
				+ TBI[1][1][2] * F_M1_I[2];
		F_M_B[1][2] = TBI[1][2][0] * F_M1_I[0] + TBI[1][2][1] * F_M1_I[1]
				+ TBI[1][2][2] * F_M1_I[2];

		// printf("F_M_B[1] %lf %lf %lf \n",F_M_B[1][0],F_M_B[1][1],F_M_B[1][2]);
		// PAUSE();

		// F_M_B[1][0] = -F_M_B[0][0];
		// F_M_B[1][1] = -F_M_B[0][1];
		// F_M_B[1][2] = -F_M_B[0][2];

		// T_M_B[1][0] = -T_M_B[0][0];
		// T_M_B[1][1] = -T_M_B[0][1];
		// T_M_B[1][2] = -T_M_B[0][2];

		T_M_B[1][0] = (TBI[1][0][0] * T_M1_I[0] + TBI[1][0][1] * T_M1_I[1]
				+ TBI[1][0][2] * T_M1_I[2]);
		T_M_B[1][1] = (TBI[1][1][0] * T_M1_I[0] + TBI[1][1][1] * T_M1_I[1]
				+ TBI[1][1][2] * T_M1_I[2]);
		T_M_B[1][2] = (TBI[1][2][0] * T_M1_I[0] + TBI[1][2][1] * T_M1_I[1]
				+ TBI[1][2][2] * T_M1_I[2]);

		/* Rotate Forces to body 0 frame */
		F_M_B[0][0] = (TBI[0][0][0] * F_M_I[0] + TBI[0][0][1] * F_M_I[1]
				+ TBI[0][0][2] * F_M_I[2]);
		F_M_B[0][1] = (TBI[0][1][0] * F_M_I[0] + TBI[0][1][1] * F_M_I[1]
				+ TBI[0][1][2] * F_M_I[2]);
		F_M_B[0][2] = (TBI[0][2][0] * F_M_I[0] + TBI[0][2][1] * F_M_I[1]
				+ TBI[0][2][2] * F_M_I[2]);

		T_M_B[0][0] = (TBI[0][0][0] * T_M_I[0] + TBI[0][0][1] * T_M_I[1]
				+ TBI[0][0][2] * T_M_I[2]);
		T_M_B[0][1] = (TBI[0][1][0] * T_M_I[0] + TBI[0][1][1] * T_M_I[1]
				+ TBI[0][1][2] * T_M_I[2]);
		T_M_B[0][2] = (TBI[0][2][0] * T_M_I[0] + TBI[0][2][1] * T_M_I[1]
				+ TBI[0][2][2] * T_M_I[2]);

		/* Compute Moments from F_MB */ //M_M = rcg_magnet cross F_MB
		M_M[0][0] = -rcg_magnet[2] * F_M_B[0][1] + rcg_magnet[1] * F_M_B[0][2];
		M_M[0][1] = rcg_magnet[2] * F_M_B[0][0] - rcg_magnet[0] * F_M_B[0][2];
		M_M[0][2] = -rcg_magnet[1] * F_M_B[0][0] + rcg_magnet[0] * F_M_B[0][1];
		//Moment on aircraft 1
		M_M[1][0] = -rcg1_magnet[2] * F_M_B[1][1]
				+ rcg1_magnet[1] * F_M_B[1][2];
		M_M[1][1] = rcg1_magnet[2] * F_M_B[1][0] - rcg1_magnet[0] * F_M_B[1][2];
		M_M[1][2] = -rcg1_magnet[1] * F_M_B[1][0]
				+ rcg1_magnet[0] * F_M_B[1][1];
		//Zero out for debugging
		if (MAGCONSTANT) {
			// M_M[0][0] = 0;
			// M_M[0][1] = 0;
			// M_M[0][2] = 0;
			// M_M[0][0] = 0;
			// M_M[1][1] = 0;
			// M_M[2][2] = 0;
			// T_M_B[0][0] = 0;
			// T_M_B[0][1] = 0;
			// T_M_B[0][2] = 0;
			// T_M_B[1][0] = 0;
			// T_M_B[1][1] = 0;
			// T_M_B[1][2] = 0;
		}

		if (F_M_B[0][1] < 0) {
			//printf("Repulsive Force \n");
			// F_M_B[0][1] = 0;
			// F_M_B[1][1] = 0;
		}
	}
}

void Horizon(double ptp[3]) {
	int ii;
	double f;

	f = (double) HNOISETYPE;

	if (HNOISETYPE) // %%White Noise(Uniform)
	{
		for (ii = 0; ii < 3; ii++) {
			ptp[ii] = ptp[ii] - f * sigmah[ii]
					+ 2 * f * sigmah[ii] * randNormal() + f * biash[ii];
		}
	}
}

void ACCEL(double uvwdot[3]) {
	int ii, jj;
	double mid[3], f;
	//%%%This function will add in ACCEL errors
	//%%%%%%%ACCEL NOISE%%%%%%%%

	f = (double) ACCELNOISETYPE;

	if (ACCELNOISETYPE) // %%All errors
	{
		for (ii = 0; ii < 3; ii++) {
			accelnoise[ii] = randNormal() * f * accelsigmanoise[ii];
			uvwdot[ii] = uvwdot[ii] + f * accelbias[ii] + accelnoise[ii];
		}
		for (ii = 0; ii < 3; ii++) {
			mid[ii] = 0;
			for (jj = 0; jj < 3; jj++) {
				mid[ii] = mid[ii] + Tgyroaccel[ii][jj] * uvwdot[jj];
			}
		}
		for (ii = 0; ii < 3; ii++) {
			uvwdot[ii] = 0;
			for (jj = 0; jj < 3; jj++) {
				uvwdot[ii] = uvwdot[ii] + Orthogonalityaccel[ii][jj] * mid[jj];
			}
		}
	}

}

void PQR(double pqr[3]) {
	int ii, jj;
	double mid[3], f;
	//%%%This function will add in PQR errors
	//%%%%%%%PQR NOISE%%%%%%%%

	f = (double) PQRNOISETYPE;

	//printf("PQR = %f %f %f \n",pqr[0],pqr[1],pqr[2]);

	if (PQRNOISETYPE) // %%All errors
	{
		for (ii = 0; ii < 3; ii++) {
			pqrnoise[ii] = randNormal() * f * sigmanoise[ii];
			pqr[ii] = pqr[ii] + f * pqrbias[ii] + pqrnoise[ii];
		}
		for (ii = 0; ii < 3; ii++) {
			mid[ii] = 0;
			for (jj = 0; jj < 3; jj++) {
				mid[ii] = mid[ii] + Tgyro[ii][jj] * pqr[jj];
			}
		}
		for (ii = 0; ii < 3; ii++) {
			pqr[ii] = 0;
			for (jj = 0; jj < 3; jj++) {
				pqr[ii] = pqr[ii] + Orthogonality[ii][jj] * mid[jj];
			}
		}
	}
	//printf("PQR = %f %f %f \n",pqr[0],pqr[1],pqr[2]);
}

void GPS(double xyz[3]) {
	int ii;
	double f;
	//%%%This function will add in GPS errors

	f = (double) GPSNOISETYPE;

	//%%%%%%%XYZ NOISE%%%%%%%%
	if (GPSNOISETYPE) // %%Random walk
	{
		//%%Position Noise
		for (ii = 0; ii < 3; ii++) {
			xyz[ii] = xyz[ii] + xbias[ii] + xnoise[ii];
			N01[ii] = randNormal();
			xnoise[ii] = N01[ii] * f * posNoise[ii];
			xbias[ii] = xbias[ii] * epsilonP
					+ f * posBias[ii] * sqrt(1 - pow(epsilonP, 2)) * N01[ii];
		}
	}
}

void GPSVEL(double xyzdot[3]) {
	int ii;
	double f;
	//%%%This function will add in GPS errors

	f = (double) GPSNOISETYPE;

	//%%%%%%%XYZ NOISE%%%%%%%%
	if (GPSNOISETYPE) // %%Random walk
	{
		//%%Velocity Noise
		for (ii = 0; ii < 3; ii++) {
			xyzdot[ii] = xyzdot[ii] + f * xdotbias[ii] + xdotnoise[ii];
			N01[ii] = randNormal();
			xdotnoise[ii] = N01[ii] * f * velNoise[ii];
		}
	}
}

void SensorErrors() {
	int ii, jj, j, k;
	double xyz[3], ptp[3], pqr[3], p, q, r, sptp[3], norm, noiselevel, slope;
	double xbar[3], ybar[3], zbar[3], xhat[3], yhat[3], zhat[3], xn, yn, zn,
			R[3][3];
	double phi, theta, psi, sinpsi, cospsi, sinphi, cosphi, x, y, z, xyzdot[3],
			ptpdot[3];
	double sphi, ctheta, cphi, stheta, spsi, cpsi, ttheta, TBInoise[3][3], s;
	double summ, rkalfa[4], krkbody[3][4], uvwdot[3], uvw[3];

	if (USESENSORS) {
		/* Define the Runge-Kutta Constants */
		rkalfa[0] = 1.00000000;
		rkalfa[1] = 2.00000000;
		rkalfa[2] = 2.00000000;
		rkalfa[3] = 1.00000000;

		for (ii = 0; ii < N; ii++) {
			StateE[ii] = State[ii];
			StateEDot[ii] = StateDot[ii];
		}
		if (GPSnext >= 1 / (TIMESTEP * GPSupdate)) {
			GPSnext = 0;
		}
		GPSnext++;
		if (GYROnext >= 1 / (TIMESTEP * GYROupdate)) {
			GYROnext = 0;
		}
		GYROnext++;
		if (ACCELnext >= 1 / (TIMESTEP * ACCELupdate)) {
			ACCELnext = 0;
		}
		ACCELnext++;
		for (ii = 0; ii < NBODIES; ii++) {
			xyz[0] = State[ii * NSTATE];
			xyz[1] = State[ii * NSTATE + 1];
			xyz[2] = State[ii * NSTATE + 2];
			ptp[0] = State[ii * NSTATE + 3];
			ptp[1] = State[ii * NSTATE + 4];
			ptp[2] = State[ii * NSTATE + 5];
			pqr[0] = State[ii * NSTATE + 9];
			pqr[1] = State[ii * NSTATE + 10];
			pqr[2] = State[ii * NSTATE + 11];
			xyzdot[0] = StateDot[ii * NSTATE];
			xyzdot[1] = StateDot[ii * NSTATE + 1];
			xyzdot[2] = StateDot[ii * NSTATE + 2];
			uvwdot[0] = StateDot[ii * NSTATE + 6];
			uvwdot[1] = StateDot[ii * NSTATE + 7];
			uvwdot[2] = StateDot[ii * NSTATE + 8];
			if (ACCELnext == 1) {
				//get new accelerometer measurement
				ACCEL(uvwdot);
				//filter signal
				uvwdot[0] = 0.5 * ACCELUVWDOT[0][ii] + 0.5 * uvwdot[0];
				uvwdot[1] = 0.5 * ACCELUVWDOT[1][ii] + 0.5 * uvwdot[1];
				uvwdot[2] = 0.5 * ACCELUVWDOT[2][ii] + 0.5 * uvwdot[2];
				ACCELUVWDOT[0][ii] = uvwdot[0];
				ACCELUVWDOT[1][ii] = uvwdot[1];
				ACCELUVWDOT[2][ii] = uvwdot[2];
			} else {
				//use previous ACCEL measurement
				uvwdot[0] = ACCELUVWDOT[0][ii];
				uvwdot[1] = ACCELUVWDOT[1][ii];
				uvwdot[2] = ACCELUVWDOT[2][ii];
			}
			if (GYROnext == 1) {
				//Get new Rate GYRO update
				PQR(pqr);
				GYROPQR[0][ii] = pqr[0];
				GYROPQR[1][ii] = pqr[1];
				GYROPQR[2][ii] = pqr[2];
				//Add HORIZON Sensor Errors(Assume update rate of HORIZON sensor is the same as RATEGYRO)
				Horizon(ptp);
				//Let the current measurement be = A*HPTP+ptp*B
				ptp[0] = 0.9 * HPTP[0][ii] + 0.1 * ptp[0];
				ptp[1] = 0.9 * HPTP[1][ii] + 0.1 * ptp[1];
				ptp[2] = 0.9 * HPTP[2][ii] + 0.1 * ptp[2];
				HPTP[0][ii] = ptp[0];
				HPTP[1][ii] = ptp[1];
				HPTP[2][ii] = ptp[2];
				phi = ptp[0];
				theta = ptp[1];
				psi = ptp[2];
				p = pqr[0];
				q = pqr[1];
				r = pqr[2];
				ptpdot[0] = p + sin(phi) * tan(theta) * q
						+ cos(phi) * tan(theta) * r;
				ptpdot[1] = cos(phi) * q - sin(phi) * r;
				ptpdot[2] = (sin(phi) / cos(theta)) * q
						+ (cos(phi) / cos(theta)) * r;
			} else {
				//Use previous gyro measurement
				pqr[0] = GYROPQR[0][ii];
				pqr[1] = GYROPQR[1][ii];
				pqr[2] = GYROPQR[2][ii];
				//And previous PTP measurement
				ptp[0] = HPTP[0][ii];
				ptp[1] = HPTP[1][ii];
				ptp[2] = HPTP[2][ii];
				phi = ptp[0];
				theta = ptp[1];
				psi = ptp[2];
				p = pqr[0];
				q = pqr[1];
				r = pqr[2];
				//     /* Compute Derivatives at Current Value */
				ptpdot[0] = p + sin(phi) * tan(theta) * q
						+ cos(phi) * tan(theta) * r;
				ptpdot[1] = cos(phi) * q - sin(phi) * r;
				ptpdot[2] = (sin(phi) / cos(theta)) * q
						+ (cos(phi) / cos(theta)) * r;
			}
			//Save PTP measurement
			phi = ptp[0];
			theta = ptp[1];
			psi = ptp[2];
			StateE[ii * NSTATE + 3] = phi;
			StateE[ii * NSTATE + 4] = theta;
			StateE[ii * NSTATE + 5] = psi;
			//Contruct TIB from ptp
			ctheta = cos(theta);
			stheta = sin(theta);
			ttheta = stheta / ctheta;
			cphi = cos(phi);
			sphi = sin(phi);
			spsi = sin(psi);
			cpsi = cos(psi);
			TBInoise[0][0] = ctheta * cpsi;
			TBInoise[0][1] = ctheta * spsi;
			TBInoise[0][2] = -stheta;
			TBInoise[1][0] = sphi * stheta * cpsi - cphi * spsi;
			TBInoise[1][1] = sphi * stheta * spsi + cphi * cpsi;
			TBInoise[1][2] = sphi * ctheta;
			TBInoise[2][0] = cphi * stheta * cpsi + sphi * spsi;
			TBInoise[2][1] = cphi * stheta * spsi - sphi * cpsi;
			TBInoise[2][2] = cphi * ctheta;
			//Add GPS Sensor Noise
			if (GPSnext == 1) {
				//Get new GPS update
				GPS(xyz);
				GPSVEL(xyzdot);
				if (ii == 0) {
					xyz[0] = 0.99 * GPSXYZ[0][ii] + 0.01 * xyz[0];
					xyz[1] = 0.99 * GPSXYZ[1][ii] + 0.01 * xyz[1];
					xyz[2] = 0.99 * GPSXYZ[2][ii] + 0.01 * xyz[2];
					xyzdot[0] = 0.99 * GPSXYZ[3][ii] + 0.01 * xyzdot[0];
					xyzdot[1] = 0.99 * GPSXYZ[4][ii] + 0.01 * xyzdot[1];
					xyzdot[2] = 0.99 * GPSXYZ[5][ii] + 0.01 * xyzdot[2];
				} else {
					xyz[0] = 0.9 * GPSXYZ[0][ii] + 0.1 * xyz[0];
					xyz[1] = 0.9 * GPSXYZ[1][ii] + 0.1 * xyz[1];
					xyz[2] = 0.9 * GPSXYZ[2][ii] + 0.1 * xyz[2];
					xyzdot[0] = 0.9 * GPSXYZ[3][ii] + 0.1 * xyzdot[0];
					xyzdot[1] = 0.9 * GPSXYZ[4][ii] + 0.1 * xyzdot[1];
					xyzdot[2] = 0.9 * GPSXYZ[5][ii] + 0.1 * xyzdot[2];
				}
				GPSXYZ[0][ii] = xyz[0];
				GPSXYZ[1][ii] = xyz[1];
				GPSXYZ[2][ii] = xyz[2];
				GPSXYZ[3][ii] = xyzdot[0];
				GPSXYZ[4][ii] = xyzdot[1];
				GPSXYZ[5][ii] = xyzdot[2];
				//Get u,v,w from xyzdot
				uvw[0] = TBInoise[0][0] * xyzdot[0] + TBInoise[0][1] * xyzdot[1]
						+ TBInoise[0][2] * xyzdot[2];
				uvw[1] = TBInoise[1][0] * xyzdot[0] + TBInoise[1][1] * xyzdot[1]
						+ TBInoise[1][2] * xyzdot[2];
				uvw[2] = TBInoise[2][0] * xyzdot[0] + TBInoise[2][1] * xyzdot[1]
						+ TBInoise[2][2] * xyzdot[2];
				GPSUVW[0][ii] = uvw[0];
				GPSUVW[1][ii] = uvw[1];
				GPSUVW[2][ii] = uvw[2];
			} else {
				//Get GPS update from integration of Accelerometer
				//Accelerometer Measures uvwdot integrate once to get uvw
				uvw[0] = uvwdot[0] * TIMESTEP + GPSUVW[0][ii];
				uvw[1] = uvwdot[1] * TIMESTEP + GPSUVW[1][ii];
				uvw[2] = uvwdot[2] * TIMESTEP + GPSUVW[2][ii];
				GPSUVW[0][ii] = uvw[0];
				GPSUVW[1][ii] = uvw[1];
				GPSUVW[2][ii] = uvw[2];
				//Now rotate to inertial frame to get xyzdot(Transpose of TBI)
				xyzdot[0] = TBInoise[0][0] * uvw[0] + TBInoise[1][0] * uvw[1]
						+ TBInoise[2][0] * uvw[2];
				xyzdot[1] = TBInoise[0][1] * uvw[0] + TBInoise[1][1] * uvw[1]
						+ TBInoise[2][1] * uvw[2];
				xyzdot[2] = TBInoise[0][2] * uvw[0] + TBInoise[1][2] * uvw[1]
						+ TBInoise[2][2] * uvw[2];
				//Now integrate again to get xyz
				xyz[0] = xyzdot[0] * TIMESTEP + GPSXYZ[0][ii];
				xyz[1] = xyzdot[1] * TIMESTEP + GPSXYZ[1][ii];
				xyz[2] = xyzdot[2] * TIMESTEP + GPSXYZ[2][ii];
				//Save xyz,xyzdot
				GPSXYZ[0][ii] = xyz[0];
				GPSXYZ[1][ii] = xyz[1];
				GPSXYZ[2][ii] = xyz[2];
				GPSXYZ[3][ii] = xyzdot[0];
				GPSXYZ[4][ii] = xyzdot[1];
				GPSXYZ[5][ii] = xyzdot[2];
			}
			//Save xyz
			StateE[ii * NSTATE] = xyz[0];
			StateE[ii * NSTATE + 1] = xyz[1];
			StateE[ii * NSTATE + 2] = xyz[2];
			//Save uvw
			StateE[ii * NSTATE + 6] = uvw[0];
			StateE[ii * NSTATE + 7] = uvw[1];
			StateE[ii * NSTATE + 8] = uvw[2];
			//Save xyzdot and ptpdot in derivatives
			StateEDot[ii * NSTATE] = xyzdot[0];
			StateEDot[ii * NSTATE + 1] = xyzdot[1];
			StateEDot[ii * NSTATE + 2] = xyzdot[2];
			StateEDot[ii * NSTATE + 3] = ptpdot[0];
			StateEDot[ii * NSTATE + 4] = ptpdot[1];
			StateEDot[ii * NSTATE + 5] = ptpdot[2];
			StateE[ii * NSTATE + 9] = pqr[0];
			StateE[ii * NSTATE + 10] = pqr[1];
			StateE[ii * NSTATE + 11] = pqr[2];
			//Save zint and xint
			StateE[ii * NSTATE + 12] = State[ii * NSTATE + 12];
			StateE[ii * NSTATE + 13] = State[ii * NSTATE + 13];
			StateE[ii * NSTATE + 14] = State[ii * NSTATE + 14];
		}
		if (NBODIES > 1) {
			if (VISNAVNOISETYPE == -1) {
				VisNav[0] = StateE[0] - StateE[NSTATE + 0];
				VisNav[1] = StateE[1] - StateE[NSTATE + 1];
				VisNav[2] = StateE[2] - StateE[NSTATE + 2];
				VisNav[3] = StateEDot[0] - StateEDot[NSTATE + 0];
				VisNav[4] = StateEDot[1] - StateEDot[NSTATE + 1];
				VisNav[5] = StateEDot[2] - StateEDot[NSTATE + 2];
			} else {
				VisNav[0] = State[0] - State[NSTATE + 0];
				VisNav[1] = State[1] - State[NSTATE + 1];
				VisNav[2] = State[2] - State[NSTATE + 2];
				VisNav[3] = StateDot[0] - StateDot[NSTATE + 0];
				VisNav[4] = StateDot[1] - StateDot[NSTATE + 1];
				VisNav[5] = StateDot[2] - StateDot[NSTATE + 2];
			}
			//Sensor errors depend on distance(1 m = 1mm Noise 10 m = 1cm Noise)
			if (VISNAVNOISETYPE) {
				norm = sqrt(
						pow(VisNav[0], 2) + pow(VisNav[1], 2)
								+ pow(VisNav[2], 2));
				slope = (0.01 - 0.001) / (9);
				noiselevel = (double) VISNAVNOISETYPE
						* (slope * (norm - 1) + 0.001);
				//printf("%lf \n",noiselevel);
				if (noiselevel < 0)
					noiselevel = 0;
				VisNav[0] = VisNav[0] + (-1 + 2 * randNormal()) * noiselevel;
				VisNav[1] = VisNav[1] + (-1 + 2 * randNormal()) * noiselevel;
				VisNav[2] = VisNav[2] + (-1 + 2 * randNormal()) * noiselevel;
			}
			//Use ACCELEROMETER FOR XYZDOT
			uvwdot[0] = ACCELUVWDOT[0][1];
			uvwdot[1] = ACCELUVWDOT[1][1];
			uvwdot[2] = ACCELUVWDOT[2][1];
			uvw[0] = uvwdot[0] * TIMESTEP + ACCELUVW[0][1];
			uvw[1] = uvwdot[1] * TIMESTEP + ACCELUVW[1][1];
			uvw[2] = uvwdot[2] * TIMESTEP + ACCELUVW[2][1];
			StateEDot[NSTATE + 0] = TBInoise[0][0] * uvw[0]
					+ TBInoise[1][0] * uvw[1] + TBInoise[2][0] * uvw[2];
			StateEDot[NSTATE + 1] = TBInoise[0][1] * uvw[0]
					+ TBInoise[1][1] * uvw[1] + TBInoise[2][1] * uvw[2];
			StateEDot[NSTATE + 2] = TBInoise[0][2] * uvw[0]
					+ TBInoise[1][2] * uvw[1] + TBInoise[2][2] * uvw[2];
		}
	}
	if (USESENSORS == 0) {
		for (ii = 0; ii < N; ii++) {
			StateE[ii] = State[ii];
			StateEDot[ii] = StateDot[ii];
		}
		VisNav[0] = State[0] - State[NSTATE + 0];
		VisNav[1] = State[1] - State[NSTATE + 1];
		VisNav[2] = State[2] - State[NSTATE + 2];
		VisNav[3] = StateDot[0] - StateDot[NSTATE + 0];
		VisNav[4] = StateDot[1] - StateDot[NSTATE + 1];
		VisNav[5] = StateDot[2] - StateDot[NSTATE + 2];
	}
}

void Sensors_Setup() {
	int i, j;
	////SENSOR STUFF///////
	for (j = 0; j < NBODIES; j++) {
		for (i = 0; i < 3; i++) {
			ACCELUVW[i][j] = State[j * NSTATE + 6 + i];
			ACCELUVWDOT[i][j] = 0;
			GYROPQR[i][j] = State[j * NSTATE + 9 + i];
			HPTP[i][j] = State[j * NSTATE + 3 + i];
			GPSXYZ[i][j] = State[j * NSTATE + i];
			GPSUVW[i][j] = State[j * NSTATE + 6 + i];
		}
		GPSXYZ[3][j] = cos(HPTP[2][j]) * GPSUVW[0][j]
				- sin(HPTP[2][j]) * GPSUVW[1][j];
		GPSXYZ[4][j] = sin(HPTP[2][j]) * GPSUVW[0][j]
				+ cos(HPTP[2][j]) * GPSUVW[1][j];
		GPSXYZ[5][j] = 0;
	}
	for (i = 0; i < 3; i++) {
		sigmaturnon[i] = randUniform() * sigmaturnonIO[i];
		ScaleFactor[i] = (randUniform() * ScaleFactorIO[i] + 1);
	}
	XYcross = XYcrossIO * randUniform();
	YZcross = YZcrossIO * randUniform();
	XZcross = XZcrossIO * randUniform();
	misalign1 = randUniform() * misalign1IO;
	misalign2 = randUniform() * misalign2IO;
	misalign3 = randUniform() * misalign3IO;
	//Accelerometer
	for (i = 0; i < 3; i++) {
		accelsigmaturnon[i] = randUniform() * accelsigmaturnonIO[i];
		AccelScaleFactor[i] = (randUniform() * AccelScaleFactorIO[i] + 1);
		accelmisalign[0] = randUniform() * accelmisalignIO[0];
	}
	accelXYcross = accelXYcrossIO * randUniform();
	accelYZcross = accelYZcrossIO * randUniform();
	accelXZcross = accelXZcrossIO * randUniform();
	epsilonP = 0.999;
	epsilonV = 0.999;
	for (i = 0; i < 3; i++) {
		N01[i] = randNormal();
		xbias[i] = posBias[i];
		xnoise[i] = posNoise[i] * N01[i];
		xdotnoise[i] = -velNoise[i] + (2 * velNoise[i]) * randNormal();
		xdotbias[i] = 0;
		pqrdrift[i] = sigmadrift[i];
		pqrbias[i] = sigmaturnon[i] + pqrdrift[i];
	}
	//////////////SETUP PQR NOISE///////////////
	Orthogonality[0][0] = ScaleFactor[0];
	Orthogonality[0][1] = XYcross;
	Orthogonality[0][2] = XZcross;
	Orthogonality[1][0] = XYcross;
	Orthogonality[1][1] = ScaleFactor[1];
	Orthogonality[1][2] = YZcross;
	Orthogonality[2][0] = XZcross;
	Orthogonality[2][1] = YZcross;
	Orthogonality[2][2] = ScaleFactor[2];
	Tgyro[0][0] = cos(misalign1) * cos(misalign3);
	Tgyro[0][1] = cos(misalign1) * sin(misalign3);
	Tgyro[0][2] = -sin(misalign1);
	Tgyro[1][0] = sin(misalign2) * sin(misalign1) * cos(misalign3)
			- cos(misalign2) * sin(misalign3);
	Tgyro[1][1] = sin(misalign2) * sin(misalign1) * sin(misalign3)
			+ cos(misalign2) * cos(misalign3);
	Tgyro[1][2] = sin(misalign2) * cos(misalign1);
	Tgyro[2][0] = cos(misalign2) * sin(misalign1) * cos(misalign3)
			+ sin(misalign2) * sin(misalign3);
	Tgyro[2][1] = cos(misalign2) * sin(misalign1) * sin(misalign3)
			- sin(misalign2) * cos(misalign3);
	Tgyro[2][2] = cos(misalign2) * cos(misalign1);
	//////////////SETUP ACCEL NOISE///////////////
	Orthogonalityaccel[0][0] = AccelScaleFactor[0];
	Orthogonalityaccel[0][1] = accelXYcross;
	Orthogonalityaccel[0][2] = accelXZcross;
	Orthogonalityaccel[1][0] = accelXYcross;
	Orthogonalityaccel[1][1] = AccelScaleFactor[1];
	Orthogonalityaccel[1][2] = accelYZcross;
	Orthogonalityaccel[2][0] = accelXZcross;
	Orthogonalityaccel[2][1] = accelYZcross;
	Orthogonalityaccel[2][2] = AccelScaleFactor[2];
	Tgyroaccel[0][0] = cos(accelmisalign[0]) * cos(accelmisalign[2]);
	Tgyroaccel[0][1] = cos(accelmisalign[0]) * sin(accelmisalign[2]);
	Tgyroaccel[0][2] = -sin(accelmisalign[0]);
	Tgyroaccel[1][0] = sin(accelmisalign[1]) * sin(accelmisalign[0])
			* cos(accelmisalign[2])
			- cos(accelmisalign[1]) * sin(accelmisalign[2]);
	Tgyroaccel[1][1] = sin(accelmisalign[1]) * sin(accelmisalign[0])
			* sin(accelmisalign[2])
			+ cos(accelmisalign[1]) * cos(accelmisalign[2]);
	Tgyroaccel[1][2] = sin(accelmisalign[1]) * cos(accelmisalign[0]);
	Tgyroaccel[2][0] = cos(accelmisalign[1]) * sin(accelmisalign[0])
			* cos(accelmisalign[2])
			+ sin(accelmisalign[1]) * sin(accelmisalign[2]);
	Tgyroaccel[2][1] = cos(accelmisalign[1]) * sin(accelmisalign[0])
			* sin(accelmisalign[2])
			- sin(accelmisalign[1]) * cos(accelmisalign[2]);
	Tgyroaccel[2][2] = cos(accelmisalign[1]) * cos(accelmisalign[0]);
}

void printState(double vec[], char name[]) {
	int ii, jj;
	for (ii = 0; ii < BODIES; ii++) {
	  printf("%s Body = %d = \n", name,ii);
		for (jj = 0; jj < NSTATE; jj++) {
			printf("%0.20f\n", vec[ii * NSTATE + jj]);
		}
	}
}

void Get_Files() {
	int i;
	char dummy[256];
	FILE *infile = NULL;
	int Nverts, ii, jj, OVERRIDE;
	double W_mag, W_angle;
	char inTime[256], inBodyProp[256], inBodyAero[256], inIMProp[256],
			inErr[256], inConn[256];
	char inTimeRPM[256], inRotoProp[256], inSpDamp[256], inFlag[256],
			inAero[256], inMag[256], inMC[256];
	char inSaveOUT[256], inSaveLOG[256], inSavePROP[256], inSaveFileOUT[256],
			inSaveFileAUX[256], inSaveFilePROP[256], inSaveForce[256],
			inMCOUT[256];
	char inControl[256], inATM[256], inControlOUT[256], inSimFile[256],
	  inSim[256], inLinear[256], inNominal[256],inFdbkFile[256];

	/* Read in the required Text File names */
	//printf("Opening Input File \n");
	printf("Using Default Input File located at Input_Files/MultiMeta.ifiles \n");
	strcpy(FilesIn,"Input_Files/MultiMeta.ifiles");
	infile = fopen(FilesIn, "r");
	if (infile == NULL) { 
	  file_error(FilesIn,"Input_Files");
	}
	//printf("Opened Input File \n");
	fscanf(infile, "%s", inTimeFile);
	fscanf(infile, "%s", inBodyPropFile);
	fscanf(infile, "%s", inBodyAeroFile);
	fscanf(infile, "%s", inControlFile);
	fscanf(infile, "%s", inSimFile);
	fscanf(infile, "%s", inATMFile);
	fscanf(infile, "%s", inMagFile);
	fscanf(infile, "%s", inSpDampFile);
	fscanf(infile, "%s", inMPCFile);
	fscanf(infile, "%s", inErrFile);
	fscanf(infile, "%s", CONNFILE);
	fscanf(infile, "%s", inFdbkFile);
	fscanf(infile, "%s", inMCFile);
	fscanf(infile, "%s", inSaveFile);
	fscanf(infile, "%s", inNoiseOUT);
	fscanf(infile, "%s", inControlOUTFile);
	fscanf(infile, "%s", inForceFile);
	fscanf(infile, "%s", inWindFile);
	fscanf(infile, "%s", inBLENDFileOUT);
	fscanf(infile, "%s", inCoeffOUT);	
	fscanf(infile, "%s", inMCOUT);
	fscanf(infile, "%s", inLinear);
	fscanf(infile, "%s", inNominal);
	fscanf(infile, "%s", inFuselageLocation);
	fscanf(infile, "%s", inSweepsLocation);
	strcpy(inFuse,inFuselageLocation);

	///IMPORT FEEDBACK LINEARIZATION PARAMETERS
	infile = fopen(inFdbkFile, "r");
	if (infile == NULL) {
	  file_error(inFdbkFile,"FdbkFile");
	}
	fscanf(infile,"%lf %s \n",&WNQ,dummy);
	fscanf(infile,"%lf %s \n",&KPZ,dummy);
	fscanf(infile,"%lf %s \n",&WNP,dummy);
	fscanf(infile,"%lf %s \n",&KPPSI,dummy);
	fscanf(infile,"%lf %s \n",&QMAX,dummy);
	//fscanf(infile,"%lf %s \n",&PMAX,dummy);
	fclose(infile);

	////////OPEN TIME FILE////////////
	strcpy(inTime, inTimeFile);
	infile = fopen(inTime, "r");
	if (infile == NULL) {
	  file_error(inTime,"TimeFile");
	}
	fscanf(infile, "%lf %s", &Time[0], dummy); /* Load in initial time */
	fscanf(infile, "%lf %s", &Time[1], dummy); /* Load in final time */
	fscanf(infile, "%lf %s", &Time[2], dummy); /* Load in time increment with contact */
	fscanf(infile, "%lf %s", &Time[3], dummy); /* Load in time increment with no contact */
	fscanf(infile, "%d %s", &record_incr, dummy); /* Load in record incrementer */

	if (infile != NULL)
	  fclose(infile);
	/* Read the Control System File */
	strcpy(inControl, inControlFile);
	infile = fopen(inControl, "r");
	if (infile == NULL) {
	  file_error(inControl,"ControlFile");
	}
	fscanf(infile, "%d %s", &CONTROLON, dummy); /* Load in FCS Update Flag */
	fscanf(infile, "%d %s", &CONTROLTYPE, dummy); /* Load in FCS Update Flag */
	fscanf(infile, "%d %s", &FCSUPDATE, dummy); /* Load in FCS Update Flag */

	/* Read the Simulation File */
	strcpy(inSim, inSimFile);
	infile = fopen(inSim, "r");
	if (infile == NULL) {
	  file_error(inSim,"SimFile");
	}
	fscanf(infile, "%d %s", &MODE, dummy); /* Load in FCS Update Flag */
	// if ((MODE == 2) || (MODE == 3))
	// 	IMONTECARLO = 1;
	fscanf(infile, "%d %s", &MCRUN, dummy); /* Load in FCS Update Flag */
	fscanf(infile, "%d %s", &IBODYAERO, dummy); /* Load in FCS Update Flag */
	fscanf(infile, "%d %s", &IGRAVITY, dummy); /* Load in FCS Update Flag */
	fscanf(infile, "%d %s", &ICRASH, dummy); /* Load in Contact Flag */
	fscanf(infile, "%d %s", &USESENSORS, dummy); /* Load in Contact Flag */
	fscanf(infile, "%d %s", &SMARTSIMULATING, dummy); /* Load in Contact Flag */
	fscanf(infile, "%d %s", &ICONTACT, dummy); /* Load in Contact Flag */
	fscanf(infile, "%d %s", &IMAGNET, dummy); /* Load in Magnet Flag */
	fscanf(infile, "%d %s", &Nx, dummy); /* Load in Magnet Flag */
	fscanf(infile, "%d %s", &Ny, dummy); /* Load in Magnet Flag */
	fscanf(infile, "%d %s", &Nz, dummy); /* Load in Magnet Flag */

	infile = fopen(inATMFile, "r");
	if (infile == NULL) {
	  file_error(inATMFile,"ATMFile");
	}
	fscanf(infile, "%d %s", &ICONSTANT, dummy);
	fscanf(infile, "%lf %lf %lf %s", &ICONSTANTSCALE[0], &ICONSTANTSCALE[1],
			&ICONSTANTSCALE[2], dummy);
	if ((ICONSTANTSCALE[0] + ICONSTANTSCALE[1] + ICONSTANTSCALE[2]) == 0)
		ICONSTANT = 0;
	fscanf(infile, "%lf %lf %lf %s", &FREQ[0], &FREQ[1], &FREQ[2], dummy);
	fscanf(infile, "%d %s", &ITURB, dummy);
	fscanf(infile, "%lf %lf %lf %s", &TURBLEVEL[0], &TURBLEVEL[1],
			&TURBLEVEL[2], dummy); /* Load in Windscale Flag */
	if ((TURBLEVEL[0] + TURBLEVEL[1] + TURBLEVEL[2]) == 0)
		ITURB = 0;
	fscanf(infile, "%d %s", &IWRF, dummy); /* Load in WRF Scale Flag */
	fscanf(infile, "%lf %lf %lf %s", &IWRFSCALE[0], &IWRFSCALE[1],
			&IWRFSCALE[2], dummy); /* Load in WRF Scale Flag */
	if ((IWRFSCALE[0] + IWRFSCALE[1] + IWRFSCALE[2]) == 0)
		IWRF = 0;
	fscanf(infile, "%d %s", &OVERRIDE, dummy);
	if (OVERRIDE) {
		fscanf(infile, "%s ", &PATH);
	} else {
		//Better idea
                #if (__linux__)
		strcpy(PATH,"/home/carlos/Work/Georgia_Tech/WRF_Wind_Data/Data25dx_HF_0/");
                #endif
                #if (__macintosh__) || (__Macintosh__) || (__APPLE__) || (__MACH__)
		strcpy(PATH,"/Users/carlos/Work/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/");
                #endif
		#if (WIN32) || (_WINDOWS) || (__WIN32__) || (__WINDOWS__) || (__WIN64__) || (_WIN32) 
		printf("Test \n");
		PAUSE();
                #endif
	}

	/* Read in the Body Properties File */
	strcpy(inBodyProp, inBodyPropFile);
	infile = fopen(inBodyProp, "r");
	if (infile == NULL) {
	  file_error(inBodyProp,"BodyPropFile");
	}
	fscanf(infile, "%lf %s", &WEIGHT, dummy); /* Load in weight */
	fscanf(infile, "%lf %s", &Iner[0][0][0], dummy); /* Load in Ixx */
	fscanf(infile, "%lf %s", &Iner[0][1][1], dummy); /* Load in Iyy */
	fscanf(infile, "%lf %s", &Iner[0][2][2], dummy); /* Load in Izz */
	fscanf(infile, "%lf %s", &Iner[0][0][1], dummy); /* Load in Ixy */
	fscanf(infile, "%lf %s", &Iner[0][0][2], dummy); /* Load in Ixz */
	fscanf(infile, "%lf %s", &Iner[0][1][2], dummy); /* Load in Iyz */

	///Scale the weight
	// double scalefactor = 1.5;
	// WEIGHT = scalefactor*WEIGHT;
	// Iner[0][0][0] = scalefactor*Iner[0][0][0]; 
	// Iner[0][1][1] = scalefactor*Iner[0][1][1]; 
	// Iner[0][2][2] = scalefactor*Iner[0][2][2]; 
	
	GRAVITY = 0;
	fscanf(infile, "%lf %s", &GRAVITY, dummy); /* Load in Gravity */
	//Some files don't have gravity in them. If so it defaults to 9.81.
	//Note that gravity can be turned off in .SIM file by switching
	//IGRAVITY to 0
	if (!GRAVITY) {
	  GRAVITY = 9.81;
	}
	//printf("Gravity = %lf\n",GRAVITY);
	if (infile != NULL){
	  fclose(infile);
	}
	/* Read in the Aerodynamics File */
	strcpy(inAero, inBodyAeroFile);
	infile = fopen(inAero, "r");
	if (infile == NULL) {
	  file_error(inAero,"AeroFile");
	}

	fscanf(infile, "%d %s", &AEROMODEL, dummy);
	fscanf(infile, "%d %s", &VORTEXMODEL, dummy);
	fscanf(infile, "%d %s", &INTERACT, dummy);
	fscanf(infile, "%d %s", &TIBCONSTANT, dummy);
	fscanf(infile, "%d %s", &VSKIP, dummy);
	fscanf(infile, "%d %s", &FIXED, dummy);
	fscanf(infile, "%d %s", &WINGPANELS, dummy);
	fscanf(infile, "%lf %s", &C_L_0, dummy); //* Load in C_L_0 */
	fscanf(infile, "%lf %s", &C_D_0, dummy); //* Load in C_D_0 */
	fscanf(infile, "%lf %s", &C_m_0, dummy);
	fscanf(infile, "%lf %s", &C_D_u, dummy);
	fscanf(infile, "%lf %s", &C_L_alpha, dummy);
	fscanf(infile, "%lf %s", &C_D_alpha, dummy);
	fscanf(infile, "%lf %s", &C_m_alpha, dummy);
	fscanf(infile, "%lf %s", &C_m_alpha_dot, dummy);
	fscanf(infile, "%lf %s", &C_m_u, dummy);
	fscanf(infile, "%lf %s", &C_L_q, dummy);
	fscanf(infile, "%lf %s", &C_m_q, dummy);
	fscanf(infile, "%lf %s", &C_L_de, dummy);
	fscanf(infile, "%lf %s", &C_m_de, dummy);
	fscanf(infile, "%lf %s", &C_x_delThrust, dummy);
	fscanf(infile, "%lf %s", &C_y_beta, dummy);
	fscanf(infile, "%lf %s", &C_l_beta, dummy);
	fscanf(infile, "%lf %s", &C_n_beta, dummy);
	fscanf(infile, "%lf %s", &C_l_p, dummy);
	fscanf(infile, "%lf %s", &C_n_p, dummy);
	fscanf(infile, "%lf %s", &C_l_r, dummy);
	fscanf(infile, "%lf %s", &C_n_r, dummy);
	fscanf(infile, "%lf %s", &C_l_da, dummy);
	fscanf(infile, "%lf %s", &C_n_da, dummy);
	fscanf(infile, "%lf %s", &C_y_dr, dummy);
	fscanf(infile, "%lf %s", &C_l_dr, dummy);
	fscanf(infile, "%lf %s", &C_n_dr, dummy);
	fscanf(infile, "%lf %s", &C_y_p, dummy);
	fscanf(infile, "%lf %s", &C_y_r, dummy);
	fscanf(infile, "%lf %s", &Sarea[0], dummy);
	fscanf(infile, "%lf %s", &wspan[0], dummy);
	fscanf(infile, "%lf %s", &c, dummy);
	fscanf(infile, "%lf %s", &Vtrim, dummy);
	fscanf(infile, "%lf %s", &mainwx, dummy);
	fscanf(infile, "%lf %s", &htailx, dummy);
	fscanf(infile, "%lf %s", &vtailx, dummy);
	fscanf(infile, "%lf %s", &vtailz, dummy);
	fscanf(infile, "%lf %s", &tailspan, dummy);

	//Read in the Magnet File
	strcpy(inMag, inMagFile);
	infile = fopen(inMag, "r");
	if (infile == NULL) {
	  file_error(inMag,"MagFile");
	}
	fscanf(infile, "%lf %s", &J, dummy);
	fscanf(infile, "%lf %s", &msizeI[0], dummy);
	fscanf(infile, "%lf %s", &msizeI[1], dummy);
	fscanf(infile, "%lf %s", &msizeI[2], dummy);
	fscanf(infile, "%lf %s", &DENMAGNET, dummy);

	//Read the contact File
	strcpy(inSpDamp, inSpDampFile);
	infile = fopen(inSpDamp, "r");
	if (infile == NULL) {
	  file_error(inSpDamp,"SpDampFile");
	}
	fscanf(infile, "%lf %lf %s", &K[0][0], &K[0][1], dummy); /* Load in K1n and K1t */
	fscanf(infile, "%lf %lf %s", &K[1][0], &K[1][1], dummy); /* Load in K2n and K2t */
	fscanf(infile, "%lf %lf %s", &C[0][0], &C[0][1], dummy); /* Load in C1n and C1t */
	fscanf(infile, "%lf %lf %s", &C[1][0], &C[1][1], dummy); /* Load in C2n and C2t */
	fscanf(infile, "%lf %s", &mu, dummy); /* Load in mu */
	fscanf(infile, "%lf %lf %lf %s", &rcg1_plane[0], &rcg1_plane[1],
			&rcg1_plane[2], dummy);
	fscanf(infile, "%lf %lf %lf %lf %lf %lf %s", &obj[0], &obj[1], &obj[2],
			&obj[3], &obj[4], &obj[5], dummy);
	fscanf(infile, "%d %s", &Nverts, dummy); /* Load in Number of Vertices */
	if (NVERTS != Nverts) {
		printf("NVERTS does not equal Nverts(%d) - Change NVERTS in Code\n",
				Nverts);
		exit(1);
	}
	for (i = 0; i < Nverts; i++) {
		fscanf(infile, "%lf %lf %lf %s", &rVertex[0][i], &rVertex[1][i],
				&rVertex[2][i], dummy);
	}
	if (infile != NULL)
	  fclose(infile);

	//Sensor Errors File
	infile = fopen(inErrFile, "r");
	if (infile == NULL) {
	  file_error(inErrFile,"ErrFile");
	}
	fscanf(infile, "%s", dummy);
	fscanf(infile, "%lf %s", &GPSNOISETYPE, dummy);
	fscanf(infile, "%lf %lf %lf %s", &posBias[0], &posBias[1], &posBias[2],
			dummy);
	fscanf(infile, "%lf %lf %lf %s", &posNoise[0], &posNoise[1], &posNoise[2],
			dummy);
	fscanf(infile, "%lf %lf %lf %s", &velNoise[0], &velNoise[1], &velNoise[2],
			dummy);
	fscanf(infile, "%lf %s", &taupos, dummy);
	fscanf(infile, "%lf %s", &tauvel, dummy);
	fscanf(infile, "%lf %s", &GPSupdate, dummy);
	fscanf(infile, "%s", dummy);
	fscanf(infile, "%lf %s", &PQRNOISETYPE, dummy);
	fscanf(infile, "%lf %lf %lf %s", &sigmanoise[0], &sigmanoise[1],
			&sigmanoise[2], dummy);
	fscanf(infile, "%lf %lf %lf %s", &sigmaturnonIO[0], &sigmaturnonIO[1],
			&sigmaturnonIO[2], dummy);
	fscanf(infile, "%lf %lf %lf %s", &sigmadrift[0], &sigmadrift[1],
			&sigmadrift[2], dummy);
	fscanf(infile, "%lf %s", &XYcrossIO, dummy);
	fscanf(infile, "%lf %s", &YZcrossIO, dummy);
	fscanf(infile, "%lf %s", &XZcrossIO, dummy);
	fscanf(infile, "%lf %lf %lf %s", &misalign1IO, &misalign2IO, &misalign3IO,
			dummy);
	fscanf(infile, "%lf %lf %lf %s", &ScaleFactorIO[0], &ScaleFactorIO[1],
			&ScaleFactorIO[2], dummy);
	fscanf(infile, "%lf %s", &GYROupdate, dummy);
	fscanf(infile, "%s", dummy);
	fscanf(infile, "%lf %s", &HNOISETYPE, dummy);
	fscanf(infile, "%lf %lf %lf %s", &sigmah[0], &sigmah[1], &sigmah[2], dummy);
	fscanf(infile, "%lf %lf %lf %s", &biash[0], &biash[1], &biash[2], dummy);
	fscanf(infile, "%s", dummy);
	fscanf(infile, "%lf %s", &ACCELNOISETYPE, dummy);
	fscanf(infile, "%lf %lf %lf %s", &accelsigmanoise[0], &accelsigmanoise[1],
			&accelsigmanoise[2], dummy);
	fscanf(infile, "%lf %lf %lf %s", &accelsigmaturnonIO[0],
			&accelsigmaturnonIO[1], &accelsigmaturnonIO[2], dummy);
	fscanf(infile, "%lf %lf %lf %s", &sigmadrift[0], &sigmadrift[1],
			&sigmadrift[2], dummy);
	fscanf(infile, "%lf %s", &accelXYcrossIO, dummy);
	fscanf(infile, "%lf %s", &accelYZcrossIO, dummy);
	fscanf(infile, "%lf %s", &accelXZcrossIO, dummy);
	fscanf(infile, "%lf %lf %lf %s", &accelmisalignIO[0], &accelmisalignIO[1],
			&accelmisalignIO[2], dummy);
	fscanf(infile, "%lf %lf %lf %s", &AccelScaleFactorIO[0],
			&AccelScaleFactorIO[1], &AccelScaleFactorIO[2], dummy);
	fscanf(infile, "%lf %s", &ACCELupdate, dummy);
	fscanf(infile, "%s", dummy);
	fscanf(infile, "%lf %s", &VISNAVNOISETYPE, dummy);

	strcpy(inSaveOUT, inSaveFile);
	outfile = fopen(inSaveOUT, "w");
	if (outfile == NULL) {
	  file_error(inSaveOUT,"SaveOUTFile");
	}
	strcpy(inControlOUT, inControlOUTFile);
	controlfile = fopen(inControlOUT, "w");
	if (controlfile == NULL) {
	  file_error(inControlOUT,"ControlOUTFile");
	}

	forcefile = fopen(inForceFile, "w");
	if (forcefile == NULL) {
	  file_error(inForceFile,"ForceFile");
	}

	windoutfile = fopen(inWindFile, "w");
	if (windoutfile == NULL) {
	  file_error(inWindFile,"WindFile");
	}

	blendfile = fopen(inBLENDFileOUT, "w");
	if (blendfile == NULL) {
	  file_error(inBLENDFileOUT,"BLENDFileOUT");
	}
	noisefile = fopen(inNoiseOUT, "w");
	if (noisefile == NULL) {
	  file_error(inNoiseOUT,"NoiseOUTFile");
	}

	//Open Linearization Files
	if (MODE == 1) {
	  linearfile = fopen(inLinear, "w");
	  if (linearfile == NULL) {file_error(inLinear,"LinearFile");}
	  nominalfile = fopen(inNominal, "w");
	  if (nominalfile == NULL) {file_error(inNominal,"NominalFile");}
	}

	//Open fmeta file
	fmetafile = fopen("Fmeta.txt","w");

	//Open Monte Carlo File
	if (IMONTECARLO) 	  {
	  mcfile = fopen(inMCFile, "r");
	  if (mcfile == NULL) {
	  file_error(inMCFile,"MCFile");
	  }
	  fscanf(mcfile, "%d %s", &NUMMC, dummy); /* Load in Monte Carlo Flag */
	  mcoutfile = fopen(inMCOUT, "w");
	  if (mcoutfile == NULL) {
	  file_error(inMCOUT,"MCOUTFile");
	  }
	}
}

void Record_State() {
	int length, ii, jj;

	fprintf(outfile, "%.12f ", TCURRENT);
	fprintf(controlfile, "%.12f ", TCURRENT);
	fprintf(forcefile, "%.12f ", TCURRENT);
	fprintf(windoutfile, "%.12f ", TCURRENT);
	fprintf(blendfile, "%.12f ", TCURRENT);
	fprintf(noisefile, "%.12f ", TCURRENT);
	for (ii = 0; ii < BODIES; ii++) {
		for (jj = 0; jj < NSTATE; jj++) {
			fprintf(outfile, " %.12f ", State[ii * NSTATE + jj]);
			fprintf(noisefile, " %.12f ", StateE[ii * NSTATE + jj]);
		}
		fprintf(controlfile, "%.12f %.12f %.12f %.12f ", DE[ii], DA[ii], DR[ii],
				DELTHRUST[ii]);
		fprintf(forcefile, " %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f", F_A[ii][0],
			F_A[ii][1], F_A[ii][2], M_A[ii][0], M_A[ii][1], M_A[ii][2],LIFT_A[ii],DRAG_A[ii]);
		//fprintf(forcefile, " %.12f %.12f %.12f %.12f %.12f %.12f ", F_C[ii][0],
		//F_C[ii][1], F_C[ii][2], M_C[ii][0], M_C[ii][1], M_C[ii][2]);
		//fprintf(forcefile, " %.12f %.12f %.12f %.12f %.12f %.12f ",
		//F_M_B[ii][0], F_M_B[ii][1], F_M_B[ii][2], T_M_B[ii][0],
		//T_M_B[ii][1], T_M_B[ii][2]);
		fprintf(windoutfile, "%lf %lf %lf ", WINDB[0][ii], WINDB[1][ii],
				WINDB[2][ii]);
	}
	fprintf(outfile, "\n");
	fprintf(noisefile, "\n");
	fprintf(controlfile, "\n");
	fprintf(forcefile, "\n");
	fprintf(windoutfile, "\n");
	fprintf(blendfile, "%lf %lf %lf %lf %lf %lf \n", THOLD, TY, PSIPATH, PSIY,
			WAYPOINTOFFSET, WAYPOINT);

}
void Final_Check() {
	if ((GOODCONTACT == 0) && (MODE != 3)) {
		printf("---------------Connection Failed-------------- \n");
	}
	// if (MODE == 3) {
	// 	//Check and see if final state is closer than initial state
	// 	double dinitial = sqrt(
	// 			pow(INITIAL[0] - INITIAL[NSTATE + 0], 2)
	// 					+ pow(INITIAL[1] - INITIAL[NSTATE + 1], 2)
	// 					+ pow(INITIAL[2] - INITIAL[NSTATE + 2], 2));
	// 	double dfinal = sqrt(
	// 			pow(State[0] - State[NSTATE + 0], 2)
	// 					+ pow(State[1] - State[NSTATE + 1], 2)
	// 					+ pow(State[2] - State[NSTATE + 2], 2));
	// 	printf("XYZ INITIAL = %f %f %f \n", INITIAL[NSTATE],
	// 			INITIAL[NSTATE + 1], INITIAL[NSTATE + 2]);
	// 	printf("dfinal %f dinitial %f \n", dfinal, dinitial);
	// 	if (dfinal < dinitial) {
	// 		printf("---------------Final State Closer-------------- \n");
	// 	} else {
	// 		printf("------------Final State Farther Away------------ \n");
	// 	}
	// }
}

void InteractionCheck() {
	//Compute Maximum WIND Environment
	double windnorm = sqrt(pow(WIND[0][0], 2) + pow(WIND[1][0], 2) + pow(WIND[2][0], 2));
	if (windnorm > WINDMAX) {
	  WINDMAX = windnorm;
	}

	//Magnet Check 
	if ((GOODCONTACT == 1) && (numContacts > 0) && (MAGCONSTANT == 0)) {
	  MAGCONSTANT = 1;
	}

	/////////////////Check for Crashing A/C/////////////
	if (ICRASH) {
	  double crash = sqrt(pow(State[0] - State[NSTATE], 2)	+ pow(State[1] - State[NSTATE + 1], 2) + pow(State[2] - State[NSTATE + 2], 2));
	  if (crash < wspan[0] / 3) {
	    printf("------------------------------Crash Detected = %lf------------------------------\n",TCURRENT);
	    ICRASH = 1;
	    TFINAL = 0;
	  }
	}

	if (MAGDISTANCE > 1000) {
	  printf("---------------------------Aircraft Distance Exceeded = %lf----------------------\n",TCURRENT);
	  printf("Distance %lf \n", MAGDISTANCE);
	  TFINAL = 0;
	  ICRASH = 2;
	}

	if ((State[2] > 0) || (State[NSTATE + 2] > 0)) {
	  printf("-----------------------Aircraft has crashed into the ground = %lf-----------------\n",TCURRENT);
	  TFINAL = 0;
	  ICRASH = 3;
	  printf("Altitude %lf \n", State[2]);
	  printf("Altitude %lf \n", State[NSTATE + 2]);
	}

	if ((GOODCONTACT == 1) && (!IMONTECARLO)) {
	  if (CONNECTION == 0) {
	    CONNECTION = 1;
	    printf("Connection Achieved %lf \n", TCURRENT);
	    if (TCURRENT < TFINAL) {
	      TFINAL = TCURRENT + 5;
	    }
	    printf("F_M_B[body][0] = %lf \n", F_M_B[0][1]);
	  }
	}

	/* Record Time and States */
	if (MODE == 2) {
	  if ((numContacts > 0) && (GOODCONTACT == 1)) {
	    printf("Connection Achieved t = %lf \n", TCURRENT);
	    tConnect = TCURRENT;
	    TCURRENT = TFINAL + 100;
	  }
	}
}

void PoorMansContactModel() {
	int ii, jj, compute[NBODIES], body0, body1;
	double xI, yI, zI, x1, y1, z1, x0, y0, z0, rB_JointI[3][NBODIES], psi0,
			psi1;
	double xB, yB, zB, u0, u1, v0, v1, w0, w1, p0, p1, q0, q1, r0, r1, phi0,
			phi1, theta0;
	double uJ0, vJ0, wJ0, uJ1, vJ1, wJ1, uJ1I, vJ1I, wJ1I, uJB, vJB, wJB,
			theta1;
	double F_CI[3], Fx, Fy, Fz, Mx, My, Mz, M_CI[3], Fx1, Fy1, Fz1;

	//Unwrap State
	for (ii = 0; ii < NUMCONNECTIONS; ii++) {
		body0 = CONNECTMAT[ii][0];
		body1 = CONNECTMAT[ii][1];
		x0 = State[body0 * NSTATE + 0];
		x1 = State[body1 * NSTATE];
		y0 = State[body0 * NSTATE + 1];
		y1 = State[body1 * NSTATE + 1];
		z0 = State[body0 * NSTATE + 2];
		z1 = State[body1 * NSTATE + 2];
		phi0 = State[body0 * NSTATE + 3];
		phi1 = State[body1 * NSTATE + 3];
		theta0 = State[body0 * NSTATE + 4];
		theta1 = State[body1 * NSTATE + 4];
		psi0 = State[body0 * NSTATE + 5];
		psi1 = State[body1 * NSTATE + 5];
		u0 = State[body0 * NSTATE + 6];
		u1 = State[body1 * NSTATE + 6];
		v0 = State[body0 * NSTATE + 7];
		v1 = State[body1 * NSTATE + 7];
		w0 = State[body0 * NSTATE + 8];
		w1 = State[body1 * NSTATE + 8];
		p0 = State[body0 * NSTATE + 9];
		p1 = State[body1 * NSTATE + 9];
		q0 = State[body0 * NSTATE + 10];
		q1 = State[body1 * NSTATE + 10];
		r0 = State[body0 * NSTATE + 11];
		r1 = State[body1 * NSTATE + 11];
		//Joint location of 1st body
		rB_JointI[0][body0] = TIB[body0][0][0] * rB_Joint[ii][0][0]
				+ TIB[body0][0][1] * rB_Joint[ii][0][1]
				+ TIB[body0][0][2] * rB_Joint[ii][0][2];
		rB_JointI[1][body0] = TIB[body0][1][0] * rB_Joint[ii][0][0]
				+ TIB[body0][1][1] * rB_Joint[ii][0][1]
				+ TIB[body0][1][2] * rB_Joint[ii][0][2];
		rB_JointI[2][body0] = TIB[body0][2][0] * rB_Joint[ii][0][0]
				+ TIB[body0][2][1] * rB_Joint[ii][0][1]
				+ TIB[body0][2][2] * rB_Joint[ii][0][2];
		//Joint Location of second body
		rB_JointI[0][body1] = TIB[body1][0][0] * rB_Joint[ii][1][0]
				+ TIB[body1][0][1] * rB_Joint[ii][1][1]
				+ TIB[body1][0][2] * rB_Joint[ii][1][2];
		rB_JointI[1][body1] = TIB[body1][1][0] * rB_Joint[ii][1][0]
				+ TIB[body1][1][1] * rB_Joint[ii][1][1]
				+ TIB[body1][1][2] * rB_Joint[ii][1][2];
		rB_JointI[2][body1] = TIB[body1][2][0] * rB_Joint[ii][1][0]
				+ TIB[body1][2][1] * rB_Joint[ii][1][1]
				+ TIB[body1][2][2] * rB_Joint[ii][1][2];
		//Compute Linear Error
		xI = x1 + rB_JointI[0][body1] - x0 - rB_JointI[0][body0];
		yI = y1 + rB_JointI[1][body1] - y0 - rB_JointI[1][body0];
		zI = z1 + rB_JointI[2][body1] - z0 - rB_JointI[2][body0];
		//Rotate to Body 0 Frame
		xB = TBI[body0][0][0] * xI + TBI[body0][0][1] * yI
				+ TBI[body0][0][2] * zI;
		yB = TBI[body0][1][0] * xI + TBI[body0][1][1] * yI
				+ TBI[body0][1][2] * zI;
		zB = TBI[body0][2][0] * xI + TBI[body0][2][1] * yI
				+ TBI[body0][2][2] * zI;
		//Velocity of Joint on body 0
		uJ0 = u0 + -r0 * rB_Joint[ii][0][1] + q0 * rB_Joint[ii][0][2];
		vJ0 = v0 + r0 * rB_Joint[ii][0][0] + -p0 * rB_Joint[ii][0][2];
		wJ0 = w0 + -q0 * rB_Joint[ii][0][0] + p0 * rB_Joint[ii][0][1];
		//Velocity of Joint on Body 1
		uJ1 = u1 + -r1 * rB_Joint[ii][1][1] + q1 * rB_Joint[ii][1][2];
		vJ1 = v1 + r1 * rB_Joint[ii][1][0] + -p1 * rB_Joint[ii][1][2];
		wJ1 = w1 + -q1 * rB_Joint[ii][1][0] + p1 * rB_Joint[ii][1][1];
		//Rotate Velocity to inertial
		uJ1I = TIB[body1][0][0] * uJ1 + TIB[body1][0][1] * vJ1
				+ TIB[body1][0][2] * wJ1;
		vJ1I = TIB[body1][1][0] * uJ1 + TIB[body1][1][1] * vJ1
				+ TIB[body1][1][2] * wJ1;
		wJ1I = TIB[body1][2][0] * uJ1 + TIB[body1][2][1] * vJ1
				+ TIB[body1][2][2] * wJ1;
		//Rotate Velocity to Body 0 
		uJB = TBI[body0][0][0] * uJ1I + TBI[body0][0][1] * vJ1I
				+ TBI[body0][0][2] * wJ1I;
		vJB = TBI[body0][1][0] * uJ1I + TBI[body0][1][1] * vJ1I
				+ TBI[body0][1][2] * wJ1I;
		wJB = TBI[body0][2][0] * uJ1I + TBI[body0][2][1] * vJ1I
				+ TBI[body0][2][2] * wJ1I;
		Fx = KL[0] * xB - CL[0] * (uJ0 - uJB);
		Fy = KL[1] * yB - CL[1] * (vJ0 - vJB);
		Fz = KL[2] * zB - CL[2] * (wJ0 - wJB);
		//Body 0 Forces
		F_C[body0][0] = F_C[body0][0] + Fx;
		F_C[body0][1] = F_C[body0][1] + Fy;
		F_C[body0][2] = F_C[body0][2] + Fz;
		//Body 1 Forces
		F_CI[0] = -1
				* (TIB[body0][0][0] * Fx + TIB[body0][0][1] * Fy
						+ TIB[body0][0][2] * Fz);
		F_CI[1] = -1
				* (TIB[body0][1][0] * Fx + TIB[body0][1][1] * Fy
						+ TIB[body0][1][2] * Fz);
		F_CI[2] = -1
				* (TIB[body0][2][0] * Fx + TIB[body0][2][1] * Fy
						+ TIB[body0][2][2] * Fz);
		//Rotate to body 1
		Fx1 = 1
				* (TBI[body1][0][0] * F_CI[0] + TBI[body1][0][1] * F_CI[1]
						+ TBI[body1][0][2] * F_CI[2]);
		Fy1 = 1
				* (TBI[body1][1][0] * F_CI[0] + TBI[body1][1][1] * F_CI[1]
						+ TBI[body1][1][2] * F_CI[2]);
		Fz1 = 1
				* (TBI[body1][2][0] * F_CI[0] + TBI[body1][2][1] * F_CI[1]
						+ TBI[body1][2][2] * F_CI[2]);
		F_C[body1][0] = F_C[body1][0] + Fx1;
		F_C[body1][1] = F_C[body1][1] + Fy1;
		F_C[body1][2] = F_C[body1][2] + Fz1;
		//Compute Rotational Springs and Dampers
		Mx = -KR[0] * (phi0 - phi1) - CR[0] * (p0 - p1);
		My = -KR[1] * (theta0 - theta1) - CR[1] * (q0 - q1);
		Mz = -KR[2] * (psi0 - psi1) - CR[2] * (r0 - r1);
		M_C[body0][0] = M_C[body0][0] + Mx;
		M_C[body0][1] = M_C[body0][1] + My;
		M_C[body0][2] = M_C[body0][2] + Mz;
		//Add negative to body1 
		M_CI[0] = -1
				* (TIB[body0][0][0] * Mx + TIB[body0][0][1] * My
						+ TIB[body0][0][2] * Mz);
		M_CI[1] = -1
				* (TIB[body0][1][0] * Mx + TIB[body0][1][1] * My
						+ TIB[body0][1][2] * Mz);
		M_CI[2] = -1
				* (TIB[body0][2][0] * Mx + TIB[body0][2][1] * My
						+ TIB[body0][2][2] * Mz);
		//Rotate to body 1
		M_C[body1][0] = M_C[body1][0]
				+ 1
						* (TBI[body1][0][0] * M_CI[0]
								+ TBI[body1][0][1] * M_CI[1]
								+ TBI[body1][0][2] * M_CI[2]);
		M_C[body1][1] = M_C[body1][1]
				+ 1
						* (TBI[body1][1][0] * M_CI[0]
								+ TBI[body1][1][1] * M_CI[1]
								+ TBI[body1][1][2] * M_CI[2]);
		M_C[body1][2] = M_C[body1][2]
				+ 1
						* (TBI[body1][2][0] * M_CI[0]
								+ TBI[body1][2][1] * M_CI[1]
								+ TBI[body1][2][2] * M_CI[2]);
		//Add effect of force offset from c.g.
		M_C[body0][0] = M_C[body0][0] + -rB_Joint[ii][0][2] * Fy
				+ rB_Joint[ii][0][1] * Fz;
		M_C[body0][1] = M_C[body0][1] + rB_Joint[ii][0][2] * Fx
				+ -rB_Joint[ii][0][0] * Fz;
		M_C[body0][2] = M_C[body0][2] + -rB_Joint[ii][0][1] * Fx
				+ rB_Joint[ii][0][0] * Fy;
		// //Body body1
		M_C[body1][0] = M_C[body1][0] + -rB_Joint[ii][1][2] * Fy1
				+ rB_Joint[ii][1][1] * Fz1;
		M_C[body1][1] = M_C[body1][1] + rB_Joint[ii][1][2] * Fx1
				+ -rB_Joint[ii][1][0] * Fz1;
		M_C[body1][2] = M_C[body1][2] + -rB_Joint[ii][1][1] * Fx1
				+ rB_Joint[ii][1][0] * Fy1;
	}
}

void SimplifiedContactModel(int connectiontype) {
	int ii, jj, compute[NBODIES], body0, body1;
	double xI, yI, zI, x1, y1, z1, x0, y0, z0, rB_JointI[3][NBODIES], psi0,
			psi1;
	double xB, yB, zB, u0, u1, v0, v1, w0, w1, p0, p1, q0, q1, r0, r1, phi0,
			phi1, theta0;
	double uJ0, vJ0, wJ0, uJ1, vJ1, wJ1, uJ1I, vJ1I, wJ1I, uJB, vJB, wJB,
			theta1;
	double F_CI[3], Fx, Fy, Fz, Mx, My, Mz, M_CI[3], Fx1, Fy1, Fz1;

	//Unwrap State
	for (ii = 0; ii < NUMCONNECTIONS; ii++) {
		body0 = CONNECTMAT[ii][0];
		body1 = CONNECTMAT[ii][1];
		x0 = State[body0 * NSTATE + 0];
		x1 = State[body1 * NSTATE];
		y0 = State[body0 * NSTATE + 1];
		y1 = State[body1 * NSTATE + 1];
		z0 = State[body0 * NSTATE + 2];
		z1 = State[body1 * NSTATE + 2];
		phi0 = State[body0 * NSTATE + 3];
		phi1 = State[body1 * NSTATE + 3];
		theta0 = State[body0 * NSTATE + 4];
		theta1 = State[body1 * NSTATE + 4];
		psi0 = State[body0 * NSTATE + 5];
		psi1 = State[body1 * NSTATE + 5];
		u0 = State[body0 * NSTATE + 6];
		u1 = State[body1 * NSTATE + 6];
		v0 = State[body0 * NSTATE + 7];
		v1 = State[body1 * NSTATE + 7];
		w0 = State[body0 * NSTATE + 8];
		w1 = State[body1 * NSTATE + 8];
		p0 = State[body0 * NSTATE + 9];
		p1 = State[body1 * NSTATE + 9];
		q0 = State[body0 * NSTATE + 10];
		q1 = State[body1 * NSTATE + 10];
		r0 = State[body0 * NSTATE + 11];
		r1 = State[body1 * NSTATE + 11];
		//Joint location of 1st body
		rB_JointI[0][body0] = TIB[body0][0][0] * rB_Joint[ii][0][0]
				+ TIB[body0][0][1] * rB_Joint[ii][0][1]
				+ TIB[body0][0][2] * rB_Joint[ii][0][2];
		rB_JointI[1][body0] = TIB[body0][1][0] * rB_Joint[ii][0][0]
				+ TIB[body0][1][1] * rB_Joint[ii][0][1]
				+ TIB[body0][1][2] * rB_Joint[ii][0][2];
		rB_JointI[2][body0] = TIB[body0][2][0] * rB_Joint[ii][0][0]
				+ TIB[body0][2][1] * rB_Joint[ii][0][1]
				+ TIB[body0][2][2] * rB_Joint[ii][0][2];
		//Joint Location of second body
		rB_JointI[0][body1] = TIB[body1][0][0] * rB_Joint[ii][1][0]
				+ TIB[body1][0][1] * rB_Joint[ii][1][1]
				+ TIB[body1][0][2] * rB_Joint[ii][1][2];
		rB_JointI[1][body1] = TIB[body1][1][0] * rB_Joint[ii][1][0]
				+ TIB[body1][1][1] * rB_Joint[ii][1][1]
				+ TIB[body1][1][2] * rB_Joint[ii][1][2];
		rB_JointI[2][body1] = TIB[body1][2][0] * rB_Joint[ii][1][0]
				+ TIB[body1][2][1] * rB_Joint[ii][1][1]
				+ TIB[body1][2][2] * rB_Joint[ii][1][2];
		//Compute Linear Error
		xI = x1 + rB_JointI[0][body1] - x0 - rB_JointI[0][body0];
		yI = y1 + rB_JointI[1][body1] - y0 - rB_JointI[1][body0];
		zI = z1 + rB_JointI[2][body1] - z0 - rB_JointI[2][body0];
		//Rotate to Body 0 Frame
		xB = TBI[body0][0][0] * xI + TBI[body0][0][1] * yI
				+ TBI[body0][0][2] * zI;
		yB = TBI[body0][1][0] * xI + TBI[body0][1][1] * yI
				+ TBI[body0][1][2] * zI;
		zB = TBI[body0][2][0] * xI + TBI[body0][2][1] * yI
				+ TBI[body0][2][2] * zI;
		//Velocity of Joint on body 0
		uJ0 = u0 + -r0 * rB_Joint[ii][0][1] + q0 * rB_Joint[ii][0][2];
		vJ0 = v0 + r0 * rB_Joint[ii][0][0] + -p0 * rB_Joint[ii][0][2];
		wJ0 = w0 + -q0 * rB_Joint[ii][0][0] + p0 * rB_Joint[ii][0][1];
		//Velocity of Joint on Body 1
		uJ1 = u1 + -r1 * rB_Joint[ii][1][1] + q1 * rB_Joint[ii][1][2];
		vJ1 = v1 + r1 * rB_Joint[ii][1][0] + -p1 * rB_Joint[ii][1][2];
		wJ1 = w1 + -q1 * rB_Joint[ii][1][0] + p1 * rB_Joint[ii][1][1];
		//Rotate Velocity to inertial
		uJ1I = TIB[body1][0][0] * uJ1 + TIB[body1][0][1] * vJ1
				+ TIB[body1][0][2] * wJ1;
		vJ1I = TIB[body1][1][0] * uJ1 + TIB[body1][1][1] * vJ1
				+ TIB[body1][1][2] * wJ1;
		wJ1I = TIB[body1][2][0] * uJ1 + TIB[body1][2][1] * vJ1
				+ TIB[body1][2][2] * wJ1;
		//Rotate Velocity to Body 0 
		uJB = TBI[body0][0][0] * uJ1I + TBI[body0][0][1] * vJ1I
				+ TBI[body0][0][2] * wJ1I;
		vJB = TBI[body0][1][0] * uJ1I + TBI[body0][1][1] * vJ1I
				+ TBI[body0][1][2] * wJ1I;
		wJB = TBI[body0][2][0] * uJ1I + TBI[body0][2][1] * vJ1I
				+ TBI[body0][2][2] * wJ1I;
		// Fx = KL[0]*xB - CL[0]*(uJ0-uJB);
		// Fy = KL[1]*yB - CL[1]*(vJ0-vJB); 
		// Fz = KL[2]*zB - CL[2]*(wJ0-wJB);
		//printf("Fconn = %lf \n",Fconn);
		// PAUSE();
		//Set Fx,Fy Fz to zero
		Fx = 0;
		Fy = 0;
		Fz = 0;
		if (connectiontype == 0) {
			//Wingtip to wingtip
			//Check to see that Fy is a magnet or magnet plus contact
			double offset = copysign(1.0, rB_Joint[ii][0][1]) * Fconn / KL[1];
			// if (copysign(1.0,yB) == copysign(1.0,rB_Joint[ii][0][1]))
			// 	{
			// 	  Fy = 0;
			// 	}
			// else
			// 	{
			Fy = KL[1] * (yB - offset) - CL[1] * (vJ0 - vJB);
			// }
			Fy = Fy + copysign(1.0, rB_Joint[ii][0][1]) * Fconn; //1000 N magnet
			//Replace with force of magnet
			//Now replace Fx,and Fz with friction and magnetic forces
			//Check to make sure magnetic force isn't thru the roof
			Fx = 20 * Fconn * xB;
			if (fabs(Fx) > Fconn * 0.4) {
				Fx = copysign(1.0, Fx) * Fconn * 0.4;
			}
			Fz = 20 * Fconn * zB;
			if (fabs(Fz) > Fconn * 0.4) {
				Fz = copysign(1.0, Fz) * Fconn * 0.4;
			}
			// Fx = Fx + KL[0]*xB - CL[0]*(uJ0-uJB);
			// Fz = Fz + KL[2]*zB - CL[2]*(wJ0-wJB);
		} else {
			//Tip to tail
			//Check to see that Fx is a magnet or magnet plus contact
			double offset = copysign(1.0, rB_Joint[ii][0][0]) * Fconn / KL[1];
			// if (copysign(1.0,yB) == copysign(1.0,rB_Joint[ii][0][1]))
			// 	{
			// 	  Fy = 0;
			// 	}
			// else
			// 	{
			Fx = KL[0] * (xB - offset) - CL[0] * (uJ0 - uJB);
			// }
			Fx = Fx + copysign(1.0, rB_Joint[ii][0][0]) * Fconn; //1000 N magnet
			//Replace with force of magnet
			//Now replace Fy,and Fz with friction and magnetic forces
			//Check to make sure magnetic force isn't thru the roof
			Fy = 20 * Fconn * yB;
			if (fabs(Fy) > 0.4 * Fconn) {
				Fy = copysign(1.0, Fy) * Fconn * 0.4;
			}
			Fz = Fconn * 20 * zB;
			if (fabs(Fz) > Fconn * 0.4) {
				Fz = copysign(1.0, Fz) * Fconn * 0.4;
			}
		}
		//Body 0 Forces
		F_C[body0][0] = F_C[body0][0] + Fx;
		F_C[body0][1] = F_C[body0][1] + Fy;
		F_C[body0][2] = F_C[body0][2] + Fz;
		//Inertial
		F_CI[0] = -1
				* (TIB[body0][0][0] * Fx + TIB[body0][0][1] * Fy
						+ TIB[body0][0][2] * Fz);
		F_CI[1] = -1
				* (TIB[body0][1][0] * Fx + TIB[body0][1][1] * Fy
						+ TIB[body0][1][2] * Fz);
		F_CI[2] = -1
				* (TIB[body0][2][0] * Fx + TIB[body0][2][1] * Fy
						+ TIB[body0][2][2] * Fz);
		//Rotate to body 1
		Fx1 = 1
				* (TBI[body1][0][0] * F_CI[0] + TBI[body1][0][1] * F_CI[1]
						+ TBI[body1][0][2] * F_CI[2]);
		Fy1 = 1
				* (TBI[body1][1][0] * F_CI[0] + TBI[body1][1][1] * F_CI[1]
						+ TBI[body1][1][2] * F_CI[2]);
		Fz1 = 1
				* (TBI[body1][2][0] * F_CI[0] + TBI[body1][2][1] * F_CI[1]
						+ TBI[body1][2][2] * F_CI[2]);
		F_C[body1][0] = F_C[body1][0] + Fx1;
		F_C[body1][1] = F_C[body1][1] + Fy1;
		F_C[body1][2] = F_C[body1][2] + Fz1;
		//Compute Rotational Springs and Dampers
		Mx = -KR[0] * (phi0 - phi1) - CR[0] * (p0 - p1);
		My = -KR[1] * (theta0 - theta1) - CR[1] * (q0 - q1);
		Mz = -KR[2] * (psi0 - psi1) - CR[2] * (r0 - r1);
		M_C[body0][0] = M_C[body0][0] + Mx;
		M_C[body0][1] = M_C[body0][1] + My;
		M_C[body0][2] = M_C[body0][2] + Mz;
		//Add negative to body1 
		M_CI[0] = -1
				* (TIB[body0][0][0] * Mx + TIB[body0][0][1] * My
						+ TIB[body0][0][2] * Mz);
		M_CI[1] = -1
				* (TIB[body0][1][0] * Mx + TIB[body0][1][1] * My
						+ TIB[body0][1][2] * Mz);
		M_CI[2] = -1
				* (TIB[body0][2][0] * Mx + TIB[body0][2][1] * My
						+ TIB[body0][2][2] * Mz);
		//Rotate to body 1
		M_C[body1][0] = M_C[body1][0]
				+ 1
						* (TBI[body1][0][0] * M_CI[0]
								+ TBI[body1][0][1] * M_CI[1]
								+ TBI[body1][0][2] * M_CI[2]);
		M_C[body1][1] = M_C[body1][1]
				+ 1
						* (TBI[body1][1][0] * M_CI[0]
								+ TBI[body1][1][1] * M_CI[1]
								+ TBI[body1][1][2] * M_CI[2]);
		M_C[body1][2] = M_C[body1][2]
				+ 1
						* (TBI[body1][2][0] * M_CI[0]
								+ TBI[body1][2][1] * M_CI[1]
								+ TBI[body1][2][2] * M_CI[2]);
		//Add effect of force offset from c.g.
		M_C[body0][0] = M_C[body0][0] + -rB_Joint[ii][0][2] * Fy
				+ rB_Joint[ii][0][1] * Fz;
		M_C[body0][1] = M_C[body0][1] + rB_Joint[ii][0][2] * Fx
				+ -rB_Joint[ii][0][0] * Fz;
		M_C[body0][2] = M_C[body0][2] + -rB_Joint[ii][0][1] * Fx
				+ rB_Joint[ii][0][0] * Fy;
		// //Body body1
		M_C[body1][0] = M_C[body1][0] + -rB_Joint[ii][1][2] * Fy1
				+ rB_Joint[ii][1][1] * Fz1;
		M_C[body1][1] = M_C[body1][1] + rB_Joint[ii][1][2] * Fx1
				+ -rB_Joint[ii][1][0] * Fz1;
		M_C[body1][2] = M_C[body1][2] + -rB_Joint[ii][1][1] * Fx1
				+ rB_Joint[ii][1][0] * Fy1;
		//Check Forces for Max Connection
		double F_mag = sqrt(
				pow(F_C[ii][0], 2) + pow(F_C[ii][1], 2) + pow(F_C[ii][2], 2));
		if (F_mag > MAXF) {
			MAXF = F_mag;
		}
	}
}

void System_Derivatives() {
	int Analytic, ii, jj, kk, ll, pp, qq, uu, vv, ww, body;
	double Uij[2][2], Vkl[2][2], Wpq[2][2], a, a1, bm, b1, c, c1, xM, yM, zM,
			int1, int2, int3;
	double Fx, Fy, Fz, mu0, U, V, W, one, R, dFx, dFy, dFz, xI, yI, zI,
			F_M1_M[3];
	double x[NBODIES], y[NBODIES], z[NBODIES], phi[NBODIES], theta[NBODIES],
			psi[NBODIES];
	double u[NBODIES], v[NBODIES], w[NBODIES], p[NBODIES], q[NBODIES],
			r[NBODIES], phiF, thetaF, xB, yB, zB;
	double VcgAtm[3], Vel, rho, SOS, Mach, AngVel[3];
	double F_R[3], F_P[3], F_W[NBODIES][3], F_T[NBODIES][3], M_R[3], M_P[3];
	double xdot, ydot, zdot, phidot, thetadot, psidot, udot, vdot, wdot, pdot,
			qdot, rdot;
	double BodyAeroLoads[6], Normal[3], rcg1_magnetI[3], rcg_magnetI[3];
	double M_T[NBODIES][3], ttheta, psiint, ptpdot[3], xydot[2], zint, xint,
			yint;
	/* Contact Analysis Variables */
	double s1n[3][NVERTS], s1t[3][NVERTS], s2n[3][NVERTS], s2t[3][NVERTS];
	double NormalB[3], rplane_vertex[3][NVERTS], rcg_vertex_I[3], rInter[3],
			rcg1_vertex_I[3];
	double delun[3][NVERTS], delut[3][NVERTS], delwt[3][NVERTS];
	double uB[3], uC[3], u1B[3], u1C[3], delu[3];
	double deluDotNormal, norm2Normal, c_star, commands[3];
	double bn[3], bt[3], norm_bn, norm_bt, lambda;
	double F_CI[NBODIES][3], M_CI[NBODIES][3], Fn[3], Ft[3], M_Cf[3], M_Cf1[3];
	double CoeffDamper1[4], CoeffDamper2[4];
	double s1ndot[3][NVERTS], s1tdot[3][NVERTS], s2ndot[3][NVERTS],
			s2tdot[3][NVERTS];
	double ctheta, stheta, spsi, cpsi, sphi, cphi, M_M[NBODIES][3], T_M1_I[3],
			T_M1_B1[3];
	double position[3], xyzdot[3], T_M1_M[3], T_M_I[3];
	double F_M_I[3], F_M1_I[3], Fmax, dist, F0, imag, d1, d2, d3, Mx, My, Mz,
			T_M1_B[3];
	double S, T, delFx, delFy, delFz, du, dv, dw, duvw, dctheta, dstheta, dsphi,
			dcphi, dspsi;
	double phim0, thetam0, psim0, phim1, thetam1, psim1, dphi, dtheta, dpsi,
			T01[3][3];
	double dcpsi, dttheta, Mx0, My0, Mz0, xp, yp, zp, crash, F_mag, d,
			F_Magnet[3], still;

	///XYZDOT AND PTPDOT
	for (body = 0; body < BODIES; body++) {
		/* Unwrap State Vector */
		x[body] = State[body * NSTATE];
		
		if (x[body] != x[body])
		  {
		    printf("Nan Detected \n");
		    exit(1);
		  }

		y[body] = State[body * NSTATE + 1];
		z[body] = State[body * NSTATE + 2];
		phi[body] = State[body * NSTATE + 3];
		theta[body] = State[body * NSTATE + 4];
		psi[body] = State[body * NSTATE + 5];
		u[body] = State[body * NSTATE + 6];
		v[body] = State[body * NSTATE + 7];
		w[body] = State[body * NSTATE + 8];
		p[body] = State[body * NSTATE + 9];
		q[body] = State[body * NSTATE + 10];
		r[body] = State[body * NSTATE + 11];
		zint = State[body * NSTATE + 12];
		xint = State[body * NSTATE + 13];
		yint = State[body * NSTATE + 14];

		/* Transformation Matrix (Transformation from I to B frame) */
		ctheta = cos(theta[body]);
		stheta = sin(theta[body]);
		ttheta = stheta / ctheta;
		cphi = cos(phi[body]);
		sphi = sin(phi[body]);
		spsi = sin(psi[body]);
		cpsi = cos(psi[body]);
		TBI[body][0][0] = ctheta * cpsi;
		TBI[body][0][1] = ctheta * spsi;
		TBI[body][0][2] = -stheta;
		TBI[body][1][0] = sphi * stheta * cpsi - cphi * spsi;
		TBI[body][1][1] = sphi * stheta * spsi + cphi * cpsi;
		TBI[body][1][2] = sphi * ctheta;
		TBI[body][2][0] = cphi * stheta * cpsi + sphi * spsi;
		TBI[body][2][1] = cphi * stheta * spsi - sphi * cpsi;
		TBI[body][2][2] = cphi * ctheta;
		/* Transformation Matrix from B to I Frame */
		TIB[body][0][0] = TBI[body][0][0];
		TIB[body][0][1] = TBI[body][1][0];
		TIB[body][0][2] = TBI[body][2][0];
		TIB[body][1][0] = TBI[body][0][1];
		TIB[body][1][1] = TBI[body][1][1];
		TIB[body][1][2] = TBI[body][2][1];
		TIB[body][2][0] = TBI[body][0][2];
		TIB[body][2][1] = TBI[body][1][2];
		TIB[body][2][2] = TBI[body][2][2];

		//Compute Initial TIB MATRIX
		if (TCURRENT == TINITIAL) {
			mateqii(TINTB, TIB, 3, 3, body);
			mateqii(TBINT, TBI, 3, 3, body);
		}

		///XYZDOT
		/*Kinematic derivatives xyzdot*/
		xdot = TIB[body][0][0] * u[body] + TIB[body][0][1] * v[body]
				+ TIB[body][0][2] * w[body];
		ydot = TIB[body][1][0] * u[body] + TIB[body][1][1] * v[body]
				+ TIB[body][1][2] * w[body];
		zdot = TIB[body][2][0] * u[body] + TIB[body][2][1] * v[body]
				+ TIB[body][2][2] * w[body];
		/* Phi,theta,Psi Derivatives*/
		phidot = p[body] + sphi * ttheta * q[body] + cphi * ttheta * r[body];
		thetadot = cphi * q[body] - sphi * r[body];
		psidot = (sphi / ctheta) * q[body] + (cphi / ctheta) * r[body];
		/*Wrap State Vector Derivatives*/
		still = 1;
		if (body == 1) {
			still = 1;
		}
		StateDot[body * NSTATE] = xdot * still;
		StateDot[body * NSTATE + 1] = ydot * LATERAL * still;
		StateDot[body * NSTATE + 2] = zdot * still;
		StateDot[body * NSTATE + 3] = phidot * LATERAL * still;
		StateDot[body * NSTATE + 4] = thetadot * still;
		StateDot[body * NSTATE + 5] = psidot * LATERAL * still;

		//Gravity Forces
		if (IGRAVITY) {
			F_W[body][0] = WEIGHT * TBI[body][0][2];
			F_W[body][1] = WEIGHT * TBI[body][1][2];
			F_W[body][2] = WEIGHT * TBI[body][2][2];
		} else {
			F_W[body][0] = 0;
			F_W[body][1] = 0;
			F_W[body][2] = 0;
		}

		/* Initialize Magnetic Forces */
		F_M_B[body][0] = 0.0;
		F_M_B[body][1] = 0.0;
		F_M_B[body][2] = 0.0;
		T_M_B[body][0] = 0.0;
		T_M_B[body][1] = 0.0;
		T_M_B[body][2] = 0.0;
		M_M[body][0] = 0.0;
		M_M[body][1] = 0.0;
		M_M[body][2] = 0.0;

		/* Initialize Contact Forces and Moments (in I Frame) */

		F_C[body][0] = 0.000000000;
		F_C[body][1] = 0.000000000;
		F_C[body][2] = 0.000000000;
		M_C[body][0] = 0.000000000;
		M_C[body][1] = 0.000000000;
		M_C[body][2] = 0.000000000;

		//Compute Velocity at each Panel
		if (AEROMODEL) {
		  VelocityPanels(body);
		}

	}

	//	printf("Kinematics Done \n");

	//Compute Interaction Forces

	//////LIFTING LINE MODEL////
	if (VORTEXMODEL) {
		VCOUNTER++;
		if ((VCOUNTER > VSKIP)) {
			VCOUNTER = 0;
			if (GOODCONTACT) {
				LiftingLineNoInteract(BODIES);
			} else {
				if (INTERACT) {
					LiftingLine(BODIES);
				} else {
					LiftingLineNoInteract(BODIES);
				}
			}
		}
	}
	
	//	printf("Lifting Line Done \n");

	///////MAGNET MODEL/////////
	if (IMAGNET != 0) {
		ComputeMagnetForce();
	}

	//////CONTACT MODEL///////
	if (ICONTACT == 1) {
		ComputeContactForce();
	}
	if ((ICONTACT == 2) && (BODIES > 1)) {
		if (connectiontype != 2) {
			//Wingtip to Wingtip
			//SimplifiedContactModel(0);
			//Tip to tail
			//SimplifiedContactModel(1);
			SimplifiedContactModel(connectiontype);
		} else {
			PoorMansContactModel();
		}
	}

	//	printf("Contact Model Done \n");

	//UVWDOT and PQRDOT
	for (body = 0; body < BODIES; body++) {
		if (IBODYAERO) {
			//Aerodynamics
			if (AEROMODEL) {
				PanelForces(BodyAeroLoads, body);
			} else {
				//Use single computation point model
				AeroForces(BodyAeroLoads, body);
			}
			F_A[body][0] = BodyAeroLoads[0];
			F_A[body][1] = BodyAeroLoads[1];
			F_A[body][2] = BodyAeroLoads[2];
			M_A[body][0] = BodyAeroLoads[3];
			M_A[body][1] = BodyAeroLoads[4];
			M_A[body][2] = BodyAeroLoads[5];
		} else {
			F_A[body][0] = 0;
			F_A[body][1] = 0;
			F_A[body][2] = 0;
			M_A[body][0] = 0;
			M_A[body][1] = 0;
			M_A[body][2] = 0;
		}

		//		printf("Aero Forces Done \n");

		/* Consolidate Total Forces and Moment (in B Frame)*/
		F_T[body][0] = F_W[body][0] + F_A[body][0] + F_M_B[body][0]
				+ F_C[body][0];
		F_T[body][1] = F_W[body][1] + F_A[body][1] + F_M_B[body][1]
				+ F_C[body][1];
		F_T[body][2] = F_W[body][2] + F_A[body][2] + F_M_B[body][2]
				+ F_C[body][2];
		M_T[body][0] = M_A[body][0] + M_M[body][0] + T_M_B[body][0]
				+ M_C[body][0];
		M_T[body][1] = M_A[body][1] + M_M[body][1] + T_M_B[body][1]
				+ M_C[body][1];
		M_T[body][2] = M_A[body][2] + M_M[body][2] + T_M_B[body][2]
				+ M_C[body][2];

		//UVWDOT
		udot = F_T[body][0] / mass[body] + r[body] * v[body]
				- q[body] * w[body];
		vdot = F_T[body][1] / mass[body] + p[body] * w[body]
				- r[body] * u[body];
		wdot = F_T[body][2] / mass[body] + q[body] * u[body]
				- p[body] * v[body];

		//PQRDOT
		int1 = M_T[body][0]
				- p[body]
						* (q[body] * Iner[body][0][2]
								- r[body] * Iner[body][0][1])
				- q[body]
						* (q[body] * Iner[body][1][2]
								- r[body] * Iner[body][1][1])
				- r[body]
						* (q[body] * Iner[body][2][2]
								- r[body] * Iner[body][1][2]);
		int2 = M_T[body][1]
				- p[body]
						* (r[body] * Iner[body][0][0]
								- p[body] * Iner[body][0][2])
				- q[body]
						* (r[body] * Iner[body][0][1]
								- p[body] * Iner[body][1][2])
				- r[body]
						* (r[body] * Iner[body][0][2]
								- p[body] * Iner[body][2][2]);
		int3 = M_T[body][2]
				- p[body]
						* (p[body] * Iner[body][0][1]
								- q[body] * Iner[body][0][0])
				- q[body]
						* (p[body] * Iner[body][1][1]
								- q[body] * Iner[body][0][1])
				- r[body]
						* (p[body] * Iner[body][1][2]
								- q[body] * Iner[body][0][2]);

		pdot = InerInv[body][0][0] * int1 + InerInv[body][0][1] * int2
				+ InerInv[body][0][2] * int3;
		qdot = InerInv[body][1][0] * int1 + InerInv[body][1][1] * int2
				+ InerInv[body][1][2] * int3;
		rdot = InerInv[body][2][0] * int1 + InerInv[body][2][1] * int2
				+ InerInv[body][2][2] * int3;

		/* Wrap State Vector Derivatives */
		still = 1;
		if (body == 1) {
			still = 1;
		}
		StateDot[body * NSTATE + 6] = udot * still;
		StateDot[body * NSTATE + 7] = vdot * LATERAL * still;
		StateDot[body * NSTATE + 8] = wdot * still;
		StateDot[body * NSTATE + 9] = pdot * LATERAL * still;
		StateDot[body * NSTATE + 10] = qdot * still;
		StateDot[body * NSTATE + 11] = rdot * LATERAL * still;

		//Integral States
		if (body == 0) {
			StateDot[body * NSTATE + 12] = z[body] - ZCOMMAND[body];
		} else {
			StateDot[body * NSTATE + 12] = -ZDELT;
		}
		if (CONTROLTYPE == 1)
		{
			StateDot[body * NSTATE + 12] = zMETA - ZCOMMAND[0];
		}

		StateDot[body * NSTATE + 13] = XDELT;
		StateDot[body * NSTATE + 14] = YDELT;

	}

	// printf("uvw_induced_B = \n");
	// for (ii = 0;ii<NPANELS*BODIES;ii++)
	//   {
	//     printf("%0.20f %0.20f %0.20f \n",uvw_induced_B[0][ii],uvw_induced_B[1][ii],uvw_induced_B[2][ii]);
	//   }
	// PAUSE();

}

void System_Derivatives_META() {
  int Analytic, ii, jj, kk, ll, pp, qq, uu, vv, ww, body,idx;
  double Uij[2][2], Vkl[2][2], Wpq[2][2], a, a1, bm, b1, c, c1, xM, yM, zM,int1, int2, int3;
  double Fx, Fy, Fz, mu0, U, V, W, one, R, dFx, dFy, dFz, xI, yI, zI,F_M1_M[3];
  double x[NBODIES], y[NBODIES], z[NBODIES], phi[NBODIES], theta[NBODIES],psi[NBODIES];
  double u[NBODIES], v[NBODIES], w[NBODIES], p[NBODIES], q[NBODIES],r[NBODIES], phiF, thetaF, xB, yB, zB;
  double VcgAtm[3], Vel, rho, SOS, Mach, AngVel[3];
  double F_R[3], F_P[3], F_W[NBODIES][3], F_T[NBODIES][3], M_R[3], M_P[3];
  double xdot, ydot, zdot, phidot, thetadot, psidot, udot, vdot, wdot, pdot,qdot, rdot;
  double BodyAeroLoads[6], Normal[3], rcg1_magnetI[3], rcg_magnetI[3];
  double M_T[NBODIES][3], ttheta, psiint, ptpdot[3], xydot[2], zint, xint,yint;
  /* Contact Analysis Variables */
  double s1n[3][NVERTS], s1t[3][NVERTS], s2n[3][NVERTS], s2t[3][NVERTS];
  double NormalB[3], rplane_vertex[3][NVERTS], rcg_vertex_I[3], rInter[3],rcg1_vertex_I[3];
  double delun[3][NVERTS], delut[3][NVERTS], delwt[3][NVERTS];
  double uB[3], uC[3], u1B[3], u1C[3], delu[3];
  double deluDotNormal, norm2Normal, c_star, commands[3];
  double bn[3], bt[3], norm_bn, norm_bt, lambda;
  double F_CI[NBODIES][3], M_CI[NBODIES][3], Fn[3], Ft[3], M_Cf[3], M_Cf1[3];
  double CoeffDamper1[4], CoeffDamper2[4];
  double s1ndot[3][NVERTS], s1tdot[3][NVERTS], s2ndot[3][NVERTS],s2tdot[3][NVERTS];
  double ctheta, stheta, spsi, cpsi, sphi, cphi, M_M[NBODIES][3], T_M1_I[3],T_M1_B1[3];
  double position[3], xyzdot[3], T_M1_M[3], T_M_I[3];
  double F_M_I[3], F_M1_I[3], Fmax, dist, F0, imag, d1, d2, d3, Mx, My, Mz,T_M1_B[3];
  double S, T, delFx, delFy, delFz, du, dv, dw, duvw, dctheta, dstheta, dsphi,dcphi, dspsi;
  double phim0, thetam0, psim0, phim1, thetam1, psim1, dphi, dtheta, dpsi,T01[3][3];
  double dcpsi, dttheta, Mx0, My0, Mz0, xp, yp, zp, crash, F_mag, d,F_Magnet[3], still;
  double F_6DOF[3],M_6DOF[3];

  //For this type of Meta Aircraft we will compute the center of mass of the composite system
  //and the state vector of the entire system
  double StateMETA[NSTATE];
  for (idx = 0;idx < NSTATE;idx++){
    StateMETA[idx] = 0;
  }
  //In Order to get the center of mass we simply use the formula xM = sum(x*mass)/massMETA
  for (body = 0;body < BODIES;body++) {
    for (idx = 0;idx < 3;idx++) {
      StateMETA[idx] += State[body * NSTATE+idx]*mass[body]/massMETA;
    }
  }
  //Phi theta and psi will be the same for all aircraft and StateMETA
  for (idx = 3;idx<6;idx++){
    StateMETA[idx] = State[idx];
  }	
  //U,V,W will operate on two points fixed on a rigid body
  //Vcenter = Vbodyi + omega X rac2cg
  //ucenter = ui + [0  -ri qi]*[xi yi zi]
  //vcenter = vi + [ri  0 -pi]*[xi yi zi]
  //vcenter = vi + [-qi pi  0]*[xi yi zi]
  double xi = -RCG2AC[0][0]; //The first 0 is x and the second zero is the first aircraft
  double yi = -RCG2AC[1][0];
  double zi = 0; //Apparently this is always zero
  double ui = State[6];
  double vi = State[7];
  double wi = State[8];
  double pi = State[9];
  double qi = State[10];
  double ri = State[11];
  StateMETA[6] = ui - ri*yi + qi*zi;
  StateMETA[7] = vi + ri*xi - pi*zi;
  StateMETA[8] = wi - qi*xi + pi*yi;
  //P,Q,R will also be the same for the entire system
  for (idx = 9;idx<12;idx++){
    StateMETA[idx] = State[idx];
  }

  // printf("===============New Iteration==========\n");
  // for (idx = 0;idx<12;idx++){
  //   printf("StateMETA = %lf \n",StateMETA[idx]);
  //   printf("State = %lf \n",State[idx]);
  // }

  //The total mass is already computed in the Master_Init() routine
  // printf("Mass = %lf \n",mass[0]);
  // printf("Mass Total = %lf \n",massMETA);

  //The Total Inertia is already computed in the Master_Init() routine
  //matrixdisp(I_b_6DOF,3,3,"Total_Inertia_Matrix");
  //matrixdisp(Inv_b_6DOF,3,3,"Total_Inverse_Inertia_Matrix");

  //Now there is no need to compute the kinematic derivatives of the total system because 
  //kinematics are preserved over the entire body.
  //Thus it is possible to simply compute the kinematic derivatives of the entire system
  for (body = 0; body < BODIES; body++) {
    /* Unwrap State Vector */
    x[body] = State[body * NSTATE];
    
    if (x[body] != x[body])
      {
	printf("Nan Detected \n");
	exit(1);
      }
    
    y[body] = State[body * NSTATE + 1];
    z[body] = State[body * NSTATE + 2];
    phi[body] = State[body * NSTATE + 3];
    theta[body] = State[body * NSTATE + 4];
    psi[body] = State[body * NSTATE + 5];
    u[body] = State[body * NSTATE + 6];
    v[body] = State[body * NSTATE + 7];
    w[body] = State[body * NSTATE + 8];
    p[body] = State[body * NSTATE + 9];
    q[body] = State[body * NSTATE + 10];
    r[body] = State[body * NSTATE + 11];
    zint = State[body * NSTATE + 12];
    xint = State[body * NSTATE + 13];
    yint = State[body * NSTATE + 14];
    
    /* Transformation Matrix (Transformation from I to B frame) */
    ctheta = cos(theta[body]);
    stheta = sin(theta[body]);
    ttheta = stheta / ctheta;
    cphi = cos(phi[body]);
    sphi = sin(phi[body]);
    spsi = sin(psi[body]);
    cpsi = cos(psi[body]);
    TBI[body][0][0] = ctheta * cpsi;
    TBI[body][0][1] = ctheta * spsi;
    TBI[body][0][2] = -stheta;
    TBI[body][1][0] = sphi * stheta * cpsi - cphi * spsi;
    TBI[body][1][1] = sphi * stheta * spsi + cphi * cpsi;
    TBI[body][1][2] = sphi * ctheta;
    TBI[body][2][0] = cphi * stheta * cpsi + sphi * spsi;
    TBI[body][2][1] = cphi * stheta * spsi - sphi * cpsi;
    TBI[body][2][2] = cphi * ctheta;
    /* Transformation Matrix from B to I Frame */
    TIB[body][0][0] = TBI[body][0][0];
    TIB[body][0][1] = TBI[body][1][0];
    TIB[body][0][2] = TBI[body][2][0];
    TIB[body][1][0] = TBI[body][0][1];
    TIB[body][1][1] = TBI[body][1][1];
    TIB[body][1][2] = TBI[body][2][1];
    TIB[body][2][0] = TBI[body][0][2];
    TIB[body][2][1] = TBI[body][1][2];
    TIB[body][2][2] = TBI[body][2][2];

    ///XYZDOT
    /*Kinematic derivatives xyzdot*/
    xdot = TIB[body][0][0] * u[body] + TIB[body][0][1] * v[body] + TIB[body][0][2] * w[body];
    ydot = TIB[body][1][0] * u[body] + TIB[body][1][1] * v[body] + TIB[body][1][2] * w[body];
    zdot = TIB[body][2][0] * u[body] + TIB[body][2][1] * v[body] + TIB[body][2][2] * w[body];
    /* Phi,theta,Psi Derivatives*/
    phidot = p[body] + sphi * ttheta * q[body] + cphi * ttheta * r[body];
    thetadot = cphi * q[body] - sphi * r[body];
    psidot = (sphi / ctheta) * q[body] + (cphi / ctheta) * r[body];
    /*Wrap State Vector Derivatives*/
    StateDot[body * NSTATE] = xdot;
    StateDot[body * NSTATE + 1] = ydot;
    StateDot[body * NSTATE + 2] = zdot;
    StateDot[body * NSTATE + 3] = phidot;
    StateDot[body * NSTATE + 4] = thetadot;
    StateDot[body * NSTATE + 5] = psidot;
    
    //Compute Velocity at each Panel
    if (AEROMODEL) {
      VelocityPanels(body);
    }
    
  } //End loop through all bodies

  //	printf("Kinematics Done \n");

  //Compute Interaction Forces

  //////LIFTING LINE MODEL////
  if (VORTEXMODEL) {
    VCOUNTER++;
    if ((VCOUNTER > VSKIP)) {
      VCOUNTER = 0;
      if (GOODCONTACT) {
	LiftingLineNoInteract(BODIES);
      } else {
	if (INTERACT) {
	  LiftingLine(BODIES);
	} else {
	  LiftingLineNoInteract(BODIES);
	}
      }
    }
  }
	
  //	printf("Lifting Line Done \n");
  
  //Initialize Forces and Moments on Composite System
  for (idx = 0;idx<3;idx++){
    F_6DOF[idx]=0;
    M_6DOF[idx]=0;
  }

  //Compute Total Forces and Moments
  for (body = 0; body < BODIES; body++) {
    //Gravity Forces
    if (IGRAVITY) {
      F_W[body][0] = WEIGHT * TBI[body][0][2];
      F_W[body][1] = WEIGHT * TBI[body][1][2];
      F_W[body][2] = WEIGHT * TBI[body][2][2];
    } else {
      F_W[body][0] = 0;
      F_W[body][1] = 0;
      F_W[body][2] = 0;
    }
    
    /* Initialize Magnetic Forces */
    F_M_B[body][0] = 0.0;
    F_M_B[body][1] = 0.0;
    F_M_B[body][2] = 0.0;
    T_M_B[body][0] = 0.0;
    T_M_B[body][1] = 0.0;
    T_M_B[body][2] = 0.0;
    M_M[body][0] = 0.0;
    M_M[body][1] = 0.0;
    M_M[body][2] = 0.0;

    /* Initialize Contact Forces and Moments (in I Frame) */
    
    F_C[body][0] = 0.000000000;
    F_C[body][1] = 0.000000000;
    F_C[body][2] = 0.000000000;
    M_C[body][0] = 0.000000000;
    M_C[body][1] = 0.000000000;
    M_C[body][2] = 0.000000000;

    if (IBODYAERO) {
      //Aerodynamics
      if (AEROMODEL) {
	PanelForces(BodyAeroLoads, body);
      } else {
	//Use single computation point model
	AeroForces(BodyAeroLoads, body);
      }
      F_A[body][0] = BodyAeroLoads[0];
      F_A[body][1] = BodyAeroLoads[1];
      F_A[body][2] = BodyAeroLoads[2];
      M_A[body][0] = BodyAeroLoads[3];
      M_A[body][1] = BodyAeroLoads[4];
      M_A[body][2] = BodyAeroLoads[5];
    } else {
      F_A[body][0] = 0;
      F_A[body][1] = 0;
      F_A[body][2] = 0;
      M_A[body][0] = 0;
      M_A[body][1] = 0;
      M_A[body][2] = 0;
    }
    
    //		printf("Aero Forces Done \n");
    
    /* Consolidate Total Forces and Moment (in B Frame)*/
    F_T[body][0] = F_W[body][0] + F_A[body][0] + F_M_B[body][0] + F_C[body][0];
    F_T[body][1] = F_W[body][1] + F_A[body][1] + F_M_B[body][1] + F_C[body][1];
    F_T[body][2] = F_W[body][2] + F_A[body][2] + F_M_B[body][2] + F_C[body][2];
    M_T[body][0] = M_A[body][0] + M_M[body][0] + T_M_B[body][0] + M_C[body][0];
    M_T[body][1] = M_A[body][1] + M_M[body][1] + T_M_B[body][1] + M_C[body][1];
    M_T[body][2] = M_A[body][2] + M_M[body][2] + T_M_B[body][2] + M_C[body][2];

    //Now compute the total forces and moments on the cg of the composite system
    for (idx = 0;idx<3;idx++){
      F_6DOF[idx]+=F_T[body][idx];
      M_6DOF[idx]+=M_T[body][idx];
    }
    //Roll Moment
    M_6DOF[0]+=RCG2AC[1][body]*F_T[body][2];
    //Pitch Moment
    M_6DOF[1]+=-RCG2AC[0][body]*F_T[body][2];
    //Yaw Moment
    M_6DOF[2]+=-RCG2AC[1][body]*F_T[body][0] + RCG2AC[0][body]*F_T[body][1];

    //Integral States
    if (body == 0) {
      StateDot[body * NSTATE + 12] = z[body] - ZCOMMAND[body];
    } else {
      StateDot[body * NSTATE + 12] = -ZDELT;
    }
    if (CONTROLTYPE == 1)
      {
	StateDot[body * NSTATE + 12] = zMETA - ZCOMMAND[0];
      }
    
    StateDot[body * NSTATE + 13] = XDELT;
    StateDot[body * NSTATE + 14] = YDELT;
    
  }

  //Now We can compute the dynamic derivatives on the composite system
  //UVWDOT
  double uMETA = StateMETA[6];
  double vMETA = StateMETA[7];
  double wMETA = StateMETA[8];
  double pMETA = StateMETA[9];
  double qMETA = StateMETA[10];
  double rMETA = StateMETA[11];
  double udotMETA = F_6DOF[0] / massMETA + rMETA * vMETA - qMETA * wMETA;
  double vdotMETA = F_6DOF[1] / massMETA + pMETA * wMETA - rMETA * uMETA;
  double wdotMETA = F_6DOF[2] / massMETA + qMETA * uMETA - pMETA * vMETA;
  
  //PQRDOT
  int1 = M_6DOF[0] - pMETA * (qMETA * I_b_6DOF[0][2] - rMETA * I_b_6DOF[0][1]) - qMETA * (qMETA * I_b_6DOF[1][2] - rMETA * I_b_6DOF[1][1]) - rMETA * (qMETA * I_b_6DOF[2][2] - rMETA * I_b_6DOF[1][2]);
  int2 = M_6DOF[1] - pMETA * (rMETA * I_b_6DOF[0][0]	- pMETA * I_b_6DOF[0][2]) - qMETA	* (rMETA * I_b_6DOF[0][1] - pMETA * I_b_6DOF[1][2]) - rMETA * (rMETA * I_b_6DOF[0][2] - pMETA * I_b_6DOF[2][2]);
  int3 = M_6DOF[2] - pMETA * (pMETA * I_b_6DOF[0][1]	- qMETA * I_b_6DOF[0][0]) - qMETA	* (pMETA * I_b_6DOF[1][1] - qMETA * I_b_6DOF[0][1]) - rMETA * (pMETA * I_b_6DOF[1][2] - qMETA * I_b_6DOF[0][2]);
  double pdotMETA = Inv_b_6DOF[0][0] * int1 + Inv_b_6DOF[0][1] * int2 + Inv_b_6DOF[0][2] * int3;
  double qdotMETA = Inv_b_6DOF[1][0] * int1 + Inv_b_6DOF[1][1] * int2 + Inv_b_6DOF[1][2] * int3;
  double rdotMETA = Inv_b_6DOF[2][0] * int1 + Inv_b_6DOF[2][1] * int2 + Inv_b_6DOF[2][2] * int3;

  /* Wrap State Vector Derivatives */
  double StateDotMETA[NSTATE];
  body = 0;
  StateDotMETA[body * NSTATE + 6] = udotMETA;
  StateDotMETA[body * NSTATE + 7] = vdotMETA;
  StateDotMETA[body * NSTATE + 8] = wdotMETA;
  StateDotMETA[body * NSTATE + 9] = pdotMETA;
  StateDotMETA[body * NSTATE + 10] = qdotMETA;
  StateDotMETA[body * NSTATE + 11] = rdotMETA;

  //At this point we have uvwdot and pqrdot at the composite cg. It is now time to 
  //compute uvwdot and pqrdot at each of the aircraft locations
  //I'm pretty sure if ptpi=ptpMETA and pqri=pqrMETA than pqrdoti=pqrdotMETA
  //As for uvwdot I would imagine you simply derivate the two points fixed on a rigid body and get
  //Hang on. Gonna do this on paper.
  //Vacdot = Vcgdot - rcg2ac X omegadot
  for (body = 0;body < BODIES;body++){
     StateDot[body * NSTATE + 6] = udotMETA - (RCG2AC[1][body]*rdotMETA);
     StateDot[body * NSTATE + 7] = vdotMETA - (-RCG2AC[0][body]*rdotMETA);
     StateDot[body * NSTATE + 8] = wdotMETA - (-RCG2AC[1][body]*pdotMETA+RCG2AC[0][body]*qdotMETA);
     StateDot[body * NSTATE + 9] = pdotMETA;
     StateDot[body * NSTATE + 10] = qdotMETA;
     StateDot[body * NSTATE + 11] = rdotMETA;
  }

}

void Stability_Analysis() {
	int ii, jj, kk;
	double *StateDot0 = vecallocatedbl(BODIES * NSTATE);
	printf("Computing Linear Matrix \n");
	printState(State, "State");

	///REVISIT REVISIT REVISIT
	//Set all controls to zero
	// DELTHRUST[0] = 0;
	// DE[0] = 0;

	//Set some flags
	LATERAL = 1;
	VSKIP = 0;
	//Get Nominal Derivatives
	System_Derivatives();
	printState(StateDot, "StateDot");

	//Compute DA[body] and DR[body] to zero out roll moment and yaw moment
	double pdot,Qa,rho,vel,ub,wb,rdot,DET;
	rho = Density();
	for (ii = 0;ii<BODIES;ii++)
	  {
	    pdot = StateDot[ii*NSTATE + 9];
	    rdot = StateDot[ii*NSTATE + 11];
	    //pdot + pdotcontrols ~= L/Ixx + Lcontrols/Ixx
	    //rdot + rdotcontrols ~= N/Izz + Ncontrols/Izz
	    //L = Qa*S*b*(C_l_da*DA + C_l_dr*DR)
	    //N = Qa*S*b*(C_n_da*DA + C_n_dr*DR)
	    ub = State[ii*NSTATE + 6];
	    wb = State[ii*NSTATE + 8];
	    vel = sqrt(pow(ub, 2.0000) + pow(wb, 2.0000));
	    Qa = rho * pow(vel, 2.00) * wspan[ii] * Sarea[ii];
	    DET = 1/(C_l_da*C_n_dr - C_l_dr*C_n_da);
	    DA[ii] = DET*(-2/Qa)*(C_n_dr*pdot*Iner[ii][0][0] - C_l_dr*rdot*Iner[ii][2][2]);
	    DR[ii] = DET*(-2/Qa)*(-C_n_da*pdot*Iner[ii][0][0] + C_l_da*rdot*Iner[ii][2][2]);
	    printf("Controls(DA,DR) = %lf %lf \n",DA[ii],DR[ii]);
	  }

	if (AEROMODEL)
	  {
	    printf("uvw_induced_B = \n");
	    for (ii = 0;ii<NPANELS*BODIES;ii++)
	      {
		printf("%0.20f %0.20f %0.20f \n",uvw_induced_B[0][ii],uvw_induced_B[1][ii],uvw_induced_B[2][ii]);
	      }
	  }
	// PAUSE();

	// //Recompute Derivatives
	System_Derivatives();
	printState(StateDot, "StateDot");
	
	//Compute Cost
	double udot, wdot, qdot, cost = 0;
	for (ii = 0; ii < BODIES; ii++) {
		udot = StateDot[ii * NSTATE + 6];
		wdot = StateDot[ii * NSTATE + 8];
		qdot = StateDot[ii * NSTATE + 10];
		cost = cost + sqrt(udot * udot + wdot * wdot + qdot * qdot);
	}
	printf("Cost = %lf \n", cost);
	printState(StateDot, "StateDot");
	veceq(StateDot0, StateDot, BODIES * NSTATE);
	double **ALINEAR = matrixallocatedbl(NSTATE * BODIES, NSTATE * BODIES);
	double **BLINEAR = matrixallocatedbl(NSTATE * BODIES, BODIES * 4);
	double DEL = 1e-6; //DEL = 1e-6
	///FORWARD DIFFERENCING

	for (ii = 0; ii < NSTATE * BODIES; ii++) {
	  // printf("ii = %d \n",ii);
		//Perturb the state vector
		State[ii] = State[ii] + DEL;

		//Compute State Derivatives
		System_Derivatives();

		//Compute Column of A matrix
		for (jj = 0; jj < NSTATE * BODIES; jj++) {
			ALINEAR[jj][ii] = (StateDot[jj] - StateDot0[jj]) / DEL;
		}
		// printState(StateDot,"StateDot");
		//Revert State Back to normal
		State[ii] = State[ii] - DEL;
	}

	////MIDPOINT METHOD
	// for (ii = 0; ii < NSTATE * BODIES; ii++) {
	//   // printf("ii = %d \n",ii);
	// 	//Perturb the state vector
	// 	State[ii] = State[ii] - DEL;

	// 	//Compute State Derivatives
	// 	System_Derivatives();

	// 	//Save StateDot into StateDot0
	// 	veceq(StateDot0, StateDot, BODIES * NSTATE);

	// 	//Perturb State vector forward
	// 	State[ii] = State[ii] + 2*DEL;

	// 	//Compute System Derivatives
	// 	System_Derivatives();

	// 	//Compute Column of A matrix
	// 	for (jj = 0; jj < NSTATE * BODIES; jj++) {
	// 	  ALINEAR[jj][ii] = (StateDot[jj] - StateDot0[jj]) / (2*DEL);
	// 	}
	// 	// printState(StateDot,"StateDot");
	// 	//Revert State Back to normal
	// 	State[ii] = State[ii] - DEL;
	// }

	for (ii = 0; ii < BODIES; ii++) {
		//iith body
		for (jj = 0; jj < 4; jj++) {
			//Perturb State and obtain new state derivatives
			switch (jj) {
			case 0:
				//de
				DE[ii] = DE[ii] + DEL;
				System_Derivatives();
				DE[ii] = DE[ii] - DEL;
				break;
			case 1:
				//da
				DA[ii] = DA[ii] + DEL;
				System_Derivatives();
				DA[ii] = DA[ii] - DEL;
				break;
			case 2:
				//dr
				DR[ii] = DR[ii] + DEL;
				System_Derivatives();
				DR[ii] = DR[ii] - DEL;
				break;
			case 3:
				//dthrust
				DELTHRUST[ii] = DELTHRUST[ii] + DEL;
				System_Derivatives();
				DELTHRUST[ii] = DELTHRUST[ii] - DEL;
				break;
			}
			//Compute Column of B matrix
			for (kk = 0; kk < NSTATE * BODIES; kk++) {
				BLINEAR[kk][4 * ii + jj] = (StateDot[kk] - StateDot0[kk]) / DEL;
			}
		}
	}

	//Record Nominal Configuration and A matrix
	for (ii = 0; ii < NSTATE * BODIES; ii++) {
		fprintf(nominalfile, "%f \n", State[ii]);
		for (jj = 0; jj < NSTATE * BODIES; jj++) {
			fprintf(linearfile, "%f ", ALINEAR[ii][jj]);
		}
		fprintf(linearfile, "\n");
	}
	for (ii = 0; ii < NSTATE * BODIES; ii++) {
		for (jj = 0; jj < 4 * BODIES; jj++) {
			fprintf(linearfile, "%f ", BLINEAR[ii][jj]);
		}
		fprintf(linearfile, "\n");
	}

	for (ii = 0; ii < BODIES; ii++) {
		//Output Controls
		printf("Controls(DT,DE,DA,DR) = %lf %lf %lf %lf \n", DELTHRUST[ii],
				DE[ii], DA[ii], DR[ii]);
		fprintf(nominalfile,"%lf \n %lf \n %lf \n %lf \n", DELTHRUST[ii],
				DE[ii], DA[ii], DR[ii]);
	}
	}

void Runge_Kutta(int irun) {
	int i, j, k, counter, threshold, contact, length, linearize, linearizePSI;
	double dt, dtContact, dtNoContact, summ, rkalfa[4], krkbody[N][4], tNominal,
			sNominal[N];
	/* Contact Analysis Variables */
	int ii, jj, index, numChange, Stationary, UpdateContact[NVERTS];
	double BigTime, SmallTime, PrevTime, tt[NVERTS], xp, yp, zp, tau, tprev;
	double PrevState[N];
	double x, y, z, px, py, pz, distance, crash, windnorm;

	//printf("Time Simulation Started \n");
	#ifdef JOYSTICK
	int joy_fd, *axis=NULL,num_of_axis=0,num_of_buttons=0;
	char *button = NULL,name_of_joystick[NAME_LENGTH];
	struct js_event js;
	cout << "Let's try and open the joystick \n";
	if( ( joy_fd = open(JOY_DEV,O_RDONLY)) == -1 ) {
	  cout << "Couldn't open joysticks \n";
	  exit(1);
	} else {
	  cout << "Success!!! \n";
	}
	cout << "Now let's get information from the joystick \n";
	ioctl(joy_fd,JSIOCGAXES,&num_of_axis);
	cout << "Number of axes = " << num_of_axis << endl;
	ioctl(joy_fd,JSIOCGBUTTONS,&num_of_buttons);
	cout << "Number of buttons = " << num_of_buttons << endl;
	ioctl(joy_fd,JSIOCGNAME(NAME_LENGTH),&name_of_joystick);
	cout << "Name of Joystick = " << name_of_joystick << endl;
	cout << "Allocating Memory for Joysticks. Cross your fingers" << endl;
	axis = (int *) calloc(num_of_axis,sizeof(int));
	button = (char *) calloc(num_of_buttons,sizeof(int));
	cout << "Success!!!! \n";
	cout << "Setting non-blocking mode \n";
	fcntl(joy_fd,F_SETFL,O_NONBLOCK);
	#endif

	//Record Initial State to File
	if (irun == MCRUN)
		Record_State();

	/* Define the Runge-Kutta Constants */
	rkalfa[0] = 1.00000000;
	rkalfa[1] = 2.00000000;
	rkalfa[2] = 2.00000000;
	rkalfa[3] = 1.00000000;

	//Initialize Parameters
	counter = 0;
	threshold = record_incr;
	NEXT = 0;

	//Initialize Sensor Errors
	SensorErrors();

	//Initialize Derivatives
	#ifdef JOYSTICK
	System_Derivatives_META();
	#else
	System_Derivatives();
	#endif
	
 	//Used for tracking time 
	#ifdef RUN_VISUALIZER
	TIMER track_time; //Timer to track time this TIMER class has a constructor
	//function that initializes time
	double time_since_start;
	int itime = 0;
	MATLAB state_XYZ,state_PTP; //variables for xyz and ptp
	state_XYZ.zeros(3,1,"XYZ");
	state_PTP.zeros(3,1,"PTP");
	int idx;
	double xVis,yVis,zVis,phiVis,thetaVis,psiVis;
	#endif

	while (TCURRENT < TFINAL) {

	  //Read Joystick State
	  #ifdef JOYSTICK
	  read(joy_fd,&js,sizeof(struct js_event));
	  switch (js.type & ~JS_EVENT_INIT) 
	    {
	    case JS_EVENT_AXIS:
	      axis[js.number] = js.value;
	      break;
	    case JS_EVENT_BUTTON:
	      button[js.number] = js.value;
	      break;
	    }
	  if (button[1] != NULL && PRESS == 0) {
	    PRESS = 1;
	    if (AUTOPILOT) {
	      cout << "AUTOPILOT OFF \n";
	      AUTOPILOT = 0;
	    } else {
	      cout << "AUTOPILOT ON \n";
	      AUTOPILOT = 1;
	    }
	  }
	  if (button[1] == NULL && PRESS == 1){
	    PRESS = 0;
	  }
	  //Link Axes to control if AUTOPILOT is off
	  double plusminus1;
	  if (!AUTOPILOT) {
	    for (int b = 0;b<BODIES;b++) {
              DEU[b] = -(10*PI/180)*axis[3]/(double)32000;
	      DAU[b] = -(10*PI/180)*axis[2]/(double)32000;
	      //DELTHRUST(1) = (MAXTHRUST-T0)/C_x_delThrust;
	      //DELTHRUST(-1) = (-T0)/C_x_DELThrust;
	      //DELTHRUST(x) = (MAXTHRUST*(x+1)/2-T0)/C_x_delThrust;
	      plusminus1 = -(axis[1]/(double)32000)-1.0; //offset by 1.0;
	      DELTHRUST[b] = (MAXTHRUST*(plusminus1+1)/2-T0)/C_x_delThrust;
	      DRU[b] = (30*PI/180)*axis[0]/(double)32000;
	    }
	  }
	  #endif
	  

	  //Compute Sensor Errors at Current State
	  SensorErrors();

	  //Compute Control if Necessary
	  //If autopilot is on run everything like normal.
	  //otherwise the control routine will set the state
	  //of the controls
	  if (AUTOPILOT) {
	    if ((counter % FCSUPDATE == 0) && (CONTROLON)) {
	      if (CONTROLTYPE == 0) {
		//PID
		ControlMETATRIM();
              }
	      if (CONTROLTYPE == 1)
		{
		  //FeedbackLinearization
		  ControlMETA();
                }
              }
          }

	  if (irun == MCRUN) {
	    ///Print Current Percentage
	    #ifdef RUN_VISUALIZER
	    if (TCURRENT > itime) {
	      printf("Simulation Time = %lf \n",TCURRENT);
              #ifdef JOYSTICK
	      cout << "Aileron Angle (deg) = " << DA[0] << endl;
	      cout << "Thrust (+-1) = " << DELTHRUST[0] << endl;
	      cout << "Elevator Angle (deg) = " << DE[0] << endl;
	      cout << "Rudder Angle (deg) = " << DR[0] << endl;
	      #endif
	      itime++;
	    }
	    #else
	    if (100 * TCURRENT / Time[1] > NEXT) {
	      NEXT += 1;
	      /* Display How Far Along */
	      printf("Time=%lf \n", TCURRENT);
	      printf("Time Simulation is %i Percent Complete \n", NEXT);
	    }
	    #endif
	    
	  }

	  /* Store Nominal State Values */
	  tNominal = TCURRENT;
	  for (j = 0; j < N; j++) {sNominal[j] = State[j];}

	  //Store Prev Contact States
	  if (ICONTACT == 1) {
	    PrevTime = TCURRENT;
	    for (j = 0; j < N; j++) {PrevState[j] = State[j];}
	    for (ii = 0; ii < IVERTS; ii++) {
	        PrevContactList[0][ii] = ContactList[0][ii];
		PrevContactList[1][ii] = ContactList[1][ii];
		PrevContactList[2][ii] = ContactList[2][ii];
		PrevContactList[3][ii] = ContactList[3][ii];
	    }
  	  }

	  /* 4th Order Runge-Kutta Integration */
	  for (j = 0; j < 4; j++) {

	      /* State Values to Evaluate Derivatives */
    	      if (j != 0) {
          	  TCURRENT = tNominal + TIMESTEP / rkalfa[j];
		  for (k = 0; k < N; k++) {State[k] = sNominal[k] + krkbody[k][j - 1] / rkalfa[j];}
	      }

	      /* Compute Derivatives at Current Value */
	      #ifdef JOYSTICK
	      System_Derivatives_META();
              #else
	      System_Derivatives();
              #endif

	      /* Runge-Kutta Constants */
	      for (k = 0; k < N; k++) {	krkbody[k][j] = TIMESTEP * StateDot[k];}
	   }

	   /* Step Time */
	   TCURRENT = tNominal + TIMESTEP;

	  /* Step States */
	  for (j = 0; j < N; j++) {
	     summ = 0.00000000;
	     for (k = 0; k < 4; k++) {summ = summ + rkalfa[k] * krkbody[j][k];}
	     State[j] = sNominal[j] + summ / 6.00000000;
	  }

	  /* Step Controls */
	  for (j = 0; j < BODIES; j++) {
	     double sigma;
	     sigma = 600*0.001/TIMESTEP;
	     DE[j] = DE[j] + TIMESTEP * (sigma * (DEU[j] - DE[j]));
	     DA[j] = DA[j] + TIMESTEP * (sigma * (DAU[j] - DA[j]));
	     DR[j] = DR[j] + TIMESTEP * (sigma * (DRU[j] - DR[j]));
	     DELTHRUST[j] = DELTHRUST[j] + TIMESTEP * (sigma * (DELTHRUSTU[j] - DELTHRUST[j]));
	  }

	  //Record State
	  if (irun == MCRUN) {
	    counter++;
	    if (counter >= threshold) {
	      Record_State();
	      threshold = threshold + record_incr;
	    }
	   }

	  //Check for New Contacts
	  if (ICONTACT == 1) {
	    NewContacts(PrevTime, PrevState);
	  }

	  ////Interaction Check (DON'T NEED THIS ANYMORE)
	  //This code was build out the MMACSV10 which involved a soft
	  //contact model. This is no longer used for the multibody
	  //simulation
	  // if (NBODIES == 2) {
	  //   InteractionCheck();
	  // }

	  #ifdef RUN_VISUALIZER
	  //Check for real time clock 
	  time_since_start = track_time.getTimeSinceStart();
	  if (TCURRENT < time_since_start-1e-2) {
	    printf("Computation too slow, increase your timestep \n");
	  }
	  while (TCURRENT > time_since_start) {
	    // printf("Computation too fast - Sleeping for %lf seconds \n",timestep);
	    // printf("Time = %lf \n",time);
	    // printf("Time since start = %lf \n",time_since_start);
	    cross_sleep(TIMESTEP);
	    time_since_start = track_time.getTimeSinceStart();
	  }

	  ///This is where we will pass the state of all bodies to the statetime
	  //variable
	  //Set the state of the ground and sky planes
	  state_XYZ.mult_eq(0);
	  state_PTP.mult_eq(0);
	  //Shift ground plane and skydome down 10 feet
	  state_XYZ.plus_eq(10.0);
	  statetime.setState(state_XYZ,state_PTP,1); //ground
	  statetime.setState(state_XYZ,state_PTP,2); //sky
	  statetime.setTime(TCURRENT);
	  for (idx = 0;idx < BODIES;idx++) {
	    xVis = State[idx*NSTATE];
	    yVis = State[idx*NSTATE + 1];
	    zVis = State[idx*NSTATE + 2];
	    phiVis = State[idx*NSTATE + 3];
	    thetaVis = State[idx*NSTATE + 4];
	    psiVis = State[idx*NSTATE + 5];
	    state_XYZ.set(1,1,xVis); state_XYZ.set(2,1,yVis); state_XYZ.set(3,1,zVis);
	    state_PTP.set(1,1,phiVis); state_PTP.set(2,1,thetaVis); state_PTP.set(3,1,psiVis);
	    statetime.setState(state_XYZ,state_PTP,idx+1+2);
	    // state_XYZ.disp();
	    // state_PTP.disp();
	  }
	  // if (NBODIES == 2) {
	  //   InteractionCheck();
	  // }

	  #endif

	}

	if (IMONTECARLO) { Final_Check(); }
}

int ImportState(double State[]) {
	int body, ii, jj, xdistance, ydistance, rows, cols;
	double sphi, ctheta, cphi, stheta, spsi, cpsi, ttheta, TBInoise[3][3];
	double u, v, w, dely,CG_6DOF[2],thickness,rfactor;
	double plusorminus = 1, xlocation, ylocation;
	char dummy[256];

	if (ICONTACT == 1) {
	  dely = 2.06;
	} else {
	  dely = 2.04;
	}
	
	CG_6DOF[0] = 0;
	CG_6DOF[1] = 0;

	u = Vtrim;
	v = 0;
	w = 0;
	body = BODIES - 1;
	int bodyflag;
	//printf("(Body index, X, Y) \n");
	//Read in rows and columns
	fscanf(ldfile, "%d %d %s ", &rows, &cols, dummy);
	//printf("rows = %d cols = %d \n", rows, cols);
	for (jj = 1; jj <= rows; jj++) {
	  for (ii = 0; ii < cols; ii++) {
	    fscanf(ldfile, "%d ", &bodyflag);
	    //printf("bodyflag = %d \n",bodyflag);
	    ylocation = (double) ii * dely;
	    //xlocation = (double)(rows-1)*2;
	    xlocation = -(double) (jj - 1) * 2.04;
	    if (bodyflag == 1) {
	      if (ii+1 > NUM_COL)
		{
		  NUM_COL++;
		}
	      if (jj + 1 > NUM_ROW && ii == 0)
		{
		  NUM_ROW++;
		}
	      body++;
	      State[body * NSTATE] = xlocation;
	      State[body * NSTATE + 1] = ylocation;
	      //printf("%d %f %f \n", body, xlocation, ylocation);
	      /////KEEP THE SAME
	      //INITIAL CONDITIONS OF ALL AIRCRAFT
	      //Initial Conditions
	      //Initial State Vector
	      //REVISIT REVISIT REVISIT
	      //Hardcoded Initial State
	      #ifdef JOYSTICK
	      State[body * NSTATE + 2] = -2; //z
	      #else
	      State[body * NSTATE + 2] = -200; //z
	      #endif
	      State[body * NSTATE + 3] = 0 * PI / 180; //phi
	      State[body * NSTATE + 5] = 0 * PI / 180; //psi
	      State[body * NSTATE + 6] = u; //u
	      State[body * NSTATE + 7] = v; //v
	      State[body * NSTATE + 8] = w; //w
	      State[body * NSTATE + 9] = 0; //p
	      State[body * NSTATE + 10] = 0; //q
	      State[body * NSTATE + 11] = 0; //r
	      DR[body] = 0;
	      DE[body] = 0;
	      DELTHRUST[body] = 0;
	      DA[body] = 0;
	      /* Transformation Matrix (Transformation from I to B frame) */
	      ctheta = cos(0);
	      stheta = sin(0);
	      ttheta = stheta / ctheta;
	      cphi = cos(0);
	      sphi = sin(0);
	      spsi = sin(0);
	      cpsi = cos(0);
	      TBI[body][0][0] = ctheta * cpsi;
	      TBI[body][0][1] = ctheta * spsi;
	      TBI[body][0][2] = -stheta;
	      TBI[body][1][0] = sphi * stheta * cpsi - cphi * spsi;
	      TBI[body][1][1] = sphi * stheta * spsi + cphi * cpsi;
	      TBI[body][1][2] = sphi * ctheta;
	      TBI[body][2][0] = cphi * stheta * cpsi + sphi * spsi;
	      TBI[body][2][1] = cphi * stheta * spsi - sphi * cpsi;
	      TBI[body][2][2] = cphi * ctheta;
	      /* Transformation Matrix from B to I Frame */
	      TIB[body][0][0] = TBI[body][0][0];
	      TIB[body][0][1] = TBI[body][1][0];
	      TIB[body][0][2] = TBI[body][2][0];
	      TIB[body][1][0] = TBI[body][0][1];
	      TIB[body][1][1] = TBI[body][1][1];
	      TIB[body][1][2] = TBI[body][2][1];
	      TIB[body][2][0] = TBI[body][0][2];
	      TIB[body][2][1] = TBI[body][1][2];
	      TIB[body][2][2] = TBI[body][2][2];
	      //Compute Save x and y locations of bodies
	      xyzAC[0][body] = xlocation;
	      xyzAC[1][body] = ylocation;
	    }
	  }
	}

	///Compute CG of 6DOF system assuming everyone weighs the same
	for (ii = 0;ii<body+1;ii++)
	  {
	    CG_6DOF[0] = CG_6DOF[0] + (1/((double)body+1))*xyzAC[0][ii];
	    CG_6DOF[1] = CG_6DOF[1] + (1/((double)body+1))*xyzAC[1][ii];
	  }
	//This is for control
	GMETA = matzeros(2,2*(body+1));
	Gt = matzeros(2*(body+1),2);
	GGt = matzeros(2,2);
	GGtINV = matzeros(2,2);
	GGtINVUFL = matzeros(2,1);
	METACONTROLS = matzeros(2*(body+1),1);
	FMETA = matrixallocatedbl(2,1);
	Gamma_META = matrixallocatedbl(2,1);
	UFL = matrixallocatedbl(2,1);
	I_b_6DOF = matrixallocatedbl(3,3);
	for (ii = 0;ii<body+1;ii++)
	  {
	    RCG2AC[0][ii] = xyzAC[0][ii] - CG_6DOF[0];
	    RCG2AC[1][ii] = xyzAC[1][ii] - CG_6DOF[1];
	  }
	
	//Read Spring and Damping Properties
	fscanf(ldfile, "%lf %lf %s", &KL[0], &KL[1], dummy);
	fscanf(ldfile, "%lf %lf %s", &CL[0], &CL[1], dummy);
	KL[2] = KL[0]; //Tangent forces
	CL[2] = CL[0];
	//KR[0] and CR[0] are function of KL[1]
	thickness = 0.12*c;
	//REVISIT REVISIT REVISIT (Crank up rotational springs to make it act more rigid)
	rfactor = 10;
	fscanf(ldfile, "%lf %s", &Fconn, dummy);
	///Try to import KR 
	KR[0] = 0;KR[1] = 0;KR[2] = 0;
	fscanf(ldfile, "%lf %lf %lf %s", &KR[0], &KR[1], &KR[2],dummy);
	//Try to read CR as well
	CR[0] = 0;CR[1] = 0;CR[2] = 0;
	fscanf(ldfile, "%lf %lf %lf %s", &CR[0], &CR[1], &CR[2],dummy);	
	if (KR[0] <= 0 && KR[1] <= 0 && KR[2] <= 0)
	  {
	    printf("Using Rfactor = %lf \n",rfactor);
	    KR[0] = 10 * KL[1] * rfactor*pow(thickness,2)/4;
	    CR[0] = 10 * CL[1] * rfactor*pow(thickness,2)/4;
	    //KR[1] and CR[1] are actually functions of KL[0] and KL[2] which are identical
	    KR[1] = KL[2] * rfactor*pow(c,2)/4;
	    CR[1] = CL[2] * rfactor*pow(c,2)/4;
	    //KR[2] and CR[2] are functions of KL[1]
	    KR[2] = KL[1] * rfactor*pow(c,2)/4;;
	    CR[2] = CL[1] * rfactor*pow(c,2)/4;;
	  }
	
	printf("KL = %lf %lf %lf \n",KL[0],KL[1],KL[2]);
	printf("CL = %lf %lf %lf \n",CL[0],CL[1],CL[2]);
	printf("KR = %lf %lf %lf \n",KR[0],KR[1],KR[2]);
	printf("CR = %lf %lf %lf \n",CR[0],CR[1],CR[2]);
	// PAUSE();
	if (body > NBODIES) {
		printf("Number of Bodies Exceeded \n");
		exit(1);
	}
	return (body + 1);
}

void Master_Init() {
	int ii, jj, nn;
	double det;

	//printf("Master Initialization \n");

	//Figure out how many bodies there actually are
	BODIES = 0;
	ldfile = fopen(CONNFILE, "r");
	if (ldfile == NULL) {
	    file_error(CONNFILE,"CONNFILE");
	}
	BODIES = ImportState(State);
	METABODIES = BODIES;
	//Generate the Connection Matrix and Joint Locations
	double d, dj, xd, yd;
	int idx = BODIES - 1, skipbody = -1, jdx, ll, flag = 1, CONNECTIONTYPES[2];
	double CONNIDX[NBODIES][NBODIES];
	CONNECTIONTYPES[0] = 0;
	CONNECTIONTYPES[1] = 0;
	NUMCONNECTIONS = 0;
	for (ii = 0; ii < BODIES; ii++) {
		//Figure out which bodies the ith body is connected to.
		for (jj = 0; jj < BODIES; jj++) {
			if (jj != ii) {
				xd = State[ii * NSTATE + 0] - State[jj * NSTATE + 0];
				yd = State[ii * NSTATE + 1] - State[jj * NSTATE + 1];
				dj = sqrt(xd * xd + yd * yd);
				//Only search for front to back or left to right
				if (dj <= 2.05) {
					//printf("Connection Found %d %d \n",ii,jj);
					//See if this connection already exists
					int connectiondne = 1;
					for (ll = 0; ll < NUMCONNECTIONS; ll++) {
						if ((CONNECTMAT[ll][0] == ii)
								&& (CONNECTMAT[ll][1] == jj)) {
							connectiondne = 0;
						}
					}
					if (connectiondne) {
						//you have found a connection
						NUMCONNECTIONS++;
						CONNECTMAT[NUMCONNECTIONS - 1][0] = jj;
						CONNECTMAT[NUMCONNECTIONS - 1][1] = ii;
						///Figure out what kind of connection this is
						rB_Joint[NUMCONNECTIONS - 1][0][0] = (State[ii * NSTATE
								+ 0] - State[jj * NSTATE + 0]) / 2;
						rB_Joint[NUMCONNECTIONS - 1][0][1] = (State[ii * NSTATE
								+ 1] - State[jj * NSTATE + 1]) / 2;
						rB_Joint[NUMCONNECTIONS - 1][0][2] = (State[ii * NSTATE
								+ 2] - State[jj * NSTATE + 2]) / 2;
						if (rB_Joint[NUMCONNECTIONS - 1][0][0] == 0) {
							//Wingtip to Wingtip
							CONNECTIONTYPES[0] = 1;
						}
						if (rB_Joint[NUMCONNECTIONS - 1][0][1] == 0) {
							//Tip to tail
							CONNECTIONTYPES[1] = 1;
						}
						rB_Joint[NUMCONNECTIONS - 1][1][0] =
								-rB_Joint[NUMCONNECTIONS - 1][0][0];
						rB_Joint[NUMCONNECTIONS - 1][1][1] =
								-rB_Joint[NUMCONNECTIONS - 1][0][1];
						rB_Joint[NUMCONNECTIONS - 1][1][2] =
								-rB_Joint[NUMCONNECTIONS - 1][0][2];
					}
				}
			}
		}
	}
	//printf("Connect Matrix \n");
	// for (ii = 0; ii < NUMCONNECTIONS; ii++) {
	// 	printf("%d %d \n", CONNECTMAT[ii][0], CONNECTMAT[ii][1]);
	// }
	//Figure out what contact model to use
	connectiontype = CONNECTIONTYPES[0] + CONNECTIONTYPES[1];
	if (connectiontype != 2) {
		if (CONNECTIONTYPES[0] == 1 || BODIES == 1) {
			connectiontype = 0;
			//printf("Wingtip to Wingtip Contact Model \n");
		} else {
			connectiontype = 1;
			//printf("Tip to Tail Contact Model \n");
		}
	} else {
	  //printf("Poor Mans Contact Model \n");
	}
	//PAUSE();

	//Time Initialization
	TINITIAL = Time[0];
	TIMESTEP = Time[2];
	//Compute Mass, Inertia,Span, and area
	Iner[0][1][0] = Iner[0][0][1];
	Iner[0][2][0] = Iner[0][0][2];
	Iner[0][2][1] = Iner[0][1][2];
	for (nn = 0; nn < BODIES; nn++) {
		mass[nn] = WEIGHT / GRAVITY;
		wspan[nn] = wspan[0];
		Sarea[nn] = Sarea[0];
		for (ii = 0; ii < 3; ii++) {
			for (jj = 0; jj < 3; jj++) {
				Iner[nn][ii][jj] = Iner[0][ii][jj];
			}
		}
		/* Calculate the Inverse of the Mass Moment of Inertia Matrix */
		det = Iner[nn][0][1] * Iner[nn][1][2] * Iner[nn][2][0]
				- Iner[nn][0][2] * Iner[nn][1][1] * Iner[nn][2][0]
				+ Iner[nn][0][2] * Iner[nn][1][0] * Iner[nn][2][1]
				- Iner[nn][0][0] * Iner[nn][1][2] * Iner[nn][2][1]
				+ Iner[nn][0][0] * Iner[nn][1][1] * Iner[nn][2][2]
				- Iner[nn][0][1] * Iner[nn][1][0] * Iner[nn][2][2];
		InerInv[nn][0][0] = (Iner[nn][1][1] * Iner[nn][2][2]
				- Iner[nn][1][2] * Iner[nn][2][1]) / det;
		InerInv[nn][0][1] = (Iner[nn][0][2] * Iner[nn][2][1]
				- Iner[nn][0][1] * Iner[nn][2][2]) / det;
		InerInv[nn][0][2] = (Iner[nn][0][1] * Iner[nn][1][2]
				- Iner[nn][0][2] * Iner[nn][1][1]) / det;
		InerInv[nn][1][0] = (Iner[nn][1][2] * Iner[nn][2][0]
				- Iner[nn][1][0] * Iner[nn][2][2]) / det;
		InerInv[nn][1][1] = (Iner[nn][0][0] * Iner[nn][2][2]
				- Iner[nn][0][2] * Iner[nn][2][0]) / det;
		InerInv[nn][1][2] = (Iner[nn][0][2] * Iner[nn][1][0]
				- Iner[nn][0][0] * Iner[nn][1][2]) / det;
		InerInv[nn][2][0] = (Iner[nn][1][0] * Iner[nn][2][1]
				- Iner[nn][1][1] * Iner[nn][2][0]) / det;
		InerInv[nn][2][1] = (Iner[nn][0][1] * Iner[nn][2][0]
				- Iner[nn][0][0] * Iner[nn][2][1]) / det;
		InerInv[nn][2][2] = (Iner[nn][0][0] * Iner[nn][1][1]
				- Iner[nn][0][1] * Iner[nn][1][0]) / det;
	}//for nn = 0

	///Now Compute vectors from cg to a/c aero center
	//We must also compute the change in Inertia
        //And the total Mass
	double ** s = matrixallocatedbl(3,3);
	double ** stranspose = matrixallocatedbl(3,3);
	double ** sst = matrixallocatedbl(3,3);
	double ** I_cg_ii = matrixallocatedbl(3,3);
	double ** I_b_1 = matrixallocatedbl(3,3);
	for (ii = 0;ii<3;ii++)
	  {
	    for (jj = 0;jj<3;jj++)
	      {
	    	I_b_1[ii][jj] = Iner[0][ii][jj];
	      }
	  }
	//matrixdisp(I_b_1,3,3,"I_b_1");
	for (ii = 0;ii<BODIES;ii++)
	  {
	    massMETA += mass[ii];
	    ///Compute skew(rcg2ac)
	    s[0][0] = 0;s[0][1] = 0;s[0][2] = RCG2AC[1][ii];
	    s[1][0] = 0;s[1][1] = 0;s[1][2] = -RCG2AC[0][ii];
	    s[2][0] = -RCG2AC[0][ii];s[2][1] = RCG2AC[1][ii];s[2][2] = 0;
	    //Compute s*s'
	    transpose_pre(stranspose,s,3,3);
	    matmult(sst,s,3,3,stranspose,3,3);
	    matplusminus(I_cg_ii,I_b_1,sst,mass[0],3,3);
	    matplusminus(I_b_6DOF,I_b_6DOF,I_cg_ii,1.00,3,3);
	  }
	
	//We need to invert I_b_6DOF. For simplicity we shall assume that I_b_6DOF is diagonal
	Inv_b_6DOF = matzeros(3,3);
	for (ii = 0;ii<3;ii++){
	  Inv_b_6DOF[ii][ii] = 1/I_b_6DOF[ii][ii];
	}

	//Check FCSUPDATE
	if (FCSUPDATE <= 0)
		FCSUPDATE = 1;
	//Set maximum Thrust
	MAXTHRUST = 200;
	//VORTEX MODEL CHECK
	if (VORTEXMODEL) {
		AEROMODEL = 1;
	};
	//Allocate Panels for Panel Code
	NPANELS = 1;
	//Stability Analysis Check
	if ((MODE == 1) && (ICONTACT != 1)) {
		LATERAL = 0;
	};

	if (AEROMODEL) {
		//Allocate Memory
		AllocatePanels();
		//Compute Location of Panels
		SetupPanels();
		//Compute Aerodynamic Split
		//We have a problem. The split is computed
		//before the computation of the split
		//That is AeroSplit runs before AeroFit().
		//Can we put a break statement in here or something?
		AeroSplit();
	}

	//Initialize Magnets
	if (ICONTACT != 1) {
		IMAGNET = 0;
	} else {
		rcg_magnet[0] = 0;
		rcg_magnet[1] = wspan[0] / 2;
		rcg_magnet[2] = 0;
		rcg1_magnet[0] = 0;
		rcg1_magnet[1] = -wspan[0] / 2;
		rcg1_magnet[2] = 0;
		//Compute Maximum Magnetic Force
		MAGNETSCALE = 1;
		msize[0] = MAGNETSCALE * msizeI[0];
		msize[1] = MAGNETSCALE * msizeI[1];
		msize[2] = MAGNETSCALE * msizeI[2];
		double F[3], T[3];
		double z = msize[2] + 0.0001;
		Fconn = MagForce(F, T, 0, 0, z, IMAGNET);
	}
	// printf("IMAGNET = %d \n",IMAGNET);
	// PAUSE();

	if (ICONTACT == 1) {
		xobjL = obj[0];
		xobjR = obj[1];
		yobjL = obj[2];
		yobjR = obj[3];
		zobjL = obj[4];
		zobjR = obj[5];
		aobj = xobjR - xobjL;
		bobj = yobjR - yobjL;
		cobj = zobjR - zobjL;
		thetal = atan2(bobj, aobj);
		thetar = PI - thetal;
		for (ii = 0; ii < NVERTS; ii++) {
			rcg_vertex[0][ii] = rVertex[0][ii];
			rcg_vertex[1][ii] = rVertex[1][ii];
			rcg_vertex[2][ii] = rVertex[2][ii];
		}
	}

	///WIND FIELD SETUP
	//Now we must allocate memory for WIND
	WIND = matrixallocatedbl(3, NPANELS * NBODIES);
	WINDB = matrixallocatedbl(3, NPANELS * NBODIES);
	uOLD = vecallocatedbl(NPANELS * NBODIES);
	vOLD = vecallocatedbl(NPANELS * NBODIES);
	wOLD = vecallocatedbl(NPANELS * NBODIES);
	if ((IWRF) || (IMONTECARLO)) {
		UVWSTARTUP();
	}
	if ((ITURB) || (IMONTECARLO)) {
		UVWTURBSTARTUP();
	}
	//MonteCarlo Setup
	if (!IMONTECARLO) {
		NUMMC = 0;
		MCRUN = 0;
	} else {
		printf("---------------------------\n");
		printf("Running Case %d from Monte Carlo File \n", MCRUN);
		if (MCRUN != 0)
			NUMMC = MCRUN;
	}

	//Get composite cg matrices
	double xMETA = 0, yMETA = 0, zMETA = 0;
	double xelev, yelev, zelev, x, y, z;
	for (ii = 0; ii < METABODIES; ii++) {
		xMETA = xMETA + (1 / (double) METABODIES) * State[ii * NSTATE];
		yMETA = yMETA + (1 / (double) METABODIES) * State[ii * NSTATE + 1];
		zMETA = zMETA + (1 / (double) METABODIES) * State[ii * NSTATE + 2];
	}
	//Composite cg is now x,y,zMETA
	for (ii = 0; ii < METABODIES; ii++) {
		//Compute location of elevator
		x = State[ii * NSTATE];
		y = State[ii * NSTATE + 1];
		z = State[ii * NSTATE + 2];
		elevGx[ii] = (x - xMETA) + 1;
		yelev = y;
		zelev = z;
		elevG[ii] = yMETA - y;
	}
	for (ii = 0; ii < METABODIES; ii++) {
		idx = (ii) * 2;
		GMIX[idx][0] = 1; //ua
		GMIX[idx][1] = 0;
		GMIX[idx + 1][0] = elevG[ii] / (wspan[0]); //uphie1
		GMIX[idx + 1][1] = elevGx[ii]; //ue
	}
	
}

void Init() {
	int ii, jj;

	//printf("Current Run Initialization \n");

	//Reset Time
	TCURRENT = TINITIAL;
	TFINAL = Time[1];
	//Aero Model Stuff
	if (VORTEXMODEL) {
		VCOUNTER = VSKIP;
		COLDSTART = 1;
		//Reset induced velocity and Gammaiprev
		for (ii = 0; ii < NPANELS * BODIES; ii++) {
			//assume initial condition is -Vinf*alfa
			uvw_induced_B[0][ii] = 0;
			uvw_induced_B[1][ii] = 0;
			uvw_induced_B[2][ii] = (double) 0.1509;
			Gammaiprev[ii] = 0;
		}
	}
	//Magnet Stuff
	MAGDISTANCE = 100;
	MAGCONSTANT = 0;
	MAXF = 0;
	//Contact Model Stuff
	GOODCONTACT = 0;
	CONNECTION = 0;
	TCONTACT = 0;
	numContacts = 0;
	tConnect = 0;
	if (ICONTACT == 1) {
		int i, j;
		/* Zero out Contact State */
		for (i = 0; i < IVERTS; i++) {
			for (j = 0; j < 3; j++) {
				State[NVRTST * i + NBODIES * NSTATE + j] = 0.0000;
				State[NVRTST * i + NBODIES * NSTATE + j + 3] = 0.0000;
				State[NVRTST * i + NBODIES * NSTATE + j + 6] = 0.0000;
				State[NVRTST * i + NBODIES * NSTATE + j + 9] = 0.0000;
			}
		}
		/* Initialize Contact List */
		for (ii = 0; ii < IVERTS; ii++) {
			ContactList[0][ii] = 0.0;
			ContactList[1][ii] = 0.0000;
			ContactList[2][ii] = 0.0000;
			ContactList[3][ii] = 0.0000;
		}
	}

	//Set Shifts to zero
	xshift = 0;
	yshift = 0;
	if (ITURB) {
		for (ii = 0; ii < NBODIES * NPANELS; ii++) {
			xshiftT[ii] = 0;
			yshiftT[ii] = 0;
		}
	}
	//Zero Wind Filters
	for (ii = 0; ii < NPANELS * NBODIES; ii++) {
		uOLD[ii] = 0;
		vOLD[ii] = 0;
		wOLD[ii] = 0;
		//Zero out WINDS
		WIND[0][ii] = 0;
		WIND[1][ii] = 0;
		WIND[2][ii] = 0;
		WINDB[0][ii] = 0;
		WINDB[1][ii] = 0;
		WINDB[2][ii] = 0;
	}
	WINDMAX = -1;
	//Control Variables
	XDELT = 0;
	YDELT = 0;
	ZDELT = 0;
	THOLD = 1;
	TY = 0;
	STARTY = 0;
	PSIY = 0;
	PSIPATH = 1;
	WAYPOINTOFFSET = 1;
	WAYPOINT = 0;
	//Zero out Controls
	for (ii = 0; ii < BODIES; ii++) {
		DE[ii] = 0;
		DR[ii] = 0;
		DELTHRUST[ii] = 0;
		DA[ii] = 0;
	}
	//Set up Parent Commands
	UCOMMAND[0] = Vtrim;
	ZCOMMAND[0] = State[2]; //Initial altitude
	MSLOPE = State[5];
	// cout << "MSLOPE = " << MSLOPE << endl;
	BINTERCEPT = State[1] - tan(MSLOPE) * State[0];
	//Reset Sensor Errors
	GPSnext = 0;
	ACCELnext = 0;
	GYROnext = 0;
	for (ii = 0; ii < 6; ii++) {
		VisNav[ii] = 0;
	}
	Sensors_Setup();
	//Reseed random number generators
	int seed = time(&rawtime);
	srand(seed);
	seed = rand()*rand()*time(&rawtime);
	srand(seed);

	///CREATE RANDOM XOFFSET AND YOFFSET
	XOFFSET = 100*randUniform();
	YOFFSET = 100*randUniform();
	PSIOFFSET = 3.14*randUniform();

	// ///REVISIT REVISIT REVISIT
	// XOFFSET = 50;
	// YOFFSET = -82;
	// PSIOFFSET = -0.6908;
	//cout << "X,Y,PSI OFFSET " << XOFFSET << " " << YOFFSET << " " << PSIOFFSET << endl;
	//cout << "what" << endl;

}

void Output_Info(int ii) {
	if ((MCRUN == 0)) {
		printf("----------------------------------------------\n");
		printf("Running Case %d out of %d \n", ii, NUMMC);
	}
	if (ICONSTANT) {
		printf("ICONSTANTSCALE = %lf %lf %lf \n", ICONSTANTSCALE[0],
				ICONSTANTSCALE[1], ICONSTANTSCALE[2]);
		printf("FREQ = %lf %lf %lf \n", FREQ[0], FREQ[1], FREQ[2]);
	}
	if (ITURB) {
		printf("TURBLEVEL = %lf %lf %lf \n", TURBLEVEL[0], TURBLEVEL[1],
				TURBLEVEL[2]); /* Load in Windscale Flag */
	}
	if (IWRF) {
		printf("IWRF = %lf %lf %lf \n", &IWRFSCALE[0], &IWRFSCALE[1],
				&IWRFSCALE[2]); /* Load in WRF Scale Flag */
	}

}

//This is for MonteCarlo
void Read_New_State() {
	int ii;
	double d;
	//printf("Reading New State \n");
	for (ii = 0; ii < NBODIES * NSTATE; ii++) {
		fscanf(mcfile, "%lf ", &d);
		State[ii] = d;
		INITIAL[ii] = State[ii];
		//printf("State = %lf \n",d);
	}
	//Reset Vtrim
	Vtrim = State[6];

	//Read Constant Winds
	fscanf(mcfile, "%d ", &ICONSTANT);
	fscanf(mcfile, "%lf %lf %lf ", &ICONSTANTSCALE[0], &ICONSTANTSCALE[1],
			&ICONSTANTSCALE[2]);
	if ((ICONSTANTSCALE[0] + ICONSTANTSCALE[1] + ICONSTANTSCALE[2]) == 0)
		ICONSTANT = 0;
	fscanf(mcfile, "%lf %lf %lf ", &FREQ[0], &FREQ[1], &FREQ[2]);

	//Read Turbulence
	fscanf(mcfile, "%d ", &ITURB);
	fscanf(mcfile, "%lf %lf %lf ", &TURBLEVEL[0], &TURBLEVEL[1], &TURBLEVEL[2]); /* Load in Windscale Flag */
	if ((TURBLEVEL[0] + TURBLEVEL[1] + TURBLEVEL[2]) == 0)
		ITURB = 0;

	//Read WRF
	fscanf(mcfile, "%d ", &IWRF); /* Load in WRF Scale Flag */
	fscanf(mcfile, "%lf %lf %lf ", &IWRFSCALE[0], &IWRFSCALE[1], &IWRFSCALE[2]); /* Load in WRF Scale Flag */
	if ((IWRFSCALE[0] + IWRFSCALE[1] + IWRFSCALE[2]) == 0)
		IWRF = 0;

}

void Output_State() {
	int ii;
	//printf("Outputting State \n");
	for (ii = 0; ii < NSTATE * NBODIES; ii++) {
		fprintf(mcoutfile, "%.12f ", INITIAL[ii]);
	}
	fprintf(mcoutfile, "%.12f %.12f %d %.12f %d %d %d \n", tConnect,
			MAGDISTANCE, ICRASH, WINDMAX, ICONSTANT, IWRF, ITURB);
}

void Percent_Connection() {
	//Compute Percent Connection
	WL =
			(Vtrim - 20)
					+ ICONSTANT
							* (100 * FREQ[0] + 100 * FREQ[1] + 100 * FREQ[2]
									+ 10 * ICONSTANTSCALE[0]
									+ 10 * ICONSTANTSCALE[1]
									+ 10 * ICONSTANTSCALE[2])
					+ IWRF
							* (10 * IWRFSCALE[0] + 10 * IWRFSCALE[1]
									+ 10 * IWRFSCALE[2])
					+ ITURB
							* (10 * TURBLEVEL[0] + 10 * TURBLEVEL[1]
									+ 10 * TURBLEVEL[2]);

	if (WL != prevWL) {
		//printf("WL = %d prevWL = %d \n",WL,prevWL);
		pconnection = 100 * numconnection / numtotal;
		PERCENT[iwl] = pconnection;
		WLMAT[iwl] = prevWL;
		iwl++;
		printf("------------------------------------------\n");
		printf("Percent Connection @ L = %d : %lf \n", prevWL, pconnection);
		if ((pconnection < 30) && (SMARTSIMULATING == 1)) {
			printf(
					"Simulation Finished Percent Connection Lower Than Threshold\n");
			irun = NUMMC;
			iwl--;
		} else {
			numconnection = 0;
			numtotal = 0;
			prevWL = WL;
		}
	}
}

void ComputeAeroDynamics(double TotalAeroLoads[])
{
  //First we run angle of attack sweeps
  int body,ii,jj,ss,sWRF,sTURB,sCONSTANT;
  double BodyAeroLoads[6],sNominal[N];
  double PState[12],C_L_0_T,C_D_0_T,Vtrim,rho,q,qhat,C_m_q_T,C_L_q_T,del;

  /////////////MAKE A SUBROUTINE FOR THIS//////////////////////

  //Make sure to set windmodel to zero
  sWRF = 0;
  sTURB = 0;
  sCONSTANT = 0;
  if (IWRF)
    {
      sWRF = 1;
      IWRF = 0;
    }
  if (ITURB)
    {
      sTURB = 1;
      ITURB = 0;
    }
  if (ICONSTANT)
    {
      sCONSTANT = 1;
      ICONSTANT = 0;
    }

  for (jj = 0;jj<6;jj++)
    {
      TotalAeroLoads[jj] = 0;
    }
  Lift_Total = 0;
  Drag_Total = 0;
  if (AEROMODEL) {
    for (body = 0;body<BODIES;body++)
      {
	VelocityPanels(body);
      }	
  }
  if (VORTEXMODEL) {
    LiftingLine(BODIES);
  }
  for (body = 0;body<BODIES;body++)
    {
      if (AEROMODEL) {
	LIFT_A[body] = 0;
	DRAG_A[body] = 0;
	PanelForces(BodyAeroLoads, body);
      } else {
	// printf("Using AeroForce() \n");
	AeroForces(BodyAeroLoads, body);
      }
      // for (ii = 0;ii<6;ii++)
      // 	{
      // 	  printf("BodyAeroLoads = %lf \n",BodyAeroLoads[ii]);
      // 	}
      Lift_Total = Lift_Total + LIFT_A[body];
      Drag_Total = Drag_Total + DRAG_A[body];
      for (jj = 0;jj<6;jj++)
	{
	  TotalAeroLoads[jj] = TotalAeroLoads[jj] + BodyAeroLoads[jj];
	}
      //Add pitch roll and yaw moments from offset c.g.
      TotalAeroLoads[3] = TotalAeroLoads[3] + RCG2AC[1][body]*BodyAeroLoads[2]; 
      TotalAeroLoads[4] = TotalAeroLoads[4] - RCG2AC[0][body]*BodyAeroLoads[2];
      TotalAeroLoads[5] = TotalAeroLoads[5] - RCG2AC[1][body]*BodyAeroLoads[0] + RCG2AC[0][body]*BodyAeroLoads[1];

      if (lodfile != NULL)
	{
	  fprintf(lodfile,"%lf %lf ",LL,DD);
	}

    }

  //But turn it back on when you're done
  if (sWRF)
    {
      IWRF = 1;
    }
  if (sTURB)
    {
      ITURB = 1;
    }
  if (sCONSTANT)
    {
      ICONSTANT = 1;
    }

  ////////////////////////////////////////////////////
}

void AeroSweeps()
{  
  //First we run angle of attack sweeps
  int body,ii,jj,ss,kk,sWRF,sTURB,sCONSTANT;
  double BodyAeroLoads[6],TotalAeroLoads[6],sNominal[N],State_Backup[N];
  double PState[12],C_L_0_T,C_D_0_T,Vtrim,rho,q,qhat,C_m_q_T,C_L_q_T,del;

  //Open file to write to
  FILE* coefffile;

  ///Make a copy of the current state vector
  for(ii=0;ii<N;ii++){State_Backup[ii]=State[ii];}

  //Set Nominal State
  //sNominal[0,1,2] = x,y,z should already be set
  //sNominal[3,4,5] = phi,theta,psi should already be set

  char aoafile[256];
  sprintf(aoafile, "%s%s",inSweepsLocation,"AOA_Sweeps.FIT");
  coefffile = fopen(aoafile,"w");
  if (coefffile == NULL){file_error(aoafile,"aoafile");}

  ////Do angle of attack sweeps
  // fscanf(coefffile,"%lf \n",&C_m_0f);
  // fscanf(coefffile,"%lf \n",&C_L_0f);
  // fscanf(coefffile,"%lf \n",&C_D_0f);
  // fscanf(coefffile,"%lf \n",&C_L_alphaf);
  // fscanf(coefffile,"%lf \n",&C_D_alphaf);
  // fscanf(coefffile,"%lf \n",&C_m_alphaf);
  double alpha1;
  for (ii = 0;ii<21;ii++)
    {
      alpha1 = -10*PI/180 + (double)ii*PI/180;
      //printf("Alpha = %lf \n",alpha1);
      sNominal[6] = 20*cos(alpha1); //u
      sNominal[7] = 0; //v
      sNominal[8] = 20*sin(alpha1); //w
      sNominal[9] = 0; //p
      sNominal[10] = 0; //q
      sNominal[11] = 0; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  for (kk = 6;kk<12;kk++)
	    {
	      State[jj*NSTATE+kk] = sNominal[kk];
	    }
	}
      ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
      ///Emily said to try and increase the Fixed iterations during the Stability Analysis.
      ComputeAeroDynamics(TotalAeroLoads);
      
      // PAUSE();

      if (coefffile != NULL)
      {
	fprintf(coefffile,"%0.20f %0.20f %0.20f %0.20f \n",alpha1,Lift_Total,Drag_Total,TotalAeroLoads[4]);
      }
      
    }
  
  fclose(coefffile);

  // printf("Recompute Aero Loads \n");
  // ComputeAeroDynamics(TotalAeroLoads);

  // for (ii = 0;ii<BODIES;ii++)
  //   {
  //     printf("Controls = %lf %lf %lf %lf \n",DE[ii],DA[ii],DR[ii],DELTHRUST[ii]);
  //   }
  // for (ii = 0;ii<6;ii++)
  //   {
  //     printf("TotalForces AOA Sweep Alpha = 10deg = %lf \n",TotalAeroLoads[ii]);
  //   }
  // exit(1);
      
  char pitchfile[256];
  sprintf(pitchfile, "%s%s",inSweepsLocation,"Pitch_Sweeps.FIT");
  coefffile = fopen(pitchfile,"w");
  if (coefffile == NULL) {file_error(pitchfile,"pitchfile");}
  
  ////Pitch Rate Terms
  // fscanf(coefffile,"%lf \n",&C_m_qf);
  // fscanf(coefffile,"%lf \n",&C_L_qf);
  double q1;
  for (ii = 0;ii<21;ii++)
    {
      q1 = -1 + (double)ii/10;
      //printf("q = %lf \n",q1);
      sNominal[6] = 20; //u
      sNominal[7] = 0; //v
      sNominal[8] = 0; //w
      sNominal[9] = 0; //p
      sNominal[10] = q1; //q
      sNominal[11] = 0; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  for (kk = 6;kk<12;kk++)
	    {
	      State[jj*NSTATE+kk] = sNominal[kk];
	    }
	  ///U,V and W will be slightly different due to the offset in CG
	  State[jj*NSTATE+6] = sNominal[6] - sNominal[11]*RCG2AC[1][jj];
	  State[jj*NSTATE+7] = sNominal[7] + sNominal[11]*RCG2AC[0][jj];
	  State[jj*NSTATE+8] = sNominal[8] - sNominal[10]*RCG2AC[0][jj] + sNominal[9]*RCG2AC[1][jj];
	}
      ////////COMPUTE AERODYNAMICS AT THIS PITCH RATE
      ComputeAeroDynamics(TotalAeroLoads);

      if (coefffile != NULL)
      {
	fprintf(coefffile,"%0.20f %0.20f %0.20f \n",q1,Lift_Total,TotalAeroLoads[4]);
      }
      
    }

  fclose(coefffile);

  ////Roll Rate Terms
  // fscanf(coefffile,"%lf \n",&C_y_pf);
  // fscanf(coefffile,"%lf \n",&C_l_pf);
  // fscanf(coefffile,"%lf \n",&C_n_pf);
  char rollfile[256];
  sprintf(rollfile, "%s%s",inSweepsLocation,"Roll_Sweeps.FIT");
  coefffile = fopen(rollfile,"w");
  if (coefffile == NULL) {file_error(rollfile,"rollfile");}

  double p1;
  for (ii = 0;ii<21;ii++)
    {
      p1 = -1 + (double)ii/10;
      //printf("p = %lf \n",p1);
      sNominal[6] = 19.79; //u
      sNominal[7] = 0; //v
      sNominal[8] = 1.086; //w
      sNominal[9] = p1; //p
      sNominal[10] = 0; //q
      sNominal[11] = 0; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  for (kk = 6;kk<12;kk++)
	    {
	      State[jj*NSTATE+kk] = sNominal[kk];
	    }
	  ///U,V and W will be slightly different due to the offset in CG
	  State[jj*NSTATE+6] = sNominal[6] - sNominal[11]*RCG2AC[1][jj];
	  State[jj*NSTATE+7] = sNominal[7] + sNominal[11]*RCG2AC[0][jj];
	  State[jj*NSTATE+8] = sNominal[8] - sNominal[10]*RCG2AC[0][jj] + sNominal[9]*RCG2AC[1][jj];
	}
      ////////COMPUTE AERODYNAMICS AT THIS Roll RATE
      ComputeAeroDynamics(TotalAeroLoads);

      if (coefffile != NULL)
      {
	fprintf(coefffile,"%0.20f %0.20f %0.20f %0.20f \n",p1,TotalAeroLoads[1],TotalAeroLoads[3],TotalAeroLoads[5]);
      }
      
    }

  fclose(coefffile);

  //////////SIDESLIP SWEEPS
  // fscanf(coefffile,"%lf \n",&C_l_betaf);
  // fscanf(coefffile,"%lf \n",&C_n_betaf);
  // fscanf(coefffile,"%lf \n",&C_y_betaf);
  char sideslipfile[256];
  sprintf(sideslipfile, "%s%s",inSweepsLocation,"SideSlip_Sweeps.FIT");
  coefffile = fopen(sideslipfile,"w");
  if (coefffile == NULL) {file_error(sideslipfile,"sideslipfile");}

  double v1;
  for (ii = 0;ii<21;ii++)
    {
      v1 = -5 + (double)ii/2;
      //printf("v = %lf \n",v1);
      sNominal[6] = 19.79; //u
      sNominal[7] = v1; //v
      sNominal[8] = 1.086; //w
      sNominal[9] = 0; //p
      sNominal[10] = 0; //q
      sNominal[11] = 0; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  for (kk = 6;kk<12;kk++)
	    {
	      State[jj*NSTATE+kk] = sNominal[kk];
	    }
	}
      ////////COMPUTE AERODYNAMICS AT THIS Roll RATE
      ComputeAeroDynamics(TotalAeroLoads);

      if (coefffile != NULL)
      {
	fprintf(coefffile,"%0.20f %0.20f %0.20f %0.20f \n",v1,TotalAeroLoads[1],TotalAeroLoads[3],TotalAeroLoads[5]);
      }
      
    }

  fclose(coefffile);

  ////////YAW RATE SWEEPS
  // fscanf(coefffile,"%lf \n",&C_y_rf);
  // fscanf(coefffile,"%lf \n",&C_n_rf);
  // fscanf(coefffile,"%lf \n",&C_l_rf);
  char yawfile[256];
  sprintf(yawfile, "%s%s",inSweepsLocation,"Yaw_Sweeps.FIT");
  coefffile = fopen(yawfile,"w");
  if (coefffile == NULL) {file_error(yawfile,"yawfile");}

  double r1;
  for (ii = 0;ii<21;ii++)
    {
      r1 = -1 + (double)ii/10;
      //printf("r = %lf \n",r1);
      sNominal[6] = 19.79; //u
      sNominal[7] = 0; //v
      sNominal[8] = 1.086; //w
      sNominal[9] = 0; //p
      sNominal[10] = 0; //q
      sNominal[11] = r1; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  for (kk = 6;kk<12;kk++)
	    {
	      State[jj*NSTATE+kk] = sNominal[kk];
	    }
	  ///U,V and W will be slightly different due to the offset in CG
	  State[jj*NSTATE+6] = sNominal[6] - sNominal[11]*RCG2AC[1][jj];
	  State[jj*NSTATE+7] = sNominal[7] + sNominal[11]*RCG2AC[0][jj];
	  State[jj*NSTATE+8] = sNominal[8] - sNominal[10]*RCG2AC[0][jj] + sNominal[9]*RCG2AC[1][jj];
	}
      ////////COMPUTE AERODYNAMICS AT THIS Roll RATE
      ComputeAeroDynamics(TotalAeroLoads);

      if (coefffile != NULL)
      {
	fprintf(coefffile,"%0.20f %0.20f %0.20f %0.20f \n",r1,TotalAeroLoads[1],TotalAeroLoads[3],TotalAeroLoads[5]);
      }
      
    }

  fclose(coefffile);

  ///Make sure to set state vector back to normal in the event that something got goofed
  for(ii=0;ii<N;ii++){State[ii]=State_Backup[ii];}

  //exit(1);

}

void Lift2Drag_Analysis()
{
  //First we run angle of attack sweeps
  int body,ii,jj,ss,kk,sWRF,sTURB,sCONSTANT;
  double BodyAeroLoads[6],TotalAeroLoads[6],sNominal[N],State_Backup[N];
  double PState[12],C_L_0_T,C_D_0_T,Vtrim,rho,q,qhat,C_m_q_T,C_L_q_T,del;

  ///Make a copy of the current state vector
  for(ii=0;ii<N;ii++){State_Backup[ii]=State[ii];}

  printf("Opening L/D File \n");
  lodfile = fopen("LifttoDrag.txt","w");

  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  for (ii = 0;ii<100;ii++)
    {
      T0 = 0;
      double alpha1=((double)ii/10)*PI/180;
      sNominal[6] = 20*cos(alpha1);
      sNominal[7] = 0; //v
      sNominal[8] = 20*sin(alpha1);
      sNominal[9] = 0; //p
      sNominal[10] = 0; //q
      sNominal[11] = 0; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  DE[jj] = 0;
	  DELTHRUST[jj] = 0;
	  DR[jj] = 0;
	  DA[jj] = 0;
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  for (kk = 6;kk<12;kk++)
	    {
	      State[jj*NSTATE+kk] = sNominal[kk];
	    }
	}
      ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
      ComputeAeroDynamics(TotalAeroLoads);  
      fprintf(lodfile,"\n");
    }

}

void AeroFit()
{
  //First we run angle of attack sweeps
  int body,ii,jj,ss,kk,sWRF,sTURB,sCONSTANT;
  double BodyAeroLoads[6],TotalAeroLoads[6],sNominal[N],State_Backup[N];
  double PState[12],C_L_0_T,C_D_0_T,Vtrim,rho,q,qhat,C_m_q_T,C_L_q_T,del;
  //Open file to write to
  FILE* coefffile;

  //printf("Running Aero Fit \n");

  //Make sure to set windmodel to zero
  sWRF = 0;
  sTURB = 0;
  sCONSTANT = 0;
  if (IWRF)
    {
      sWRF = 1;
      IWRF = 0;
    }
  if (ITURB)
    {
      sTURB = 1;
      ITURB = 0;
    }
  if (ICONSTANT)
    {
      sCONSTANT = 1;
      ICONSTANT = 0;
    }

  ///Make a copy of the current state vector
  for(ii=0;ii<N;ii++){State_Backup[ii]=State[ii];}

  //Ok so set all Fuselage values to zero
  C_m_0f = 0.0;
  C_L_0f = 0.0; 
  C_D_0f = 0.0;
  C_L_alphaf = 0.0; 
  C_D_alphaf = 0.0; 
  C_m_alphaf = 0.0; 
  C_m_qf = 0.0; 
  C_L_qf = 0.0; 
  C_l_pf = 0.0; 
  C_n_pf = 0.0; 
  C_y_pf = 0.0; 
  C_l_betaf = 0.0; 
  C_n_betaf = 0.0; 
  C_y_betaf = 0.0; 
  C_y_rf = 0.0; 
  C_n_rf = 0.0; 
  C_l_rf = 0.0; 

  // PAUSE();
  
  rho = Density();
  Vtrim = 20;

  //Initialize Sensor Errors
  //printf("Initializing Sensor Errors \n");
  SensorErrors();

  //Initialize everything
  //printf("Initializing Derivatives \n");
  System_Derivatives();
  //printf("Derivatives Initialized \n");

  //First we need to get all zero lift coefficients;
  double alpha1 = 0;
  sNominal[6] = 20*cos(alpha1); //u
  sNominal[7] = 0; //v
  sNominal[8] = 20*sin(alpha1); //w
  sNominal[9] = 0; //p
  sNominal[10] = 0; //q
  sNominal[11] = 0; //r
  for (jj = 0;jj<BODIES;jj++)
    {
      //x,y,z should never change
      //phi theta psi should never change
      //u,v,w
      for (kk = 6;kk<12;kk++)
	{
	  State[jj*NSTATE+kk] = sNominal[kk];
	}
    }
  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  ComputeAeroDynamics(TotalAeroLoads);  

  ///Compute C_L_0
  double Qa = 0.5*rho*(pow(sNominal[6],2)+pow(sNominal[8],2))*Sarea[0];
  double Lift_Ideal = Qa*(C_L_0);
  double Lift_Fus = Lift_Ideal - Lift_Total;
  C_L_0f = Lift_Fus/(Qa);
  //Compute C_D_0
  double Drag_Ideal = Qa*(C_D_0);
  double Drag_Fus = Drag_Ideal - Drag_Total;
  C_D_0f = Drag_Fus/Qa;
  //Compute C_m_0
  double Pitch_Moment_Ideal = Qa*c*C_m_0;
  double Pitch_Moment_Fus = Pitch_Moment_Ideal - TotalAeroLoads[4];
  C_m_0f = Pitch_Moment_Fus/(Qa*c);
  //printf("%lf \n",C_m_0f);
  //printf("%lf \n",C_L_0f);
  //printf("%lf \n",C_D_0f);
  
  ///////////////COMPUTE ALPHA COEFFICIENTS
  alpha1 = 5*PI/180;
  sNominal[6] = 20*cos(alpha1); //u
  sNominal[7] = 0; //v
  sNominal[8] = 20*sin(alpha1); //w
  sNominal[9] = 0; //p
  sNominal[10] = 0; //q
  sNominal[11] = 0; //r
  for (jj = 0;jj<BODIES;jj++)
    {
      //x,y,z should never change
      //phi theta psi should never change
      //u,v,w
      for (kk = 6;kk<12;kk++)
	{
	  State[jj*NSTATE+kk] = sNominal[kk];
	}
    }
  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  ComputeAeroDynamics(TotalAeroLoads);  

  ///Compute C_L_alpha
  Qa = 0.5*rho*(pow(sNominal[6],2)+pow(sNominal[8],2))*Sarea[0];
  Lift_Ideal = Qa*(C_L_0 + C_L_alpha*alpha1);
  Lift_Fus = Lift_Ideal - Lift_Total;
  C_L_alphaf = Lift_Fus/(Qa*alpha1);
  //Compute C_D_alpha
  Drag_Ideal = Qa*(C_D_0 + C_D_alpha*alpha1*alpha1);
  Drag_Fus = Drag_Ideal - Drag_Total;
  C_D_alphaf = Drag_Fus/(Qa*alpha1*alpha1);
  //Compute C_m_alpha
  Pitch_Moment_Ideal = Qa*c*(C_m_0 + C_m_alpha*alpha1);
  Pitch_Moment_Fus = Pitch_Moment_Ideal - TotalAeroLoads[4];
  C_m_alphaf = Pitch_Moment_Fus/(Qa*c*alpha1);
  //printf("%lf \n",C_L_alphaf);
  //printf("%lf \n",C_D_alphaf);
  //printf("%lf \n",C_m_alphaf);

  //////////PITCH COEFFICIENTS
  double q1 = 0.25;
  sNominal[6] = 20;
  sNominal[7] = 0; //v
  sNominal[8] = 0;
  sNominal[9] = 0; //p
  sNominal[10] = q1; //q
  sNominal[11] = 0; //r
  for (jj = 0;jj<BODIES;jj++)
    {
      //x,y,z should never change
      //phi theta psi should never change
      //u,v,w
      for (kk = 6;kk<12;kk++)
	{
	  State[jj*NSTATE+kk] = sNominal[kk];
	}
    }
  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  ComputeAeroDynamics(TotalAeroLoads);  

  ///Compute C_L_alpha
  double vel2 = (pow(sNominal[6],2)+pow(sNominal[8],2));
  Qa = 0.5*rho*vel2*Sarea[0];
  qhat = q1*c/(2*sqrt(vel2));
  Lift_Ideal = Qa*(C_L_0 + C_L_q*qhat);
  Lift_Fus = Lift_Ideal - Lift_Total;
  C_L_qf = Lift_Fus/(Qa*qhat);
  //Compute C_m_alpha
  //printf("C_m_q = %lf \n",C_m_q);
  Pitch_Moment_Ideal = Qa*c*(C_m_0 + C_m_q*qhat);
  Pitch_Moment_Fus = Pitch_Moment_Ideal - TotalAeroLoads[4];
  C_m_qf = Pitch_Moment_Fus/(Qa*c*qhat);
  //printf("%lf \n",C_m_qf);
  //printf("%lf \n",C_L_qf);

  ///ROLL RATE COEFFICIENTS
  double p1 = 0.25;
  sNominal[6] = 19.79;
  sNominal[7] = 0; //v
  sNominal[8] = 1.086;
  sNominal[9] = p1; //p
  sNominal[10] = 0; //q
  sNominal[11] = 0; //r
  for (jj = 0;jj<BODIES;jj++)
    {
      //x,y,z should never change
      //phi theta psi should never change
      //u,v,w
      for (kk = 6;kk<12;kk++)
	{
	  State[jj*NSTATE+kk] = sNominal[kk];
	}
    }
  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  ComputeAeroDynamics(TotalAeroLoads);  

  ///Compute C_y_p
  vel2 = (pow(sNominal[6],2)+pow(sNominal[8],2));
  Qa = 0.5*rho*vel2*Sarea[0];
  double phat = p1*wspan[0]/(2*sqrt(vel2));
  double Y_Ideal = Qa*(C_y_p*phat);
  double Y_Fus = Y_Ideal - TotalAeroLoads[1];
  C_y_pf = Y_Fus/(Qa*phat);
  // printf("phat,TotalAeroLoads[1],Y_Ideal,Y_Fus,C_y_pf %lf %lf %lf %lf %lf \n",phat,TotalAeroLoads[1],Y_Ideal,Y_Fus,C_y_pf);
  // ComputeAeroDynamics(TotalAeroLoads);  
  // printf("TotalAeroLoads[1] %lf \n",TotalAeroLoads[1]);
  // PAUSE();
  //Compute C_l_p
  double Roll_Moment_Ideal = Qa*wspan[0]*(C_l_p*phat);
  double Roll_Moment_Fus = Roll_Moment_Ideal - TotalAeroLoads[3];
  C_l_pf = Roll_Moment_Fus/(Qa*wspan[0]*phat);
  //Compute C_n_p
  double Yaw_Moment_Ideal = Qa*wspan[0]*(C_n_p*phat);
  double Yaw_Moment_Fus = Yaw_Moment_Ideal - TotalAeroLoads[5];
  C_n_pf = Yaw_Moment_Fus/(Qa*wspan[0]*phat);
  //printf("%lf \n",C_l_pf);
  //printf("%lf \n",C_n_pf);
  //printf("%lf \n",C_y_pf);

  ///SIDESLIP COEFFICIENTS
  double v1 = 0.25;
  sNominal[6] = 19.79;
  sNominal[7] = v1; //v
  sNominal[8] = 1.086;
  sNominal[9] = 0; //p
  sNominal[10] = 0; //q
  sNominal[11] = 0; //r
  for (jj = 0;jj<BODIES;jj++)
    {
      //x,y,z should never change
      //phi theta psi should never change
      //u,v,w
      for (kk = 6;kk<12;kk++)
	{
	  State[jj*NSTATE+kk] = sNominal[kk];
	}
    }
  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  ComputeAeroDynamics(TotalAeroLoads);  

  ///Compute C_y_beta
  vel2 = (pow(sNominal[6],2)+pow(sNominal[8],2));
  double beta = asin(v1/sqrt(vel2));
  Qa = 0.5*rho*vel2*Sarea[0];
  Y_Ideal = Qa*(C_y_beta*beta);
  Y_Fus = Y_Ideal - TotalAeroLoads[1];
  C_y_betaf = Y_Fus/(Qa*beta);
  //Compute C_l_beta
  Roll_Moment_Ideal = Qa*wspan[0]*(C_l_beta*beta);
  Roll_Moment_Fus = Roll_Moment_Ideal - TotalAeroLoads[3];
  C_l_betaf = Roll_Moment_Fus/(Qa*wspan[0]*beta);
  //Compute C_n_p
  Yaw_Moment_Ideal = Qa*wspan[0]*(C_n_beta*beta);
  Yaw_Moment_Fus = Yaw_Moment_Ideal - TotalAeroLoads[5];
  C_n_betaf = Yaw_Moment_Fus/(Qa*wspan[0]*beta);
  //printf("%lf \n",C_l_betaf);
  //printf("%lf \n",C_n_betaf);
  //printf("%lf \n",C_y_betaf);

  ///AND FINALL YAW RATE COEFFICIENTS
  double r1 = 0.25;
  sNominal[6] = 19.79;
  sNominal[7] = 0; //v
  sNominal[8] = 1.086;
  sNominal[9] = 0; //p
  sNominal[10] = 0; //q
  sNominal[11] = r1; //r
  for (jj = 0;jj<BODIES;jj++)
    {
      //x,y,z should never change
      //phi theta psi should never change
      //u,v,w
      for (kk = 6;kk<12;kk++)
	{
	  State[jj*NSTATE+kk] = sNominal[kk];
	}
    }
  ////////COMPUTE AERODYNAMICS AT THIS ANGLE OF ATTACK///////////
  ComputeAeroDynamics(TotalAeroLoads);  

  ///Compute C_y_r
  vel2 = (pow(sNominal[6],2)+pow(sNominal[8],2));
  double rhat = r1*wspan[0]/(2*sqrt(vel2));
  Qa = 0.5*rho*vel2*Sarea[0];
  Y_Ideal = Qa*(C_y_r*rhat);
  Y_Fus = Y_Ideal - TotalAeroLoads[1];
  C_y_rf = Y_Fus/(Qa*rhat);
  //Compute C_l_r
  Roll_Moment_Ideal = Qa*wspan[0]*(C_l_r*rhat);
  Roll_Moment_Fus = Roll_Moment_Ideal - TotalAeroLoads[3];
  C_l_rf = Roll_Moment_Fus/(Qa*wspan[0]*rhat);
  //Compute C_n_r
  Yaw_Moment_Ideal = Qa*wspan[0]*(C_n_r*rhat);
  Yaw_Moment_Fus = Yaw_Moment_Ideal - TotalAeroLoads[5];
  C_n_rf = Yaw_Moment_Fus/(Qa*wspan[0]*rhat);
  //printf("%lf \n",C_y_rf);
  //printf("%lf \n",C_n_rf);
  //printf("%lf \n",C_l_rf);

  ///BUT THESE ARE WHAT WE ARE FITTING THEM TO BE
  sprintf(temp, "%s%s%d%s",inFuse,"Fuselage", (int) WINGPANELS, ".Coeff");
  coefffile = fopen(temp,"w");
  if (coefffile == NULL) {
	  printf("Fuselage Location Inputted Incorrectly \n");
	  file_error(temp,"Fuselage Coefficients File");
  }
  fprintf(coefffile,"%lf \n",C_m_0f);
  fprintf(coefffile,"%lf \n",C_L_0f);
  fprintf(coefffile,"%lf \n",C_D_0f);
  fprintf(coefffile,"%lf \n",C_L_alphaf);
  fprintf(coefffile,"%lf \n",C_D_alphaf);
  fprintf(coefffile,"%lf \n",C_m_alphaf);
  fprintf(coefffile,"%lf \n",C_m_qf);
  fprintf(coefffile,"%lf \n",C_L_qf);
  fprintf(coefffile,"%lf \n",C_l_pf);
  fprintf(coefffile,"%lf \n",C_n_pf);
  fprintf(coefffile,"%lf \n",C_y_pf);
  fprintf(coefffile,"%lf \n",C_l_betaf);
  fprintf(coefffile,"%lf \n",C_n_betaf);
  fprintf(coefffile,"%lf \n",C_y_betaf);
  fprintf(coefffile,"%lf \n",C_y_rf);
  fprintf(coefffile,"%lf \n",C_n_rf);
  fprintf(coefffile,"%lf \n",C_l_rf);  
  fclose(coefffile);

  ///Make sure to set state vector back to normal in the event that something got goofed
  for(ii=0;ii<N;ii++){State[ii]=State_Backup[ii];}

  //Just to check run AeroSweeps()
  AeroSweeps();

  //But turn it back on when you're done
  if (sWRF)
    {
      IWRF = 1;
    }
  if (sTURB)
    {
      ITURB = 1;
    }
  if (sCONSTANT)
    {
      ICONSTANT = 1;
    }

}

////////I NO LONGER USE THIS SUBROUTINE 
void ComputePerturbation()
{
  int body,ii,jj,ss;
  double BodyAeroLoads[6],TotalAeroLoads[6],Lift_Total,Drag_Total,sNominal[N];
  double PState[12],C_L_0_T,C_D_0_T,Vtrim,rho,q,qhat,C_m_q_T,C_L_q_T,del;
  //Open file to write to
  FILE*coefffile;
  coefffile = fopen(inCoeffOUT,"w");
  
  rho = Density();
  Vtrim = 20;

  //Initialize Sensor Errors
  SensorErrors();

  //Initialize everything
  System_Derivatives();

  //PAUSE();

  //Set Nominal State
  //sNominal[0,1,2] = x,y,z should already be set
  //sNominal[3,4,5] = phi,theta,psi should already be set
  //Find u and w such that angle of attack is minimized
  double alpha1 = 0,ALFA_PREV=-2*PI/180;
  double step = 3*PI/180;
  for (ii = 0;ii<2000;ii++)
    {
      sNominal[6] = 20*cos(alpha1); //u
      sNominal[7] = 0; //v
      sNominal[8] = 20*sin(alpha1); //w
      sNominal[9] = 0; //p
      sNominal[10] = 0; //q
      sNominal[11] = 0; //r
      for (jj = 0;jj<BODIES;jj++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  State[jj*NSTATE+6] = sNominal[6];
	  State[jj*NSTATE+8] = sNominal[8];
	}
      System_Derivatives();
      if (copysign(1,ALFA_TOTAL) != copysign(1,ALFA_PREV))
	{
	  step = -step/2;
	}
      ALFA_PREV = ALFA_TOTAL;
      alpha1 = alpha1 + step;
      printf("Iteration = %d \n",ii);
      printf("Alfa = %lf \n",ALFA_PREV);
    }

  del = 1e-8;
  for (ss = 6;ss<12;ss++)
    {
      ///Perturb State
      for (jj = 0; jj < 12; jj++){
	PState[jj] = sNominal[jj];}
      if (ss == 8)
	{
	  del = 1e-2;
	  //del = 1e-2;
	}
      else
	{
	  del = 1e-4;
	}
      if (ss != 6)
	{
	  PState[ss] = PState[ss] + del;
	}
      //Propagate Transformation
      for (ii = 0;ii<BODIES;ii++)
	{
	  //x,y,z should never change
	  //phi theta psi should never change
	  //u,v,w
	  State[ii*NSTATE+6] = PState[6] - RCG2AC[1][ii]*PState[11];
	  State[ii*NSTATE+7] = PState[7] + RCG2AC[0][ii]*PState[11];
	  State[ii*NSTATE+8] = PState[8] - RCG2AC[0][ii]*PState[10] + RCG2AC[1][ii]*PState[9];
	  State[ii*NSTATE+9] = PState[9];
	  State[ii*NSTATE+10] = PState[10];
	  State[ii*NSTATE+11] = PState[11];
	}
      printState(State,"State");
      ///Compute Forces and Moments
      for (jj = 0;jj<6;jj++)
	{
	  TotalAeroLoads[jj] = 0;
	}
      Lift_Total = 0;
      Drag_Total = 0;
      if (AEROMODEL) {
	for (body = 0;body<BODIES;body++)
	  {
	    VelocityPanels(body);
	  }	
      }
      if (VORTEXMODEL) {
	LiftingLine(BODIES);
      }
      for (body = 0;body<BODIES;body++)
	{
	  if (AEROMODEL) {
	    LIFT_A[body] = 0;
	    DRAG_A[body] = 0;
	    PanelForces(BodyAeroLoads, body);
	  } else {
	    AeroForces(BodyAeroLoads, body);
	  }
	  Lift_Total = Lift_Total + LIFT_A[body];
	  Drag_Total = Drag_Total + DRAG_A[body];
	  for (jj = 0;jj<6;jj++)
	    {
	      TotalAeroLoads[jj] = TotalAeroLoads[jj] + BodyAeroLoads[jj];
	    }
	  //Add pitch roll and yaw moments from offset c.g.
	  TotalAeroLoads[3] = TotalAeroLoads[3] + RCG2AC[1][body]*BodyAeroLoads[2]; 
	  TotalAeroLoads[4] = TotalAeroLoads[4] - RCG2AC[0][body]*BodyAeroLoads[2];
	  TotalAeroLoads[5] = TotalAeroLoads[5] - RCG2AC[1][body]*BodyAeroLoads[0] + RCG2AC[0][body]*BodyAeroLoads[1];
	}
      if (AEROMODEL)
	{
	  // Drag_Total = -TotalAeroLoads[0]*cos(ALFA_FUS) - TotalAeroLoads[2]*sin(ALFA_FUS);
	  // Lift_Total = TotalAeroLoads[0]*sin(ALFA_FUS) - TotalAeroLoads[2]*cos(ALFA_FUS);
	  printf("ss = %d \n",ss);
	  for (ii = 0;ii<6;ii++)
	    {
	      printf("TotalAeroLoads[ii] = %0.20f \n",TotalAeroLoads[ii]);
	    }
	}
      printf("Drag, Lift = %lf %lf \n",Drag_Total,Lift_Total);
      //PAUSE();
      // if (ss == 6)
      // 	{
      // 	  C_L_0_T = Lift_Total/(0.5*rho*pow(Vtrim,2)*Sarea[0]);
      // 	  C_D_0_T = Drag_Total/(0.5*rho*pow(Vtrim,2)*Sarea[0]);
      // 	  C_m_0_T = TotalAeroLoads[4]/(0.5*rho*pow(Vtrim,2)*Sarea[0]*c);
      // 	  printf("%lf %lf %lf \n",C_L_0_T,C_D_0_T,C_m_0_T);
      // 	}
      // if (ss == 10)
      // 	{
      // 	  q = del;
      // 	  qhat = c * q / (2 * Vtrim);
      // 	  C_m_q_T = (TotalAeroLoads[4]/(0.5*rho*pow(Vtrim,2)*Sarea[0]*c) - C_m_0_T)/qhat;
      // 	  C_L_q_T = (Lift_Total/(0.5*rho*pow(Vtrim,2)*Sarea[0])-C_L_0_T)/qhat;
      // 	  //printf("%lf %lf \n",C_L_q_T,C_m_q_T);
      // 	}

      if (coefffile != NULL)
      {
          fprintf(coefffile,"%0.20f %0.20f %0.20f %0.20f %0.20f %0.20f \n",Lift_Total,TotalAeroLoads[1],Drag_Total,TotalAeroLoads[3],TotalAeroLoads[4],TotalAeroLoads[5]);
      }
    }
}

#ifdef RUN_VISUALIZER
void Visualizer(VISOBJECT *statetime,int argc,char** argv,int BODIES) {

  printf("Initializing Visualizer\n");

  //OPENGL glhandle_g; -- This variable is created in opengl.h
  //Once you call this the entire routine stops and nothing can run after this
  //In order to run anything else you need to get a boost thread going 
  //So you need to give it a pointer to an object file with a method called
  //getState(MATLAB,MATLAB) and getTime() There is a VISOBJECT class prototype
  //in opengl.h that you can use to do what you need. 
  //The variable glhandle_g is an OPENGL variable. The prototype for Initialize
  //Is below for help
  //void Initialize(VISOBJECT*,int,char**,int,int,int,int);
  //The prototype doesn't help very much so I've included the declaration from opengl.cpp
  //void OPENGL::Initialize(VISOBJECT* statetime_in,int argc,char** argv,int inFarplane,int inWidth,int inHeight,int defaultcamera) {
  //The variable in here I really care the most about is default camera
  //In the code the defaultcamera variable is given to the function Initialize which is a method for the variable camera. The type of variable is cameracontrol  
  //void Initialize(int,MATLAB,MATLAB,int);
  //camera.Initialize(NumObjects,state.cg,state.ptp,defaultcamera);
  //void CameraControl::Initialize(int NumObjects,MATLAB state_cg,MATLAB state_ptp,int defaultcamera)
  //ctype = defaultcamera;
  //if ctype < number of objects these are follow cameras
  //els if ctype < 2*(number of objects) these are origin cameras
  //So if we make defaultcamera = number of objects the camera will
  //default to an origin camera however the number of objects in the
  //space are (if only 1 aircraft)
  //Ground Plane
  //Sky Dowme
  //Aircraft 1
  //So the ctypes are as follows
  //ctype = 0 - follow ground plane
  //1 - follow sky dome
  //2 - follow aircraft 1
  //3 - origin ground plane
  //4 - origin sky dome
  //5 - origin aircraft 1
  //6 - free camera
  //So basically what we want is actually 2*NUMOBJECTS-1
  //But NUMOBJECTS = BODIES + 2
  int NUMOBJECTS = BODIES + 2;
  glhandle_g.Initialize(statetime,argc,argv,10000,600,600,2*NUMOBJECTS-1);

  return;

}

#endif 


int main(int argc, char** argv) {
	int ii, jj;
	double d, Nominal[N], start, mcend, ddummy;

	printf("Meta Aircraft written by Carlos Montalvo 2011-2016 \n");

	// FILE*Gainfile;
	// Gainfile = fopen("Gain.txt","r");
	// fscanf(Gainfile,"%lf %lf \n",&G1,&G2);
	//KV = G2;

	//Get Initial Time
	Get_Time();

	//Read Initial Parameters from Input File
	Get_Files();

	//Master Initialization
	Master_Init();

	for (ii = 0; ii <= NUMMC; ii++) {
		if ((IMONTECARLO) && (ii > 0)) {
			//Get new State
			Read_New_State();

			//Check Connection
			if (MODE == 2)
				Percent_Connection();

			//Output Info
			Output_Info(ii);

		}

		if (ii >= MCRUN) {
			//Current Run Init
			Init();

			//HERE IS WHERE WE NEED TO RUN THE FITTING PROCEDURE
			if ((BODIES == 1) && (AEROMODEL))
			  {
			    AeroFit();
			  }
			//Run the Runge_Kutta on Current State
			if ((MODE != 4) && (MODE != 3))
			  {
			    #ifdef RUN_VISUALIZER
			    //Kick off the Runge_Kutta loop as a thread
			    boost::thread runAirplanes(Runge_Kutta,ii);
			    //Kick off the Visualizer as a thread as well
			    //OK SO WE NEED TO HARCODE MULTIMETA.IFILES UNFORTUNATELY.
			    //BECAUSE OPENGL USES THE ARGC AND ARGV STUFF 
			    statetime.Initialize(TCURRENT,BODIES + 2); //Give the visualizer routine the current time and the number of bodies
			    //Now that we know how many bodies there are we need to generate the file of body ourselves
			    ifstream visfile;
			    visfile.open(argv[1]);
			    if (!visfile.is_open()) {
			      printf("In Order to Kick Off the Visualizer Properly You Need to Specify Where the Visualizer_Setup.txt file is Located \n");
			      printf("Destroying All Threads...\n");
			      pthread_cancel(runAirplanes.native_handle());
			    }
			    string input[11];
			    int NOBJ = 0;
			    int idx;
			    for (idx = 0;idx < 11;idx++) {
			      getline(visfile,input[idx]);
			      //cout << input[idx] << endl;
			    }
			    visfile.close();
			    //Now output file again
			    ofstream visout;
			    visout.open(argv[1]);
			    for (idx = 0;idx < 8;idx++) {
			      if (idx == 1) {
				visout << BODIES + 2 << "\t" << "!Number of objects in window" << endl;
			      }
			      else
				{
				  visout << input[idx] << endl;
				}
			    }
			    //Now we repeat line 8,9 and 10 for the number of bodies
			    for (idx = 0;idx < BODIES;idx++) {
			      visout << input[8] << endl;
			      visout << input[9] << endl;
			      visout << input[10] << endl;
			    }
			    //exit(1);
			    //The extra 2 is for the ground plane and the sky plane
			    boost::thread runVisualizer(Visualizer,&statetime,argc,argv,BODIES);
			    //Unfortunately the simulation will continue after the 
			    //the thread is kicked off therefore we need to add a few things
			    //Wait two seconds to make sure all threads have kicked off properly
			    cross_sleep(2);
			    char key = ' ';
			    while (key != 'q') 
			      {
				cout << "Press ESC to quit" << endl;
				cin >> key;
			      }
			    printf("Destroying All Threads...\n");
			    pthread_cancel(runAirplanes.native_handle());
			    pthread_cancel(runVisualizer.native_handle());
			    printf("Function Quit \n");
			    
			    #else
			    Runge_Kutta(ii);
			    #endif
			  }
			if (MODE == 4)
			  {
			    AeroSweeps();
			  }
			if (MODE == 3)
			  {
			    Lift2Drag_Analysis();
			  }

			//Check for Connection
			if (ii > 0) {
				if (GOODCONTACT) {
					numconnection++;
				}
				numtotal++;
			}
		}

		if (IMONTECARLO) {
			//Output Current Run
			Output_State();
		}
	}

	//Run Stability Analysis
	if (MODE == 1) {
		Stability_Analysis();
	} else if (MODE == 2) {
		pconnection = 100 * numconnection / numtotal;
		PERCENT[iwl] = pconnection;
		WLMAT[iwl] = prevWL;
		iwl++;
		printf("--------------------------------------\n");
		printf("Final Percent Connection @ WL = %d : %lf% \n", prevWL,
				pconnection);
		printf("Percent Connection Summary \n");
		for (ii = 0; ii < iwl; ii++) {
			printf("WL = %lf , Percent Connection = %lf% \n", WLMAT[ii],
					PERCENT[ii]);
		}
		printf("--------------------------------------\n");
	}

	//Print End Time
	//Print_Elapsed_Time();

}

