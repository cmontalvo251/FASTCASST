#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

double State[6],StateDot[6];

void Yonnet(double F_M[],double msize[],double x,double y,double z,double J)
{

  //This gives you the magnet force on magnet 2

  int ii,jj,kk,ll,pp,qq;
  double Jprime,mu0;
  double a1,b1,c1,a2,b2,c2,Uij[2][2],Vkl[2][2];
  double Wpq[2][2],Fx,Fy,Fz,U,W,R,V,one,dFx,dFy,dFz,PI;

  PI = 3.141592654;

  Jprime = J;
  a1 = msize[0]/2;
  b1 = msize[1]/2;
  c1 = msize[2]/2;
  a2 = msize[0]/2;
  b2 = msize[1]/2;
  c2 = msize[2]/2;

  //%%Compute U,V,W
  for (ii = 0;ii<2;ii++)
    {
      for(jj = 0;jj<2;jj++)
	{
	  Uij[ii][jj] = x - a1*pow(-1,ii+1) + a2*pow(-1,jj+1);
	  Vkl[ii][jj] = y - b1*pow(-1,ii+1) + b2*pow(-1,jj+1);
	  Wpq[ii][jj] = z - c1*pow(-1,ii+1) + c2*pow(-1,jj+1);
	}
    }

  Fx = 0;Fy = 0;Fz = 0;
  for (ii = 0;ii<2;ii++)
    {
      for (jj = 0;jj<2;jj++)
	{
	  for (kk = 0;kk<2;kk++)
	    {
	      for( ll = 0;ll<2;ll++)
		{
		  for( pp = 0;pp<2;pp++)
		    {
		      for (qq = 0;qq<2;qq++)
			{
			  U = Uij[ii][jj];
			  V = Vkl[kk][ll];
			  W = Wpq[pp][qq];
			  R = sqrt(pow(U,2) + pow(V,2) + pow(W,2));
			  one = pow(-1,ii+jj+kk+ll+pp+qq);
			  dFx = 0.5*(pow(V,2)-pow(W,2))*log(R-U)+U*V*log(R-V)+V*W*atan2(U*V,R*W)+0.5*R*U;
			  dFy = 0.5*(pow(U,2)-pow(W,2))*log(R-V)+U*V*log(R-U)+U*W*atan2(U*V,R*W)+0.5*R*V;
			  dFz = -U*W*log(R-U)-V*W*log(R-V)+U*V*atan2(U*V,R*W)-R*W;
			  Fx = Fx + one*dFx;
			  Fy = Fy + one*dFy;
			  Fz = Fz + one*dFz;
			}
		    }
		}
	    }
	}
    }
  F_M[0] = Fx;
  F_M[1] = Fy;
  F_M[2] = Fz;

  mu0 = 4*PI/pow(10,7); ////permeability of free space in T*m/A
  //%mu0 = 1;
  F_M[0] = J*Jprime/(4*PI*mu0)*F_M[0];
  F_M[1] = J*Jprime/(4*PI*mu0)*F_M[1];
  F_M[2] = J*Jprime/(4*PI*mu0)*F_M[2];

}

void System_Derivatives()
{
  double x,y,z,xdot,ydot,zdot;

  x = State[0];
  y = State[1];
  z = State[2];
  xdot = State[3];
  ydot = State[4];
  zdot = State[5];

  StateDot[0] = xdot;
  StateDot[1] = ydot;
  StateDot[2] = zdot;

  /* Initialize Force Vector */
  double F_M[3],msize[3],J,distance[3],mass; 
  F_M[0] = 0;
  F_M[1] = 0;
  F_M[2] = 0;
  msize[0] = 0.0508;
  msize[1] = 0.0508;
  msize[2] = 0.0213;
  J = 1.29;
  mass = 0.40922;

  Yonnet(F_M,msize,x,y,z-1,J);

  StateDot[3] = F_M[0]/mass;
  StateDot[4] = F_M[1]/mass;
  StateDot[5] = F_M[2]/mass;

}

int main()
{

  int ii,jj,kk;
  double x[11],y[11],z[50],inc,height;

  FILE * infile = fopen("Initial.txt","r");

  fscanf(infile,"%lf \n",&State[0]);
  fscanf(infile,"%lf \n",&State[1]);
  fscanf(infile,"%lf \n",&State[2]);
  fscanf(infile,"%lf \n",&State[3]);
  fscanf(infile,"%lf \n",&State[4]);
  fscanf(infile,"%lf \n",&State[5]);

  /* Define the Runge-Kutta Constants */
  double rkalfa[4];
  rkalfa[0] = 1.00000000;
  rkalfa[1] = 2.00000000;
  rkalfa[2] = 2.00000000;
  rkalfa[3] = 1.00000000;

  double TCURRENT,TFINAL = 10,sNominal[6],tNominal,krkbody[6][4];
  int N = 6,j,i,k;
  double TIMESTEP = 0.001,summ;

  FILE *outfile = fopen("Magnet.txt","w");
  double PrevState[3];
  PrevState[0] = State[0];
  PrevState[1] = State[1];
  PrevState[2] = State[2];
  while ((State[2] < 1) && (TCURRENT < 10))
    {
      //Save PrevState
      PrevState[0] = State[0];
      PrevState[1] = State[1];
      PrevState[2] = State[2];

      fprintf(outfile,"%lf %lf %lf %lf \n",TCURRENT,State[0],State[1],State[2]);
      //printf("TCURRENT = %lf \n",TCURRENT);
      /* Store Nominal State Values */
      tNominal = TCURRENT;
      for (j=0; j<N; j++) sNominal[j] = State[j];

      /* 4th Order Runge-Kutta Integration */
      for (j=0; j<4; j++)
	{
	  /* State Values to Evaluate Derivatives */
	  if (j != 0)
	    {	
	      TCURRENT = tNominal + TIMESTEP/rkalfa[j];
	      for (k=0; k<N; k++) State[k] = sNominal[k] + krkbody[k][j-1]/rkalfa[j];
	    }
	  /* Compute Derivatives at Current Value */
	  System_Derivatives();

	  /* Runge-Kutta Constants */
	  for (k=0; k<N; k++) krkbody[k][j] = TIMESTEP*StateDot[k];
	}

      /* Step Time */
      TCURRENT = tNominal + TIMESTEP;
      
      /* Step States */
      for (j=0; j<N; j++)
	{
	  summ = 0.00000000;
	  for (k=0; k<4; k++) summ = summ + rkalfa[k]*krkbody[j][k];
	  State[j] = sNominal[j] + summ/6.00000000;
	}
    }
}
