//%%This is the front end of uvwout(), run this script  */
/* %before you run uvwout You only need to run it once */

#define dim 40
#define tlength 601

/* %%%%%%%%GLOBALS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

char* PATH="/home/carlos/Georgia_Tech/Grad_Research/WRF_Wind_Data/Data25dx_HF_0/";
char* temp=(char*)malloc(8);
int markX,markY,markZ,markT,parameters[5],mark1[2],mark2[2];
int bounds,boundflag,pathlength,dx,dy,ztop;
double xcoord[dim],ycoord[dim],zcoord[dim],terrain[dim][dim],tcoord[tlength];
double U0[dim][dim][dim],Udt[dim][dim][dim],V0[dim][dim][dim],Vdt[dim][dim][dim];
double W0[dim][dim][dim],Wdt[dim][dim][dim];
char* U0name=(char*)malloc(256);
char* Udtname =(char*)malloc(256);
char* V0name=(char*)malloc(256);
char* Vdtname =(char*)malloc(256);
char* W0name=(char*)malloc(256);
char* Wdtname =(char*)malloc(256);

void importwind(double outmat[dim][dim][dim],char* file)
{
  FILE* fid=NULL;
  int ii,jj,kk,nii,njj;
  double inmat[dim][dim][dim];
  printf("Importing File = %s \n",file);
  fid = fopen(file,"r");
  if (fid == NULL)
    {
      printf("Wind Data File defined incorrectly \n");
      printf("%s \n",file);
      exit(1);
    }
  for(jj=0;jj<dim;jj++)
    {
      for(ii=0;ii<dim;ii++)
	{
	  for(kk=0;kk<dim;kk++)
	    {
	      fscanf(fid,"%lf",&inmat[ii][kk][jj]);
	      outmat[ii][kk][jj] = inmat[ii][kk][jj];
	    }
	}
    }
  fclose(fid);
  /* %U0 = U0(end:-1:1,end:-1:1,:); */
  /* for(kk=0;kk<dim;kk++) */
  /*   { */
  /*     for(ii=0;ii<dim;ii++) */
  /* 	{ */
  /* 	  for(jj=0;jj<dim;jj++) */
  /* 	    { */
  /* 	      nii = dim-ii-1; */
  /* 	      njj = dim-jj-1; */
  /* 	      outmat[ii][jj][kk] = inmat[nii][njj][kk]; */
  /* 	    } */
  /* 	} */
  /*   } */

}

int find(double invec[],int row,double value)
{
  int idx,counter;
  idx = 0;
  counter = row;
  //printf("counter = %d \n",counter);
  while(idx < row)
    {
      if ((invec[idx] > value))
	{
	  counter = idx-1;
	  idx = row + 1;
	}
      idx++;
    }
  //printf("counter = %d \n",counter);
  return counter;

}
void uvwwind(double wind[3],double xstar,double ystar,double zstar,double tstar)
{
  int stepX,stepY,stepZ,stepT,extrapX,extrapY,extrapZ,extrapT;
  int tinterp,x1,x2,y1,y2,z1,z2,tt,coord1[4],coord2[4],ii;
  int cord1[2],cord2[2];
  double uvw[3][2],xpts2[2],ypts2[2],zpts2[2],zpts1,xpts1,ypts1;
  double u8[8],v8[8],w8[8],u4[4],v4[4],w4[4],uslope,vslope,wslope;
  double u2[2],v2[2],w2[2],u,v,w,tpts[2];

  /*   %%This function will take in x,y,z(m),t(sec) and location and  */
  /*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
  /*   %so many globals must be defined. location is a string that */
  /*   %contains the location of the data to be interpolated. */

  /*   %Note when you plug in zstar this code will compute height above */
  /*   %ground. So make sure you give this code the absolute height and */
  /*   %this code will compute height above ground. */

  stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
  extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
  tinterp = 2;

  //%%Check X */
  if (markX == dim-1)
    {
      markX--;
    }
  if ((xstar >= xcoord[markX]) && (xstar <= xcoord[markX+1]))
    {
      /*Your in between the markers so keep going */
      markX = markX;
    }
  else
    {
      //%Find markX
      if (xstar > xcoord[dim-1])
	{
	  //%use endpt
	  markX = dim-1;
	  stepX = -1;
	  extrapX = 1;
	}
      else if (xstar < xcoord[0])
	{
	  //%use starpt */
	  markX = 0;
	  stepX = 1;
	  extrapX = 1;
	}
      else
	{
	  markX = find(xcoord,dim,xstar);
	  if (markX == dim-1)
	    {
	      markX--;
	    }
	  else if (markX <= -1)
	    {
	      markX = 0;
	    }
	}
    }

  //printf("xstar = %lf \n",xstar);
  //printf("markX = %d \n",markX);

  /* %%Check Y */
  if (markY == dim-1);
  {
    markY--;
  }
  if ((ystar >= ycoord[markY]) && (ystar <= ycoord[markY+1]))
    {
      //Your in between the markers so keep going */
      markY=markY;
    }
  else
    {
      //%Find markY */
      if (ystar > ycoord[dim-1])
	{
	  //use endpt */
	  markY = dim-1;
	  stepY = -1;
	  extrapY = 1;
	}
      else if (ystar < ycoord[0])
	{
	  markY = 0;
	  stepY = 1;
	  extrapY = 1;
	}
      else
	{
	  markY = find(ycoord,dim,ystar);
	  if (markY == dim-1)
	    {
	      markY = markY--;
	    } 
	  else if (markY <= -1)
	    {
	      markY = 0;
	    }
	}
    }

  //printf("ystar = %lf \n",ystar);
  //printf("markY = %d \n",markY);

  if ((zstar >= zcoord[markZ]) && (zstar <= zcoord[markZ+1]))
    {
      //%%Your in between the markers so keep going */
      markZ = markZ;
    }
  else
    {
      //%Find markZ
      if (zstar > zcoord[dim-1])
	{
	  //%use endpt */
	  markZ = dim-1;
	  stepZ = -1;
	  extrapZ = 1;
	}
      else if (zstar < zcoord[0])
	{
	  markZ = 0;
	  stepZ = 1;
	  extrapZ = 1;
	}
      else
	{
	  markZ = find(zcoord,dim,zstar);
	  if (markZ == dim-1)
	    {
	      markZ--;
	    }
	  else if (markZ <= -1)
	    {
	      markZ = 0;
	    }
	}
    }

  //printf("zstar = %lf \n",zstar);
  //printf("markZ = %d \n",markZ);

  //%%Check T */
  if (markT == tlength)
    {
      markT = markT - 1;
    }
  if ((tstar >= tcoord[markT]) && (tstar <= tcoord[markT+1]))
    {
      //%%Your in between the markers so keep going */
      markT = markT;
    }
  else
    {
      //%Find markT */
      if (tstar > tcoord[dim-1])
	{
	  //%use endpt
	  markT = tlength-1;
	  extrapT = 1;
	}
      else if (tstar < tcoord[0])
	{
	  //%use start pt */
	  markT = 0;
	  extrapT = 1;
	}
      else
	{
	  markT = find(tcoord,dim,tstar);
	  if (markT == tlength-1)
	    {
	      markT--;
	    }
	  else if (markT < 0)
	    {
	      markT = 0;
	    }
	}
      //%%Import U,V,W maps since markT changed */
      strcpy(U0name,PATH);
      sprintf(temp,"%s%d%s","U",(int)tcoord[markT],".txt");
      strcat(U0name,temp);
  
      strcpy(V0name,PATH);
      sprintf(temp,"%s%d%s","V",(int)tcoord[markT],".txt");
      strcat(V0name,temp);
  
      strcpy(W0name,PATH);
      sprintf(temp,"%s%d%s","W",(int)tcoord[markT],".txt");
      strcat(W0name,temp);
  
      /*%%only import at markT */
      importwind(U0,U0name);
      importwind(V0,V0name);
      importwind(W0,W0name);
      if (extrapT)
	{
	  tinterp = 1;
	}
      else
	{
	  //%%import markT + 1
	  strcpy(Udtname,PATH);
	  sprintf(temp,"%s%d%s","U",(int)tcoord[markT+1],".txt");
	  strcat(Udtname,temp);

	  strcpy(Vdtname,PATH);
	  sprintf(temp,"%s%d%s","V",(int)tcoord[markT+1],".txt");
	  strcat(Vdtname,temp);

	  strcpy(Wdtname,PATH);
	  sprintf(temp,"%s%d%s","W",(int)tcoord[markT+1],".txt");
	  strcat(Wdtname,temp);

	  importwind(Udt,Udtname);
	  importwind(Vdt,Vdtname);
	  importwind(Wdt,Wdtname);
	}
    }

  //printf("tstar = %lf \n",tstar);
  //printf("markT = %d \n",markT);
  

  /*%%Interpolation Scheme */
  for (tt = 0;tt<tinterp;tt++)
    {
      //%Interpolate Spatially */
      //%%To start we have 8 discrete point (8 corners of a cube) */
      xpts2[0] = xcoord[markX];
      xpts2[1] = xcoord[markX+stepX];
      ypts2[0] = ycoord[markY];
      ypts2[1] = ycoord[markY+stepY];
      zpts2[0] = zcoord[markZ];
      zpts2[1] = zcoord[markZ+stepZ];
      x1 = markX;x2 = markX+stepX;
      y1 = markY;y2 = (markY+stepY);
      z1 = markZ;z2 = markZ+stepZ;
      if (tt == 0)
	{
	  //%%Use U0,V0,W0 */
	  u8 = {U0[y1][x1][z1],U0[y1][x2][z1],U0[y2][x2][z1],U0[y2][x1][z1],U0[y1][x1][z2],U0[y1][x2][z2],U0[y2][x2][z2],U0[y2][x1][z2]};
	  v8 = {V0[y1][x1][z1],V0[y1][x2][z1],V0[y2][x2][z1],V0[y2][x1][z1],V0[y1][x1][z2],V0[y1][x2][z2],V0[y2][x2][z2],V0[y2][x1][z2]};
	  w8 = {W0[y1][x1][z1],W0[y1][x2][z1],W0[y2][x2][z1],W0[y2][x1][z1],W0[y1][x1][z2],W0[y1][x2][z2],W0[y2][x2][z2],W0[y2][x1][z2]};
	}
      else
	{
	  //%%Use Udt][Vdt][Wdt */
	  u8 = {Udt[y1][x1][z1],Udt[y1][x2][z1],Udt[y2][x2][z1],Udt[y2][x1][z1],Udt[y1][x1][z2],Udt[y1][x2][z2],Udt[y2][x2][z2],Udt[y2][x1][z2]};
	  v8 = {Vdt[y1][x1][z1],Vdt[y1][x2][z1],Vdt[y2][x2][z1],Vdt[y2][x1][z1],Vdt[y1][x1][z2],Vdt[y1][x2][z2],Vdt[y2][x2][z2],Vdt[y2][x1][z2]};
	  w8 = {Wdt[y1][x1][z1],Wdt[y1][x2][z1],Wdt[y2][x2][z1],Wdt[y2][x1][z1],Wdt[y1][x1][z2],Wdt[y1][x2][z2],Wdt[y2][x2][z2],Wdt[y2][x1][z2]};
	}

      //for (ii = 0;ii<8;ii++)
      //{
      //printf("v8 = %lf \n",v8[ii]);
      //}
  
      //%%%%%interpZ%%%%%%%%%%%% */
  
      if (extrapZ)
	{
	  //%%You don't need to interpolate on z and you can just use */
	  //%%the values at markZ or z1 */
	  zpts1 = zpts2[0];
	  u4 = {u8[0],u8[1],u8[2],u8[3]};
	  v4 = {v8[0],v8[1],v8[2],v8[3]};
	  w4 = {w8[0],w8[1],w8[2],w8[3]};
	  bounds = 1;
	}
      else
	{
	  //%%Interpolate Between Z points(interpolate pts 1-4 and 5-8) */
	  //%Pts 1,5 : 2,6 : 3,7 : 4,8 */
	  coord1 = {0,1,2,3};
	  coord2 = {4,5,6,7};
	  for(ii = 0;ii<4;ii++)
	    {
	      uslope = (u8[coord2[ii]]-u8[coord1[ii]])/(zpts2[1]-zpts2[0]);
	      vslope = (v8[coord2[ii]]-v8[coord1[ii]])/(zpts2[1]-zpts2[0]);
	      wslope = (w8[coord2[ii]]-w8[coord1[ii]])/(zpts2[1]-zpts2[0]);
	      u4[ii] = uslope*(zstar-zpts2[0])+u8[coord1[ii]];
	      v4[ii] = vslope*(zstar-zpts2[0])+v8[coord1[ii]];
	      w4[ii] = wslope*(zstar-zpts2[0])+w8[coord1[ii]];
	    }
	  zpts1 = zstar;
	}
  
      //%%%%%interpY%%%%%%%%%%% */

      if (extrapY)
	{
	//%%You don't need to interpolate on y */
 	  ypts1 = ypts2[0];
	  u2 = {u4[0],u4[1]};
	  v2 = {v4[0],v4[1]};
	  w2 = {w4[0],w4[1]};
	  bounds = 1;
	}
      else
	{
	  //%%Interpolate between Y points(interpolate pts 1-2 and 3-4) */
	  //%%Pts 1,4 : 2,3 */
	  cord1 = {0,1};
	  cord2 = {3,2};
	  for (ii = 0;ii<2;ii++)
	    {
	      uslope = (u4[cord2[ii]]-u4[cord1[ii]])/(ypts2[1]-ypts2[0]);
	      vslope = (v4[cord2[ii]]-v4[cord1[ii]])/(ypts2[1]-ypts2[0]);
	      wslope = (w4[cord2[ii]]-w4[cord1[ii]])/(ypts2[1]-ypts2[0]);
	      u2[ii] = uslope*(ystar-ypts2[0])+u4[cord1[ii]];
	      v2[ii] = vslope*(ystar-ypts2[0])+v4[cord1[ii]];
	      w2[ii] = wslope*(ystar-ypts2[0])+w4[cord1[ii]];
	    }
	  ypts1 = ystar;
	}
  
      //%%%%interpX%%%%%%%%%%%% */
      if (extrapX)
	{
	  //%%You don't need to interpolate on x */
	  xpts1 = xpts2[0];
	  u = u2[0];
	  v = v2[0];
	  w = w2[0];
	  bounds = 1;
	}
      else
	{
	  //%%Interpolate between X points */
	  uslope = (u2[1]-u2[0])/(xpts2[1]-xpts2[0]);
	  vslope = (v2[1]-v2[0])/(xpts2[1]-xpts2[0]);
	  wslope = (w2[1]-w2[0])/(xpts2[1]-xpts2[0]);
	  u = uslope*(xstar-xpts2[0])+u2[0];
	  v = vslope*(xstar-xpts2[0])+v2[0];
	  w = wslope*(xstar-xpts2[0])+w2[0];
	  xpts1 = xstar;
	}
  
      //%%%%Save wind values%%%%% */
  
      uvw[0][tt] = u;
      uvw[1][tt] = v;
      uvw[2][tt] = w;

    }


  if (extrapT)
    {
      //Answer is just first entry of uvw
      wind[0] = uvw[0][0];
      wind[1] = uvw[1][0];
      wind[2] = uvw[2][0];
    }
  else
    {
      //%%Interpolate on T */
      tpts = {tcoord[markT],tcoord[markT+1]};
      u2 = {uvw[0][0],uvw[0][1]};
      v2 = {uvw[1][0],uvw[1][1]};
      w2 = {uvw[2][0],uvw[2][1]};

      uslope = (u2[1]-u2[0])/(tpts[1]-tpts[0]);
      vslope = (v2[1]-v2[0])/(tpts[1]-tpts[0]);
      wslope = (w2[1]-w2[0])/(tpts[1]-tpts[0]);
      u = uslope*(tstar-tpts[0])+u2[0];
      v = vslope*(tstar-tpts[0])+v2[0];
      w = wslope*(tstar-tpts[0])+w2[0];
      wind[0] = u;
      wind[1] = v;
      wind[2] = w;
    }

  if (bounds && boundflag)
    {
      printf("You went out of bounds at T = %lf XYZ = %lf , %lf , %lf \n",tstar,xstar,ystar,zstar);
      boundflag = 0;
    }

}


void UVWSTARTUP()
{
  FILE *infile=NULL;
  int ii,jj;

  /* %%%%%%%%%%%Define Data Location%%%%%%%%%%%%%% */

  printf("Using Wind File = %s \n",PATH);

  /* %%%%%%%%%%%Initialize Variables%%%%%%%%%%%%% */

  markX = 0;markY = 0;markZ = 0;markT = 0;
  bounds = 0;boundflag = 1;
  mark1 = {0,1};
  mark2 = {0,1};

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /* %%%%%%%%Import Extra Parameters File%%%%%%%%% */

  pathlength = strlen(PATH);  
  char* ParametersFile = "Parameters.txt";
  char* inParameters = (char*)malloc(pathlength+strlen(ParametersFile));
  strcpy(inParameters,PATH);
  strcat(inParameters,ParametersFile);
  infile = fopen(inParameters,"r");
  if (infile == NULL)
    {
      printf("Parameters.txt File defined incorrectly \n");
      printf("%s \n",inParameters);
      exit(1);
    }
  for (ii = 0;ii<5;ii++)
  {
    fscanf(infile,"%d,",&parameters[ii]);
  }
  fclose(infile);
  
  char* ZcoordFile = "Zcoord.txt";
  char* inZcoord = (char*)malloc(pathlength+strlen(ZcoordFile));
  strcpy(inZcoord,PATH);
  strcat(inZcoord,ZcoordFile);
  infile = fopen(inZcoord,"r");
  if (infile == NULL)
    {
      printf("Zcoord.txt File defined incorrectly \n");
      printf("%s \n",inZcoord);
      exit(1);
    }
  for (ii = 0;ii<parameters[4];ii++)
  {
    fscanf(infile,"%lf,",&zcoord[ii]);
  }
  fclose(infile);
  
  char* TimesFile = "SampleTimes.txt";
  char* inTimes = (char*)malloc(pathlength+strlen(TimesFile));
  strcpy(inTimes,PATH);
  strcat(inTimes,TimesFile);
  infile = fopen(inTimes,"r");
  if (infile == NULL)
    {
      printf("SampleTimes.txt File defined incorrectly \n");
      printf("%s \n",inTimes);
      exit(1);
    }
  for (ii = 0;ii<601;ii++)
  {
    fscanf(infile,"%lf,",&tcoord[ii]);
  }
  fclose(infile);

  char* HeightFile = "THeight.txt";
  char* inHeight = (char*)malloc(pathlength+strlen(HeightFile));
  strcpy(inHeight,PATH);
  strcat(inHeight,HeightFile);
  infile = fopen(inHeight,"r");
  if (infile == NULL)
    {
      printf("THeight.txt File defined incorrectly \n");
      printf("%s \n",inHeight);
      exit(1);
    }
  for (ii = 0;ii<dim;ii++)
    {
      for (jj = 0;jj<dim;jj++)
	{
	  fscanf(infile,"%lf ",&terrain[ii][jj]);
	}
    }
  fclose(infile);

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /* %%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% */

  dx = parameters[0];
  dy = parameters[1];
  ztop = parameters[2];
  printf("Grid Size = %d %s \n",dx,"m");
  printf("Maximum Height = %d %s \n",ztop,"m");

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /* %%%%%%%%%%%Create xcoord and ycoord%%%%%%% */
  for (ii = 0;ii<dim;ii++)
    {
      xcoord[ii] = -(double)dx*(dim-1)/2 + ii*dx;
      ycoord[ii] = -(double)dy*(dim-1)/2 + ii*dy;
      //printf("%lf \n",xcoord[ii]);
    }

  /* %%%%%%%Import Initial UVW matrices%%%%%%%%% */
  
  sprintf(temp,"%s%d%s","U",(int)tcoord[0],".txt");
  strcpy(U0name,PATH);
  strcat(U0name,temp);
  sprintf(temp,"%s%d%s","U",(int)tcoord[1],".txt");
  strcpy(Udtname,PATH);
  strcat(Udtname,temp);
  sprintf(temp,"%s%d%s","V",(int)tcoord[0],".txt");
  strcpy(V0name,PATH);
  strcat(V0name,temp);
  sprintf(temp,"%s%d%s","V",(int)tcoord[1],".txt");
  strcpy(Vdtname,PATH);
  strcat(Vdtname,temp);
  sprintf(temp,"%s%d%s","W",(int)tcoord[0],".txt");
  strcpy(W0name,PATH);
  strcat(W0name,temp);
  sprintf(temp,"%s%d%s","W",(int)tcoord[1],".txt");
  strcpy(Wdtname,PATH);
  strcat(Wdtname,temp);

  importwind(U0,U0name);
  importwind(Udt,Udtname);
  importwind(V0,V0name);
  importwind(Vdt,Vdtname);
  importwind(W0,W0name);
  importwind(Wdt,Wdtname);
  
  printf("UVW Initialization Complete \n");
}


