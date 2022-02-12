//Carlos Montalvo General 6DOF Animation Program
//Developed March 2012
//Edited and Revised 2020
//References
// Websites:
// http://pyopengl.sourceforge.net/
// http://nehe.gamedev.net/
// www.pygame.org
// http://sourceforge.net/apps/mediawiki/predef/index.php?title=Operating_Systems
// easybmp.sourceforge.net
// http://ernesthuntley.wordpress.com/2010/08/20/smooth-keyboard-input-with-glut/
// Texts:
// OpenGL-Programming Guide The Official Guide to Learning - Schreiner et Al.
// For linux
// sudo apt-get install freeglut3-dev
// For windows
// download glut
//obj file
// v x y z vertex coordinates
// ...
// vt u v texture coordinates
// ...
// vn x y z normal coordinates
// usemtl .bmp (24bit 256x256)
// s off
// f v/vt/vn v/vt/vn v/vt/vn face values
// f ...

//////////DEFINE LIBRARIES TO LOAD/////////////

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <time.h>
#include "opengl.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace boost;

//This creates a variable called glhandle_g (_g for global)
OPENGL glhandle_g;
boost::mutex statemutex; //this is so we can access the glhandle_g.state variables externally and internally
boost::mutex timemutex;
boost::mutex controlmutex;

/////////Define Letters/////
/////Numbers and Special characters///
GLubyte Letters[2][24] = {{0xC0,0x30,0xC0,0x30,0xC0,0x30,0xC0,0x30,0xC0,0x30,0xFF,0xF0,0xFF,0xF0,0xC0,0x30,0xC0,0x30,0xC0,0x30,0xFF,0xF0,0xFF,0xF0},{0x06,0x00,0x0F,0x00,0x0F,0x00,0x19,0x80,0x19,0x80,0x30,0xC0,0x30,0xC0,0x60,0x60,0x60,0x60,0xE0,0x70,0xC0,0x30,0x00,0x00,
    }};
GLubyte Numbers[10][24] = {{0x7E,0x00,0xFF,0x00,0xE3,0x00,0xD3,0x00,0xD3,0x00,0xD3,0x00,0xCB,0x00,0xCB,0x00,0xCB,0x00,0xC7,0x00,0xFF,0x00,0x7E,0x00},{0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0xD8,0x00,0xF8,0x00,0x78,0x00,0x38,0x00,0x18,0x00},{0xFF,0x00,0xFF,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x00,0xC3,0x00,0xE7,0x00,0x7E,0x00,0x3C,0x00},{0xFE,0x00,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x7E,0x00,0x7E,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0xFF,0x00,0xFE,0x00,
  },{0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x7F,0x80,0xFF,0x80,0xC6,0x00,0x66,0x00,0x36,0x00,0x1E,0x00,0x0E,0x00},{0xFE,0x00,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0xFF,0x00,0xFE,0x00,0xC0,0x00,0xC0,0x00,0xFF,0x00,0xFF,0x00},{0x7F,0x80,0xE1,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xE1,0xC0,0xFF,0x80,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xFF,0x80,0x7F,0x80},{0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0xFF,0x00,0xFF,0x00},{0x3E,0x00,0x7F,0x00,0xE3,0x80,0xC1,0x80,0xE3,0x80,0x7F,0x00,0x3E,0x00,0x63,0x00,0xC1,0x80,0xE3,0x80,0x7F,0x00,0x3E,0x00},{0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x7F,0x00,0xFF,0x00,0xC3,0x00,0xC3,0x00,0xFF,0x00,0x7E,0x00,
}};
GLubyte Special[2][24] = {{0x00,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x00,0x00},{0xF0,0x00,0xF0,0x00,0xF0,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};

//Constructor
OPENGL::OPENGL() {
  ok = 0;
  counter = 0;
}

void OPENGL::loop(int argc,char** argv,char fileroot[],int inFarplane,int inWidth,int inHeight) {
  bool Full;
  int NumObjects,ii;
  char objectfile[256]={NULL};
  double scalefactor;
  char err[256];
  int filefound = 0;
  ifstream file;
  //Initialize itime which governs when to print to std out
  itime = 0;
  printf("Initializeing OpenGL Rendering Environment\n");
  if (fileroot != NULL) {
    strcat(objectfile,fileroot);
  }
  strcat(objectfile,"Input_Files/Render.txt");
  file.open(objectfile);
  if (!file.is_open()) {
    	sprintf(err,"Error Opening File = %s \n",objectfile);
      error(err);
    } else {
      printf("Opened Render.txt File = %s \n",objectfile);
      string input;
      getline(file,input);
      savepics = atoi(input.c_str());
      if (savepics) {
        printf("Saving Each Frame to Image File \n");
      }
      //Initial Parameters
      Full = 0; //Fullscreen set to zeron
      getline(file,input);
      NumObjects = atoi(input.c_str());
      if (NumObjects) {
        printf("Number of Objects to Render = %d \n",NumObjects);
      }
      printf("Initializing State History\n");
      state.Initialize(NumObjects);
      if (state.ok)	 {
        printf("State Ok!! \n");
        //Initialize Camera
        getline(file,input);
        double camera_offset_zaxis = atof(input.c_str());
        //Default Camera
        getline(file,input);
        int defaultcamera = atof(input.c_str());
        camera.Initialize(NumObjects,state.cg,state.ptp,defaultcamera,camera_offset_zaxis);
        //Initialize Window
        Farplane = inFarplane;
        Height = inHeight;
        Width = inWidth;
        int argc = 1;
        WindowInitialize(Full,argc,argv);
        //Initialize Mouse Inputs
        mouse.Initialize();
        //Setup OBJs
        objs.Initialize(NumObjects);
        if (objs.ok) {
	         for(ii = 0;ii<NumObjects;ii++) {
              //Import Objects
              string trash;
              char delimeter(' '); //It is always ' ' <- a space
              getline(file,input, delimeter);
              getline(file,trash);
              strcpy(objectfile, input.c_str());
              printf("Object = %s \n",objectfile);
              getline(file,input);
              scalefactor = atof(input.c_str());
              objs.Load(objectfile,ii+1,1,scalefactor);
              if (!objs.ok) {
		            //printf("Object File not specified properly \n");
                sprintf(err,"%s %s","Objectfile Not Specified Properly: ",objectfile);
                error(err);
              }
		          getline(file,input);
              double scale = atof(input.c_str());
		          state.positionscale.set(ii+1,1,scale);
		          printf("Position Scale Factor = %lf \n",state.positionscale.get(ii+1,1));
            }
        } else {
          sprintf(err,"%s","OBJSETUP Error");
          error(err);
        }
        //Initialize Setup Routines
      	simtime = 0;
        initial = 0;
        glutDisplayFunc(&DrawGLScene);
        glutIdleFunc(&MainLoop);
        glutReshapeFunc(&ResizeGLScene);
        //Setup Keyboard Control
        keyboard.Initialize();
        printf("Starting Visualizer \n");
        ready = 1; //this tells everyone else that you're working
      	//Kick Off Program
	      glutMainLoop(); 
      } else {
        sprintf(err,"%s","StateHistory Error");
      }
    }
  printf("Visualizer Initialized and Running... \n");
  PAUSE(); //If all is good the program will hang here. Hence the need for boost
}

void ResizeGLScene(int inWidth,int inHeight)
{
  if (inHeight == 0)
    {
      inHeight = 1;
    }
  glhandle_g.Height = inHeight;
  glhandle_g.Width = inWidth;
  glViewport(0, 0, inWidth, inHeight);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (double)inWidth/(double)inHeight, 0.1,glhandle_g.Farplane);
  glMatrixMode(GL_MODELVIEW);
}

void MainLoop()
{
  DrawGLScene();
}

void KeyPressed(unsigned char key,int x,int y)
{
  usleep(100);

  //printf("%u \n",key);
  if ((key == 'u') || (key == 'j') || (key == 'h') || (key == 'k') || (key == 't') || (key == 'g') || (key == 'm') || (key == 'n')) {
    controlmutex.lock();    
  }
  //TAER
  //u is pull up
  if (key == 'u') {
    glhandle_g.state.control[2]-=0.1;
  }
  //j is push down
  if (key == 'j') {
    glhandle_g.state.control[2]+=0.1;
  }
  //h is roll left
  if (key == 'h') {
    glhandle_g.state.control[1]-=0.1;
  }
  //k is roll right
  if (key == 'k') {
    glhandle_g.state.control[1]+=0.1;
  }
  //t is increase thrust
  if (key == 't') {
   glhandle_g.state.control[0]+=0.1;
   //printf("Increasing thrust %lf \n",glhandle_g.state.control[0]);
  }
  if (key == 'g') {
   glhandle_g.state.control[0]-=0.1; 
  }
  //m is yaw right
  if (key == 'm') {
   glhandle_g.state.control[3]+=0.1;  
  }
  //n is yaw left
  if (key == 'n') {
   glhandle_g.state.control[3]-=0.1;   
  }
  //Check for saturation
  for (int i = 0;i<NUMCONTROLS;i++) {
    if (abs(glhandle_g.state.control[i]) > 1.0) {
      glhandle_g.state.control[i] = copysign(1.0,glhandle_g.state.control[i]);
    }
    //printf("%lf ",glhandle_g.state.control[i]);
  }
  //printf("\n");
  if ((key == 'u') || (key == 'j') || (key == 'h') || (key == 'k') || (key == 't') || (key == 'g') || (key == 'm') || (key == 'n')) {
    controlmutex.unlock();    
  }

  if ((key == 27) || (key == 'q'))
    {
      printf("Quitting OpenGL \n");
      glutDestroyWindow(glhandle_g.figure);
    }
  if (key == 'c')
    {
      glhandle_g.camera.SwitchCamera();
    }
  if (key == 'o')
    {
      glhandle_g.camera.zoom(-1);
    }
  if (key == 'i')
    {
      glhandle_g.camera.zoom(1);
    }
  if (key == 'w')
    {
      glhandle_g.camera.pitch(1);
    }
  if (key == 's')
    {
      glhandle_g.camera.pitch(-1);
    }
  if (key == 'a')
    {
      glhandle_g.camera.yaw(-1);
    }
  if (key == 'd')
    {
      glhandle_g.camera.yaw(1);
    }
  if (key == ' ')
    {
      printf("Playing... \n");
      glhandle_g.state.advance = 1;
    }
  if (key == 'l')
    {
      if (glhandle_g.keyboard.lightflag == 0)
	{
	  glEnable(GL_LIGHTING);
	  glhandle_g.keyboard.lightflag = 1;
	}
      else
	{
	  glDisable(GL_LIGHTING);
	  glhandle_g.keyboard.lightflag = 0;
	}
    }
  
}

void DrawGLScene()
{
  int ii,jj;
  double cg[3],ptp[3];
  if (glhandle_g.initial == 0)
    {
      glhandle_g.initial = glutGet(GLUT_ELAPSED_TIME);
    }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  //It's possible to change the state of the objects externally using
  //StateHistory::setState() so we need to create a lock here
  statemutex.lock();
  //Get new camera position
  glhandle_g.camera.Update(glhandle_g.state);
  statemutex.unlock();

  //Set up camera
  gluLookAt(glhandle_g.camera.pos[0],glhandle_g.camera.pos[1],glhandle_g.camera.pos[2],glhandle_g.camera.target[0],glhandle_g.camera.target[1],glhandle_g.camera.target[2],glhandle_g.camera.up[0],glhandle_g.camera.up[1],glhandle_g.camera.up[2]);
  
  //Draw all Objects
  for(ii = 1;ii<=glhandle_g.state.objects;ii++)
    {
      glBindTexture(GL_TEXTURE_2D,glhandle_g.objs.texture[ii-1]);
      //state.cg and state.ptp are MATLAB operators that are Nx3
      //so you must use setters and getters
      statemutex.lock();
      for (jj = 1;jj<=3;jj++)
  	{
  	  cg[jj-1] = glhandle_g.state.cg.get(jj,ii);
	  ptp[jj-1] = glhandle_g.state.ptp.get(jj,ii);
  	}
      statemutex.unlock();

      glTranslatef(cg[0],cg[1],cg[2]);
      glRotatef(ptp[2]*RAD2DEG,0,0,1);
      glRotatef(ptp[1]*RAD2DEG,0,1,0);
      glRotatef(ptp[0]*RAD2DEG,1,0,0);
      //Draw object
      glCallList(glhandle_g.objs.gl_list[ii-1]);
      //Now rotate back
      glRotatef(-ptp[0]*RAD2DEG,1,0,0);
      glRotatef(-ptp[1]*RAD2DEG,0,1,0);
      glRotatef(-ptp[2]*RAD2DEG,0,0,1);
      //Translate Back
      glTranslatef(-cg[0],-cg[1],-cg[2]);
    }

  ////////////Draw The time on the screen///////////////
  glColor3f(1.0,1.0,1.0);
  glRasterPos3f(glhandle_g.camera.target[0],glhandle_g.camera.target[1],glhandle_g.camera.target[2]);
  double offsety = -glhandle_g.Height/2.5;
  double offsetx = glhandle_g.Width/2.1;
  char timechar[256];
  int asciicode;
  timemutex.lock();
  sprintf(timechar,"%lf ",glhandle_g.state.T);
  timemutex.unlock();
  for (ii = 0;ii<6;ii++)
    {
      asciicode = timechar[ii]-48;
      if (asciicode < 0)
  	{
  	  glBitmap(12,12,offsetx,offsety,10,0,Special[1]);
  	  //glBitmap(12,12,offsetx,offsety,10,0,Numbers[ii]);
  	}
      else
  	{
  	  glBitmap(12,12,offsetx,offsety,10,0,Numbers[asciicode]);
  	}
    }
  //printf("offsetx = %lf \n",offsetx);
  //printf("offsety = %lf \n",offsety);
  ////////////////////////////////////////////////////////////////


  #ifdef PRINT_TIME //to stdout
  if (glhandle_g.state.T > glhandle_g.itime) {
    printf("Visualization time = %lf \n",glhandle_g.state.T);
    glhandle_g.itime++;
    glhandle_g.state.cg.disp();
    glhandle_g.state.ptp.disp();
  }
  #endif
  glutSwapBuffers();
  ///////////////////////////////////////////////////////////
}

///////////////////STATEHISTORY/////////////////////////

void StateHistory::Initialize(int NumberofObjects) 
{
  ok = 0;

  //Set Time to zero initially
  setTime(0);

  //Set number of objects
  numobjects_ = NumberofObjects;

  //Initialize cg and ptp variables
  cg.zeros(3,numobjects_,"state.cg");
  ptp.zeros(3,numobjects_,"state.ptp");

  //Initialize Controls
  controlmutex.lock();
  for (int i = 0;i<NUMCONTROLS;i++) {
    control[i] = 0;
  }
  controlmutex.unlock();

  //Initialize Position Scale variable
  positionscale.zeros(numobjects_,1,"positionscale");

  //Initialize Variables
  advance = 0;
  objects = numobjects_;

  ok = 1;

};

void StateHistory::setTime(double simtime)
{
  timemutex.lock();
  T = simtime;
  timemutex.unlock();
}

void StateHistory::UpdateRender(double time,MATLAB cgin,MATLAB ptpin,int objectnumber,double keyboardOut[]) {

  //?Set Time
  setTime(time);
  
  //Set Position of All Objects  
  statemutex.lock();
  for (int idx = 1;idx<=3;idx++) {
    cg.set(idx,objectnumber,positionscale.get(objectnumber,1)*cgin.get(idx,1));
    ptp.set(idx,objectnumber,ptpin.get(idx,1));
  }
  //cg.disp();
  statemutex.unlock();

  //Set Keyboard state
  controlmutex.lock();
  for (int i = 0;i<NUMCONTROLS;i++) {
    keyboardOut[i] = control[i];
  }
  controlmutex.unlock();
}

void StateHistory::getState(MATLAB cgout,MATLAB ptpout)
{
  ptpout.overwrite(ptp);
  cgout.overwrite(cg);
}

double StateHistory::getTime()
{
  return simtime_;
}

/////////////////CAMERA CONTROL////////////////

void CameraControl::Initialize(int NumObjects,MATLAB state_cg,MATLAB state_ptp,int defaultcamera,double cameraoffset_zaxis)
{
  ///Set Z offset
  camera_zoffset = cameraoffset_zaxis;

  int ii;
  objects = NumObjects;
  // if (defaultcamera > NumObjects) {
  //   defaultcamera = 0;
  // }
  //Default is follow cam behind first object but I've since changed this to be an 
  //input to this function
  ctype = defaultcamera;
  rearview = 50;
  //state.cg and state.ptp are MATLAB operators that are 3xN
  //so you must use setters and getters
  cg[0] = state_cg.get(1,1);
  cg[1] = state_cg.get(2,1);
  cg[2] = state_cg.get(3,1);
  ptp[0] = state_ptp.get(1,1);
  ptp[1] = state_ptp.get(2,1);
  ptp[2] = state_ptp.get(3,1);
  psi = ptp[2];
  freepos[0]=0;
  freepos[1]=0;
  freepos[2]=0;
  freetarget[0]=-10;
  freetarget[1]=0;
  freetarget[2]=-1;
  pos[0] = cg[0] - rearview*cos(psi);
  pos[1] = cg[1] - rearview*sin(psi);
  pos[2] = cg[2];
  for (ii = 0;ii<3;ii++)
    {
      target[ii] = cg[ii];
    }
  //Based on target and camera position we need to compute
  //psi and theta
  //First xchat = (pos-target)/norm
  ComputeRange();
  theta = asin(-xcamera[2]);
  psi = atan2(xcamera[1],xcamera[0]);
  ComputeUp();
  ctype--;
  SwitchCamera();
}

void CameraControl::SwitchCamera()
{
  int ii;
  //Everytime this function runs the variable ctype is increased by 1
  
  //if ctype is equal to 2*objects it means the variable is about to
  //roll over to zero which means we need to save the location of the
  //free camera so when we return to it the camera will be the same
  if (ctype == 2*objects)
    {
      //save location and target of camera
      for (ii=0;ii<3;ii++)
	{
	  freepos[ii] = pos[ii];
	  freetarget[ii] = target[ii];
	}
    }

  //Increment ctype by 1.
  ctype++;

  //If ctype is greater than 2*objects reset the ctype to zero
  if (ctype > 2*objects)
    {
      ctype = 0;
    }


  ///Now we need to set the camera to what we want

  //If ctype is equal to 2*objects we are at the free object camera
  if (ctype == 2*objects)
    {
      for (ii = 0;ii<3;ii++)
	{
	  pos[ii] = freepos[ii];
	  target[ii] = freetarget[ii];
	}
      ComputeRange();
      theta = asin(-xcamera[2]);
      psi = atan2(xcamera[1],xcamera[0]);
      ComputeUp();
      printf("Free Camera \n");
    }

  //#camera 0-objects-1 = follow cameras
  if (ctype < objects)    {
    printf("Following Object = %d \n",ctype+1);
  }

  //#objects-objects*2 - origin cameras
  else if (ctype < 2*objects) {
    printf("Origin Camera Pointing at Object = %d \n",ctype-objects+1);
  }

}

void CameraControl::ComputeUp()
{
  up[0] = -cos(psi)*sin(theta);
  up[1] = -sin(psi)*sin(theta);
  up[2] = -cos(theta);
}

void CameraControl::ComputeRange()
{
  int ii;
  targetdistance = 0;
  for (ii = 0;ii<3;ii++)
    {
      xcamera[ii] = target[ii] - pos[ii];
      targetdistance = targetdistance + pow(xcamera[ii],2);
    }
  targetdistance = sqrt(targetdistance);
  for (ii = 0;ii<3;ii++)
    {
      xcamera[ii] = xcamera[ii]/targetdistance;
    }
}

void CameraControl::zoom(double direction)
{
  int ii;
  if (ctype < objects)
    {
      rearview = rearview - direction;
      //printf("%lf \n",rearview);
      if (rearview < 1)rearview = 1;
    }
  else
    {
      for (ii = 0;ii<3;ii++)
	{
	  pos[ii] = pos[ii] + direction*xcamera[ii];
	  target[ii] = target[ii] + direction*xcamera[ii];
	}
      ComputeRange();
    }
}

void CameraControl::pitch(double direction)
{
  theta = theta - direction*(double)0.1;
  ComputeForward();
  ComputeUp();
  if (ctype >= objects)
    {
      ComputeNewTarget();
    }
}

void CameraControl::yaw(double direction)
{
  psi = psi - direction*(double)0.1;
  ComputeForward();
  ComputeUp();
  if (ctype >= objects)
    {
      ComputeNewTarget();
    }
}

void CameraControl::ComputeForward()
{
  xcamera[0] = cos(psi)*cos(theta);
  xcamera[1] = cos(theta)*sin(psi);
  xcamera[2] = -sin(theta);
}

void CameraControl::ComputeNewTarget()
{
  int ii;
  for(ii = 0;ii<3;ii++)
    {
      target[ii] = pos[ii] + targetdistance*xcamera[ii];
    }
}

void CameraControl::Update(StateHistory state)
{
  int idx,ii;

  if (ctype < objects)
    {
      //in this case ctype is also the coordinate we use for cg
      //and ptp
      idx = ctype;
      for(ii = 0;ii<3;ii++)
      {
        cg[ii] = state.cg.get(ii+1,idx+1);
        ptp[ii] = state.ptp.get(ii+1,idx+1);
        target[ii] = cg[ii];
      }
      double ctheta,stheta,spsi,cpsi;
      ctheta = cos(theta);
      stheta = sin(theta);
      cpsi = cos(psi);
      spsi = sin(psi);
      pos[0] = cg[0] - ctheta*cpsi*rearview;
      pos[1] = cg[1] - ctheta*spsi*rearview;
      pos[2] = cg[2] + stheta*rearview;
      ComputeRange();
      theta = asin(-xcamera[2]);
      psi = atan2(xcamera[1],xcamera[0]);
      ComputeUp();
    }
    else if(ctype < 2*objects) {
      //in this case our position is zero but our target
      //is the cg of the aircraft
      idx = ctype - objects;
      for(ii = 0;ii<3;ii++) {
        target[ii] = state.cg.get(ii+1,idx+1);
        pos[ii] = 0;
      }
      ComputeRange();
      theta = asin(-xcamera[2]);
      psi = atan2(xcamera[1],xcamera[0]);
      ComputeUp();

      //Add Offset but only if it's an origina camera
      pos[2]-=camera_zoffset;
    }
  
}

//////////////////MAINWINDOW///////////////////////

void OPENGL::WindowInitialize(bool Full,int argc,char** argv)
{
  FILE* file;
  char name[256],OS[256];
  GLfloat LightDiffuse[4],LightPosition[4];
  
  printf("Initializing Window \n");
  glutInit(&argc,argv);
  sprintf(OS,"%s","Unknown System");
  #if (__linux__)
  sprintf(OS,"%s","Linux");
  #endif
  #if (__macintosh__) || (__Macintosh__) || (__APPLE__) || (__MACH__)
  sprintf(OS,"%s","Macintosh");
  #endif
  if (getenv("OS")!=NULL)
    {
      sprintf(OS,getenv("OS"));
    }
  sprintf(name,"Multi Object Visualizer with Interactive Environment (Running on %s)",OS);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(Width,Height);
  glutInitWindowPosition(100,100);
  figure = glutCreateWindow(name);
  if (Full){
    glutFullScreen();
  }
  //Basic Setup
  glEnable(GL_TEXTURE_2D);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (double)Width/(double)Height, 0.1,Farplane);
  glMatrixMode(GL_MODELVIEW);
  LightDiffuse[0] = 1.0;
  LightDiffuse[1] = 1.0;
  LightDiffuse[2] = 1.0;
  LightDiffuse[3] = 0.0;
  LightPosition[0] = 0.0;
  LightPosition[1] = 0.0;
  LightPosition[2] = -2.0;
  LightPosition[3] = 0.0;
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glLightfv(GL_LIGHT1,GL_DIFFUSE,LightDiffuse);
  glLightfv(GL_LIGHT1,GL_POSITION,LightPosition);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHTING);
  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
  glDisable(GL_LIGHTING);
  //glLoadIdentity();

  if (savepics)
    {
      printf("Allocating Memory \n");
      framebuffer = (Image*)malloc(sizeof(Image));
      printf("%d \n",Width);
      printf("%d \n",Height);
      framebuffer->sizeX = (uint32_t)Width;
      framebuffer->sizeY = (uint32_t)Height;
      int bits = 3;
      int size = framebuffer->sizeX * framebuffer->sizeY * bits;
      printf("%d \n",size);
      framebuffer->data = (char *)malloc(size);
      printf("Memory Allocated \n");
    }

  printf("Window Initialized \n");
  
}

///////////////MOUSECONTROL///////////////////

void MouseControl::Initialize()
{

  printf("Initializing Mouse Inputs \n");

  mousex = 0;mousey = 0;
  prevTime = 0;
  GetNewMouse = (double)1/(double)120;
  glutMotionFunc(&MouseMotion);
  glutPassiveMotionFunc(&SaveMouse);
  glutMouseFunc(&Mouse);

  printf("Mouse Initialized \n");
  
}

void SaveMouse(int x,int y)
{
  glhandle_g.mouse.mousex = x;
  glhandle_g.mouse.mousey = y;
}

void Mouse(int button,int state,int x,int y)
{
  if (button == 3)
    {
      glhandle_g.camera.zoom((double)0.5);
    }
  if (button == 4)
    {
      //printf("Zooming out \n");
      glhandle_g.camera.zoom((double)-0.5);
    }
}

void MouseMotion(int x,int y)
{
  double delx,dely,currenttime,timeInterval;

  delx = (x-glhandle_g.mouse.mousex)/(double)100;
  dely = (y-glhandle_g.mouse.mousey)/(double)100;
  glhandle_g.camera.yaw(delx);
  glhandle_g.camera.pitch(-dely);
  
  currenttime = glutGet(GLUT_ELAPSED_TIME);
  timeInterval = (currenttime - glhandle_g.mouse.prevTime)/(double)1000;
  if (timeInterval > glhandle_g.mouse.GetNewMouse)
    {
      glhandle_g.mouse.prevTime = currenttime;
      glhandle_g.mouse.mousex = x;
      glhandle_g.mouse.mousey = y;
    }

}

/////////////////OBJsetup/////////////////////////

void OBJSETUP::Initialize(int NumObjects)
{
  int ii;

  printf("Initializing OBJ Import \n");

  for(ii = 0;ii<MAX_TEXTURE;ii++)
    {
      glGenTextures(1,&texture[ii]);
      gl_list[ii] = glGenLists(1);
    }

  printf("OBJ Import Initialized \n");

  ok = 1;

}

void OBJSETUP::Load(char filename[],int number,double scale,double amount)
{

  ok = 1;
  printf("Importing OBJ File: %s \n", filename);
  int ii,jj,p;
  char* memory = NULL;
  //Save Directory of Object File
  char directory[256];
  char dummy[256];
  int loc=0;
  char * pch;
  char name[256];
  std::string test = filename;
  loc = test.find("/");
  //printf("%d \n",loc);
  sprintf(name,"%s",filename);
  pch = strtok(name,"/");
  sprintf(dummy,"%s",pch);
  if (loc == 0)
    {
      sprintf(directory,"%s","/");
      loc = 1;
    }
  else
    {
      loc = 0;
    }
  //printf("%s \n",pch);
  while (pch != NULL)
  {
    pch = strtok (NULL, "/");
    if (pch != NULL)
      {
	//printf ("%s\n",pch);
	if (loc == 0)
	  {
	    sprintf(directory,"%s",dummy);
	    loc = 1;
	  }
	else
	  {
	    strcat(directory,dummy);
	  }
	strcat(directory,"/");
	sprintf(dummy,"%s",pch);
      }
  }
  printf("Directory = %s \n",directory);

  size_t bytes = ObjLoadFile(filename, &memory);
  ObjModel* model = ObjLoadModel(memory, bytes,number,directory);

  glNewList(gl_list[number-1],GL_COMPILE);
  glEnable(GL_TEXTURE_2D);
  glFrontFace(GL_CCW);
  printf("scale = %lf \n",amount);
  if (!scale)
    {
      amount = 1;
    }
  // if (number > 2)
  //   {
  //     //glColor3f(255,255,0); //rgb
  //   }
  // else
  //   {
  //     //glColor3f(1.0,1.0,1.0);
  //   }
  for (ii = 0;ii<model->nTriangle;ii++)
    {
      //printf("Loading Face %d \n",ii);
      // for (jj = 0;jj<3;jj++)
      // 	{
      // 	  printf("vertices %d \n",model->TriangleArray[ii].Vertex[jj]);
      // 	  printf("normals %d \n",model->TriangleArray[ii].Normal[jj]);
      // 	  printf("texcoords %d \n",model->TriangleArray[ii].TexCoord[jj]);
      // 	}
      //PAUSE();
      glBegin(GL_POLYGON);
      for (jj = 0;jj<3;jj++)
	{
	  if (model->TriangleArray[ii].Normal[jj] > 0)
	    {
	      p = model->TriangleArray[ii].Normal[jj]-1;
	      glNormal3f(model->NormalArray[p].x*amount,model->NormalArray[p].y*amount,model->NormalArray[p].z*amount);
	    }
	  if (number > -1)
	    {
	      if (model->TriangleArray[ii].TexCoord[jj] >0)
		{
		  p = model->TriangleArray[ii].TexCoord[jj]-1;
		  glTexCoord2f(model->TexCoordArray[p].u,model->TexCoordArray[p].v);
		}
	    }
	  p = model->TriangleArray[ii].Vertex[jj]-1;
	  glVertex3f(model->VertexArray[p].x*amount,model->VertexArray[p].y*amount,model->VertexArray[p].z*amount);
	}
      glEnd();
    }
  glDisable(GL_TEXTURE_2D);
  glEndList();


  free(model->NormalArray);
  free(model->TexCoordArray);
  free(model->TriangleArray);
  free(model->VertexArray);
  free(model);

}

ObjModel* OBJSETUP::ObjLoadModel(char* memory, size_t size,int number,char* directory)
{
  char* p = NULL, * e = NULL;
  ObjModel* ret = (ObjModel*) calloc(1, sizeof(ObjModel));
  memset(ret, 0, sizeof(ObjModel));
  ret->material.exist = 0;
  
  p = memory;
  e = memory + size;
	
  while (p != e)
    {
           if (memcmp(p, "vn", 2) == 0) ret->nNormal++;
      else if (memcmp(p, "vt", 2) == 0) ret->nTexCoord++;
      else if (memcmp(p, "v",  1) == 0) ret->nVertex++;
      else if (memcmp(p, "f",  1) == 0) ret->nTriangle++;
      else if (memcmp(p, "mtllib",6) == 0){
	ret->material.exist = 1;
	sscanf(p,"mtllib %s ",&ret->material.file);
	printf("Material File Found: %s \n",ret->material.file);
      }
      else if (memcmp(p, "usemtl",6) == 0){
	sscanf(p,"usemtl %s ",&ret->material.texname);
	//printf("Using Texture File: %s \n",ret->material.texname);
      }
	
	   while (*p++ != (char) 0x0A);
    }

  ret->VertexArray   = (ObjVertex*)   malloc(sizeof(ObjVertex) * ret->nVertex);
  ret->NormalArray   = (ObjNormal*)   malloc(sizeof(ObjNormal) * ret->nNormal);
  ret->TexCoordArray = (ObjTexCoord*) malloc(sizeof(ObjTexCoord) * ret->nTexCoord);
  ret->TriangleArray = (ObjTriangle*) malloc(sizeof(ObjTriangle) * ret->nTriangle);

  p = memory;
	
  int nV = 0, nN = 0, nT = 0, nF = 0;

  if (ret->material.exist)
    {
      char materialfilename[256];
      ret->material.texexist = 0;
      printf("Loading Material File: %s \n",ret->material.file);
      //We need to concatenate the directory so that the code will find it
      sprintf(materialfilename,"%s",directory);
      strcat(materialfilename,ret->material.file);
      printf("%s \n",materialfilename);
      char* mtlmem = NULL;
      size_t bytes = ObjLoadFile(materialfilename,&mtlmem);
      char* s = NULL,*d=NULL;
      s = mtlmem;
      d = mtlmem + bytes;
      while (s != d)
	{
  	  if (memcmp(s, "map_Kd",6) == 0){
  	    sscanf(s,"map_Kd %s ",&ret->material.texfile);
  	    printf("Texture File Found: %s \n",ret->material.texfile);
  	    ret->material.texexist = 1;
  	  }
	  
  	  while (*s++ != (char) 0x0A);
  	}
  }


  if (ret->material.texexist)
    {
      Image *image1;
      int test;
      printf("Loading Texturefile: %s \n",ret->material.texfile);
      char texturefilename[256];
      //We need to concatenate the directory so that the code will find it
      sprintf(texturefilename,"%s",directory);
      strcat(texturefilename,ret->material.texfile);
      printf("%s \n",texturefilename);
      glBindTexture(GL_TEXTURE_2D,texture[number-1]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
      image1 = (Image*)malloc(sizeof(Image));
      test = ImageLoad(texturefilename,image1);
      if (test)
	{
	  glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);
	  gluBuild2DMipmaps(GL_TEXTURE_2D, 3, image1->sizeX,image1->sizeY, GL_RGB, GL_UNSIGNED_BYTE, image1->data);
	}
      //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    }
  
  while (p != e)
    {
      if (memcmp(p, "vn", 2) == 0)
      {
         sscanf(p, "vn %f %f %f", &ret->NormalArray[nN].x,
                                  &ret->NormalArray[nN].y,
                                  &ret->NormalArray[nN].z);
         nN++;
      }
      else if (memcmp(p, "vt", 2) == 0)
      {
         sscanf(p, "vt %f %f", &ret->TexCoordArray[nT].u,
                               &ret->TexCoordArray[nT].v);
         nT++;
      }
      else if (memcmp(p, "v", 1) == 0) /* or *p == 'v' */
      {
         sscanf(p, "v %f %f %f", &ret->VertexArray[nV].x,
                                 &ret->VertexArray[nV].y,
                                 &ret->VertexArray[nV].z);
         nV++;
      }
      else if (memcmp(p, "f", 1) == 0) /* or *p == 'f' */
      {
         sscanf(p, "f %d/%d/%d %d/%d/%d %d/%d/%d", &ret->TriangleArray[nF].Vertex[0],
                                                   &ret->TriangleArray[nF].TexCoord[0],
                                                   &ret->TriangleArray[nF].Normal[0],
                                                   &ret->TriangleArray[nF].Vertex[1],
                                                   &ret->TriangleArray[nF].TexCoord[1],
                                                   &ret->TriangleArray[nF].Normal[1],
                                                   &ret->TriangleArray[nF].Vertex[2],
                                                   &ret->TriangleArray[nF].TexCoord[2],
                                                   &ret->TriangleArray[nF].Normal[2]);
         nF++;
      }

      while (*p++ != (char) 0x0A);
   }
     
   return ret;
}

int OBJSETUP::ImageLoad(char *filename, Image *image) {
    FILE *file;
    unsigned long size;                 // size of the image in bytes.
    unsigned long i;                    // standard counter.
    unsigned short int planes;          // number of planes in image (must be 1) 
    unsigned short int bpp;             // number of bits per pixel (must be 24)
    char temp;                          // temporary color storage for bgr-rgb conversion.
    int bits;

    // make sure the file is there.
    if ((file = fopen(filename, "rb"))==NULL)
    {
	printf("File Not Found : %s\n",filename);
	char err[256];
	sprintf(err,"%s %s","File not found: ",filename);
	error(err);
	ok = 0;
	return 0;
    }

    int test = sizeof(unsigned long);
    printf("Size of unsigned long = %d \n",test);

    // seek through the bmp header, up to the width/height:
    fseek(file, 18, SEEK_CUR);

    // read the width
    if ((i = fread(&image->sizeX, 4, 1, file)) != 1) {
	printf("Error reading width from %s.\n", filename);
	char err[256];
	sprintf(err,"%s %s","Error Reading width from ",filename);
	error(err);
	return 0;
    }
    printf("Width of %s: %lu\n", filename, image->sizeX);
    
    // read the height 
    if ((i = fread(&image->sizeY, 4, 1, file)) != 1) {
	printf("Error reading height from %s.\n", filename);
	char err[256];
	sprintf(err,"%s %s","Error reading height from : ",filename);
	error(err);
	return 0;
    }
    printf("Height of %s: %lu\n", filename, image->sizeY);
    
    // read the planes
    if ((fread(&planes, 2, 1, file)) != 1) {
	printf("Error reading planes from %s.\n", filename);
	char err[256];
	sprintf(err,"%s %s","Error reading planes from : ",filename);
	error(err);
	return 0;
    }
    if (planes != 1) {
	printf("Planes from %s is not 1: %u\n", filename, planes);
	char err[256];
	sprintf(err,"%s %s","Planes is not 1 from: ",filename);
	error(err);
	return 0;
    }
    if ((i = fread(&bpp, 2, 1, file)) != 1) {
	printf("Error reading bpp from %s.\n", filename);
	char err[256];
	sprintf(err,"%s %s","Error reading bpp from: ",filename);
	error(err);
	return 0;
    }

    // calculate the size (assuming 24 bits or 3 bytes per pixel).
    // read the bpp
    bits = (bpp)/8;
    printf("BPP = %d \n",bpp);
    size = image->sizeX * image->sizeY * bits;

    if (bpp != 24) {
    	printf("Bpp from %s is not 24: %u\n", filename, bpp);
	char err[256];
	sprintf(err,"%s %s","Bpp is not 24 from: ",filename);
	error(err);
    	return 0;
    }
	
    // seek past the rest of the bitmap header.
    fseek(file, bits, SEEK_CUR);

    // read the data. 
    image->data = (char *) malloc(size);
    if (image->data == NULL) {
	printf("Error allocating memory for color-corrected image data");
	char err[256];
	sprintf(err,"%s %s","Error allocating memory for color-correct image data: ",filename);
	error(err);
	return 0;	
    }

    if ((i = fread(image->data, size, 1, file)) != 1) {
	printf("Error reading image data from %s.\n", filename);
	char err[256];
	sprintf(err,"%s %s","Error reading image data: ",filename);
	error(err);
	return 0;
    }

    for (i=0;i<size;i+=3) { // reverse all of the colors. (bgr -> rgb)
	temp = image->data[i];
	image->data[i] = image->data[i+2];
	image->data[i+2] = temp;
    }
    
    // we're done.
    fclose(file);
    return 1;
}

size_t OBJSETUP::ObjLoadFile(char* szFileName, char** memory)
{
	size_t bytes = 0;
	FILE* file = fopen(szFileName, "rt");

	if (file != NULL)
	  {
	    fseek(file, 0, SEEK_END);
	    size_t end = ftell(file);
	    fseek(file, 0, SEEK_SET);

	    *memory = (char*) malloc(end);
	    bytes = fread(*memory, sizeof(char), end, file);
	    fclose(file);
	  }
	else
	  {
	    //printf("File not found: %s \n",szFileName);
	    char err[256];
	    sprintf(err,"%s %s","File below not found \n",szFileName);
	    error(err);
	    ok = 0;
	  }

	return bytes;
}

//////////KEYBOARD CONTROL//////////

void KeyboardControl::Initialize()
{
  lightflag = 0;
  glutKeyboardFunc(&KeyPressed);
}


