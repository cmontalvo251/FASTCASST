//Carlos Montalvo General 6DOF Animation Program
//Developed March 2012
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

/////////OPENGL LIBRARIES/////////

#if (__linux__)
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#if (__macintosh__) || (__Macintosh__) || (__APPLE__) || (__MACH__)
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#endif

/////////DEFINES/////////////

#define PI 3.141592653
#define MAX_TEXTURE 40
#define NUMBUILDINGS 30

/////////Define Letters/////
GLubyte Letters[2][24] = {{0xC0,0x30,0xC0,0x30,0xC0,0x30,0xC0,0x30,0xC0,0x30,0xFF,0xF0,0xFF,0xF0,0xC0,0x30,0xC0,0x30,0xC0,0x30,0xFF,0xF0,0xFF,0xF0},{0x06,0x00,0x0F,0x00,0x0F,0x00,0x19,0x80,0x19,0x80,0x30,0xC0,0x30,0xC0,0x60,0x60,0x60,0x60,0xE0,0x70,0xC0,0x30,0x00,0x00,
    }};
GLubyte Numbers[10][24] = {{0x7E,0x00,0xFF,0x00,0xE3,0x00,0xD3,0x00,0xD3,0x00,0xD3,0x00,0xCB,0x00,0xCB,0x00,0xCB,0x00,0xC7,0x00,0xFF,0x00,0x7E,0x00},{0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0x18,0x00,0xD8,0x00,0xF8,0x00,0x78,0x00,0x38,0x00,0x18,0x00},{0xFF,0x00,0xFF,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x00,0xC3,0x00,0xE7,0x00,0x7E,0x00,0x3C,0x00},{0xFE,0x00,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x7E,0x00,0x7E,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0xFF,0x00,0xFE,0x00,
  },{0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x7F,0x80,0xFF,0x80,0xC6,0x00,0x66,0x00,0x36,0x00,0x1E,0x00,0x0E,0x00},{0xFE,0x00,0xFF,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0xFF,0x00,0xFE,0x00,0xC0,0x00,0xC0,0x00,0xFF,0x00,0xFF,0x00},{0x7F,0x80,0xE1,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xE1,0xC0,0xFF,0x80,0xC0,0x00,0xC0,0x00,0xC0,0x00,0xFF,0x80,0x7F,0x80},{0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0xFF,0x00,0xFF,0x00},{0x3E,0x00,0x7F,0x00,0xE3,0x80,0xC1,0x80,0xE3,0x80,0x7F,0x00,0x3E,0x00,0x63,0x00,0xC1,0x80,0xE3,0x80,0x7F,0x00,0x3E,0x00},{0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x03,0x00,0x7F,0x00,0xFF,0x00,0xC3,0x00,0xC3,0x00,0xFF,0x00,0x7E,0x00,
}};
GLubyte Special[2][24] = {{0x00,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x3E,0x00,0x00,0x00},{0xF0,0x00,0xF0,0x00,0xF0,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};

///Building Locations
double XBUILDING[NUMBUILDINGS],YBUILDING[NUMBUILDINGS],ZBUILDING[NUMBUILDINGS];

/////////GLOBAL FUNCTIONS////////

void PAUSE()
{
  printf("%s","Function Paused\n");
  printf("Type in any number(1-inf) and press enter to continue\n");
  int  a;
  scanf("%i",&a);
}

void error(char *output)
{
  printf("Error -> %s \n",output);
  FILE*file;
  file = fopen("movie.err","w");
  fprintf(file,"%s \n",output);
  fclose(file);
}

/////////////CLASSES/////////////////

class StateHistory
{
public:
  int advance,objects,filesize,rowdx,ok;
  double freq,maxval,**data,T,scale,timestep,**finaldata;
  double *cg,*ptp;
  void Initialize(char*,int,int);
  void GetNewState();
};

class CameraControl
{
public:
  int objects,ctype;
  double rearview;
  double cg[3],ptp[3],freepos[3],freetarget[3],targetdistance;
  double pos[3],target[3],up[3],xcamera[3],psi,theta;
  void Initialize(int);
  void ComputeRange();
  void ComputeUp();
  void Update(double*,double*);
  void SwitchCamera();
  void ComputeForward();
  void zoom(double);
  void ComputeNewTarget();
  void pitch(double);
  void yaw(double);
};

class MainWindow
{
public:
  int figure,Width,Height;
  void Initialize(int,int,bool,int,char**);
};

class MouseControl
{
public:
  int mousex,mousey;
  double prevTime,GetNewMouse;
  void Initialize();
};

class OBJsetup
{
public:
  int objects,ok;
  GLuint texture[MAX_TEXTURE];
  GLuint gl_list[MAX_TEXTURE];
  void Initialize(int);
  void Load(char*,int,double,double);
  void MTL(char*);
};

class GLDraw
{
public:
  int Farplane;
  double time,initial;
  void Initialize(int);
};

class KeyboardControl
{
public:
  int lightflag;
  void Initialize();
};

///////////STRUCTURES FOR OBJLOADER/////
#pragma pack(1)
struct BITMAPINFOHEADER
{
  uint32_t biSize;
  uint32_t biWidth;
  uint32_t biHeight;
  uint16_t biPlanes;
  uint16_t biBitCount;
  uint32_t biCompression;
  uint32_t biSizeImage;
  uint32_t biPxmeter;
  uint32_t biPymeter;
  uint32_t biColors;
  uint32_t biImportantColors;
};

struct BITMAPFILEHEADER
{
  uint16_t bfType;
  uint32_t bfSize;
  uint16_t bfReserved1;
  uint16_t bfReserved2;
  uint32_t bfOffBits;
};
#pragma pack()

struct ObjVertex
{
  GLfloat x, y, z;
};

typedef ObjVertex ObjNormal;

struct ObjTexCoord
{
  GLfloat u, v;
};

struct ObjTriangle
{
   int Vertex[3];
   int Normal[3];
   int TexCoord[3];
};

class MTL
{
public:
  char file[256];
  char texname[256];
  char texfile[256];
  bool exist;
  bool texexist;
};

struct Image {
  //unsigned long sizeX;
  //unsigned long sizeY;
  uint32_t sizeX;
  uint32_t sizeY;
    char *data;
};

struct ObjModel
{
   int nVertex, nNormal, nTexCoord, nTriangle;

  ObjVertex* VertexArray;
  ObjNormal* NormalArray;
  ObjTexCoord* TexCoordArray;
  ObjTriangle* TriangleArray;
  MTL material;
};

//////////GLOBAL VARIABLES//////

double rad2deg = (double)180/PI;
int savepics,counter=0;
Image *framebuffer;
MainWindow window;
MouseControl mouse;
CameraControl camera;
StateHistory state;
OBJsetup objs;
GLDraw draw;
KeyboardControl keyboard;

//////////SUBROUTINE FOR OBJLOADER///////////

size_t ObjLoadFile(char* szFileName, char** memory)
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
	    printf("File not found: %s \n",szFileName);
	    char err[256];
	    sprintf(err,"%s %s","File not found: %s \n",szFileName);
	    error(err);
	    objs.ok = 0;
	  }

	return bytes;
}

int ImageLoad(char *filename, Image *image) {
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
	objs.ok = 0;
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

ObjModel* ObjLoadModel(char* memory, size_t size,int number,char* directory)
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
      glBindTexture(GL_TEXTURE_2D,objs.texture[number-1]);
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

/////////////////OBJsetup/////////////////////////

void OBJsetup::Initialize(int NumObjects)
{
  int ii;

  printf("Initializing OBJ Import \n");

  for(ii = 0;ii<MAX_TEXTURE;ii++)
    {
      glGenTextures(1,&texture[ii]);
      gl_list[ii] = glGenLists(1);
    }

  printf("OBJ Import Initialized \n");

}

void OBJsetup::Load(char filename[],int number,double scale,double amount)
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
  printf("%s\n",directory);

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

///////////////////STATEHISTORY/////////////////////////

void StateHistory::Initialize (char statefile[],int Numobjects,int FPS) 
{
  FILE* file;
  int rowlength = Numobjects*6,ii,jj,newfilesize;
  double row[rowlength+1],val,time,newstate0[rowlength+1];
  double t0,t1,alfa,newstate1[rowlength+1];

  ok = 0;

  //Allocate Memory for cg and ptp
  cg = (double*)malloc((Numobjects*3)*sizeof(double));
  ptp = (double*)malloc((Numobjects*3)*sizeof(double));

  printf("Loading State History File: %s \n",statefile);
  //Initialize Variables
  freq = 1/(double)FPS;
  maxval = 50;
  advance = 0;
  objects = Numobjects;
  //Open file and start to read everyline
  file = fopen(statefile,"r");
  printf("%s \n",statefile);
  if (!file)
    {
      ok = 0;
      char err[256];
      sprintf(err,"%s %s","File Not Found: ",statefile);
      error(err);
      //printf("File Not Found \n");
      return;
    }
  ok = 1;
  //Obtain File Size
  printf("Obtaining File Size \n");
  // filesize = 0;
  // while(!feof(file))
  //   {
  //     for (ii = 0;ii<rowlength+1;ii++)
  // 	{
  // 	  fscanf(file,"%lf ",&row[ii]);
  // 	  printf("Current Row = %d \n",ii);
  // 	}
  //     filesize++;
  //   }
  // rewind(file);
  fscanf(file,"%d ",&filesize);
  printf("File Size Obtained \n");
  if (filesize <= 0)
    {
      printf("State History File Defined Improperly \n");
      error("State History File Defined Improperly");
      filesize = 1;
      ok = 0;
      return;
    }
  //Allocate Memory for data
  printf("Allocating Memory \n");
  data = (double**)malloc(filesize*sizeof(double*));
  for (ii = 0;ii<filesize;ii++)
    {
      data[ii] = (double*)malloc((rowlength+1)*sizeof(double));
      printf("Memory Status = %lf \n",100*(double)ii/(double)filesize);
    }
  printf("Memory Allocated \n");
  printf("Reading Data \n");
  //Now read every row of file and save it in data
  for (ii = 0;ii<filesize;ii++)
    {
      for (jj = 0;jj<rowlength+1;jj++)
	{
	  fscanf(file,"%lf ",&data[ii][jj]);
	}
      //Get maximum cg coordinate for skydome
      for (jj = 1;jj<objects*3+1;jj++)
	{
	  val = fabs(data[ii][jj]);
	  if (val > maxval)
	    {
	      maxval = val;
	    }
	}
    }
  printf("Data Read \n");
  //Print everything just to be sure
  //  for (ii = 0;ii<filesize;ii++)
  //    {
  //      for (jj = 0;jj<rowlength+1;jj++)
  // 	 {
  // 	   printf("%lf ",data[ii][jj]);
  // 	 }
  //      printf(" \n");
  //    }
  // printf("%d \n",filesize);
  // PAUSE();
  T = data[0][0];
  //Compute Scaling Factor
  printf("Scaling World: %lf \n",scale);
  timestep = data[1][0] - data[0][0];
  printf("Input File Timestep %lf \n",timestep);
  printf("Frame Rate = %d \n",FPS);
  if (freq < timestep)
    {
      printf("Interpolating Data \n");
      newfilesize = (filesize*timestep)*FPS;
      timestep = freq;
      printf("Number of Frames = %d \n",newfilesize);
      time = 0;
      ///Allocate memory for newdata
      finaldata = (double**)malloc(newfilesize*sizeof(double*));
      for (ii = 0;ii<newfilesize;ii++)
	{
	  finaldata[ii] = (double*)malloc((rowlength+1)*sizeof(double));
	} 
      for (ii = 0;ii<rowlength+1;ii++)
	{
	  newstate0[ii] = data[0][ii];
	}
      rowdx = 1;
      for (ii = 0;ii<newfilesize;ii++)
	{
	  t0 = newstate0[0];
	  if (rowdx >= filesize)
	    {
	      rowdx=filesize-1;
	      t0 = t0-freq;
	    }

	  for(jj = 0;jj<rowlength+1;jj++)
	    {
	      newstate1[jj] = data[rowdx][jj];
	    }
	  t1 = newstate1[0];
	  alfa = (time-t0)/(t1-t0);
	  if (t1 == t0){alfa == 0;}
	  for(jj=0;jj<rowlength+1;jj++)
	    {
	      finaldata[ii][jj] = newstate0[jj] + (newstate1[jj]-newstate0[jj])*alfa;
	      //printf("%lf ",finaldata[ii][jj]);
	      //printf("%lf ",newstate0[jj]);
	    }
	  //printf("\n");
	  // for(jj=0;jj<rowlength+1;jj++)
	  //   {
	  //     printf("%lf ",newstate1[jj]);
	  //   }
	  // printf("\n");
	  //PAUSE();
	  time = time + freq;
	  if (time >= t1)
	    {
	      rowdx++;
	      for(jj=0;jj<rowlength+1;jj++)
		{
		  newstate0[jj] = newstate1[jj];
		}
	    }
	  //printf("rowdx = %d \n",rowdx);
	  //printf("alfa = %lf \n",alfa);
	}
      filesize = newfilesize;
      printf("Interpolation Done \n");
    }
  else if (freq > timestep)
    {
      //Extrapolate statehistory file
      //Allocate identical file for final data and copy entire contents of
      //data into final data
      filesize = (timestep/freq)*filesize;
      finaldata = (double**)malloc(filesize*sizeof(double*));
      for (ii = 0;ii<filesize;ii++)
	{
	  finaldata[ii] = (double*)malloc((rowlength+1)*sizeof(double));
	} 
      counter = 0;
      int increment = freq/timestep;
      for (ii = 0;ii<filesize;ii++)
	{
	  for(jj = 0;jj<rowlength+1;jj++)
	    {
	      finaldata[ii][jj] = data[counter][jj];
	    }
	  counter = counter + increment;
	}
      counter = 0;
    }
  else
    {
      //Allocate identical file for final data and copy entire contents of
      //data into final data
      finaldata = (double**)malloc(filesize*sizeof(double*));
      for (ii = 0;ii<filesize;ii++)
	{
	  finaldata[ii] = (double*)malloc((rowlength+1)*sizeof(double));
	} 
      for (ii = 0;ii<filesize;ii++)
	{
	  for(jj = 0;jj<rowlength+1;jj++)
	    {
	      finaldata[ii][jj] = data[ii][jj];
	    }
	}
    }
  rowdx = 0;
  printf("objects = %d \n",objects);
  ///Finally Set up cg and ptp vectors
  for(ii = 0;ii<objects*3;ii++)
    {
      cg[ii] = finaldata[0][ii+1];
      ptp[ii] = finaldata[0][objects*3+1+ii];
      // printf("cg = %lf , ptp = %lf \n",cg[ii],ptp[ii]);
      // printf("state = %lf , %lf , %lf \n",cg[0],cg[1],cg[2]);
      // printf("ptp = %lf , %lf , %lf \n",ptp[0],ptp[1],ptp[2]);
    }

  // printf("state = %lf , %lf , %lf \n",cg[0],cg[1],cg[2]);
  // printf("ptp = %lf , %lf , %lf \n",ptp[0],ptp[1],ptp[2]);

  //PAUSE();

  //Print everything just to be sure
  //  for (ii = 0;ii<filesize;ii++)
  //    {
  //      for (jj = 0;jj<rowlength+1;jj++)
  // 	 {
  // 	   printf("%lf ",finaldata[ii][jj]);
  // 	 }
  //      printf(" \n");
  //    }
  // printf("%d \n",filesize);

  fclose(file);
  
};

void StateHistory::GetNewState()
{
  int ii,rowlength=objects*6;
  double newstate[rowlength+1];
  rowdx = rowdx + advance;
  //printf("%d \n",rowdx);
  if (rowdx > filesize-1)
    {
      rowdx = filesize-1;
      advance = 0;
    }
  for(ii = 0;ii<rowlength+1;ii++)
    {
      newstate[ii] = finaldata[rowdx][ii];
    }
  T = newstate[0];
  for(ii = 0;ii<objects*3;ii++)
    {
      cg[ii] = newstate[ii+1];
      ptp[ii] = newstate[objects*3+1+ii];
      //printf("cg = %lf , ptp = %lf \n",cg[ii],ptp[ii]);
    }

}

/////////////////CAMERA CONTROL////////////////

void CameraControl::Initialize(int NumObjects)
{
  int ii;
  objects = NumObjects;
  ctype = 0;
  rearview = 50;
  //Default is follow cam behind first object
  cg[0] = state.cg[0];cg[1] = state.cg[1];cg[2] = state.cg[2];
  ptp[0] = state.ptp[0];ptp[1] = state.ptp[1];ptp[2] = state.ptp[2];
  psi = ptp[2];
  freepos[0] = 0;freepos[1]=0;freepos[2]=0;
  freetarget[0] = -10;freetarget[1]=0;freetarget[2]=-1;
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
}

void CameraControl::Update(double cgall[],double ptpall[])
{
  int idx,ii;

  if (ctype < objects)
    {
      //in this case ctype is also the coordinate we use for cg
      //and ptp
      idx = ctype;
      for(ii = 0;ii<3;ii++)
	{
	  cg[ii] = cgall[3*idx+ii];
	  ptp[ii] = ptpall[3*idx+ii];
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
  else if(ctype < 2*objects)
    {
      //in this case our position is zero but our target
      //is the cg of the aircraft
      idx = ctype - objects;
      for(ii = 0;ii<3;ii++)
	{
	  target[ii] = cgall[3*idx+ii];
	  pos[ii] = 0;
	}
      ComputeRange();
      theta = asin(-xcamera[2]);
      psi = atan2(xcamera[1],xcamera[0]);
      ComputeUp();
    }

}
void CameraControl::SwitchCamera()
{
  int ii;
  if (ctype == 2*objects)
    {
      //save location and target of camera
      for (ii=0;ii<3;ii++)
	{
	  freepos[ii] = pos[ii];
	  freetarget[ii] = target[ii];
	}
    }
  ctype++;
  if (ctype > 2*objects)
    {
      ctype = 0;
    }
  if (ctype == 2*objects)
    {
      printf("Free Camera \n");
      for (ii = 0;ii<3;ii++)
	{
	  pos[ii] = freepos[ii];
	  target[ii] = freetarget[ii];
	}
      ComputeRange();
      theta = asin(-xcamera[2]);
      psi = atan2(xcamera[1],xcamera[0]);
      ComputeUp();
    }
  //#camera 0-objects-1 = follow camera
  //#objects-objects*2 - origin cameras
  //#last - free camera

}

void CameraControl::ComputeUp()
{
  up[0] = -cos(psi)*sin(theta);
  up[1] = -sin(psi)*sin(theta);
  up[2] = -cos(theta);
}
void CameraControl::ComputeForward()
{
  xcamera[0] = cos(psi)*cos(theta);
  xcamera[1] = cos(theta)*sin(psi);
  xcamera[2] = -sin(theta);
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

void CameraControl::ComputeNewTarget()
{
  int ii;
  for(ii = 0;ii<3;ii++)
    {
      target[ii] = pos[ii] + targetdistance*xcamera[ii];
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

//////////////////MAINWINDOW///////////////////////

void MainWindow::Initialize(int inWidth,int inHeight,bool Full,int argc,char** argv)
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
  Width = inWidth;
  Height = inHeight;
  glutInitWindowSize(Width,Height);
  glutInitWindowPosition(100,100);
  figure = glutCreateWindow(name);
  if (Full){glutFullScreen();}
  //Basic Setup
  glEnable(GL_TEXTURE_2D);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (double)Width/(double)Height, 0.1,draw.Farplane);
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

//////////////OPENGL METHODS/////////////////////////

void MouseMotion(int x,int y)
{
  double delx,dely,currenttime,timeInterval;

  delx = (x-mouse.mousex)/(double)100;
  dely = (y-mouse.mousey)/(double)100;
  camera.yaw(delx);
  camera.pitch(-dely);
  
  currenttime = glutGet(GLUT_ELAPSED_TIME);
  timeInterval = (currenttime - mouse.prevTime)/(double)1000;
  if (timeInterval > mouse.GetNewMouse)
    {
      mouse.prevTime = currenttime;
      mouse.mousex = x;
      mouse.mousey = y;
    }

}

void SaveMouse(int x,int y)
{
  mouse.mousex = x;
  mouse.mousey = y;
}

void Mouse(int button,int state,int x,int y)
{
  if (button == 3)
    {
      camera.zoom((double)0.5);
    }
  if (button == 4)
    {
      //printf("Zooming out \n");
      camera.zoom((double)-0.5);
    }
}

void DrawGLScene()
{
  int ii,jj;
  double cg[3],ptp[3];
  if (draw.initial == 0)
    {
      draw.initial = glutGet(GLUT_ELAPSED_TIME);
    }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  //Set up camera
  gluLookAt(camera.pos[0],camera.pos[1],camera.pos[2],camera.target[0],camera.target[1],camera.target[2],camera.up[0],camera.up[1],camera.up[2]);
  
  //Draw Sky
  glBindTexture(GL_TEXTURE_2D,objs.texture[0]);
  glCallList(objs.gl_list[0]);

  //Draw Ground
  glBindTexture(GL_TEXTURE_2D,objs.texture[1]);
  glCallList(objs.gl_list[1]);

  //Draw Buildings
  for (ii = 0;ii<NUMBUILDINGS;ii++)
    {
      glBindTexture(GL_TEXTURE_2D,objs.texture[ii+2]);
      glTranslatef(XBUILDING[ii],YBUILDING[ii],ZBUILDING[ii]);
      glCallList(objs.gl_list[ii+2]);
      glTranslatef(-XBUILDING[ii],-YBUILDING[ii],-ZBUILDING[ii]);
    }

  for(ii = 0;ii<state.objects;ii++)
    {
      glBindTexture(GL_TEXTURE_2D,objs.texture[ii+2+NUMBUILDINGS]);
      for (jj = 0;jj<3;jj++)
	{
	  cg[jj] = state.cg[3*ii+jj];
	  ptp[jj] = state.ptp[3*ii+jj];
	}
      glTranslatef(cg[0],cg[1],cg[2]);
      glRotatef(ptp[2]*rad2deg,0,0,1);
      glRotatef(ptp[1]*rad2deg,0,1,0);
      glRotatef(ptp[0]*rad2deg,1,0,0);
      //Draw object
      glCallList(objs.gl_list[ii+2+NUMBUILDINGS]);
      //Now rotate back
      glRotatef(-ptp[0]*rad2deg,1,0,0);
      glRotatef(-ptp[1]*rad2deg,0,1,0);
      glRotatef(-ptp[2]*rad2deg,0,0,1);
      //Translate Back
      glTranslatef(-cg[0],-cg[1],-cg[2]);
    }

  //Draw An F on the screen
  glColor3f(1.0,1.0,1.0);
  glRasterPos3f(camera.target[0],camera.target[1],camera.target[2]);
  double offsetx = -window.Height/2.5;
  double offsety = window.Width/2.1;
  char time[256];
  int asciicode;
  sprintf(time,"%lf ",state.T);
  for (ii = 0;ii<6;ii++)
    {
      asciicode = time[ii]-48;
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

  glutSwapBuffers();

  //Save Image
  //if (counter == 0)
  if ((savepics) && (state.advance))
    {
      //printf("Exporting Frame \n");
      glPixelStorei(GL_PACK_ALIGNMENT,1);
      glReadPixels(0,0,window.Width,window.Height,GL_RGB,GL_UNSIGNED_BYTE,framebuffer->data);
      int i;
      char temp;
      char fname[32];
      sprintf(fname,"Frames/Movie_%04d.bmp",counter);
      counter++;
      //Create a new file for writing
      FILE *pFile = fopen(fname, "wb");
      BITMAPINFOHEADER BMIH;
      BMIH.biSize = (uint32_t)sizeof(BITMAPINFOHEADER);
      BMIH.biWidth = framebuffer->sizeX;
      BMIH.biHeight = framebuffer->sizeY;
      BMIH.biPlanes = (uint16_t)1;
      BMIH.biBitCount = (uint16_t)24;
      BMIH.biCompression = (uint32_t)0;
      BMIH.biSizeImage = (uint32_t)(framebuffer->sizeX * framebuffer->sizeY * 3); 
      BMIH.biColors = 0;
      BMIH.biImportantColors = 0;
      BITMAPFILEHEADER bmfh;
      bmfh.bfType = 'B'+('M'<<8);
      bmfh.bfOffBits = sizeof(BITMAPFILEHEADER) + BMIH.biSize; 
      bmfh.bfSize = bmfh.bfOffBits + BMIH.biSizeImage;
      bmfh.bfReserved1 = 0;
      bmfh.bfReserved2 = 0;
      //Write the bitmap file header
      fwrite(&bmfh, 1, sizeof(BITMAPFILEHEADER), pFile);
      //And then the bitmap info header
      fwrite(&BMIH, 1, sizeof(BITMAPINFOHEADER), pFile);
      //Finally, write the image data itself 
      //-- the data represents our drawing
      for (i=0;i<BMIH.biSizeImage;i+=3) 
	{
	  temp = framebuffer->data[i];
	  framebuffer->data[i] = framebuffer->data[i+2];
	  framebuffer->data[i+2] = temp;
	}
      fwrite(framebuffer->data, 1, BMIH.biSizeImage, pFile);
      fclose(pFile);
    }

  //printf("state = %lf , %lf , %lf \n",state.cg[0],state.cg[1],state.cg[2]);
  //printf("ptp = %lf , %lf , %lf \n",state.ptp[0],state.ptp[1],state.ptp[2]);

  //Get New cg and ptp coordinates
  state.GetNewState();
  //Get new camera position
  camera.Update(state.cg,state.ptp);

}

void ResizeGLScene(int Width,int Height)
{
  if (Height == 0)
    {
      Height = 1;
    }
  window.Height = Height;
  window.Width = Width;
  glViewport(0, 0, Width, Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (double)Width/(double)Height, 0.1,draw.Farplane);
  glMatrixMode(GL_MODELVIEW);

}

void MainLoop()
{
  DrawGLScene();
}

void KeyPressed(unsigned char key,int x,int y)
{
  usleep(100);

  if (key == 27)
    {
      glutDestroyWindow(window.figure);
    }
  if (key == 'c')
    {
      camera.SwitchCamera();
    }
  if (key == 'o')
    {
      camera.zoom(-1);
    }
  if (key == 'i')
    {
      camera.zoom(1);
    }
  if (key == 'w')
    {
      camera.pitch(1);
    }
  if (key == 's')
    {
      camera.pitch(-1);
    }
  if (key == 'a')
    {
      camera.yaw(-1);
    }
  if (key == 'd')
    {
      camera.yaw(1);
    }
  if (key == ' ')
    {
      printf("Playing... \n");
      state.advance = 1;
    }
  if (key == 'l')
    {
      if (keyboard.lightflag == 0)
	{
	  glEnable(GL_LIGHTING);
	  keyboard.lightflag = 1;
	}
      else
	{
	  glDisable(GL_LIGHTING);
	  keyboard.lightflag = 0;
	}
    }
  
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

/////////MAIN GLDRAW/////////////

void GLDraw::Initialize(int plane)
{
  Farplane = plane;
  time = 0;
  initial = 0;
  glutDisplayFunc(&DrawGLScene);
  glutIdleFunc(&MainLoop);
  glutReshapeFunc(&ResizeGLScene);
  
}

//////////KEYBOARD CONTROL//////////

void KeyboardControl::Initialize()
{
  lightflag = 0;
  glutKeyboardFunc(&KeyPressed);
}

///////////////MAIN////////////////

int main(int argc,char** argv)
{
  FILE* file;
  bool Full;
  int Farplane,Width,Height,NumObjects,FPS,ii;
  char statefile[256],objectfile[256];
  double scalefactor;

  printf("Opening Project File = %s \n",argv[1]);
  file=fopen("movie.err","rb");
  if (file)
    {
      system("rm movie.err");
      fclose(file);
    }
  
  file = fopen(argv[1],"r");

  if (file)
    {
      fscanf(file,"%d",&savepics);
      fscanf(file,"%d",&FPS);
      if (FPS == 0)
	{
	  FPS = 1;
	}
      printf("Frame Rate = %d \n",FPS);
      //Initial Parameters
      Farplane = 10000;
      Width = 600;
      Height = 600;
      Full = 0;
      //Import StateHistory File
      fscanf(file,"%s ",&statefile);
      fscanf(file,"%lf ",&state.scale);
      fscanf(file,"%d ",&NumObjects);
      state.Initialize(statefile,NumObjects,FPS);
      if (state.ok)
	{
	  //Initialize Camera
	  camera.Initialize(NumObjects);
	  //Initialize Window
	  window.Initialize(Width,Height,Full,argc,argv);
	  //Initialize Mouse Inputs
	  mouse.Initialize();
	  //Setup OBJs
	  objs.Initialize(NumObjects);
	  //Import Sky
	  objs.Load("Viewer/sky/skydome.obj",1,1,state.scale);
	  //objs.Load("sky/skydome.obj",1,1,state.scale);
	  //Import Ground
	  objs.Load("Viewer/ground/ground.obj",2,1,state.scale*12);
	  //objs.Load("ground/ground.obj",2,1,state.scale);

	  //Import Buildings
	  double buildingfactor = 3;
	  char BuildingName[256];
	  int counter = 1;
	  srand(time(NULL));
	  double num = 0,radius;
	  for (ii = 0;ii<NUMBUILDINGS;ii++)
	    {
	      sprintf(BuildingName,"Viewer/buildings/building%d.obj",counter);
	      counter++;
	      if (counter > 5)counter = 1;
	      buildingfactor = rand() % 2 + 3;
	      objs.Load(BuildingName,ii+3,1,state.scale*buildingfactor);
	      num = ii*2*PI/(NUMBUILDINGS);
	      radius = rand() % 5*state.scale + 35*state.scale;
	      XBUILDING[ii] = radius*sin(num);
	      YBUILDING[ii] = radius*cos(num);
	      ZBUILDING[ii] = -2*state.scale;
	    }

	  if (objs.ok)
	    {
	      for(ii = 0;ii<NumObjects;ii++)
		{
		  //Import Objects
		  fscanf(file,"%s",&objectfile);
		  fscanf(file,"%lf",&scalefactor);
		  objs.Load(objectfile,ii+3+NUMBUILDINGS,1,scalefactor);
		  if (!objs.ok)
		    {
		      printf("Object File not specified properly \n");
		      char err[256];
		      sprintf(err,"%s %s","Objectfile Not Specified Properly: ",objectfile);
		      error(err);
		      return 0;
		    }
		}
	    }
	  else
	    {
	      printf("Could Not Find Ground and sky files \n");
	      char err[256];
	      sprintf(err,"%s","Could Not Find Ground and Sky Files");
	      error(err);
	      return 0;
	    }
	  //Initialize Setup Routines
	  draw.Initialize(Farplane);
      
	  //Setup Keyboard Control
	  keyboard.Initialize();
      
	  printf("Starting Simulation \n");
      
	  //Kick Off Program
	  glutMainLoop();
	}
      else
	{
	  printf("State History Error: %s \n",statefile);
	  //char err[256];
	  //sprintf(err,"%s %s","File Not Found: ",statefile);
	  //error(err);
	}
    }
  else
    {
      printf("File Not Found: %s \n",argv[1]);
      char err[256];
      sprintf(err,"%s %s","File Not Found: ",argv[1]);
      error(err);
    }

  return 0;

}

