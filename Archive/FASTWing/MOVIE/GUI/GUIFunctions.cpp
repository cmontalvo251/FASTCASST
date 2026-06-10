#include <stdio.h>
#include <iostream>
#include <string.h>
#include "moviegui.h"
#include "GUIFunctions.h"
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <stdlib.h>

/////////DEFINES////////////

#define MAX_OBJECTS 20

////////////////GLOBALS////////

int FPS=0;
double worldscale;
char statefilename[256];
int NumObjects=0;
char objectfilenames[MAX_OBJECTS][256];
double scalefactor[MAX_OBJECTS];
int savepics=0;
int stateok=0;
char filename[256];

///////////////////////////////

MainGUI::MainGUI()
{
}

MainGUI::~MainGUI()
{
}

void MainGUI::Setup()
{
   Ui.setupUi(this);
   QObject::connect(Ui.playMovieButton,SIGNAL(clicked()),this,SLOT(PlayMovie()));
   QObject::connect(Ui.BrowseButton,SIGNAL(clicked()),this,SLOT(Browse()));
   QObject::connect(Ui.NewButton,SIGNAL(clicked()),this,SLOT(New()));
   QObject::connect(Ui.SaveButton,SIGNAL(clicked()),this,SLOT(Save()));
   QObject::connect(Ui.ImportObjectButton,SIGNAL(clicked()),this,SLOT(Import()));
   QObject::connect(Ui.importStateFile,SIGNAL(clicked()),this,SLOT(ImportState()));
   QObject::connect(Ui.DeleteObjectButton,SIGNAL(clicked()),this,SLOT(Delete()));
   QObject::connect(Ui.convertButton,SIGNAL(clicked()),this,SLOT(BMP2AVI()));
}

///////SLOT FUNCTIONS//////

void MainGUI::BMP2AVI()
{
  QString value;
  QByteArray ba;
  char* str;
  //First Grab Frame Rate
  value = Ui.framerateEdit->text();
  ba = value.toLatin1();
  str = ba.data();
  FPS = atoi(str);

  if (FPS != 0)
    {
      char command[1000];
      sprintf(command,"%s%d%s","mencoder -ovc lavc -mf fps=",FPS,":type=bmp 'mf://Frames/*.bmp' -o ~/Desktop/Movie.avi");
      system(command);
      QMessageBox msgBox;
      msgBox.setText("Video Finished");
      msgBox.setInformativeText("Your video has been converted");
      int ret = msgBox.exec();
    }
  else
    {
      QMessageBox msgBox;
      msgBox.setText("Error!");
      msgBox.setInformativeText("Frame Rate Not defined Correctly");
      int ret = msgBox.exec();
    }
}

void MainGUI::PlayMovie()
{
  char command[1000];
  int ret = Save();
  if (ret)
    {
      Ui.statusBar->setText("Opening OpenGL Window");
      #if (__linux__)
         sprintf(command,"%s","Viewer/LinuxGL.exe ");
      #endif
      #if (__macintosh__) || (__Macintosh__) || (__APPLE__) || (__MACH__)
	 sprintf(command,"%s","Viewer/MacGL.exe ");
      #endif
      #if (__WIN32__)
	 sprintf(command,"%s","Viewer/MacGL.exe ");
      #endif
      strcat(command,filename);
      //printf("%s \n",command);
      system(command);
      FILE*file;
      if (file = fopen("movie.err","r"))
	{
	  char error[256];
	  sprintf(error,"%s"," ");
	  char dummy[256];
	  while (!feof(file))
	    {
	      fscanf(file,"%s ",&dummy);
	      strcat(error,dummy);
	    }
	  QMessageBox msgBox;
	  msgBox.setText("Error Found");
	  msgBox.setInformativeText(error);
	  msgBox.setStandardButtons(QMessageBox::Ok);
	  msgBox.setDefaultButton(QMessageBox::Ok);
	  int ret = msgBox.exec();
	  system("rm movie.err");
	}
    }
}

void MainGUI::Delete()
{
  int row = Ui.tableWidget->currentRow();
  if ((NumObjects > 0) && (row > -1))
    {
      QMessageBox msgBox;
      msgBox.setText("Object Deletion");
      msgBox.setInformativeText("Are you sure you want to delete this object?");
      msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
      msgBox.setDefaultButton(QMessageBox::No);
      int ret = msgBox.exec();
      switch (ret) 
	{
	case QMessageBox::Yes:
	  // Yes was clicked
	  NumObjects--;
	  Ui.tableWidget->removeRow(row);
	  if (row < NumObjects-1)
	    {
	      //We must shift everything in objectfilenames and scalefactor by 1
	      int ii;
	      for (ii = row+1;ii<NumObjects;ii++)
		{
		  scalefactor[ii-1] = scalefactor[ii];
		  sprintf(objectfilenames[ii-1],"%s",objectfilenames[ii]);
		}
	    }
	  Ui.statusBar->setText("Object Deleted");
	}
    }
  else
    {
      Ui.statusBar->setText("No Objects to Delete - You must click the row");
    }
}


void MainGUI::Import()
{
  char* dummy;
  NumObjects++;
  if (NumObjects > MAX_OBJECTS)
    {
      Ui.statusBar->setText("Maximum Object Count Exceeded");
      NumObjects--;
    }
  else
    {
      QString inputfile = QFileDialog::getOpenFileName(this,tr("Choose Object File..."),".",tr("Object Files (*.obj)"));
      if (!inputfile.isEmpty())
	{
	  QByteArray ba = inputfile.toLatin1();
	  dummy = ba.data();
	  sprintf(objectfilenames[NumObjects-1] ,"%s ",dummy);
	  scalefactor[NumObjects-1] = 1;
	  //Now populate row with these two values
	  Ui.tableWidget->insertRow(NumObjects-1);
	  QTableWidgetItem *newObject = new QTableWidgetItem(tr(objectfilenames[NumObjects-1]));
	  Ui.tableWidget->setItem(NumObjects-1,0,newObject);
	  sprintf(dummy,"%lf ",scalefactor[NumObjects-1]);
	  QTableWidgetItem *newScale = new QTableWidgetItem(tr(dummy));
	  Ui.tableWidget->setItem(NumObjects-1,1,newScale);
	  Ui.statusBar->setText("Object Loaded");
	}
    }
}

void MainGUI::ImportState()
{
  QString inputfile = QFileDialog::getOpenFileName(this,tr("Choose State File..."),".",tr("Any File (*)"));
  if (!inputfile.isEmpty())
    {
      QByteArray ba = inputfile.toLatin1();
      sprintf(statefilename,"%s ",ba.data());
      Ui.statehistoryEdit->setText(statefilename);
      Ui.statusBar->setText("State File Loaded");
      stateok=1;
    }
}

int MainGUI::Save()
{
  QTableWidgetItem *newscale;
  QString value;
  QByteArray ba;
  char* str;
  int ii;
  //printf("%s \n",filename);
  if (NumObjects > 0)
    {
      if (stateok)
	{
	  //We're good so grab everything from the widgets and put it into
	  //the file
	  //First Grab Frame Rate
	  value = Ui.framerateEdit->text();
	  ba = value.toLatin1();
	  str = ba.data();
	  FPS = atoi(str);
	  //Now grab the worldscale value
	  value = Ui.worldScaleEdit->text();
	  ba = value.toLatin1();
	  str = ba.data();
	  worldscale = atof(str);
	  //Now Grab the statehistory file name
	  value = Ui.statehistoryEdit->text();
	  ba = value.toLatin1();
	  sprintf(statefilename,"%s",ba.data());
	  //printf("StateFilename = %s \n",statefilename);
	  //Now Grab all the scale factor since that's the only thing that would
	  //change
	  for (ii = 0;ii<NumObjects;ii++)
	    {
	      newscale = Ui.tableWidget->item(ii,1);
	      value = newscale->text();
	      ba = value.toLatin1();
	      str = ba.data();
	      scalefactor[ii] = atof(str);
	      //printf("Scale = %lf \n",scalefactor[ii]);
	    }
	  //Finally save the savepics value
	  savepics = Ui.SaveCheckBox->isChecked();
	  //Ok Now check and see if filename is defined
	  //printf("%s \n",filename);
	  FILE* fid = fopen(filename,"r");
	  if (fid)
	    {
	      fclose(fid);
	      //File exists so let's write everything to this file
	      fid = fopen(filename,"w");
	      fprintf(fid,"%d \n",savepics);
	      fprintf(fid,"%d \n",FPS);
	      fprintf(fid,"%s \n",statefilename);
	      fprintf(fid,"%lf \n",worldscale);
	      fprintf(fid,"%d \n",NumObjects);
	      for (ii = 0;ii<NumObjects;ii++)
		{
		  fprintf(fid,"%s \n",objectfilenames[ii]);
		  fprintf(fid,"%lf \n",scalefactor[ii]);
		}
	      fclose(fid);
	      Ui.statusBar->setText("File Saved");
	      return 1;
	    }
	  else
	    {
	      QString outputfile = QFileDialog::getSaveFileName(this,tr("Save Project File As..."),".",tr("MOVIE Files (*.movie)"));
	      QByteArray ba = outputfile.toLatin1();
	      sprintf(filename,"%s",ba.data());
	      fid = fopen(filename,"w");
	      fprintf(fid,"%d \n",savepics);
	      fprintf(fid,"%d \n",FPS);
	      fprintf(fid,"%s \n",statefilename);
	      fprintf(fid,"%d \n",NumObjects);
	      for (ii = 0;ii<NumObjects;ii++)
		{
		  fprintf(fid,"%s \n",objectfilenames[ii]);
		  fprintf(fid,"%lf \n",scalefactor[ii]);
		}
	      fclose(fid);
	      Ui.statusBar->setText("File Saved");
	      return 1;
	      //printf("%s \n",filename);
	    }
	}
      else
	{
	  Ui.statusBar->setText("No State History File");
	  return 0;
	}
    }
  else
    {
      Ui.statusBar->setText("No Objects to Save");
      return 0;
    }
}

void MainGUI::New()
{
  int ii;
  if (NumObjects != 0)
    {
      //Clear All contents
      for (ii = 0;ii<NumObjects;ii++)
	{
	  scalefactor[ii] = 1;
	  Ui.tableWidget->removeRow(0);
	}
      NumObjects = 0;
      FPS = 0;
      stateok = 0;
      savepics = 0;
      Ui.framerateEdit->setText("0");
      Ui.statehistoryEdit->clear();
      Ui.worldScaleEdit->clear();
      Ui.statusBar->setText("New Project File Created");
      Ui.SaveCheckBox->setChecked(0);
      sprintf(filename,"%s","######");
    }
  else
    {
      //Do Nothing
    }
}

void MainGUI::Browse()
{
  QString inputfile = QFileDialog::getOpenFileName(this,tr("Choose Project File..."),".",tr("Movie Files (*.movie)"));
  if (!inputfile.isEmpty())
    {
      //printf("opening file \n");
      QByteArray ba = inputfile.toLatin1();
      sprintf(filename,"%s",ba.data());
      char dummy[256];
      int fint;
      FILE* fid = fopen(filename,"r");
      //printf("%s \n",filename);
      if (fid)
	{
	  fint = fscanf(fid,"%d ",&savepics);
	  if (savepics == 1)
	    {
	      Ui.SaveCheckBox->setChecked(1);
	      //printf("%d \n",savepics);
	    }
	  fint = fscanf(fid,"%d ",&FPS);
	  fint = fscanf(fid,"%s ",&statefilename);
	  stateok = 1;
	  fint = fscanf(fid,"%lf ",&worldscale);
	  //printf("State Filename = %s \n",statefilename);
	  fint = fscanf(fid,"%d ",&NumObjects);
	  //printf("Number of Objects = %d \n",NumObjects);
	  if (NumObjects > MAX_OBJECTS)
	    {
	      NumObjects = MAX_OBJECTS;
	      printf("Maximum Object Count Exceeded \n");
	    }
	  int ii;
	  for (ii=0;ii<NumObjects;ii++)
	    {
	      fint = fscanf(fid,"%s ",&dummy);
	      strcpy(objectfilenames[ii],dummy);
	      fint = fscanf(fid,"%lf ",&scalefactor[ii]);
	      //Add row to matrix
	      Ui.tableWidget->insertRow(ii);
	      //Put objectfile name in 1st column and scale in 2nd
	      QTableWidgetItem *newObject = new QTableWidgetItem(tr(objectfilenames[ii]));
	      Ui.tableWidget->setItem(ii,0,newObject);
	      sprintf(dummy,"%lf ",scalefactor[ii]);
	      QTableWidgetItem *newScale = new QTableWidgetItem(tr(dummy));
	      Ui.tableWidget->setItem(ii,1,newScale);
	    }
	  sprintf(dummy,"%d",FPS);
	  Ui.framerateEdit->setText(dummy);
	  Ui.statehistoryEdit->setText(statefilename);
	  Ui.statusBar->setText(filename);
	  sprintf(dummy,"%lf",worldscale);
	  Ui.worldScaleEdit->setText(dummy);
	  fclose(fid);
      	}
    }
  //printf("%s \n",filename);
}

