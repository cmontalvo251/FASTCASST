/////////MAIN GUI APPLICATION//////////

#include <stdio.h>
#include "moviegui.h"
#include "GUIFunctions.h"


/////////GLOBALS//////////

int main(int argc,char** argv)
{
  QApplication gui(argc,argv);
  MainGUI window;
  window.Setup();
  window.show();
  gui.exec();
}

//http://sector.ynet.sk/qt4-tutorial/my-first-qt-gui-application.html
