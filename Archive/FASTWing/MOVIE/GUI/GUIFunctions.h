#include "moviegui.h"

class MainGUI:public QDialog
{
  Q_OBJECT
public:
   Ui_Dialog Ui;
   QDialog Dialog;
   void Setup();
  MainGUI();
  ~MainGUI();
public slots:
  void PlayMovie();
  void Browse();
  void New();
  int Save();
  void Import();
  void ImportState();
  void Delete();
  void BMP2AVI();
};

