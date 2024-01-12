The easiest way to make an eclipse project is to create a project with
an existing makefile 

1.) Create the makefile and test it (make sure the -g flag is set) 

$ make

2.) Open eclipse and click File->New..->C++ Project->C/C++ Makefile
with existing code 

3.) Edit the keys

Window->Preferences->General->Keys->Emacs

4.) Try and debug once and choose the file to debug(Run.exe) 

choose the gdb/mi configuration

5.) Add the EclipseMeta.ifiles to the input arguments

Run->Run Configurations...->(x)Arguments
Make sure that the code you are debugging is correct


