This repo contains numerous folders and f95/cpp files. Here are the
folders listed below 

plotting/ - this folder contains 4-D plotting routines in order plot
the results of the WRF data after you post processed it.

README_Extra/ - This contains numerous amount of README files. I would
recommend looking at EXAMPLE_WRF_SIM_START2FINISH.txt which explains
alot of the files. 

Scripts/ - This is where all my scripts are. There are BASH scripts, 


You want to start in Scripts/MATLAB_Scripts/createlesdata.m

This has alot of important information in it. However, this code
mainly just runs runWRFNCLcode. This file is located in
Scripts/BASH_Scripts/runWRFNCLcode which does alot 
of things automatically. That file has a variabled on line 9 called
DATA. The value of this variable must be the same as the ROOT variable
on line 9 of savelesdata.ncl (again read through
EXAMPLE_WRF_SIM_....txt for more help)

As for the f95 files and cpp files here is how you compile them.

The f95 file is a standalone application to import the wind and compute it using 4D interpolation at x,y,z and time. To compile it simply type 

$ gfortran wrf*.f95 -w

The cpp commands have not been tested in recent years so you may need to edit those slightly
