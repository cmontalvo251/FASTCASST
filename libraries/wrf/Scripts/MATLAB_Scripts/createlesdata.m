%%This Will Read in the data outputted by savedata.ncl
close all
clear
clc

tic

%%Terrain
%%if you want to edit the terrain go into WRFV3/run/ and 
%%change the terrain_data.txt to whatever you want
%%this is an add on that I created to accomodate different terrain


%%If you're creating a restart point this needs to be done
%%%%%%namelist.input file%%%%%%
  %run_minutes = 30
  %start_minute = 0
  %history_interval_m = 5
  %restart = .false.
  %restart_interval_m = 30
  
%%runWRFNCLcode%%%%%%%%
  %RESTART = 1
  
%%createlesdata%%%%%%%  
  %%CONSOLIDATE = 0  

%%%Once you created the restart point and you want to use it
%make RESTART = 0 and CONSOLIDATE = 1 and change namelist.input to
  %run_minutes = 10
  %start_minute = 30
  %history_interval_s = 1
  %restart = .true.
  %restart_interval_m = 30
  
CONSOLIDATE = 1; %%Consolidate the data into smaller text files

%Run WRF Code if you need to
system('./runWRFNCLcode'); 
%%this is a bash script that runs ideal.exe,wrf.exe,renames
%the output file and converts it to text format
%if you want to change the file names or anything you must
%change the code itself

if CONSOLIDATE
  consolidate
end


%%To change wrf model change /run/namelist.input
%%the data will then be outputted in a raw .nc file in /run/
%%The file will then be written to text files into a folder defined
%%in the function convertNC2TXT and savelesdata.ncl
%%Finally the program consolidate will read in the raw data and 
%%consolidate it into fewer text files. 

%%So if you run something and you want to save everything you 
%have 3 things to save

%%1. the raw wrfles.nc
%%2. the text files in /Raw_Data/  
%%3. the fewer test files in /Data/

%%both 2 and 3 have different location depending on user
%definitions

%plotlesdata

toc