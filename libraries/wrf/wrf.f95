!!!!WRF 4-D Interpolator Written by Carlos Montalvo July 15th, 2015

!!TO COMPILE AND RUN THIS CODE

! gfortran wrf*.f95 -w
! ./a.out

!Identical MATLAB Code is
!dataloc = 'WRF_Wind_Data/'
!UVWfrontend
!uvwout(0,0,200,0,dataloc,0)

module MAAWMDATATYPES
 integer,parameter :: MAXNALT = 200                ! Units: 'nd', Desc: 'Maximum Number of Atmosphere Altitude Table Points'
 integer,parameter :: MAXNLSE = 20                 ! Units: 'nd', Desc: 'Maximum Number of Lifting Surface Elements'
 integer,parameter :: MAXPROP = 20                 ! Units: 'nd', Desc: 'Maximum Number of Propellor Table size
 integer,parameter :: MAXNAOA = 100                ! Units: 'nd', Desc: 'Maximum Number of Aerodynamic Angle of Attack Table Points'
 integer,parameter :: MAXX = 1000                  ! Units: 'nd', Desc: 'Maximum Number of System States'
 integer,parameter :: NOACTUATORS = 1              ! Units: 'nd', Desc: Number of Actuators
 integer,parameter :: MAXAIRCRAFT = 20             ! Units: 'nd', Desc: Number of Aircraft
 integer,parameter :: MAXCOMMAND = 1000            ! Units: 'nd', Desc: Maximum number of waypoints allowed
 integer,parameter :: NDATAMAX = 20000              ! Units: 'nd', Desc: Number of data points to sample before surrogen is run
 real*8,parameter :: PI = 3.14159265358979323846   ! Units: 'nd', Desc: 'Pi'

 type ATMOSPHERESTRUCTURE
    integer :: markX = 1                             ! X coordinate Marker in interpolation routine 
    integer :: markY = 1                             ! Y coordinate Marker in interpolation routine 
    integer :: markZ = 1                             ! Z coordinate Marker in interpolation routine 
    integer :: markT = 1                             ! T coordinate Marker in interpolation routine 
    integer :: markXT = 1                            ! X coordinate Marker in turbulence interpolation routine 
    integer :: markYT = 1                            ! Y coordinate Marker in turbulence interpolation routine 
    integer :: markZT = 1                            ! Z coordinate Marker in turbulenceinterpolation routine 
    integer :: bounds = 0                            ! Flag indicating wether or not you have gone out of WRF bounds
    integer :: boundflag = 1                         ! Flag indicating wether or not you have gone out of WRF bounds
    integer :: boundsT = 0                           ! Flag indicating wether or not you have gone out of TURB bounds
    integer :: boundflagT = 1                        ! Flag indicating wether or not you have gone out of TURB bounds
    integer :: parameters(5) = 0                     ! Vector of a few parameters 
    integer :: tcoord(601) = 0                       ! Vector of time
    integer :: dim = 40                              ! Dimension of Spatial wind data matrices
    integer :: dimT = 500                            ! Dimension of Turbulence wind data matrices
    integer :: tlength = 601                         ! Dimension of time vector
    integer :: OFFON = 0                             ! Units: 'nd', Desc: 'Off/On Switch'
    integer :: TABSIZE = 0                           ! Units: 'nd', Desc: 'Table Size'
    integer :: MODNO = 1                             ! Units: 'nd', Desc: 'Model Number'
    integer :: IP = 1                                ! Units: 'nd', Desc: 'Table Pointer'
    integer :: DQFLAG = 0                            ! Units: 'nd', Desc: 'Data Quality Flag (0=Data Not Loaded Successfully, 1=Data Loaded Successfully)'
    integer :: TIMEVARYING = 0                       ! Units: 'nd', Desc: 'Static Wind field or not
    real*8 :: zcoord(40) = 0                         ! Z coordinates of wind data in z (m)
    real*8 :: xcoord(40) = 0                         ! X coordinates of wind data in x (m)
    real*8 :: ycoord(40) = 0                         ! Y coordinates of wind data in y (m)
    real*8 :: xcoordT(500) = 0                       ! X coordinates of turbulence data in x (m)
    real*8 :: ycoordT(500) = 0                       ! Y coordinates of turbulence data in y (m)
    !real*8 :: terrain(40,40) = 0                     ! Terrain height in meters
    !!!REVISIT REVISIT REVIST - Always make sure this is back to normal
    ! real*8 :: U0(1,1,1),V0(1,1,1),W0(1,1,1)          ! U,V,W velocity at timestep t (m/s)
    ! real*8 :: Udt(1,1,1),Vdt(1,1,1),Wdt(1,1,1)       ! U,V,W velocity at timestep t+dt (m/s)
    ! real*8 :: UTURB(1,1),VTURB(1,1),WTURB(1,1)       ! U,V,W turbulence at timestep t (m/s)
    real*8 :: U0(40,40,40),V0(40,40,40),W0(40,40,40) ! U,V,W velocity at timestep t (m/s)
    real*8 :: Udt(40,40,40),Vdt(40,40,40),Wdt(40,40,40) ! U,V,W velocity at timestep t+dt (m/s)
    real*8 :: UTURB(500,500),VTURB(500,500),WTURB(500,500) ! U,V,W turbulence at timestep t (m/s)
    real*8 :: dx = 0                                 ! X resolution (m)
    real*8 :: dxT = 1                                ! XY resolution of turbulence (m)
    real*8 :: dyT = 1                                ! XY resolution of turbulence (m)
    real*8 :: dy = 0                                 ! X resolution (m)
    real*8 :: ztop = 0                               ! Highest altitude to sample from (m)
    real*8 :: IWINDSCALE = 1                         ! Scale of WRF winds
    real*8 :: TURBLEVEL = 1                          ! Scale of Turbulence 
    real*8 :: Vtrim = 20                             ! Trim velocity of craft flying through winds
    real*8 :: TIMESTEP = 0.0001                      ! Timestep of Simulation
    real*8 :: WINDGUST(3) = 0                        ! Vector holding WINDGUST values at current timestep (m/s)
    real*8 :: VWAKE(3) = 0                           ! Vector holding wake data values at current timestep (m/s)
    real*8 :: WRFX = 0                               ! Wrf winds in x (m/s)
    real*8 :: WRFY = 0                               ! Wrf winds in y (m/s)
    real*8 :: WRFZ = 0                               ! Wrf winds in z (m/s)
    real*8 :: ALT = 0.0                              ! Units: 'm', Desc: 'Altitude'
    real*8 :: DEN = 1.22566                          ! Units: 'kg/m^3', Desc: 'Density'
    real*8 :: WINDSPEED = 0.0                        ! Units: 'm/s', Desc: 'Wind Speed'
    real*8 :: WINDDIR = 0.0                          ! Units: 'rad', Desc: 'Wind Direction'
    real*8 :: WINDELEV = 0.0                         ! Units: 'rad', Desc: 'Wind Elevation'
    real*8 :: XI = 0.0                               ! Units: 'm', Desc: 'Inertial X Position'
    real*8 :: YI = 0.0                               ! Units: 'm', Desc: 'Inertial Y Position'
    real*8 :: ZI = 0.0                               ! Units: 'm', Desc: 'Inertial Z Position'
    real*8 :: TIME = 0.0
    real*8 :: VXWIND = 0.0                           ! Units: 'm/s', Desc: 'Atmospheric Wind Along Inertial I Axis'
    real*8 :: VYWIND = 0.0                           ! Units: 'm/s', Desc: 'Atmospheric Wind Along Inertial J Axis'
    real*8 :: VZWIND = 0.0                           ! Units: 'm/s', Desc: 'Atmospheric Wind Along Inertial K Axis'
    real*8 :: ALTTAB(MAXNALT) = 0.0                  ! Units: 'm', Desc: 'Density Altitude Table'
    real*8 :: DENTAB(MAXNALT) = 0.0                  ! Units: 'kg/m^3', Desc: 'Density Table'
    real*8 :: VXWINDTAB(MAXNALT) = 0.0               ! Units: 'm/s', Desc: 'VXWIND Wind Table'
    real*8 :: VYWINDTAB(MAXNALT) = 0.0               ! Units: 'm/s', Desc: 'VYWIND Wind Table'
    real*8 :: VZWINDTAB(MAXNALT) = 0.0               ! Units: 'm/s', Desc: 'VZWIND Wind Table'
    real*8 :: xshift = 0                             ! Units: 'm' , 'Desc: shift in x coordinate for interpolation
    real*8 :: yshift = 0                             ! Units: 'm' , 'Desc: shift in y coordinate for interpolation
    real*8 :: zshift = 0                             ! Units: 'm' , 'Desc: shift in y coordinate for interpolation
    real*8 :: xshiftT = 0                            ! Units: 'm' , 'Desc: shift in x coordinate for interpolation
    real*8 :: yshiftT = 0                            ! Units: 'm' , 'Desc: shift in y coordinate for interpolation
    real*8 :: zshiftT = 0                            ! Units: 'm' , 'Desc: shift in y coordinate for interpolation
    real*8 :: PSIOFFSET = 0                          ! Units: 'rad' ,'Desc: used to rotate the dryden and WRF model'
    real*8 :: WAVESPEED(2) = 0                       ! Units: 'm/s', Desc: Speed that the static waves propagate to simulate time varying component
    real*8 :: RAMPTIME = 0                           ! Units: 'sec', Desc: Time to ramp in winds
    character*128 PATH
    character*256 U0name,V0name,W0name,UTurbname,Vturbname,Wturbname
    character*256 Udtname,Vdtname,Wdtname
 end type ATMOSPHERESTRUCTURE
 type MAAWMSTRUCTURE
  character(128) :: FILEINPUTFILE = ' '            ! Units: 'nd', Desc: 'File of Files Input File'
  character(128) :: ATMOSPHEREINPUTFILE = ' '      ! Units: 'nd', Desc: 'Atmosphere Input File'
  character(128) :: AIRCRAFTINPUTFILE = ' '        ! Units: 'nd', Desc: 'Aircraft Input File'
  character(128) :: CSINPUTFILE = ' '              ! Units: 'nd', Desc: 'Control System Input File'
  character(128) :: SIMINPUTFILE = ' '             ! Units: 'nd', Desc: 'Simulation Input File'
  character(128) :: ICSINPUTFILE = ' '             ! Units: 'nd', Desc: 'Initial Condition Input File'
  character(128) :: WAYINPUTFILE = ' '              ! Units: 'nd', Desc:  'Waypoint Input File'
  character(128) :: RUNLOGFILE = ' '               ! Units: 'nd', Desc: 'Run Log File'
  character(128) :: STATEOUTPUTFILE = ' '          ! Units: 'nd', Desc: 'State Output File'
  character(128) :: MISCOUTPUTFILE = ' '           ! Units: 'nd', Desc: 'Miscellaneous Output File'
  character(128) :: CONTROLOUTPUTFILE = ' '        ! Units: 'nd', Desc: 'Control Output File'
  character(128) :: FORCEOUTPUTFILE = ' '          ! Units: 'nd', Desc: 'Force Output File'
  character(128) :: ERROROUTPUTFILE = ' '          ! Units: 'nd', Desc: 'Error Output File'
  character(128) :: WINDOUTPUTFILE = ' '           ! Units: 'nd', Desc: 'Wind Output File
  character(24) :: DATEANDTIMEOFRUN = ' '          ! Units: 'nd', Desc: 'Date and Time of Run'
  real*8 :: GRAVITY = 9.81                         ! Units: 'm/s^2', Desc: 'Gravity'
  type(ATMOSPHERESTRUCTURE) :: ATM
 end type MAAWMSTRUCTURE
END module MAAWMDATATYPES

PROGRAM MAIN
  use MAAWMDATATYPES
 implicit none
 integer openflag,readflag,LENGTH,i
 character(128) inputfilename
 character(12) inputfiletype
 type(MAAWMSTRUCTURE) T
 
 T%ATMOSPHEREINPUTFILE = 'Input_Files/WRF.ATM';  

 !Load Data
 call ATMOSPHERE(T,1)

 !!!!!!!!!!!!!!!!!!!!!!!!! Echo Data !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 T%RUNLOGFILE = 'WRF.LOG'

 openflag = 0
 open(unit=25,file=T%RUNLOGFILE,iostat=openflag)
 if (openflag .ne. 0) then
  write(*,*) 'Error Opening Run Log File: ',T%RUNLOGFILE;  STOP
 end if

 !Echo All other Data
 call ATMOSPHERE(T,2)

 !Set X,Y,Z,T of Atmosphere
 do i = 1,10
    T%ATM%TIME = 0
    T%ATM%XI = i
    T%ATM%YI = 0
    T%ATM%ZI = -200
 
    !Call ATMOSPHERE
    call ATMOSPHERE(T,3)

    !Print Values
    write(*,*) 'X,Y,Z = ',T%ATM%XI,T%ATM%YI,T%ATM%ZI
    write(*,*) 'Vx,Vy,Vz = ',T%ATM%VXWIND,T%ATM%VYWIND,T%ATM%VZWIND
 end do

END PROGRAM MAIN

SUBROUTINE ATMOSPHERE(T,iflag)
 use MAAWMDATATYPES
 implicit none
 integer i,iflag,ifind,openflag,readflag,ii,jj,ierr
 real*8 m,readreal,wind,dirnominal,windnominal,winddir
 real*8 dim,dummy(40),rx,test(3),ramp
 character*11 HeightFile
 character*14 ParametersFile
 character*1 letter
 character*10 ZcoordFile,number
 character*15 TimesFile
 character*256 inParameters,inZcoord,inTimes,inHeight,temp
 type(MAAWMSTRUCTURE) T

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! COMPUTE iflag = 3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!

 if (iflag .eq. 3) then  

    !!Reset all winds to zero
    T%ATM%VXWIND = 0
    T%ATM%VYWIND = 0
    T%ATM%VZWIND = 0
    T%ATM%WRFX = 0
    T%ATM%WRFY = 0
    T%ATM%WRFZ = 0
    T%ATM%WINDGUST(1) = 0
    T%ATM%WINDGUST(2) = 0
    T%ATM%WINDGUST(3) = 0

    T%ATM%ALT = -T%ATM%ZI

  ! Constant Model

  !!!!!REVISIT REVISIT REVISIT!!!!!!
    dirnominal = T%ATM%WINDDIR
    windnominal = 0.0
    winddir = T%ATM%WINDDIR
    wind = T%ATM%WINDSPEED
    
    if (T%ATM%MODNO .eq. 1) then
       T%ATM%VXWIND = wind*cos(winddir)*cos(T%ATM%WINDELEV)
       T%ATM%VYWIND = wind*sin(winddir)*cos(T%ATM%WINDELEV)
       T%ATM%VZWIND = -wind*sin(T%ATM%WINDELEV)
       T%ATM%DEN = 1.22566;
    end if

  ! Equation Density and Constant Wind Model

  if (T%ATM%MODNO .eq. 2) then
     T%ATM%DEN = 1.22566578494891*(1.00000000-0.0000225696709*T%ATM%ALT)**4.258
     T%ATM%VXWIND = T%ATM%WINDSPEED*cos(T%ATM%WINDDIR)
     T%ATM%VYWIND = T%ATM%WINDSPEED*sin(T%ATM%WINDDIR)
     T%ATM%VZWIND = 0.0
  end if

  ! Table Look-Up Model 

  if (T%ATM%MODNO .eq. 3) then

   ifind = 0

   ! Position Pointer  

   if (T%ATM%ALT .le. T%ATM%ALTTAB(T%ATM%IP)) then 
    ifind = -1 
    do while ((ifind.ne.0) .and. (T%ATM%IP.gt.1))
     T%ATM%IP = T%ATM%IP - 1 
     if (T%ATM%ALTTAB(T%ATM%IP)   .le. T%ATM%ALT) then 
     if (T%ATM%ALTTAB(T%ATM%IP+1) .gt. T%ATM%ALT) then 
      ifind = 0
     end if
     end if 
    end do 
   end if
   if (T%ATM%ALT .gt. T%ATM%ALTTAB(T%ATM%IP+1)) then 
    ifind = 1
    do while ((ifind.ne.0) .and. (T%ATM%IP.lt.T%ATM%TABSIZE-1))
     T%ATM%IP = T%ATM%IP + 1 
     if (T%ATM%ALTTAB(T%ATM%IP)   .le. T%ATM%ALT) then 
     if (T%ATM%ALTTAB(T%ATM%IP+1) .gt. T%ATM%ALT) then 
      ifind = 0 
     end if 
     end if 
    end do 
   end if
   if (ifind .eq. 0) then
    m = (T%ATM%ALT-T%ATM%ALTTAB(T%ATM%IP))/(T%ATM%ALTTAB(T%ATM%IP+1)-T%ATM%ALTTAB(T%ATM%IP))
   else if (ifind .eq. -1) then
    m = 0.0
   else if (ifind .eq. 1) then
    m = 1.0
   end if

   ! Interpolate

   T%ATM%DEN    = T%ATM%DENTAB(T%ATM%IP)    + m*(T%ATM%DENTAB(T%ATM%IP+1)-T%ATM%DENTAB(T%ATM%IP))
   T%ATM%VXWIND = T%ATM%VXWINDTAB(T%ATM%IP) + m*(T%ATM%VXWINDTAB(T%ATM%IP+1)-T%ATM%VXWINDTAB(T%ATM%IP))
   T%ATM%VYWIND = T%ATM%VYWINDTAB(T%ATM%IP) + m*(T%ATM%VYWINDTAB(T%ATM%IP+1)-T%ATM%VYWINDTAB(T%ATM%IP))
   T%ATM%VZWIND = T%ATM%VZWINDTAB(T%ATM%IP) + m*(T%ATM%VZWINDTAB(T%ATM%IP+1)-T%ATM%VZWINDTAB(T%ATM%IP))

  end if

  if (T%ATM%MODNO .eq. 4) then
     T%ATM%DEN = 1.22566; !Keep density constant at least for now
     call WRFMODEL(T) !Call WRF model. All velocities needed are set inside this subroutine
     !//Add in Full Field Dryden Gust model
     call TURBULENCE(T)
  end if

  !!Add all winds
  T%ATM%VXWIND = T%ATM%VXWIND + T%ATM%WINDGUST(1) + T%ATM%WRFX
  T%ATM%VYWIND = T%ATM%VYWIND + T%ATM%WINDGUST(2) + T%ATM%WRFY
  T%ATM%VZWIND = T%ATM%VZWIND + T%ATM%WINDGUST(3) + T%ATM%WRFZ

  !Ramp Winds in
  if (T%ATM%MODNO .eq. 4) then
     ramp = 0
     if (T%ATM%TIME .lt. T%ATM%RAMPTIME) then
        ramp = T%ATM%TIME/T%ATM%RAMPTIME
     else
        ramp = 1
     end if
     T%ATM%VXWIND = ramp*T%ATM%VXWIND
     T%ATM%VYWIND = ramp*T%ATM%VYWIND
     T%ATM%VZWIND = ramp*T%ATM%VZWIND
  end if

  RETURN

end if

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ECHO DATA iflag = 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

if (iflag .eq. 2) then 
  write(25,*) ' '
  write(25,*) 'AAAA TTTT M   M OOOO SSSS PPPP H  H EEEE RRR  EEEE'
  write(25,*) 'A  A  TT  MM MM O  O SS   PP P H  H EE   RR R EE  '
  write(25,*) 'AAAA  TT  M M M O  O SSSS PPPP HHHH EEEE RRRR EEEE'
  write(25,*) 'A  A  TT  M   M O  O   SS PP   H  H EE   RRR  EE  '
  write(25,*) 'A  A  TT  M   M OOOO SSSS PP   H  H EEEE RR R EEEE'
  write(25,*) ' '
  write(25,*) 'Atmosphere Input File: '
  write(25,*) trim(T%ATMOSPHEREINPUTFILE)
  write(25,*) ' '
  write(25,*) 'Model Number (0=Constant, 1=Equation, 2=Table): ',T%ATM%MODNO
  write(25,*) ' '
  if (T%ATM%MODNO .eq. 1) then
   write(25,*) 'Density (kg/m^3): ',T%ATM%DEN
   write(25,*) 'Wind Speed (m/s): ',T%ATM%WINDSPEED
   write(25,*) 'Wind Direction (deg): ',57.3*T%ATM%WINDDIR
   write(25,*) 'Wind Elevation (deg): ',57.3*T%ATM%WINDELEV
   write(25,*) ' '
  end if
  if (T%ATM%MODNO .eq. 2) then
   write(25,*) 'Wind Speed (m/s): ',T%ATM%WINDSPEED
   write(25,*) 'Wind Direction (deg): ',57.3*T%ATM%WINDDIR
   write(25,*) 'Wind Elevation (deg): ',57.3*T%ATM%WINDELEV
   write(25,*) ' '
  end if
  if (T%ATM%MODNO .eq. 3) then
   write(25,*) 'Altitude (m), Density (kg/m^3), VX Wind Speed (m/s), VY Wind Speed (m/s), VZ Wind Speed (m/s)'
   write(25,*) '---------------------------------------------------------------------------------------------'
   do i=1,T%ATM%TABSIZE  
    write(25,fmt='(5e18.8)') T%ATM%ALTTAB(i),T%ATM%DENTAB(i),T%ATM%VXWINDTAB(i),T%ATM%VYWINDTAB(i),T%ATM%VZWINDTAB(i)
   end do
   write(25,*) ' '
  end if
  if (T%ATM%MODNO .eq. 4) then
     write(25,*) 'Using Wind File = ',T%ATM%PATH 
     write(25,*) 'Grid Size of WRF model = ',T%ATM%dx,' m'
     write(25,*) 'Maximum Height of WRF model = ',T%ATM%ztop,' m'
     write(25,*) 'Grid Size of Turbulence(m) = ', T%ATM%dxT
     write(25,*) 'Wind scale = ',T%ATM%IWINDSCALE
     write(25,*) 'Turbulence Scale = ',T%ATM%TURBLEVEL
     write(25,*) 'Heading Offset in WRF model(rad) = ',T%ATM%PSIOFFSET
     write(25,*) 'Wavespeed in X and Y (m/s) = ',T%ATM%WAVESPEED(1),T%ATM%WAVESPEED(2)
     write(25,*) 'Ramp Time (sec) = ',T%ATM%RAMPTIME
  end if
  write(25,*) 'Data Quality Flag (nd, 0=Data Not Loaded Successfully, 1=Data Loaded Successfully): ',T%ATM%DQFLAG

  RETURN
  
end if
  
!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOAD DATA iflag = 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 if (iflag .eq. 1) then

    write(*,*) 'Loading Atmosphere File'
    
  open(unit=94,file=T%ATMOSPHEREINPUTFILE,status='old',iostat=openflag)
  if (openflag .ne. 0) then
   write(*,*) 'Error Opening Atmosphere Input File: ',T%ATMOSPHEREINPUTFILE;  STOP
  else
     write(*,*) 'Atmosphere File opened'
  end if
  rewind(94)
  
  read(unit=94,fmt=*,iostat=readflag) readreal; T%ATM%MODNO = readreal
  if (T%ATM%MODNO .eq. 1) then
   read(unit=94,fmt=*,iostat=readflag) T%ATM%DEN
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDSPEED
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDDIR
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDELEV
  end if
  if (T%ATM%MODNO .eq. 2) then
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDSPEED
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDDIR
   read(unit=94,fmt=*,iostat=readflag) T%ATM%WINDELEV
  end if
  if (T%ATM%MODNO .eq. 3) then
   read(unit=94,fmt=*,iostat=readflag) readreal; T%ATM%TABSIZE = readreal
   do i=1,T%ATM%TABSIZE  
    read(unit=94,fmt=*,iostat=readflag) T%ATM%ALTTAB(i),T%ATM%DENTAB(i),T%ATM%VXWINDTAB(i),T%ATM%VYWINDTAB(i),T%ATM%VZWINDTAB(i)
   end do
  end if
  if (T%ATM%MODNO .eq. 4) then
     !Atmospheric Density
     read(unit=94,fmt=*,iostat=readflag) T%ATM%DEN
     read(unit=94,fmt=*,iostat=readflag) T%ATM%IWINDSCALE
     read(unit=94,fmt=*,iostat=readflag) T%ATM%TURBLEVEL
     read(unit=94,fmt=*,iostat=readflag) T%ATM%PSIOFFSET
     read(unit=94,fmt=*,iostat=readflag) T%ATM%RAMPTIME
     read(unit=94,fmt=*,iostat=readflag) T%ATM%WAVESPEED(1)
     read(unit=94,fmt=*,iostat=readflag) T%ATM%WAVESPEED(2)
     !Read PATH
     read(unit=94,fmt=*,iostat=readflag) T%ATM%PATH

     !%%%%%%%%Import Extra Parameters File%%%%%%%%% */

     ParametersFile = 'Parameters.txt';
     inParameters = trim(T%ATM%PATH)//ParametersFile
     open(unit=78,file=inParameters,status='old',iostat=ierr)
     if (ierr .ne. 0) then
        write(*,*) 'Parameters.txt File defined incorrectly. Check to make sure your path is set correctly'
        write(*,*) 'Check that your path is defined correctly in .ATM file.'
        write(*,*) ' This is your current Path:  ',T%ATM%PATH
        STOP;
     endif
     read(unit=78,fmt=*) T%ATM%parameters
     close(78)
  
     ZcoordFile = 'Zcoord.txt';
     inZcoord = trim(T%ATM%PATH)//ZcoordFile
     open(unit=78,file=inZcoord,status='old',iostat=ierr)
     if (ierr .ne. 0) then
        write(*,*) 'Zcoord.txt File defined incorrectly. Check to make sure your path is set correctly'
        write(*,*) 'Check that your path is defined correctly in .ATM file.'
        write(*,*) ' This is your current Path:  ',T%ATM%PATH
        STOP;
     endif
     read(78,*) T%ATM%zcoord
     close(78)
  
     TimesFile = 'SampleTimes.txt';
     inTimes = trim(T%ATM%PATH)//TimesFile
     open(unit=78,file=inTimes,status='old',iostat=ierr)
     if (ierr .ne. 0) then
        write(*,*) 'SampleTimes.txt File defined incorrectly. Check to make sure your path is set correctly'
        write(*,*) 'Check that your path is defined correctly in .ATM file.'
        write(*,*) ' This is your current Path:  ',T%ATM%PATH
        STOP;
     endif
     read(78,*) T%ATM%tcoord
     close(78)

     !%%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% */

     T%ATM%dx = T%ATM%parameters(1);
     T%ATM%dy = T%ATM%parameters(2);
     T%ATM%ztop = T%ATM%parameters(3);

     !%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

     !%%%%%%%%%%%Create xcoord and ycoord%%%%%%% */
     dim = T%ATM%dim
     do ii = 1,dim
        T%ATM%xcoord(ii) = -T%ATM%dx*(dim-1)/2 + (ii-1)*T%ATM%dx
        T%ATM%ycoord(ii) = -T%ATM%dy*(dim-1)/2 + (ii-1)*T%ATM%dy
     enddo

     !%%%%%%%Import Initial UVW matrices%%%%%%%%% */
  
     write(number, '(i1)' )  T%ATM%tcoord(1)
     letter = trim('U')
     T%ATM%U0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('V')
     T%ATM%V0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('W')
     T%ATM%W0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     write(number, '(i1)' )  T%ATM%tcoord(2)
     T%ATM%Wdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('U')
     T%ATM%Udtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
     letter = trim('V')
     T%ATM%Vdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'

     call IMPORTWIND(T%ATM%U0,T%ATM%U0name);
     call IMPORTWIND(T%ATM%Udt,T%ATM%Udtname);
     call IMPORTWIND(T%ATM%V0,T%ATM%V0name);
     call IMPORTWIND(T%ATM%Vdt,T%ATM%Vdtname);
     call IMPORTWIND(T%ATM%W0,T%ATM%W0name);
     call IMPORTWIND(T%ATM%Wdt,T%ATM%Wdtname);
  
     !write(*,*) 'UVW Initialization Complete'

     write(*,*) 'Dryden Initialization '

     !%%%%%%%%%%%Initialize Variables%%%%%%%%%%%%% */

     !Allocate memory for WRF interpolation
     T%ATM%boundsT = 0;
     T%ATM%boundflagT = 1;

     !%%%%%%%%%%Unwrap Parameters%%%%%%%%%%%%%%% 

     !%%%%%%%%%%%Create xcoord and ycoord%%%%%%%
     do ii = 1,T%ATM%dimT
        T%ATM%xcoordT(ii) = -T%ATM%dxT * (T%ATM%dimT - 1) / 2 + ii * T%ATM%dxT;
        T%ATM%ycoordT(ii) = -T%ATM%dyT * (T%ATM%dimT - 1) / 2 + ii * T%ATM%dyT;
     end do
     !%%%%%%%Import Initial UVW matrices%%%%%%%%%

     T%ATM%Uturbname = trim(T%ATM%PATH)//trim('Uturb.txt')
     T%ATM%Vturbname = trim(T%ATM%PATH)//trim('Vturb.txt')
     T%ATM%Wturbname = trim(T%ATM%PATH)//trim('Wturb.txt')

     call IMPORTTURB(T%ATM%UTURB, T%ATM%UTurbname);
     call IMPORTTURB(T%ATM%VTURB, T%ATM%VTurbname);
     call IMPORTTURB(T%ATM%WTURB, T%ATM%WTurbname);
     
     write(*,*) 'Turbulence Initialization Complete'

     close(94) 
     write(*,*) 'ATMOSPHERE Load Complete'
  
  end if

  T%ATM%DQFLAG = 1
  
  RETURN
end if
 
RETURN
END SUBROUTINE ATMOSPHERE

SUBROUTINE WRFMODEL(T)
  use MAAWMDATATYPES
  implicit none
  integer stepX,stepY,stepZ,stepT,extrapX,extrapY,extrapZ,extrapT,cord2(2)
  integer markX,markY,markZ,markT,gust,body
  integer tinterp,x1,x2,y1,y2,z1,z2,tt,ii,coord1(4),coord2(4),cord1(2)
  real*8 uvw(3,2),xpts2(2),ypts2(2),zpts2(2),zpts1,xpts1,ypts1,rx;
  real*8 u8(8),v8(8),w8(8),u4(4),v4(4),w4(4),uslope,vslope,wslope;
  real*8 u2(2),v2(2),w2(2),u,v,w,tpts(2),Lu,Lv,Lw,sigw,sigu,sigv,tstar;
  real*8 ugo,vgo,wgo,vatm(3),xstar,ystar,zstar,xtemp,vtemp,counter,xwidth,ywidth
  real*8 xi,yi,zi
  character*1 letter
  character*10 number
  type(MAAWMSTRUCTURE) T

  !   /*   %%This function will take in x,y,z(m),t(sec) and location and  */
  !   /*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */

  xi = T%ATM%XI
  yi = T%ATM%YI
  zi = T%ATM%ZI

  xtemp = xi
  xi = xtemp*cos(T%ATM%PSIOFFSET) + yi*sin(T%ATM%PSIOFFSET);
  yi = -xtemp*sin(T%ATM%PSIOFFSET) + yi*cos(T%ATM%PSIOFFSET);

  !!Extra term is from phase offset to add time-varying component
  xi = xi - T%ATM%WAVESPEED(1)*(T%ATM%TIME)
  yi = yi - T%ATM%WAVESPEED(2)*(T%ATM%TIME)

  !Now shift the WRF Grid so that XI,YI and ZI fall in side the cube
  T%ATM%xshift = 0
  T%ATM%yshift = 0

  xwidth = T%ATM%xcoord(T%ATM%dim)-T%ATM%xcoord(1)
  ywidth = T%ATM%ycoord(T%ATM%dim)-T%ATM%ycoord(1)

  !%Find markX
  if (xi .gt. T%ATM%xcoord(T%ATM%dim)) then
     T%ATM%xshift = -floor(abs(xi-T%ATM%xcoord(1))/xwidth)
  else if (xi .lt. T%ATM%xcoord(1)) then 
     T%ATM%xshift = floor(abs(xi-T%ATM%xcoord(T%ATM%dim))/xwidth)
  end if

  !%Find markY
  if (yi .gt. T%ATM%ycoord(T%ATM%dim)) then
     T%ATM%yshift = -floor(abs(yi-T%ATM%ycoord(1))/ywidth)
  else if (yi .lt. T%ATM%ycoord(1)) then 
     T%ATM%yshift = floor(abs(yi-T%ATM%ycoord(T%ATM%dim))/ywidth)
  end if

  xstar = xi + T%ATM%xshift*xwidth
  ystar = yi + T%ATM%yshift*ywidth
  zstar = -zi

  ! if (abs(xstar) .gt. T%ATM%xcoord(T%ATM%dim)) then
  !    write(*,*) 'x = ',xi,xstar
  !    PAUSE
  ! end if
  ! if (abs(ystar) .gt. T%ATM%ycoord(T%ATM%dim)) then
  !    write(*,*) 'y = ',yi,ystar
  !    PAUSE
  ! end if

  !write(*,*) xi,xstar

  if (T%ATM%IWINDSCALE .gt. 0) then
     tstar = T%ATM%TIME
     stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
     extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
     if (zstar .lt. 0) then
        zstar = -zstar
     endif
     tinterp = 2;

     uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
     uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

     markX = T%ATM%markX
     markY = T%ATM%markY
     markZ = T%ATM%markZ
     markT = T%ATM%markT

     !%%Check X
     if (markX .eq. T%ATM%dim) then
        markX = markX - 1;
     end if
     if ((xstar .ge. T%ATM%xcoord(markX)) .and. (xstar .le. T%ATM%xcoord(markX+1))) then
        !%%Your in between the markers so keep going
     else
        call FIND(T%ATM%xcoord,T%ATM%dim,xstar,markX)
        if (markX .eq. T%ATM%dim) then
           markX = markX - 1;
        else if (markX .le. 0) then
           markX = 1;
        end if
     end if
     !%%Check Y
     if (markY .eq. T%ATM%dim) then
        markY = markY - 1;
     end if
     if ((ystar .ge. T%ATM%ycoord(markY)) .and. (ystar .le. T%ATM%ycoord(markY+1))) then
        !%%Your in between the markers so keep going
     else
        call FIND(T%ATM%ycoord,T%ATM%dim,ystar,markY)
        if (markY .eq. T%ATM%dim) then
           markY = markY - 1;
        else if (markY .le. 0) then
           markY = 1;
        end if
     end if
     !%%Check Z
     if (markZ .eq. T%ATM%dim) then
        markZ = markZ - 1;
     end if

     if ((zstar .ge. T%ATM%zcoord(markZ)) .and. (zstar .le. T%ATM%zcoord(markZ+1))) then
        !%%Your in between the markers so keep going
     else
        !%Find markZ
        if (zstar .gt. T%ATM%zcoord(T%ATM%dim)) then
           !%use endpt
           markZ = T%ATM%dim;
           stepZ = -1;
           extrapZ = 1;
        else if (zstar .lt. T%ATM%zcoord(1)) then
           markZ = 1;
           stepZ = 1;
           extrapZ = 1;
        else
           call FIND(T%ATM%zcoord,T%ATM%dim,zstar,markZ)
           if (markZ .eq. T%ATM%dim) then
              markZ = markZ - 1;
           else if (markZ .eq. 0) then
              markZ = 1;
           end if
        end if
     end if
     !%%Check T
     if (markT .eq. T%ATM%tlength) then
        markT = markT - 1;
     end if
     if ((tstar .ge. T%ATM%tcoord(markT)) .and. (tstar .le. T%ATM%tcoord(markT+1))) then
        !%%Your in between the markers so keep going
     else
        if (T%ATM%TIMEVARYING .eq. 1) then
           !%Find markT
           if (tstar .gt. T%ATM%tcoord(T%ATM%tlength)) then
              !%use endpt
              markT = T%ATM%tlength;
              extrapT = 1;
           else if (tstar .lt. T%ATM%tcoord(1)) then
              !%use start pt
              markT = 1;
              extrapT = 1;
           else
              call FIND2(T%ATM%tcoord,T%ATM%tlength,tstar,markT)
              if (markT .eq. T%ATM%tlength) then
                 markT = markT - 1;
              else if (markT .eq. 0) then
                 markT = 1;
              end if
           end if

           !%%Import U,V,W maps since markT changed
           if (T%ATM%tcoord(markT) .lt. 10) then
              write(number, '(i1)' )  T%ATM%tcoord(markT)
           else
              if (T%ATM%tcoord(markT) .le. 99) then
                 write(number, '(i2)' )  T%ATM%tcoord(markT)
              else
                 write(number, '(i3)' )  T%ATM%tcoord(markT)
              endif
           endif
           letter = trim('U')
           T%ATM%U0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
           letter = trim('V')
           T%ATM%V0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
           letter = trim('W')
           T%ATM%W0name = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
           !%%only import at markT
           call IMPORTWIND(T%ATM%U0,T%ATM%U0name);
           call IMPORTWIND(T%ATM%V0,T%ATM%V0name);
           call IMPORTWIND(T%ATM%W0,T%ATM%W0name);
           !%U0 = U0(end:-1:1,end:-1:1,:);
           !%V0 = V0(end:-1:1,end:-1:1,:);
           !%W0 = W0(end:-1:1,end:-1:1,:);
           if (extrapT .eq. 1) then
              tinterp = 1;
           else
              !%%import markT + 1
              if (T%ATM%tcoord(markT+1) .lt. 10) then
                 write(number, '(i1)' )  T%ATM%tcoord(markT+1)
              else if (T%ATM%tcoord(markT+1) .le. 99) then
                 write(number, '(i2)' )  T%ATM%tcoord(markT+1)
              else
                 write(number, '(i3)' )  T%ATM%tcoord(markT+1)
              endif
              letter = trim('U')
              T%ATM%Udtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
              letter = trim('V')
              T%ATM%Vdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
              letter = trim('W')
              T%ATM%Wdtname = trim(T%ATM%PATH)//trim(letter)//trim(number)//'.txt'
              call IMPORTWIND(T%ATM%Udt,T%ATM%Udtname);
              call IMPORTWIND(T%ATM%Vdt,T%ATM%Vdtname);
              call IMPORTWIND(T%ATM%Wdt,T%ATM%Wdtname);
           end if !(extrapT eq. 1 ) 
        else !Not time varyine
           tinterp = 1
        end if
     end if

     !%%Interpolation Scheme
     do tt = 1,tinterp 
        !%Interpolate Spatially

        !%%To start we have 8 discrete point (8 corners of a cube)
        xpts2(1) = T%ATM%xcoord(markX)
        xpts2(2) = T%ATM%xcoord(markX+stepX);
        ypts2(1) = T%ATM%ycoord(markY)
        ypts2(2) = T%ATM%ycoord(markY+stepY);
        zpts2(1) = T%ATM%zcoord(markZ)
        zpts2(2) = T%ATM%zcoord(markZ+stepZ);
        x1 = markX;x2 = markX+stepX;
        y1 = markY;y2 = (markY+stepY);
        z1 = markZ;z2 = markZ+stepZ;
        if (tt .eq. 1) then
           !%%Use U0,V0,W0
           u8(1) = T%ATM%U0(y1,x1,z1);
           u8(2) = T%ATM%U0(y1,x2,z1);
           u8(3) = T%ATM%U0(y2,x2,z1);
           u8(4) = T%ATM%U0(y2,x1,z1);
           u8(5) = T%ATM%U0(y1,x1,z2);
           u8(6) = T%ATM%U0(y1,x2,z2);
           u8(7) = T%ATM%U0(y2,x2,z2);
           u8(8) = T%ATM%U0(y2,x1,z2);
           v8(1) = T%ATM%V0(y1,x1,z1);
           v8(2) = T%ATM%V0(y1,x2,z1);
           v8(3) = T%ATM%V0(y2,x2,z1);
           v8(4) = T%ATM%V0(y2,x1,z1);
           v8(5) = T%ATM%V0(y1,x1,z2);
           v8(6) = T%ATM%V0(y1,x2,z2);
           v8(7) = T%ATM%V0(y2,x2,z2);
           v8(8) = T%ATM%V0(y2,x1,z2);
           w8(1) = T%ATM%W0(y1,x1,z1);
           w8(2) = T%ATM%W0(y1,x2,z1);
           w8(3) = T%ATM%W0(y2,x2,z1);
           w8(4) = T%ATM%W0(y2,x1,z1);
           w8(5) = T%ATM%W0(y1,x1,z2);
           w8(6) = T%ATM%W0(y1,x2,z2);
           w8(7) = T%ATM%W0(y2,x2,z2);
           w8(8) = T%ATM%W0(y2,x1,z2);
        else
           !%%Use Udt,Vdt,Wdt
           u8(1) = T%ATM%Udt(y1,x1,z1);
           u8(2) = T%ATM%Udt(y1,x2,z1);
           u8(3) = T%ATM%Udt(y2,x2,z1);
           u8(4) = T%ATM%Udt(y2,x1,z1);
           u8(5) = T%ATM%Udt(y1,x1,z2);
           u8(6) = T%ATM%Udt(y1,x2,z2);
           u8(7) = T%ATM%Udt(y2,x2,z2);
           u8(8) = T%ATM%Udt(y2,x1,z2);
           v8(1) = T%ATM%Vdt(y1,x1,z1);
           v8(2) = T%ATM%Vdt(y1,x2,z1);
           v8(3) = T%ATM%Vdt(y2,x2,z1);
           v8(4) = T%ATM%Vdt(y2,x1,z1);
           v8(5) = T%ATM%Vdt(y1,x1,z2);
           v8(6) = T%ATM%Vdt(y1,x2,z2);
           v8(7) = T%ATM%Vdt(y2,x2,z2);
           v8(8) = T%ATM%Vdt(y2,x1,z2);
           w8(1) = T%ATM%Wdt(y1,x1,z1);
           w8(2) = T%ATM%Wdt(y1,x2,z1);
           w8(3) = T%ATM%Wdt(y2,x2,z1);
           w8(4) = T%ATM%Wdt(y2,x1,z1);
           w8(5) = T%ATM%Wdt(y1,x1,z2);
           w8(6) = T%ATM%Wdt(y1,x2,z2);
           w8(7) = T%ATM%Wdt(y2,x2,z2);
           w8(8) = T%ATM%Wdt(y2,x1,z2);
        end if


        !%%%%%interpZ%%%%%%%%%%%%

        if (extrapZ .eq. 1) then
           !%%You don't need to interpolate on z and you can just use
           !%%the values at markZ or z1
           zpts1 = zpts2(1);
           u4(1) = u8(1);
           u4(2) = u8(2);
           u4(3) = u8(3);
           u4(4) = u8(4);
           v4(1) = v8(1);
           v4(2) = v8(2);
           v4(3) = v8(3);
           v4(4) = v8(4);
           w4(1) = w8(1);
           w4(2) = w8(2);
           w4(3) = w8(3);
           w4(4) = w8(4);
           T%ATM%bounds = 1;
        else
           !%%Interpolate Between Z points(interpolate pts 1-4 and 5-8)
           !%Pts 1,5 : 2,6 : 3,7 : 4,8
           coord1 = (/1,2,3,4/);
           coord2 = (/5,6,7,8/);
           do ii = 1,4
              uslope = (u8(coord2(ii))-u8(coord1(ii)))/(zpts2(2)-zpts2(1));
              vslope = (v8(coord2(ii))-v8(coord1(ii)))/(zpts2(2)-zpts2(1));
              wslope = (w8(coord2(ii))-w8(coord1(ii)))/(zpts2(2)-zpts2(1));
              u4(ii) = uslope*(zstar-zpts2(1))+u8(coord1(ii));
              v4(ii) = vslope*(zstar-zpts2(1))+v8(coord1(ii));
              w4(ii) = wslope*(zstar-zpts2(1))+w8(coord1(ii));
           end do
           zpts1 = zstar;
        end if

        !%%%%%interpY%%%%%%%%%%%

        if (extrapY .eq. 1) then
           !%%You don't need to interpolate on y
           ypts1 = ypts2(1);
           u2(1) = u4(1);
           u2(2) = u4(2);
           v2(1) = v4(1);
           v2(2) = v4(2);
           w2(1) = w4(1);
           w2(2) = w4(2);
           T%ATM%bounds = 1;
        else
           !%%Interpolate between Y points(interpolate pts 1-2 and 3-4)
           !%%Pts 1,4 : 2,3
           cord1 = (/1,2/);
           cord2 = (/4,3/);
           do ii = 1,2
              uslope = (u4(cord2(ii))-u4(cord1(ii)))/(ypts2(2)-ypts2(1));
              vslope = (v4(cord2(ii))-v4(cord1(ii)))/(ypts2(2)-ypts2(1));
              wslope = (w4(cord2(ii))-w4(cord1(ii)))/(ypts2(2)-ypts2(1));
              u2(ii) = uslope*(ystar-ypts2(1))+u4(cord1(ii));
              v2(ii) = vslope*(ystar-ypts2(1))+v4(cord1(ii));
              w2(ii) = wslope*(ystar-ypts2(1))+w4(cord1(ii));
           end do
           ypts1 = ystar;
        end if

        !%%%%interpX%%%%%%%%%%%%
        if (extrapX .eq. 1) then
           !%%You don't need to interpolate on x
           xpts1 = xpts2(1);
           u = u2(1);
           v = v2(1);
           w = w2(1);
           T%ATM%bounds = 1;
        else
           !%%Interpolate between X points
           uslope = (u2(2)-u2(1))/(xpts2(2)-xpts2(1));
           vslope = (v2(2)-v2(1))/(xpts2(2)-xpts2(1));
           wslope = (w2(2)-w2(1))/(xpts2(2)-xpts2(1));
           u = uslope*(xstar-xpts2(1))+u2(1);
           v = vslope*(xstar-xpts2(1))+v2(1);
           w = wslope*(xstar-xpts2(1))+w2(1);
           xpts1 = xstar;
        end if

        !%%%%Save wind values%%%%%

        uvw(1,tt) = u;
        uvw(2,tt) = v;
        uvw(3,tt) = w;

     end do

     if (T%ATM%TIMEVARYING .eq. 1) then
        if (extrapT .eq. 1) then
           !//Answer is just first entry of uvw
           vatm(1) = uvw(1,1);
           vatm(2) = uvw(2,1);
           vatm(3) = uvw(3,1);
        else
           !%%Interpolate on T
           tpts(1) = T%ATM%tcoord(markT)
           tpts(2) = T%ATM%tcoord(markT+1);
           u2(1) = uvw(1,1);
           u2(2) = uvw(1,2);
           v2(1) = uvw(2,1);
           v2(2) = uvw(2,2);
           w2(1) = uvw(3,1);
           w2(2) = uvw(3,2);
           uslope = (u2(2)-u2(1))/(tpts(2)-tpts(1));
           vslope = (v2(2)-v2(1))/(tpts(2)-tpts(1));
           wslope = (w2(2)-w2(1))/(tpts(2)-tpts(1));  
           u = uslope*(tstar-tpts(1))+u2(1);
           v = vslope*(tstar-tpts(1))+v2(1);
           w = wslope*(tstar-tpts(1))+w2(1);
           vatm(1) = u;
           vatm(2) = v;
           vatm(3) = w;
        end if
     else
        !//Answer is just first entry of uvw
        vatm(1) = uvw(1,1);
        vatm(2) = uvw(2,1);
        vatm(3) = uvw(3,1);
     end if
     
     !Rotate by PSIOFFSET
     vtemp = vatm(1)
     vatm(1) = vtemp*cos(T%ATM%PSIOFFSET) + vatm(2)*sin(T%ATM%PSIOFFSET);
     vatm(2) = -vtemp*sin(T%ATM%PSIOFFSET) + vatm(2)*cos(T%ATM%PSIOFFSET);

     !//Multiply by scale
     vatm(1) = T%ATM%IWINDSCALE*vatm(1)
     vatm(2) = T%ATM%IWINDSCALE*vatm(2)
     vatm(3) = T%ATM%IWINDSCALE*vatm(3)

     if ((T%ATM%bounds .eq. 1) .and. (T%ATM%boundflag .eq. 1)) then
        !write(*,*) 'You went out of bounds at T = ',tstar,' XYZ = ',xstar,ystar,zstar
        T%ATM%boundflag = 0;
     end if
  else
     vatm(1) = 0
     vatm(2) = 0
     vatm(3) = 0
  endif

  ! if (vatm(3) .gt. 4) then
  !    write(*,*) xi,xstar,T%ATM%xshift
  !    write(*,*) yi,ystar,T%ATM%yshift
  !    write(*,*) zi,zstar,T%ATM%zshift
  !    write(*,*) vatm(1),vatm(2),vatm(3)
  !    write(*,*) 'Ypts2 = ',ypts2
  !    write(*,*) u2
  !    write(*,*) v2
  !    write(*,*) w2
  !    STOP
  ! end if
  
  T%ATM%WRFX = vatm(1)
  T%ATM%WRFY = vatm(2)
  T%ATM%WRFZ = vatm(3)

  T%ATM%markX = markX
  T%ATM%markY = markY
  T%ATM%markZ = markZ
  T%ATM%markT = markT

END SUBROUTINE WRFMODEL

SUBROUTINE TURBULENCE(T)
  use MAAWMDATATYPES
  implicit none
  integer stepX,stepY,stepZ,stepT,extrapX,extrapY,extrapZ,extrapT,cord2(2)
  integer markX,markY,markZ,markT,gust,body
  integer tinterp,x1,x2,y1,y2,z1,z2,tt,ii,coord1(4),coord2(4),cord1(2)
  real*8 uvw(3,2),xpts2(2),ypts2(2),zpts2(2),zpts1,xpts1,ypts1,rx;
  real*8 u8(8),v8(8),w8(8),u4(4),v4(4),w4(4),uslope,vslope,wslope;
  real*8 u2(2),v2(2),w2(2),u,v,w,tpts(2),Lu,Lv,Lw,sigw,sigu,sigv,tstar;
  real*8 ugo,vgo,wgo,vatm(3),xstar,ystar,zstar,xtemp,vtemp,counter
  character*1 letter
  character*10 number
  type(MAAWMSTRUCTURE) T

  !   /*   %%This function will take in x,y,z(m),t(sec) and location and  */
  !   /*   %return u,v,w(m/s). This uses a fast quad-linear interpolation */
  !   /*   %so many globals must be defined. location is a string that */
  !   /*   %contains the location of the data to be interpolated. */

  xtemp = T%ATM%XI
  T%ATM%XI = xtemp*cos(T%ATM%PSIOFFSET) + T%ATM%YI*sin(T%ATM%PSIOFFSET);
  T%ATM%YI = -xtemp*sin(T%ATM%PSIOFFSET) + T%ATM%YI*cos(T%ATM%PSIOFFSET);

  xstar = T%ATM%XI + T%ATM%xshiftT
  ystar = T%ATM%YI + T%ATM%yshiftT
  zstar = T%ATM%ZI + T%ATM%zshiftT

  if (T%ATM%TURBLEVEL .gt. 0) then
     stepX = 1;stepY = 1;stepZ = 1;stepT = 1;
     extrapX = 0;extrapY = 0;extrapZ = 0;extrapT = 0;
     if (zstar .lt. 0) then
        zstar = -zstar
     endif
     tinterp = 2;

     uvw(1,1)=0;uvw(2,1)=0;uvw(3,1)=0;
     uvw(1,2)=0;uvw(2,2)=0;uvw(3,2)=0;

     markX = T%ATM%markXT
     markY = T%ATM%markYT

     !%%Check X
     if (markX .eq. T%ATM%dimT) then
        markX = markX - 1;
     end if
     if ((xstar .ge. T%ATM%xcoordT(markX)) .and. (xstar .le. T%ATM%xcoordT(markX+1))) then
        !%%Your in between the markers so keep going
     else
        !%Find markX
        if (xstar .gt. T%ATM%xcoordT(T%ATM%dimT)) then
           !%use endpt
           markX = T%ATM%dimT
           stepX = -1
           extrapX = 1
           counter = 0
           do while ((T%ATM%xshiftT + T%ATM%XI .gt. T%ATM%xcoordT(T%ATM%dimT)) .and. (counter .lt. 20))
              T%ATM%xshiftT = -1.9 * T%ATM%xcoordT(T%ATM%dimT) + T%ATM%xshiftT
              counter = counter + 1
           end do
        else if (xstar .lt. T%ATM%xcoordT(1)) then 
           !%use starpt
           markX = 1;
           stepX = 1;
           extrapX = 1;    
           counter = 0
           do while ((T%ATM%xshiftT + T%ATM%XI .lt. T%ATM%xcoordT(1)) .and. (counter .lt. 20))
              T%ATM%xshiftT = 1.9 * T%ATM%xcoordT(T%ATM%dimT) + T%ATM%xshiftT
              counter = counter + 1
           end do
        else
           call FIND(T%ATM%xcoordT,T%ATM%dimT,xstar,markX)
           if (markX .eq. T%ATM%dimT) then
              markX = markX - 1;
           else if (markX .le. 0) then
              markX = 1;
           end if
        end if
     end if
     !%%Check Y
     if (markY .eq. T%ATM%dimT) then
        markY = markY - 1;
     end if
     if ((ystar .ge. T%ATM%ycoordT(markY)) .and. (ystar .le. T%ATM%ycoordT(markY+1))) then
        !%%Your in between the markers so keep going
     else
        !%Find markY
        if (ystar .gt. T%ATM%ycoordT(T%ATM%dimT)) then
           !%use endpt
           markY = T%ATM%dimT;
           stepY = -1;
           extrapY = 1;
           counter = 0
           do while ((T%ATM%yshiftT + T%ATM%YI .gt. T%ATM%ycoordT(T%ATM%dimT)) .and. (counter .lt. 20))
              T%ATM%yshiftT = -1.9 * T%ATM%ycoordT(T%ATM%dimT) + T%ATM%yshiftT
              counter = counter + 1
           end do
        else if (ystar .lt. T%ATM%ycoordT(1)) then
           markY = 1;
           stepY = 1;
           extrapY = 1;
           counter = 0
           do while ((T%ATM%yshiftT + T%ATM%YI .lt. T%ATM%ycoordT(1)) .and. (counter .lt. 20))
              T%ATM%yshiftT = 1.9 * T%ATM%ycoordT(T%ATM%dimT) + T%ATM%yshiftT
              counter = counter + 1;
           end do
        else
           call FIND(T%ATM%ycoordT,T%ATM%dimT,ystar,markY)
           if (markY .eq. T%ATM%dimT) then
              markY = markY - 1;
           else if (markY .le. 0) then
              markY = 1;
           end if
        end if
     end if

     !We start with 8 points
     !%%To start we have 4 discrete point (4 corners of a square)
     xpts2(1) = T%ATM%xcoordT(markX)
     xpts2(2) = T%ATM%xcoordT(markX+stepX);
     ypts2(1) = T%ATM%ycoordT(markY)
     ypts2(2) = T%ATM%ycoordT(markY+stepY);
     x1 = markX;x2 = markX+stepX;
     y1 = markY;y2 = (markY+stepY);
     !%%Use U0,V0,W0
     u4(1) = T%ATM%UTURB(y1,x1)
     u4(2) = T%ATM%UTURB(y1,x2)
     u4(3) = T%ATM%UTURB(y2,x2)
     u4(4) = T%ATM%UTURB(y2,x1)
     v4(1) = T%ATM%VTURB(y1,x1)
     v4(2) = T%ATM%VTURB(y1,x2)
     v4(3) = T%ATM%VTURB(y2,x2)
     v4(4) = T%ATM%VTURB(y2,x1)
     w4(1) = T%ATM%WTURB(y1,x1)
     w4(2) = T%ATM%WTURB(y1,x2)
     w4(3) = T%ATM%WTURB(y2,x2)
     w4(4) = T%ATM%WTURB(y2,x1)

     !%%%%%interpY%%%%%%%%%%%
     if (extrapY .eq. 1) then
        !%%You don't need to interpolate on y
        ypts1 = ypts2(1);
        u2(1) = u4(1);
        u2(2) = u4(2);
        v2(1) = v4(1);
        v2(2) = v4(2);
        w2(1) = w4(1);
        w2(2) = w4(2);
        T%ATM%boundsT = 1;
     else
        !%%Interpolate between Y points(interpolate pts 1-2 and 3-4)
        !%%Pts 1,4 : 2,3
        cord1 = (/1,2/);
        cord2 = (/4,3/);
        do ii = 1,2
           uslope = (u4(cord2(ii))-u4(cord1(ii)))/(ypts2(2)-ypts2(1));
           vslope = (v4(cord2(ii))-v4(cord1(ii)))/(ypts2(2)-ypts2(1));
           wslope = (w4(cord2(ii))-w4(cord1(ii)))/(ypts2(2)-ypts2(1));
           u2(ii) = uslope*(ystar-ypts2(1))+u4(cord1(ii));
           v2(ii) = vslope*(ystar-ypts2(1))+v4(cord1(ii));
           w2(ii) = wslope*(ystar-ypts2(1))+w4(cord1(ii));
        end do
        ypts1 = ystar;
     end if

     !%%%%interpX%%%%%%%%%%%%
     if (extrapX .eq. 1) then
        !%%You don't need to interpolate on x
        xpts1 = xpts2(1);
        u = u2(1);
        v = v2(1);
        w = w2(1);
        T%ATM%boundsT = 1;
     else
        !%%Interpolate between X points
        uslope = (u2(2)-u2(1))/(xpts2(2)-xpts2(1));
        vslope = (v2(2)-v2(1))/(xpts2(2)-xpts2(1));
        wslope = (w2(2)-w2(1))/(xpts2(2)-xpts2(1));
        u = uslope*(xstar-xpts2(1))+u2(1);
        v = vslope*(xstar-xpts2(1))+v2(1);
        w = wslope*(xstar-xpts2(1))+w2(1);
        xpts1 = xstar;
     end if

     !%%%%Save wind values%%%%%

     vatm(1) = u
     vatm(2) = v
     vatm(3) = w

     !Rotate by PSIOFFSET
     vtemp = vatm(1)
     vatm(1) = vtemp*cos(T%ATM%PSIOFFSET) + vatm(2)*sin(T%ATM%PSIOFFSET);
     vatm(2) = -vtemp*sin(T%ATM%PSIOFFSET) + vatm(2)*cos(T%ATM%PSIOFFSET);

     !//Multiply by scale
     vatm(1) = T%ATM%TURBLEVEL*vatm(1)
     vatm(2) = T%ATM%TURBLEVEL*vatm(2)
     vatm(3) = T%ATM%TURBLEVEL*vatm(3)

     if ((T%ATM%boundsT .eq. 1) .and. (T%ATM%boundflagT .eq. 1)) then
        !write(*,*) 'You went out of turbulence bounds at T = ',tstar,' XYZ = ',xstar,ystar,zstar
        T%ATM%boundflagT = 0;
     end if
  else
     vatm(1) = 0
     vatm(2) = 0
     vatm(3) = 0
  endif
  
  T%ATM%WINDGUST(1) = vatm(1)
  T%ATM%WINDGUST(2) = vatm(2)
  T%ATM%WINDGUST(3) = vatm(3)

  T%ATM%markXT = markX
  T%ATM%markYT = markY
  T%ATM%markZT = markZ

END SUBROUTINE TURBULENCE

!!Import routine to load data files placed in text files
SUBROUTINE IMPORTWIND(mat,filename)
  implicit none
  integer ii,jj,kk,nii,njj,ierr;
  real*8 mat(40,40,40),tempmat(40)
  character*256 filename

  write(*,*) 'Importing: ',filename
  
  open(unit=78,file=filename,status='old',iostat=ierr)
  if (ierr .ne. 0) then 
     write(*,*) 'Wind File defined incorrectly'
     write(*,*) filename
     STOP;
  endif
  do jj=1,40
     do ii=1,40
        read(78,*) tempmat
        do kk=1,40
           mat(ii,kk,jj) = tempmat(kk)
        enddo
     enddo
  enddo
  close(78)

END SUBROUTINE IMPORTWIND

SUBROUTINE IMPORTTURB(outmat,filename) 
  integer ii, jj, kk, nii, njj;
  character*256 filename
  real*8 outmat(500,500),temp(500)

  open(unit=78,file=filename,status='old',iostat=ierr)
  if (ierr .ne. 0) then
     write(*,*) 'Turbulence Data File defined incorrectly'
     write(*,*) filename
     STOP;
  end if
  do ii = 1,500
     read(78,*) temp
     do jj = 1,500
        outmat(ii,jj) = temp(jj)
     end do
  end do
  close(78)

END SUBROUTINE IMPORTTURB

!!!FIND FUNCTIONS FOR ATMOSPHERE MODEL

SUBROUTINE FINDGE(invec,row,val,counter)
  implicit none
  integer idx,counter,row;
  real*8 invec(400),val
  idx = 1;
  counter = row;
  do while (idx .lt. row)
     if ((invec(idx) .ge. val)) then
        counter = idx-1;
        idx = row + 1;
     endif
     idx = idx + 1
  enddo
END SUBROUTINE FINDGE

SUBROUTINE FIND(invec,row,val,counter)
  implicit none
  integer idx,counter,row;
  real*8 invec(400),val
  idx = 1;
  counter = row;
  do while (idx .lt. row)
     if ((invec(idx) .gt. val)) then
        counter = idx-1;
        idx = row + 1;
     endif
     idx = idx + 1
  enddo
END SUBROUTINE FIND

SUBROUTINE FIND2(invec,row,val,counter)
  implicit none
  integer idx,counter,row,invec(601)
  real*8 val
  idx = 1;
  counter = row;
  do while (idx .lt. row)
     if ((invec(idx) .gt. val)) then
        counter = idx-1;
        idx = row + 1;
     endif
     idx = idx + 1
  enddo
END SUBROUTINE FIND2
