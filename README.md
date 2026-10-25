# FASTCASST (Facility for Aerospace Systems and Technology Configurable Autopilot and Simulation Software Tool)
The Facility for Aerospace Systems and Technology (FAST) has put together a Configurable Autopilot and Simulation Software Tool (CASST) to simulate and test autopilots for multiple vehicles.

This FASTCASST software is an open-source configurable simulation environment developed in C++ (FAST++) and Python (FASTPy) programming languages. This simulation environment simulates a real-world environment and is designed to test actual control software of a vehicle before testing of physical hardware. The software is designed to be modular by allowing the user to build a portfolio containing the simulation input files, CAD, aerodynamics and control system of the vehicle which allows the software to work with a wide variety of vehicles including but not limited to: airplanes, multicopters, satellites, marine surface vehicles and ground vehicles. FASTCASST currently runs in four modes: simulation only (SIMONLY) mode, software in the loop mode (SIL), hardware in the loop mode (HIL) and (AUTO) mode. The SIMONLY mode allows the vehicle to be mathematically simulated for a specified time with the results being displayed as plots at the end of the simulation, whereas the SIL mode simulates the vehicle in a virtual environment in realtime with control software running in both modes on your desktop computer. OpenGL is used to render the vehicle in a 3D environment creating a Virtual Twin. The AUTO mode is intended to run on the Raspberry Pi with a Navio2 Emlid HAT with a plan to run this on the Arduino and Blue Robotics Navigator platform. 

The HIL mode requires the software to be compiled on a desktop and a microcontroller with data being transmitted via a USB serial connection. In this mode the dynamics are simulated on the desktop machine and data is sent to the microcontroller via serial to run the control dynamics. Communication runs back and forth from both machines. The SIMONLY and SIL modes have been tested on a laptop running Ubuntu using a quadcopter, airplane, satellite, tank and x8 drone. AUTO mode has been tested on a tank, car, boat and an airplane. HIL is still a work in progress (WIP). The below sections document the current status of version 10.0 and how to run and install the software

The software has a few SysML Diagrams located in the SysML folder. The Block Definition Diagrams (BDD) and Internal Block Diagrams (IBDs) will help understand the flow between modules where every block is a class defined in libraries/ folder complete with Python and C++ versions. Information transfer between blocks happens in the associated functions. 

Note that the SysML_Diagrams/hardware folder contains SysML diagrams for a autonomous airplane codenamed "Meta". These diagrams show a specific use case for an aircraft and might help with hardware integration. There is also a list of references to peer-reviewed journals and conference papers at the end of this README that includes a number of photos to real hardware.

Finally, the equations of motion in the libraries/modeling block as well as a lot of design elements and control theory are located in a textbook currently located in a separate repo: https://cmontalvo251.github.io/aerospace-mechanics-controls/frontmatter.html

# Contributing

We're always looking for real people to contribute. If you're solely going to use AI though we don't want you. We've written this software almost entirely from scratch (~80%) and we like it that way. Yes we have VScode and Gemini and Antigravity but we also do a lot of things by hand to make sure the code runs the way we want. If you'd like to contribute create a Github account and create a pull request. This blog will help you on your journey to Github and pull requests. http://fastlabtutorials.blogspot.com/2015/07/github-basics.html

# Hardware Setup

The sections that follow detail installation of the software on a Debian based Linux machine: either a Raspberry Pi or Laptop running Ubuntu. The AUTO mode currently is only supported on a Raspberry Pi with a Navio2 Emlid HAT or an Arduino MEGA. The Navio2 is complete with a GNSS (GPS), UART ports, MPU9250 (IMU1), LSM9DS1 (IMU2) and MS5611 (Barometer). There are plans to add the Blue Robotics Navigator to the list of compatible hardware but at the moment this is untested and unsupported. https://bluerobotics.com/store/comm-control-power/control/navigator/. When using the Arduino MEGA you must add a GPS, IMU, UART transmitter and Barometer to the Arduino or the software won't work. 

The UART port is used for Serial communication to a ground station computer. The ground station receives data from your drone running FASTCASST and plots everything onto a nice GUI. Directions on the ground station are included below. 

Version 10.0 does not currently have support for Windows. MinGW and Cygwin have been tested without success. 

Running Powershell as Administrator and running 'wsl --install' will install a Ubuntu Virtual machine which may work but is not tested. If you get error code 0x80370114 make sure to enable WSL in program features. This blog may help you https://randomhacksdrc.blogspot.com/2023/02/mounting-ext4-harddrive-to-windows.html 

Another option is to install VirtualBox on your windows computer and then Install Ubuntu into your virtual machine. This is also untested but more than likely will work.

Another option that would definitely work would be to simply dual boot your machine or just switch to Ubuntu Linux completely - https://randomhacksdrc.blogspot.com/2014/10/thinking-of-making-switch-to-linux.html

For the Raspberry Pi it is recommended to get the headless Emlid Raspbian version: https://drive.google.com/file/d/1k8PEGZ0j2ep0suopJIldsSffiGqJmN30/view?usp=sharing

Once your Raspberry Pi, laptop, virtual machine or Ubuntu machine is setup you may proceed with the installation below. 

Note, on the Raspberry Pi you may have ardupilot turned on by default. To turn it off type

$ sudo systemctl disable arducopter

Of course first check and see what is running by typing

$ sudo emlidtool ardupilot

# Downloading Software

The software is a simple zip download from this repo or a git clone if you have git. On Ubuntu this involves two simple commands

$ sudo apt install git

$ git clone 'https://www.github.com/cmontalvo251/FASTCASST.git'

Note on Ubuntu, it is always recommended to run an update in the event that your distribution has new software for C++ or Python

$ sudo apt update

$ sudo apt upgrade

Also, for the plotting and ground station routines, Python is required. As such, you must ensure you have a version of Python3 installed. You can check your version of Python using the command below.

$ python3 --version 

You must also install other modules in Python to get these routines to work. This requires pip3 which you can install with:

$ sudo apt install python3-pip

# Input Deck

For both the C++ and Python versions every vehicle contains there own set of input files in libraries/V_*/Input_Files/. Each vehicle has 3 files. The first is a Config.txt file used to change control variables, waypoints and different sensor rates. Simulation.txt is used to change all simulation parameters like simulation time, timestep and initial conditions. Simulation.txt is only used in SIMONLY and SIL modes while Config.txt is used for every mode. The final input file is Render.txt which is used only for SIL mode. All input files are heavily commented and should speak for themselves.

# Python Installation and Run

The Python version of FASTCASST called FASTPy is in the src folder called fast.py. You can run that simply by running.

$ python3 src/fast.py

The first time invoking that command will undoubtedly result in numerous module errors. The following modules required are the following:

$ pip3 install numpy

$ pip3 install smbus

$ pip3 install spidev

$ pip3 install pyserial

After installing those modules the simulation will run with some standard out prints and 2 files will be created. Skip below for details on the data files and how to plot them. In the preamble of src/fast.py there are a few variables that you can change that's different from FAST++ and the input deck as explained above. The variables you can change are VEHICLE, JSON_FILE_NAME, MODE, LATITUDE_ORIGIN and LONGITUDE_ORIGIN.

The VEHICLE can be x8, car, boat or any vehicle defined in the libraries/V_* folder. Adding a new vehicle is explained below in this README. JSON_FILE_NAME is the name of the JSON file used for SIL mode to operate the vehicle with a USB controller. The JSON files are housed in libraries/RCInput/Calibration. The MODE can be either SIMONLY, SIL, HIL or AUTO. At the time of this writing only SIMONLY and SIL mode are operational in FASTPy.

LATITUDE_ORIGIN and LONGITUDE_ORIGIN are used to set the origin point of the GPS.

# C++ Installation and Run 

C++ is the original version of the software but a Python version was created to ease the learning curve of FASTCASST. However, Python does not have all the same functionality that C++ does so if you want to run the entire simulation suite you need to run the C++ version. I recommend utilizing a resource to learn C++ like this one: "Learn C++ in 21 days" - http://101.lv/learn/C++/. 

It is assumed that you have 'make' and 'g++' already installed on your computer. You can check what versions you have by running 

$ make --version

$ g++ --version 

and install them if you don't have them by running

$ sudo apt install make

$ sudo apt install g++

You also need the boost libraries to run multiple threads in C++

$ sudo apt install libboost-all-dev

In order to compile the software you must first run 

$ make clean

to clean all the old modules. Then you can compile the software by running

$ make simonly MODEL="MODEL"

where MODEL is the name of the vehicle you want to compile. Again all the vehicles are shown in libraries/V_*. Adding a new vehicle is explained below. For example, if you want to compile the x8 you can run 

$ make simonly MODEL="x8"

To run the code you type

$ ./simonly.exe x8/

Then you can proceed to plotting the data.

# Plotting Data from SIMONLY

After running either FAST++ or FASTPy the simulation will create two files. The first file is located in 'data/0.csv' which contains the output of the sensors in the virtual environment while 'logs/0.csv' contains the output of the modeling routine. Think of the logs folder as the as the truth data while the data folder is the output of the sensors which may be perfect or polluted with sensor noise. Note that the sensors still operate on the principle of the actual sensors which means speed from the GPS even in a perfect environment is still obtained through a derivative filter of position. If you open either data or the logs file you will see that the first row contains a header and says which state variable or parameter is being recorded. Changing the variables outputted requires editing the source code and is not covered in this README. 

To run the plotting script you need the following modules

$ pip3 install matplotlib

You also need a custom pdf.py module that is in my Python.git repository. In order to get that module you must first clone that directory

$ git clone 'https::/www.github.com/cmontalvo251/Python.git'

Then add that new Python/pdf folder to your PYTHONPATH environment variable. To do that open ~/.bashrc 

$ nano ~/.bashrc

and then copy the line below into your file 

export PYTHONPATH=$PYTHONPATH:$HOME/Python/pdf/

Then you need to have Ubuntu update your bashrc file

$ source ~/.bashrc

Once you have those pre-requisites satisfied you can run the plotting script. Note the plotting script has the following format

$ python3 plot_simulation_data.py # MODEL

Where the '#' is a number between 1-5 and MODEL is car, satellite, boat, or any other vehicle you have defined in the libraries/V_* folder. If you'd like to add a new vehicle you can read the section on how to add new vehicles. 

The numbers 1-3 apply only to FAST++. 1 will run 'make clean', compiled the code, run FAST++ and plot it. 2 will only compile, run and plot, 3 will only run and plot. The number 5 will run and plot FASTPy while the number 4 will just plot the data in the 'data' and 'logs' folders.

The MODEL flag in the plotting script will tell the compiler which vehicle to compile the FAST++ routine. The MODEL flag is not used in FASTPy because the vehicle is set in src/fast.py in the preamble as explained in the Python installation section. 

For example, 

$ python3 plot_simulation_data.py 2 x8

will compile, run and plot the x8 simulation and sensor data for FAST++ while

$ python3 plot_simulation_data.py 5

will run and plot the FASTPy results for the VEHICLE defined in src/fast.py

# Running in SIL Mode (FASTPy)

To run the software in SIL mode simply open src/fast.py and change the MODE to 'SIL'. Then run

$ python3 src/fast.py

You simply type the letter 'q' to quit.  Also, you can use the mouse to zoom and rotate and the letter 'c' will change the viewing angle. However, you need the following OpenGL modules

$ pip3 install PyOpenGL PyOpenGL_accelerate

as well as the freeglut packages from synaptic

$ sudo apt install freeglut3 freeglut3-dev

# Running in SIL Mode (FAST++)

To run SIL mode in C++ you must clean the entire project 

$ make clean

and recompile

$ make sil MODEL="MODEL"

where "MODEL" can again be any vehicle you have defined in the libraries folder. To compile x8 you would run

$ make sil MODEL="x8"

Once compiled you run SIL mode by running

$ ./sil.exe x8/

Again type the letter 'q' to quit. Again, you can use the mouse to zoom and rotate and the letter 'c' will change the viewing angle.

# How to Add a New Controller for SIL

# How to Add a New Vehicle in the libraries/ folder

Adding a new vehicle is fairly simple but there are some fundamental steps. 

1.) First you must create an Input_Files folder with the 3 standard files: Render.txt, Simulation.txt and Config.txt

2.) You'll notice the Render.txt file points to objects defined in the objects/ folder. These are obj, mtl and bmp files. I typically make objs in Blender and export them as an obj and then create the 24 bit 256x256 texture files in GIMP but you can use anything you like provided the format is the same as the other obj files. Simply open up the obj and mtl files to see the standard format for those files.

3.) Create the controller and force files. For Python you need to create a controller.py and a force.py file. For the force.py routine is called to get the mass and inertia as well as the forces and moments. The controller.py file is used to get the controller commands depending on the sensor readings. My recommendation is to read a different vehicle to see the standard routines that are called and the input parameters for those functions. For C++ the files are *_forces.cpp, *_forces.h, *_controller.cpp, and *_controller.h where the *_forces.cpp file is used to compute the forces and moments of the vehicle while *_controller.cpp is used to compute the control commands. The * here is used to denote whatever vehicle you are creating while the header files (.h) are simply the accompanying C++ header files for standard C++ classes. 

# Arduino Setup 

The setup for Arduino involves opening the src/fastduino/fastduino.ino in the Arduino IDE and then flashing it onto an Arduino MEGA. Note that the setup guide for this is incomplete as support for the Arduino version is currently not on the high priority list. However, hopefully the comments and structure of the code in fastduino.ino will help you get started if that's something you're interested in doing.

# References

Below are a few papers written that utilize this software in the field

Maxwell Cobar and Carlos Montalvo. “The Facility for Aerospace Systems and Technology Simulation:
FASTSim - An Open-Source Configurable Software in the Loop Simulation Environment”. In: AIAA
AVIATION Forum - Chicago, IL. Final Paper Submitted May 2nd, 2022. June 2022.
 
Maxwell Cobar and Carlos Montalvo. “Takeoff and Landing of a Wingtip Connected Meta Aircraft
with Feedback Control”. In: Journal of Aircraft 58.4 (2021), pp. 733–742. doi: 10.2514/1.C035787.
url: https://doi.org/10.2514/1.C035787.

Maxwell Cobar, Carlos Montalvo, and Collin Buckner. “Disaster Response Fixed Wing UAV for Gulf of
Mexico Field Campaign”. In: AIAA SciTech Forum - Nashville, TN. Jan. 2021. doi: 10.2514/6.2021-
1408. url: https://arc.aiaa.org/doi/abs/10.2514/6.2021-1408.

Maxwell Cobar and Carlos Montalvo. “Developing Semi-Autonomous Flight Control Software For a
Meta Aircraft of Two Fixed Wing Aircraft with a Raspberry Pi + Navio2 Flight Controller”. In:
Proceedings of the Early Career Technical Conference 2019, Birmingham, Alabama, USA. Vol. 18. 3.
Nov. 2019. url: https://www.uab.edu/engineering/me/images/Documents/about/early_career_
technical_journal/Year_2019_Vol_18-Section3.pdf.

Collin Carithers et al. “Control Optimization of a Novel eVTOL/Pusher Transport Aircraft with
Known Faults”. In: AIAA AVIATION Dallas, TX. doi:10.2514/6.2019-3269. June 2019. doi: 10.2514/
6.2019-3269. url: https://arc.aiaa.org/doi/abs/10.2514/6.2019-3269.

Collin Carithers and Carlos Montalvo. “Experimental Control of Two Connected Fixed Wing Aircraft”.
In: MDPI - Aerospace 5.4 (113 October 24th, 2018). doi:10.3390/aerospace5040113. url: https://
www.mdpi.com/2226-4310/5/4/113/pdf.

Brandon Troub et al. “Low-Cost, Multi-Purpose Autopilot for Ground and Aerial Vehicles using an Ar-
duino MEGA with Transistor Array Safety Circuit”. In: AIAA AVIATION Denver, CO. doi:10.2514/6.2017-
3140. June 2017-3140.

Brandon Troub and Carlos Montalvo. “Meta Aircraft Controllability”. In: AIAA AVIATION Wash-
ington, DC, USA. 2016.

Carlos Montalvo and Mark Costello. “Meta Aircraft Flight Dynamics”. In: Journal of Aircraft 52.1
(Jan. 2015). doi:10.2514/1.C032634, pp. 107–115. doi: 10.2514/1.C032634.

Carlos Montalvo. Meta Aircraft Flight Dynamics and Controls. PhD Thesis, Georgia Institute of Tech-
nology, May 2014.

Carlos Montalvo and Mark Costello. “Meta Aircraft Connection Dynamics”. In: AIAA GNC/AFM
Minneapolis, MN. 2012. doi: 10.2514/6.2012-4677.
