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

# Python Installation and Run

The Python version of FASTCASST called FASTPy is in the src folder called fast.py. You can run that simply by running.

$ python3 src/fast.py

Note that it is assumed that you already have Python3 installed on your computer but the first time invoking that command will undoubtedly result in numerous module errors. In order to install modules on Ubuntu you must have pip3. Install that first with the following command:

$ sudo apt install python3-pip

Then you can install the following modules

$ pip3 install fdd

fdfdfd

# C++ Installation and Run 

C++ is the original version of the software but a Python version was created to ease the learning curve of FASTCASST. However, Python does not have all the same functionality that C++ does so if you want to run the entire simulation suite you need to run the C++ version. I recommend utilizing a resource to learn C++ like this one: "Learn C++ in 21 days" - http://101.lv/learn/C++/. 

This readme is still a WIP but for now I recommend opening the Makefile and trying to compile and run the software in SIMONLY mode for an airplane

you will need a few packages before you can compile

$ sudo apt install make

$ sudo apt install g++

$ sudo apt install libboost-all-dev

if you get an error try

$ sudo apt-get update 

and then try 

$ sudo apt install [package_name] --fix-missing

then you can compile the code

$ make simonly MODEL="airplane"

To run the code you type

./simonly.exe airplane/

and then run the associated python script to plot data

$ python3 plotdata.py 4

Alternatively you can run

$ ./plotdata.py 1

which will invoke the python3 interpreter and clean, compile, run and plot the results

# Plotting Data from SIMONLY

# How to Add a New Controller for SIL

# How to Add a New Vehicle in the libraries/ folder

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
