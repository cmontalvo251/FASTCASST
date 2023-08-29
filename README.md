# FASTCASST
The Facility for Aerospace Systems and Technology (FAST) has put together a Configurable Autopilot and Simulation Software Tool (CASST) to simulate and test autopilots for multiple aerospace vehicles.

This software is an open-source configurable software in the loop simulation environment developed in C++ programming language codenamed FASTCASST (Facility for
Aerospace Systems and Technology Configurable Autopilot and Simulation Software Tool). This simulation environment simulates a real-world environment and is designed to test flight control software of a vehicle before flight testing of physical hardware. The software is designed to be modular by allowing the user to build a portfolio containing the simulation input files, CAD, aerodynamics and control system of the vehicle which allows the software to work with a wide variety of vehicles including but not limited to: airplanes, multicopters, satellites and ground vehicles. FASTCASST currently runs in four modes: simulation only (SIMONLY) mode, software in the loop mode (SIL), hardware in the loop mode (HIL) and (AUTO) mode. The SIMONLY mode allows the vehicle to be mathematically simulated for a specified time with the results being displayed as plots at the end of the simulation, whereas the SIL mode simulates the vehicle in a virtual environment in realtime with flight control software running in both modes. The AUTO mode is intended to run on the Raspberry Pi with Navio Emlid HAT however future revisions will allow the user to run this on the Arduino platform. The HIL requires the software to be compiled on a desktop and hardwired to a microcontroller also running FASTCASST in HIL mode. In this mode the dynamics are simulated on the desktop machine and data is sent to the microcontroller via serial to run the control dynamics. Communication runs back and forth from both machines. The SIMONLY and SIL modes have been tested on a laptop running Ubuntu 20.04 using a quadcopter, airplane, portalcube, satellite, tank and x8 drone. AUTO mode has been tested on a tank and an airplane. HIL is still a work in progress (WIP).

Recommended steps to compile and contribute to this project

1.) Create a Github account and download this repo. If you're reading this I'm assuming you've already done this. Just in case you may want to check out my tutorial on Github. http://fastlabtutorials.blogspot.com/2015/07/github-basics.html

2.) Familiarize yourself with C++. I learned C++ using "Learn C++ in 21 days" - http://101.lv/learn/C++/

3.) Familiarize yourself with SysML_Diagrams. Start with the Software Block Diagram Definitions (BDD) and then move to the Internal Block Diagrams (IBD). Understand that every block has an associated .h and .cpp file that you can look for in the libraries/ folder. Information transfer between blocks happens in the associated functions. 

4.) Note that the SysML_Diagrams/hardware folder contains SysML diagrams for a autonomous airplane codenamed "Meta". There is no need to look at those unless you intend to build an aircraft using the FASTCASST software. If you only wish to do simulation there is no need. Note that FASTCASST is intended to run on hardware, specifically the Raspberry Pi with Navio Emlid HAT, therefore you may want to at least glance at the hardware BDD's and IBD's

5.) The equations of motion in the modeling block as well as a lot of design elements and control theory are located in a "textbook" that I wrote on Aerospace Mechanics. That book is open source and can be found here. https://github.com/cmontalvo251/LaTeX/blob/master/Aerospace_Mechanics/aerospace_mechanics.pdf

6.) This readme is still a WIP but for now I recommend opening the Makefile and trying to compile and run the software in SIMONLY mode for an airplane

$ make simonly MODEL="airplane"

and then running the associated python script to plot data

$ python3 plotdata.py

N.) For further reading on successes of this software please see the references below.

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
