# OVERALL 

* [X] ~~*Move all of these action items to issues on Github so they are easier to track and you can add comments to all of them. That way you don't have Rev_Notes and Work_Tasks.md separated in two different files.*~~ [2025-11-07]
  
# ======================================COMPLETED==================================================:

# MIGRATING FASTSENSORS:

   * [x] Move all the Arduino codes to the Microcontroller repos @done (25-08-04 09:50)

   * [x] Migrate FASTLogger into FASTCASST Archive (This was not done on the date to the right but I just forgot to check this box.) @done (25-08-04 09:50)

   * [x] Most of the sensor codes could be put into Microcontrollers honestly since it's plotting data. Some of it went to the python repo as well @done (25-08-04 09:50)

# RENDERING:

  * [x] Create CAD for all vehicles you are using @done (21-11-06 11:30)
    * [x] Car (Tank) #@done (21-10-29 23:37)
    * [x] Plane #@done (21-10-31 22:13)
    * [x] Quad #@done (21-09-30 20:40)
    * [x] X8 @done (21-11-06 11:30)

# EXTRA STUFF:
  
  * [x] Change the name to FASTCASST - Configurable Autopilot and Simulation Software Tool @done (23-01-30 16:13)

  * [x] Fix the double to integer numerical precision issue by adding a quantization flag to the Simulation.txt file. I added a terrible hack where the control_matrix and the pwm_array is sent to every routine and a #ifdef cubesat is there to switch between the two. That's terrible. What we need to do is send the double array to all routines and then somewhere in the model we quantize the array if the flag is set @done (23-02-15 10:32)

  * [x] Hey the FASTDuino code is horribly hacked together. Would be nice to actually streamline the code. That is use the sensors and hardware class and rcio class so that the ino is a bit more high level. @done (23-12-05 10:39)  

# FASTPILOT:

 * [x] Fork code to www.gitlab.com:Montalvo/FASTPilot.git - To do this just put the MODELS in FASTPilot. So all you need to do is edit the MakeFile to reference that FASTPilot folder but main.cpp will still be in github @done (22-12-02 09:50)

 * [x] Only fork code to FASTPilot in the event you are developing code that does not already exist in print @done (22-12-02 09:50)

# HIL:

  * [x] For HIL - use the HIL code from MultiSAT++ for FASTsim @done (22-05-24 18:50)

  * [x] Once HIL mode works for SIM you can deprecate the HIL simulation in MultiSAT++ (See note below. Definitely not going to deprecate. I definitely want a copy. I think the point more is to try and get FASTKit to work like MultiSAT++ does for embedded Sims.) @done (22-05-24 18:54)
  
  * [x] MultiSAT++ simulation will still need copies of the C++ repo but as you make the Model=SAT mode more like MultiSAT++ you can deprecate this simulation. Still I would like to make sure I still have a running version of MultiSAT++ to work with in the event I do E-Sail simulations. @done (22-05-24 18:52)
  
  * [x] Add hooks to FASTSim that compile DESKTOP + HIL and RPI/Arduino HIL that sends sensor data and control commands back and forth. The RPI/HIL version will accept a polluted state vector from the computer and run that through the control algorithm and then pass back the control commands to the integrator. So serialListens and SerialResponds etc will need to be written to send data back and forth @done (22-05-24 18:50)

  * [x] Testing sending sense data 1 way from desktop to Rpi - just have RPI print the data to the screen to be sure data is transmitting 1 way @done (22-10-25 12:25)

  * [x] Have the Rpi print the control matrix every time it receives new data that way you can see the update in the control signals based on the inputs and do this with the receiver taking data so you can fly the plan manually @done (22-10-25 12:53)

  * [x] Verify that HIL and Serial tests are still functioning at Work @done (22-10-26 12:49)

  * [x] Can I send data wirelessly back and forth with the Rpi at work @done (22-10-26 12:49)

  * [x] If I bring the hardwire connection home can I get the home Rpi to send data back and forth @done (22-10-26 12:49)

  * [x] Then finally test wireless at home @done (22-10-26 13:05)

  * [x] Fix the problem where RPI reads the sensors from the Navio rather than the Serial port @done (22-10-27 10:17)

  * [x] Go back and make sure that when you send data to the RPi the values update @done (22-11-08 07:48)

  * [x] Have the Rpi send data 1 way to the desktop just like you did with the desktop but do this with the receiver taking data so you can fly the airplane manually @done (22-11-08 11:56)

  * [x] Create a loop where Desktop sends data to RPi and then waits for a valid update from Rpi @done (22-11-09 10:50)

  * [x] Create a loop on desktop where it waits for signal from desktop and then pushes data back to Desktop @done (22-11-09 10:50)

  * [x] Decrease HILRATE until the code can't run any faster @done (22-11-10 20:56)

  * [x] Run Tank in SIMONLY @done (22-11-11 10:09)
  * [x] Run Tank in SIL @done (22-11-11 10:09)
  * [x] Run Tank in HIL w/o telemetry but return rates back to normal @done (22-11-11 10:09)
  * [x] Run Tank in HIL w/ telemetry @done (22-11-11 10:09)
  * [x] Merge HIL branch @done (22-11-11 10:42)

  * [x] Increase baudrate for HIL to 115200 @done (22-12-03 07:55)

# GROUND STATION:

  * [x] Create a ground station in Python for AUTO Mode (right now this is in Processing) (This will work for HIL mode as well since the Pi will be sending telemetry data) @done (22-06-13 15:38)

  * [x] Create a ground station in Python that works in SIL mode (no need for SIMONLY mode) @done (22-06-04 17:59)

  * [x] For telemetry think about writing a groundstation in python using a GUI. That might a lot easier to plot given the fact that you can use matplotlib (take a look at LISA simulation for help on GUIs and plots) Would be cool to have GUI run in SIL mode. According to the docs online you can have C++ create a text file and then Python read the file. How do we prevent both programs reading the same file? Have C++ increment [NUM].txt and then have Python read the latest. That should work @done (22-06-04 17:59)

# AVIATION:

 * [x] Update Quadcopter and Airplane dynamics @done (22-04-12 20:38)
 * [x] Run SIMONLY simulations with quadcopter and airplane models @done (22-04-22 16:21)
 * [x] Run SIL simulations with quadcopter @done (22-04-24 20:43)
 * [x] Run SIL airplane models @done (22-04-24 20:43)
 * [x] Plot all data and email to team @done (22-04-24 20:43)

# ANTX(DID NOT COMPLETE THESE TASKS):

 * [x] Make sure that the Batwing is working properly in Demo mode using SIMPILOT @done (22-04-24 20:43)
 
 * [x] Make sure telemetry is working properly using Processing @done (22-04-24 20:43)
 
 * [x] Upgrade Processing Software with whatever signals are being sent from Drone - Current iteration of software is in Processing.git/Serial/ReceiveHexDump.pde Original software used during NOAA campaign is in GROUND_STATION also in Serial folder. Be sure to check Archive folder. I'd like the ground station software to be versatile enough that it can handle any number of sensors from telemetry you just simply need to specify what sensors are being read in and in what order @done (22-04-24 20:43)
 
 * [x] Market our entire setup as an independent contractor if needed @done (22-04-24 20:44)

 * [x] Publish another paper? @done (22-04-24 20:44)

# FASTSim SysML:

   * [x] I think to make sure you have a simple transition you need to move all the C++ and HIL repos into MultiSAT as well. The reason being because HIL the simulation does not work right now in FASTSim. I think eventually you can deprecate the HIL simulation but just to make sure things still work properly you need to put a running copy into MultiSAT so it still works during the transition @done (21-12-28 22:19)
   * [x] Move HIL into C++ directory and delete HIL (call folder Hardware or something) @done (21-12-28 22:33)
   * [x] Rename all the aerodynamics.cpp routines to forces.cpp @done (21-12-30 08:01)
   * [x] Move all vehicles to a Model Folder @done (21-12-28 22:34)
   * [x] Re-org all folders in C++.git (Make C++ routines called Basic or something like that maybe Util? or COmmon? yea Common) @done (21-12-28 22:34)
   * [x] Code Orbit++ and add a sphere for Earth scaled by REARTH. Try to reuse as many routines as possible from MultiSat++. @done (21-12-30 16:47)
   * [x] deprecate Orbit++ from Aerospace folder @done (21-12-30 17:46)
   * [x] SysML diagrams for everything below to make sure everything works as intended @done (21-12-31 20:37)
   * [x] I think the HIL send should be in Dynamics (that’s the only place you need to send) @done (21-12-31 20:37)
   * [x] The HIL receive should be in Sensors (only place you need to receive) @done (21-12-31 20:37)
    * [x] Create a logical architecture that works for all vehicles in the structure that looks like the form: @done (21-12-31 20:37)
    * [x] Inner Loop @done (21-12-31 20:37)
      * [x] Inputs: Roll, pitch, yaw commands @done (21-12-31 20:37)
      * [x] Outputs: ctlcomms - us signals to RCIO class @done (21-12-31 20:37)
    * [x] Outerloop @done (21-12-31 20:37)
      * [x] Inputs: waypoint commands (x,y,altitude) @done (21-12-31 20:37)
      * [x] Outputs: roll pitch and yaw commands @done (21-12-31 20:37)
    * [x] GraveDigger - The Remote control car in the lab @done (21-12-31 20:37)
    * [x] BattleBot - the tank @done (21-12-31 20:37)
    * [x] FASTIris @done (21-12-31 20:37)
    * [x] Bumblebee @done (21-12-31 20:37)
    * [x] Meta 1 and 2 @done (21-12-31 20:38)
    * [x] Batwing @done (21-12-31 20:38)
   * [x] First Video is your struggle with something new @done (22-01-02 09:39)
   * [x] Second Video is on SysML diagrams @done (22-01-02 09:39)
   * [x] Rename C++.git to FAST.git @done (22-01-20 23:30)
   * [x] Make the software look like the SysML diagram. This will probably break everything @done (22-03-30 20:29)
   * [x] Put the Makefile in FAST that way you can do - make Demo or make Logger find a way to do this @done (21-12-29 17:27)
   * [x] I want the main.cpp for all the below routines to never have to be edited. @done (21-12-29 17:27)
      * [x] LOGGER - move from FASTSensors to here @done (21-12-29 17:27)
      * [x] DEMO - this is SimPilot @done (21-12-29 17:27)
      * [x] SIM  - This is FASTsim @done (21-12-29 17:27)
   * [x] Save FASTPilot.git on Gitlab for proprietary software @done (22-01-20 23:35)
   * [x] Merge all changes above @done (22-01-20 23:31)
   * [x] Delete HIL on Github. @done (22-01-20 23:35) I archived this instead

# FASTSim:

  * [x] Have someone else walk through README and ensure they can compile code in SIMONLY mode and run process logs to plot done (21-08-24 14:46) #Running on Laptop but not Desktop - MJC

  * [x] The sensor errors are no longer working. why? that needs to be fixed done (21-09-03 20:58)

  * [x] SIL model does not work on a clean install of ubuntu 20.04. Need to figure out why? done (21-08-20 11:36) 

 * [x] Finish the actuator dynamics. There are alot of subtasks for this one #@done (21-09-12 22:24)
   * [x] Integrate the state vector to produce State. done (21-09-06 08:31)
   * [x] Send STate to the sensor routine and receive the polluted state done (21-09-06 08:33)
   * [x] Send pollutted state to controller built by the user to produce ctlcomms #@done (21-09-06 08:34)
   * [x] Send ctlcomms to actuator dynamics to produce actuators values #@done (21-09-07 21:42)
   * [x] Pollute actuator values with ACTUATOR ERRORS by some percentage #@done (21-09-07 21:42)
   * [x] Send actuator values to aerodynamics routine written by user to produce Forces and moments #@done (21-09-07 21:43)

 * [x] Complete the controller.cpp/.h file for the quadcopter folder. This can be open source since these equations are publicly available #@done (21-09-15 22:17)

 * [x] Complete the aerodynamics.cpp/.h file for the Quadcopter folder. This can be open source since these equations are publicly available #@done (21-09-21 11:00)

 * [x] Create the minimum viable rcout class for SIMONLY and SIL #@done (21-09-25 21:54) This was kind of easy because in SIMONLY and SIL mode the RCOutput does not do anything

 * [x] Run SIL on RPI - This mode should run in realtime but not with openGL #@done (21-10-15 11:25)

 * [x] Run SIMONLY only on RPI #@done (21-10-15 11:25)

 * [x] Finish writing sensor class for FASTsim #@done (21-10-15 11:26)

  * [x] The filter for the IMU needs to run all the time and needs to be an input to the simulation so you can change that on the fly. Ok actually I have a better idea. The filter doesn't need to be outside the filter parameter needs to change from s to....wait. The whole point of the filter is to use it when in SIL/SIMONLY mode so it's not that it needs to be outside the IMU routine it just needs to be it's own function set by a parameter that is not 's' but 'ComplimentaryFilterConstant' or something like that #@done (21-10-26 08:26)

  * [x] In that regard the GPS needs a heading angle estimator using the computeSpeed() routine but the function name needs to change and be overloaded so that old functions still work properly. I'm thinking GPSGroundTrack() would be appropriate and absorb all new functionality. #@done (21-11-06 12:42)

  * [x] I'd like a function in the Sensor routine to take the IMU heading and the GPS heading and filter them together. I did something like this on the Arduino I just don't quite remember where it is but I'm sure I'll find it. #@done (21-11-07 03:23)

 * [x] Run AUTO mode on RPI on Bumblebee @done (21-11-17 13:14)

 * [x] Add telemetry to send data to ground station @done (21-11-22 11:19)

 * [x] Complete aerodynamics for X8 @done (21-11-02 16:04)

 * [x] Complete the PID controller for the X8 @done (21-11-02 16:04)

 * [x] Complete aerodynamics for Aircraft #@done (21-10-31 22:14)

 * [x] Complete the controller (PID) for the A/C #@done (21-10-31 22:14)

 * [x] Edit the Makefile so you don't have to edit it everytime you compile differently. Would be cool to do make rebuild DESKTOP SIL RCTECH or make rebuild PI AUTO FLYSKY #@done (21-11-07 03:58)

 * [x] Add Keyboard inputs to RCIO so that you can control things with a keyboard #@done (21-11-07 04:56)

# BattleBot:

  * [x] Create a CAD for the vehicle #@done (21-10-15 11:27)

  * [x] Ground Contact Model #@done (21-09-13 16:48)

  * [x] Create aerodynamics - I think the vehicle is so slow such that we don't need to worry about the forwards force but I do think we need a -k*v force in the Y direction also feed control signals to forces and moments. Also need -k*r in the yaw moment #@done (21-10-01 12:48)

  * [x] Create a controller to feed transmitter commands to 2 wheels with differential #@done (21-10-01 12:48)

  * [x] Run SIMONLY #@done (21-10-01 12:52)

  * [x] Run SIL #@done (21-10-01 13:10)

  * [x] Run AUTO mode on RPI for battle bot #@done (21-10-15 11:28)

# SIMPILOT:

 * [x] Make sure the RCInput main.cpp test works in the Test_Scripts folder in HIL.git #@done (21-09-23 12:15)

 * [x] Add navio receiver rcin protocols to current RCInput.h/.cpp and test with receiver in the Lab What exactly is your plan for RCIO? Do you want RCInput and RCOutput to be in this folder like this? RCInput.h/RCInput.cpp and RCOutput.h/RCOutput.cpp That seems the most reasonable in my opinion. #@done (21-09-23 12:19)

 * [x] (Do this in FASTSim first before you do this) Add ESC outputs to existing RCOoutput but combine everything to the minimum viable set of routines. You'll probably have to create some test scripts. What exactly is your plan for RCIO? Do you want RCInput and RCOutput to be in this folder like this? RCInput.h/RCInput.cpp and RCOutput.h/RCOutput.cpp That seems the most reasonable in my opinion. #@done (21-09-27 12:29)

 * [x] Since you just edited the RCInput and RCOutput routine go back and make sure that SIMONLY, SIL and Rpi hooks are working in FASTSim. In order to get the RPi hooks working you're going to have to compile this routine in AUTO mode on the RPI. You'll also need to hook up the PWM outputs and some servos to make sure you're getting the output you think you're supposed to get. You might need to use an o-scope #@done (21-09-27 12:47)

 * [x] Quadcopter autopilot - write a simple PID loop. This can be open source since it won't work in flight. This will require creating a folder like Portalcube. For now leave the aerodynamics blank and just sent zeros since we're just interested in the controller. However make sure that the aerodynamics.cpp/.h are at least there. #@done (21-09-15 22:15)

 * [x] After you write the loop above pull out the controller and paste it into the simpilot.exe so its easier to see #@done (21-09-15 22:15)

 * [x] Move saturation block to RCIO to saturate pwmcomms during the write loop #@done (21-09-30 20:24)

 * [x] Add pwmcomms to logfile and update plotting routine to plot pwmcomms instead of ctlcomms #@done (21-09-30 20:34)

 * [x] Change FASTsim to use pwmcomms to run actuator dynamics #@done (21-09-30 20:34)

 * [x] ACRO and STAB mode on Hardware do not seem to function properly #@done (21-10-01 11:50)

 * [x] As a final check make sure both SimPilot SIMONLY,SIL and AUTO work #@done (21-10-01 11:54)

 * [x] Do the same for FASTsim SIMONLY, SIL #@done (21-10-01 12:39)

# FASTSim Extra Sensors:

 * [x] Add barometer @done (21-12-17 16:21)
 * [x] Add ADC @done (21-12-17 16:21)
 * [x] Fix Datalogger to output a headers when you start (like one row @done (21-12-17 19:00))