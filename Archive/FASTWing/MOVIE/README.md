# MOVIE

Multi-Object Visualizer with Interactive Environment

This code was created to simulate multiple object flying in an interactive environment.

The GUI portion is written using the QT libraries and the visualizer portion is written using OpenGL. 

A youtube video showing you the code can be seen down here

https://www.youtube.com/watch?v=Ur6ahesXS3w

The visualizer (all in Viewer/ Folder) can be compiled separately as can the GUI portion (in GUI/ folder)

Make sure you have all the GLUT and QT libraries. The input files use wavefront obj files for the objects and a simulation output file that contains the 6DOF state information of all objects in the space.

6DOF information is (x,y,z,phi,theta,psi,u,v,w,p,q,r)

where x,y,z is the position

phi,theta,psi is the attitude

u,v,w are the translational velocities

and p,q,r are the angular velocities.

OPENGL AND GLUT

sudo apt-get install

freeglut3-dev
mesa - this doesn't work in 14.04 but luckily for you it is already installed in 14.04.

make sure to add -lGLUT and -lGL to the command line compile command.

For windows you need to download glut from the internet.



