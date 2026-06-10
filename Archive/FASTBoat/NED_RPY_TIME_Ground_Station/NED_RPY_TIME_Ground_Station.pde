////////////GLOBAL VARIABLES/////////////
float[] ptp = new float[3];
float[] pqr = new float[3];
float[] ned = new float[3];
float[] rx = new float[4];
float lat = 0; // initialize Latitude
float lon = 0; // initialize Longitude
float Da = 0; // Difference in altitudes
float hardware_time = 0;
float baro_altitude = 0;
float zero_altitude = 0; // Initializing zero altitude shouldn't have to change
float GPS_altitude = 0; // Initializing GPS altitude will change based on signal but should be linear 
float airspeed = 0;
float time = 0;
int numVars = 7;
int cycle = 0;
float[] received_data = new float[numVars];
float time_now = 0;
int wait = 0; //this let's us click the mouse to start the routine.
float pi = 3.141592654;

////////VIEWPORT VARIABLES//////////////////
PFont font;

///////////SERIAL VARIABLS
//In order to talk to the Arduino
int oktosend = 1;
import processing.serial.*;
Serial myport;
char[]inLine = new char[60];
float last_send = 0; //When did we last request a response
float time_wait = 5; //time to wait before another messenger is sent assuming we haven't heard back.
float time_poll = 1; //How often we poll for new data in seconds
float last_response = 0; //When did we last hear a response
int w = 0;
int slashr = 0;
int slashn = 0;

///////////////DATALOGGING VARIABLES//////////
PrintWriter file;

////FLAGS
void setup() 
{  
  //SETUP VIEW WINDOM
  size(400,400, P3D); //set view size with 3D processing
  surface.setResizable(true);

  //BACKGROUND
  // I put the background in setup not draw this causes corners to be cut off on the cube
  background(#000000);
 
  ////CREATE A FONT
  font = createFont("Courier", 18); //built in processing command
 
  myport = new Serial(this,"/dev/cu.usbserial-DN00BR0D",57600);

  ///SETUP DATALOGGING FILE
  int d = day();    // Values from 1 - 31
  int m = month();  // Values from 1 - 12
  int y = year();   // 2003, 2004, 2005, etc.
  int s = second();  // Values from 0 - 59
  int min = minute();  // Values from 0 - 59
  int h = hour();    // Values from 0 - 23
  file = createWriter("jacquelineday/Desktop/Logging_"+str(y)+"_"+str(m)+"_"+str(d)+"_"+str(h)+"_"+str(min)+"_"+str(s)+".txt");
 
  //Initialize all receive vars to zero
  for (int i = 0;i<numVars;i++){
    received_data[i] = 0;
  }
  
  println("Ready");
}

void mouseClicked() {
  wait = 1;
}

//Alright here is our draw loop
void draw() {
  
  //Put an infinite while loop so we can click to kick off this routine
  if (wait == 1) {
    //Get timer of this laptop
    time_now = millis()/1000.0;
    
    ///Have we requested data?
    if ((oktosend == 1) && ((time_now - last_response) > time_poll)) {
      //If not go ahead and request data
      SerialPutHello();
      //But don't request data again until we've received a response 
      oktosend = 0;
      //Grab the time of sending
      last_send = time_now;
    }
    
    ////Check for Response EveryLoop
    CheckForResponse();
  }
  
  //Extract Data
  ptp[0] = (pi/180.0)*received_data[0];
  ptp[1] = (pi/180.0)*received_data[1];
  ptp[2] = (pi/180.0)*received_data[2];
  lon = received_data[3];
  lat = received_data[4];
  GPS_altitude = received_data[5];
  time = received_data[6];
  //rx[0] = received_data[3];
  //rx[1] = received_data[4];
  //rx[2] = received_data[5];
  //rx[3] = received_data[6];
  //baro_altitude = received_data[7];
  //GPS_altitude = received_data[8];
  //lat = received_data[9];
  //lon = received_data[10];
  
  //Set the view port color
  background(#000000); // I kept this here if we want to revert the background to original 
  fill(#FFFFFF);
  
  //GRID 2,1 - PTP
  stroke(255);
  textFont(font, 8*((width+height)/2.0)/200.0);  //set the textfont to Courier and size 20
  textAlign(LEFT, TOP); //set the test to left and top
  text("Euler Angles (deg)\nRoll: " + degrees(ptp[0]) + "\nPitch: " + degrees(ptp[1]) + "\nYaw: " + degrees(ptp[2]), width/3.0,height/10.0);
  
   //GRID 1,1 - NED
  text("Latitude (deg): \n" + lat + "\nLongitude (deg): \n" + lon + "\nGPS Altitude (m): \n" + GPS_altitude,width/3.0,height*5.0/8.0); //
  
  text("Time: \n" + time,width/3.0,height*7.0/8.0);
  
  drawCube();
}

/////ROUTINE TO ROTATE A BOX ON THE SCREEN
void drawCube() {  
  pushMatrix();  //sets the view port window - these are openGL commands
  translate(width/2.0, height/2.0, 0); //translate the cube
  float c = (1.0/5.0)*(1.0/9.0)*0.8;
  //x is to the right and y is down
  scale(width*c,height*c,(width+height)/2.0*c); //scale the cube
  // a demonstration of the following is at 
  // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
  rotateZ(-ptp[0]); //phi 
  rotateX(ptp[1]); //theta
  rotateY(-ptp[2]); //psi in that order for standard Aerospace sequences
  buildBoxShape(); //use QUADS to draw the faces in different colors
  popMatrix(); //restore the orientation window 
}

/////ROUTINE TO DRAW A BOX
void buildBoxShape() {
  noStroke();
  beginShape(QUADS); //again openGL commands to draw a cube

  //Z+ (to the drawing area)
  fill(#00ff00); //Cube is in different colors
  vertex(-5, -5, 5);
  vertex(5, -5, 5);
  vertex(5, 5, 5);
  vertex(-5, 5, 5);

  //Z-
  fill(#0000ff); 
  vertex(-5, -5, -5);
  vertex(5, -5, -5);
  vertex(5, 5, -5);
  vertex(-5, 5, -5);

  //X-
  fill(#ff0000);
  vertex(-5, -5, -5);
  vertex(-5, -5, 5);
  vertex(-5, 5, 5);
  vertex(-5, 5, -5);

  //X+
  fill(#ffff00);
  vertex(5, -5, -5);
  vertex(5, -5, 5);
  vertex(5, 5, 5);
  vertex(5, 5, -5);

  //Y-
  fill(#ff00ff);
  vertex(-5, -5, -5);
  vertex(5, -5, -5);
  vertex(5, -5, 5);
  vertex(-5, -5, 5);

  //Y+
  fill(#00ffff);
  vertex(-5, 5, -5);
  vertex(5, 5, -5);
  vertex(5, 5, 5);
  vertex(-5, 5, 5);

  endShape(); //done with shape and quit
}

//Write data to file
void logdata() {
  //Log Data to Home Screen
  print("Logging Data: ");
  for (int idx = 0;idx<numVars;idx++) {
    print(received_data[idx] + " ");
    file.write(received_data[idx]+" ");
  }
  file.write("\n");
  file.flush();
  print("\n");
}

void SerialPutHello() {
  println("Sending w slash r");
  myport.write(119);
  myport.write(13);
  println("Sent");
}

int SerialGetHello() {
  //Consume w\r\n
  char inchar = '\0';
  println("Checking for Response",time_now);
  while(myport.available()>0){
    inchar = myport.readChar();
    if (int(inchar) == 119) {
      println("char = "+inchar + " int = ",int(inchar));
      println("w received ! \n");
      w = 1;
    }
    if ((int(inchar) == 13) && (w == 1)) {
       println("char = "+inchar + " int = ",int(inchar));
       println("slash r received! \n");
       slashr = 1;
    }
    if ((int(inchar) == 10) && (slashr+w == 2)) {
       println("char = "+inchar + " int = ",int(inchar));
       println("slash n received! \n");
       w = 0;
       slashr = 0;
       return 1;
    }
  }
  return 0;
}

void SerialGetArray() {
  for (int d = 0;d<numVars;d++) {
    int i = 0;
    char inchar = '\0';
    println("Waiting for characters");
    do {
      do {
        inchar = myport.readChar();
      } while (int(inchar) == 65535);
      println("Receiving: i = "+str(i)+" char = "+inchar+" chartoint = " + str(int(inchar)));
      inLine[i++] = inchar;
    } while ((inchar != '\r') && (i<60));
    println("Response received");

    // Format from Arduino:
    // H:nnnnnnnn 
    String inString = String.valueOf(inLine);
    
    // Now Convert from ASCII to HEXSTRING to FLOAT
    println("Converting to Float");
    String clean = inString.substring(2).replaceAll(" ","");
    String truncated = clean.substring(0,8);
    println(inString);
    println(clean);
    println(truncated);
    received_data[d] = Float.intBitsToFloat(unhex(truncated));
  }
  //Update time that we received a response
  last_response = time_now;
  logdata();
}

void CheckForResponse() {
  //If we have already requested data then we need to check for data 
  if (oktosend == 0) {
    //This routine checks for data from the drone
    int response = SerialGetHello();
    //if a response was received
    if (response == 1) {
      //A response is just w\r\n so we need to continue reading for 
      //H:nnnnnnn yadda yadda
      SerialGetArray();
      //We can reset the oktosend and request more data
      println("Response Received \n");
      oktosend = 1;
      //Then reset the timer 
      last_send = time_now;
    }
  }
 //Sometimes the message send to the hardware is not read. Lost in transmission really
 //So after time_wait seconds we will send the hardware another message. This is my solution
 //to the two generals problem. 
 //https://en.wikipedia.org/wiki/Two_Generals%27_Problem
 if ((time_now - last_send > time_wait) && (oktosend == 0)) {
   oktosend = 1;
   println("It has been " + str(time_wait) + " seconds since we requested data so we are going to try again"); 
 }
 
}
