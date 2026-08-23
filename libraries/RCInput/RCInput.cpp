#include "RCInput.h"


//constructor class
RCInput::RCInput() {
}

void RCInput::initialize() {
  //Receiver on Raspberry Pi
  //Or running as fast as possible
  #ifdef ARDUINO
  num_of_axis = RECV_N_CHANNEL; //Default to this number for arduino
  #else
  num_of_axis = 8; //default num_of_axis to 8 no matter what.
  //It might change if you have a joystick but at least it's initialized to 8
  #endif

  ///Joystick on Desktop
  #ifdef JOYSTICK
  printf("Initializing joystick \n");
  if((joy_fd = open(JOY_DEV,O_RDONLY)) == -1 ) {
    printf("Couldn't open joysticks \n");
    printf("Defaulting to Stick min \n");
    printf("joy_fd = %d \n",joy_fd);
  } else {
    printf("Getting information from the joystick \n");
    ioctl(joy_fd,JSIOCGAXES,&num_of_axis);
    ioctl(joy_fd,JSIOCGBUTTONS,&num_of_buttons);
    printf("Number of buttons = %d \n",num_of_buttons);
    ioctl(joy_fd,JSIOCGNAME(NAME_LENGTH),&name_of_joystick);
    printf("Name of Joystick = %s \n",name_of_joystick);
    printf("Allocating Buttons for Joysticks. Cross your fingers \n");
    button = (char *) calloc(num_of_buttons,sizeof(int));
    printf("Success!!!! \n");
    printf("Setting non-blocking mode \n");
    fcntl(joy_fd,F_SETFL,O_NONBLOCK);
  }

#define STRINGIFY_DETAIL(x) #x
#define STRINGIFY(x) STRINGIFY_DETAIL(x)

  std::string config_file;
  #if defined(RX)
  config_file = std::string(STRINGIFY(RX)) + "_config.json";
  #endif

  std::ifstream f(config_file);
  if (f.is_open()) {
    try {
      json j = json::parse(f);
      auto mapping = j["mapping"];

      const char* controls[5] = {"throttle", "roll", "pitch", "yaw", "autopilot_switch"};
      for (int i = 0; i < 5; i++) {
        if (mapping.contains(controls[i])) {
          auto ctrl = mapping[controls[i]];
          if (ctrl.contains("axis")) {
            axis_mappings[i].axis = ctrl["axis"].get<int>();
            axis_mappings[i].invert = ctrl.value("invert", false);
            axis_mappings[i].min = ctrl.value("min", -1.0);
            axis_mappings[i].max = ctrl.value("max", 1.0);
          }
        }
      }
      has_json_config = true;
      printf("Loaded JSON config from %s\n", config_file.c_str());
    } catch (std::exception& e) {
      printf("Error parsing %s: %s\n", config_file.c_str(), e.what());
    }
  } else {
    printf("Could not open JSON config file: %s\n", config_file.c_str());
  }
  #endif

  printstdout("Setting up RCIO System \n");
  printstdout("Allocating Axes \n");
  printstdout("Number of Axes = ");
  printstdoutint(num_of_axis);
  printstdout("\n");
  rx_array = (int *) calloc(num_of_axis,sizeof(int));
  joycomm = (int *) calloc(num_of_axis,sizeof(int));
  axis_id = (int *) calloc(num_of_axis,sizeof(int));
  printstdout("Done \n");

  //Extra stuff on RPi using a Receiver
  #ifdef RECEIVER
  for (size_t i = 0; i < num_of_axis; i++) {
    axis_id[i] = open_axis(i);
    if (axis_id[i] < 0) {
      printf("Error opening an axis \n");
      perror("open");
    }
  }
  #endif

  //Extra stuff for Arduino
  #ifdef ARDUINO
  // tell the Power Management Controller to disable the write protection of the (Timer/Counter) registers
  pmc_set_writeprotect(false);
  // enable external clock for counter TC2, channel 2
  pmc_enable_periph_clk(ID_TC8);
  // set clock tp 84MHz/2 = 42MHz
  TC_Configure(TC2, 2, TC_CMR_TCCLKS_TIMER_CLOCK1);
  // start timer
  TC_Start(TC2,2);
  // connect pins and handlers
  //ch0Handler();
  attachInterrupt(RECV_CHAN0PIN, &ch0Handler, CHANGE);
  attachInterrupt(RECV_CHAN1PIN, &ch1Handler, CHANGE);
  attachInterrupt(RECV_CHAN2PIN, &ch2Handler, CHANGE);
  attachInterrupt(RECV_CHAN3PIN, &ch3Handler, CHANGE);
  attachInterrupt(RECV_CHAN4PIN, &ch4Handler, CHANGE);
  attachInterrupt(RECV_CHAN5PIN, &ch5Handler, CHANGE);
  #endif
}

void RCInput::setStick(int val) {
  //printf("Setting stick to neutral \n");
  for (int idx = 0;idx<num_of_axis;idx++) {
    rx_array[idx] = val; //STICK_MIN #define
  }
}

void RCInput::LostCommCheck() {
  int lostcomms = 0;
  for (int idx = 0;idx<4;idx++) {
    if (rx_array[idx] == 0) {
      lostcomms = 1;
    }
  }
  if (lostcomms == 1) {
    //printstdout("Lost Communications Setting to Neutral!!!\n");
    setStickNeutral();
  }
}

void RCInput::saturationCheck() {
  for (int idx = 0;idx<num_of_axis;idx++) {
    if (rx_array[idx] < STICK_MIN) {
      //printstdout("REC Clipped \n");
      rx_array[idx] = STICK_MIN;
    }
    if (rx_array[idx] > STICK_MAX) {
      rx_array[idx] = STICK_MAX;
      //printstdout("REC Clipped \n");
    }
  }
}

void RCInput::RangeCheck() {
  int badconn = 0;
  for (int idx = 0;idx<4;idx++) {
    if (rx_array[idx] < 0 || rx_array[idx] > 2500) {
      badconn = 1;
    }
  }
  if (badconn == 1) {
    setStickNeutral();
  }
}

void RCInput::saturation_block() {
  //printf("Running Saturation Block!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  //First run a lost comms check
  LostCommCheck();
  //Then run a range check
  //RangeCheck();
  //Saturation Check
  saturationCheck();
}

void RCInput::setStickNeutral() {
  setStick(STICK_MIN); //First set all sticks to min
  //but then set the Aileron, Elevator and Rudder to mid
  rx_array[1] = STICK_MID;
  rx_array[2] = STICK_MID;
  rx_array[3] = STICK_MID;
}

int RCInput::bit2PWM(int val) {
  ///the values from the joystick are from -32678 to 32768 which is a 16 bit number
  //printf("val = %d \n");
  return (STICK_MAX-STICK_MID)*val/BIT_RANGE + STICK_MID;
}

void RCInput::readRCstate()
{
  #ifdef KEYBOARD
  for (int idx = 0;idx<num_of_axis;idx++) {
    //printf("%lf ",keyboard[idx]);
    if (idx<4) {
      rx_array[idx] = keyboard[idx]*(STICK_MAX-STICK_MIN)/2.0 + STICK_MID;
    } else {
      rx_array[idx] = STICK_MIN;
    }
  }
  //printf("\n");
  //printRCstate(-4);
  #endif

  #ifdef RECEIVER
  //printf("Reading from Receiver \n");
  for (int idx = 0;idx<num_of_axis;idx++) {
    rx_array[idx] = read_axis(idx);
  }
  #endif

  //Arduino uses interrupts to capture the data
  #ifdef ARDUINO
  for (int idx = 0;idx<num_of_axis;idx++) {
    rx_array[idx] = getRXvalue(idx);
  }
  #endif

  #ifdef JOYSTICK
  if (joy_fd != -1) {
    // cout << "Current time = " <<  << endl;
    //cout << "Reading Joystick state \n";
    read(joy_fd,&js,sizeof(struct js_event));
    // cout << "What is the joystick state? \n";
    switch (js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_AXIS:
      //if (js.number == 5) {
      //  printf("js.value and number = %d %d \n",js.number,js.value);
      //}
      joycomm[js.number] = bit2PWM(js.value);
      break;
    case JS_EVENT_BUTTON:
      button[js.number] = js.value;
      break;
    }
    mapjoy2rx();
  } else {
    setStickNeutral();
  }
  #endif

  #ifdef SIMONLY
  setStickNeutral();
  #endif

  //LostCommCheck();
  saturation_block();

  //printf("TTTT ");
  //printRCstate(-4);
  //printf("FFFF ");
}

int RCInput::invert(int val) {
  int out;
  //A value that is STICK_MIN will produce STICK_MAX and viceversa
  if (val != 0) {
    int delta = val - STICK_MID;
    int inverse = -delta;
    out = STICK_MID + inverse;
  } else {
    //A value of zero means loss of signal
    out = 0;
  }
  return out;
}

void RCInput::mapjoy2rx() {
  //First we're just going to copy everything over to make sure everything copies over properly
  for (int i = 0;i<num_of_axis;i++) {
    rx_array[i] = joycomm[i];
  }

  //Then based on the specific joystick and json file we'll map the joycomm values to the rx_array
  #ifdef JOYSTICK
  if (has_json_config) {
    // 0: throttle, 1: roll (aileron), 2: pitch (elevator), 3: yaw (rudder), 4: autopilot
    for (int i = 0; i < 5; i++) {
      int ax = axis_mappings[i].axis;
      if (ax >= 0 && ax < num_of_axis) {
        int val = joycomm[ax];
        if (axis_mappings[i].invert) {
          val = invert(val);
        }
        rx_array[i] = val;
      }
    }
    return;
  }
  #endif
}

void RCInput::printRCstate(int all) {
  //printf("Axis State = ");
  int val = num_of_axis;
  if (all < 0) {
    val = -all;
  }
  for (x = 0;x<val;x++){
    printstdoutint(rx_array[x]);
  }

  #ifdef JOYSTICK
  if (all == 1) {
    //printf(" Button State = ");
    for (x = 0;x<num_of_buttons;x++){
      printf("%d ",button[x]);
    }
  }
  #endif
  //printf("\n");
}

//This is where the interrupts happen on the Arduino
#ifdef ARDUINO
void RCInput::ch0Handler() {
  pwmHandler(0, RECV_CHAN0PIN);
}
void RCInput::ch1Handler(){
  pwmHandler(1, RECV_CHAN1PIN);
}
void RCInput::ch2Handler(){
  pwmHandler(2, RECV_CHAN2PIN);
}
void RCInput::ch3Handler(){
  pwmHandler(3, RECV_CHAN3PIN);
}
void RCInput::ch4Handler(){
  pwmHandler(4, RECV_CHAN4PIN);
}
void RCInput::ch5Handler(){
  pwmHandler(5, RECV_CHAN5PIN);
}

int RCInput::getRXvalue(int chann) {
  return rx_array_static[chann];
}

void RCInput::pwmHandler(int chann,int pin){
  unsigned int timeCurrentChange = TC2->TC_CHANNEL[2].TC_CV;
  // true if pins just went high
  if(digitalRead(pin))
    {
      timeLastChange[chann] = timeCurrentChange;
    }
  else
    {
      // clock runs at 42MHz therefore divide with 42 to get us
      rx_array_static[chann] = (timeCurrentChange - timeLastChange[chann])/42;
      //Serial.print("Channel = ");
      //Serial.print(chann);
      //Serial.print(" ");
      //Serial.print(rx_array_static[chann]);
      //Serial.print("\n");
    }
}
#endif

//Raspberry Pi specific codes
#ifdef RECEIVER
int RCInput::open_axis(int channel)
{
  char *channel_path;
  if (asprintf(&channel_path, "%s/ch%d", RCIN_SYSFS_PATH, channel) == -1) {
    err(1, "channel: %d\n", channel);
  }
  int fd = ::open(channel_path, O_RDONLY);
  free(channel_path);
  return fd;
}

//This is where we read the axis
int RCInput::read_axis(int ch)
{
  if (ch > num_of_axis)
    {	
      fprintf(stderr,"Channel number too large\n");
      return -1;
    }
  char buffer[10];
  //so this is some fancy bit shit bullcrap that only computer scientist know how this works
  if (::pread(axis_id[ch], buffer, ARRAY_SIZE(buffer), 0) < 0) {
    perror("pread");
  }
  //this function atoi converts a char to a integer
  return atoi(buffer);
}
#endif

//empty destructor class
RCInput::~RCInput(){
}
