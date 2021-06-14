//*******************ADNS pins n stuff ***************//

#include "SPI.h"

#include <math.h>  

#include <Wire.h>
#include <INA219_WE.h>
#include <math.h>

// these pins may be different on different boards
// this is for the uno
#define PIN_SS        10
#define PIN_MISO      12
#define PIN_MOSI      11
#define PIN_SCK       13

#define PIN_MOUSECAM_RESET     8
#define PIN_MOUSECAM_CS        7

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60
#define ADNS3080_PRODUCT_ID_VAL        0x17

#define PI 3.14159265359

INA219_WE ina219; // this is the instantiation of the library for the current sensor

float open_loop, closed_loop; // Duty Cycles
float vpd,vb,vref=3,iL,dutyref,current_mA; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration
float ev=0,cv=0,ei=0,oc=0; //internal signals
float Ts=0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float kpv=0.05024,kiv=15.78,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float kpi=0.02512,kii=39.4,kdi=0; // current pid.
float u0i,u1i,delta_ui,e0i,e1i,e2i; // Internal values for the current controller
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max=1, ui_min=0; //anti-windup limitation
float output_max=4, output_min=0; //anti-windup limitation
float current_limit = 1.0;
boolean Boost_mode = 0;
boolean CL_mode = 0;

unsigned int loopTrigger;

//************************** Motor Constants **************************//
unsigned long lastUpdate = 0;
int DIRRstate = LOW;              //initializing direction states
int DIRLstate = HIGH;

int DIRL = 20;                    //defining left direction pin
int DIRR = 21;                    //defining right direction pin

int pwmr = 5;                     //pin to control right wheel speed using pwm
int pwml = 9;                     //pin to control left wheel speed using pwm
//*******************************************************************//


float dx=0;
float dy=170;
double total_x = 0;
double total_x_prev = 0;
double total_y_prev = 0;
double total_y = 0;
double total_x1 = 0;
double total_y1 = 0;
int a=0;
int b=0;
int distance_x=0;
int distance_y=0;
volatile byte movementflag=0;
volatile int xydat[2];
const float distCenterToCamera = 72;// the distance from the center of the tire line to the camera
const double inflatedRadiusObst = 150;//ball radius is 2cm, from center of rover to the end of the tire is 10cm, 3cm leeway
const double outerInflatedRadius = 220; //outer inflated radius to stop before we hit the obstacle and to reduce the angle we have to take to avoid it
float rover_radius = 149;
const double angleInAvoiding = 48.6;
const double distToAvoided = 266;
double avoidanceAngle=0;

// ******************** PID compute variables *************//

float error_turn_r, error_turn, error_previous_r, error_move;
float output_previous_r, output_turn_r;
float delta_output_r;
float kp_turn_r=0.05, kd_turn_r=0.001, ki_turn_r=5;
double Setpoint;
int triggerOuterLoop=0;


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
}


struct MD{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};

struct obstacle{
  double x;//x coord
  double y;//y coord
  int flag;
};

obstacle obstArray[5];
int obstArSize = 5; //sizeof() function does not work well with array of structs so define the number of obstacles
obstacle currentObstAvoided;


const char beginEndTransm = '/';
const char endSign = ']';
const char beginSign = '[';
String commandSpeed = "";
String commandTurn = "";
String commandTurnPrev = "";
double commandTurnAngle;
String commandCoordX = "";//in cm
String commandCoordXPrev = "";//in cm
double finalCoordX = 0;//in mm
String commandCoordY = "";
String commandCoordYPrev = "";
double finalCoordY = 0;//in mm
String commandObstDist = "";
String commandObstDistPrev = "";
double obstDist=10000;
String commandObstAngle = "";
String commandObstAnglePrev = "";
String commandColor = "";
String commandColorPrev = "";
double obstAngle; //in degrees
int commandNum = 0;
double trackAngle = 0; //in degrees
double trackCoordX = 0;//in mm
double trackCoordY = 0;
double obstCoordX; //in mm
double obstCoordY; //in mm
int obstColor = 0;
double ballRadius = 20;//measured radius of 20mm
double distToFinal;//tracks the distance to reach the final coordinates

double neededAngle = 0;//angle used to go to coordinates
double initialAngle = 0;//used to track initial angle when moving to coordinates
double initialAngleAvoiding = 0;//used to track initial angle when avoiding obstacles
double initialAngleCommand = 0;//used to track initial angle when receiving an angle from web app
double angleToAvoidObst = 0;

int avoidanceMode = 0;//will be used to make sure the obstacle is avoided before doing other commands
unsigned long timeToStop = 280;// measured that it takes the rover 100ms to stop
unsigned long trackStoppingTime = 0;
int stateTurned=2;//track when the turning is finished
int stateMoved=2;//track when the moving is finished
int stateCoord=4;//track when the moving to position is finished
int stateAngleReceived=2; //tracks when the command from the web app is completed
int stateCoordReceived=2; //tracks when the command from the web app is completed
int stateSpeedReceived=2; //tracks when the command from the web app is completed
int stateAvoid=4; //tracks when the avoiding is finished


//******************Function declarations**********************

//For optical sensor
void mousecam_reset();
int mousecam_init();
void mousecam_write_reg(int reg, int val);
int mousecam_read_reg(int reg);
void mousecam_read_motion(struct MD *p);
int mousecam_frame_capture(byte *pdata);
void setupOptSen();

//For SMPS
void sampling();
float saturation( float sat_input, float uplim, float lowlim);
void pwm_modulate(float pwm_input);
float pidv( float pid_input);
float pidi(float pid_input);
void setUpSMPS();

//Functions for moving
void stopRover();
void moveforward();
void moveWithSpeed();
void PID_move_fwd(double distance);
void PID_turn_left (double theta);
void PID_turn_right (double theta);
float computePID(float inp);
void goToCoord();
void avoidObst(obstacle needsAvoiding);
void commandRover();
void moveWithSpeed();
void checkObstacles();
bool checkOption(double cx, double cy);

//Functions for communication
void serialToString();
void sendData();
void getObstCoord();
void trackCoordinates();



// *************************** setup *****************************//

void setup() {
  
  //************************** Motor Pins Defining **************************//
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  pinMode(pwmr, OUTPUT);
  pinMode(pwml, OUTPUT);
  //*******************************************************************//

  //Basic pin setups
  
  noInterrupts(); //disable all interrupts
  pinMode(13, OUTPUT);  //Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); //Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  
  pinMode(6, OUTPUT);
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c

// ***********************ADNS setup *********************//

  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Serial started");

  if(mousecam_init()==-1)
  {
    Serial.println("Mouse cam failed to init");
    while(1);
  }  
  //initialise the obstacles
  for(int i=0;i<obstArSize;i++){
    obstArray[i].x=0;
    obstArray[i].y=0;
    obstArray[i].flag=0;
  }
  currentObstAvoided = {0,0,0};
}

unsigned long messageInterval = 2500;
double moveConst = 1000;

void loop(){
  setUpSMPS(); //takes samples and computes pid values every time it loops
  trackCoordinates(); //coordinate tracking function that uses the optical sensor
  setupOptSen(); //updates the values of the sensor every loop
  while (Serial1.available() > 0) {
    serialToString(); //function that decodes the UART messages from control, after it's finished it sends info to control (sending f-n is called inside)
  }


  commandRover();
  
  if(lastUpdate+messageInterval<millis()){
    Serial.print("Track X pos: ");
    Serial.println(trackCoordX);
    Serial.print("Track Y pos: ");
    Serial.println(trackCoordY);
    
    Serial.print("Track Angle: ");
    Serial.println(trackAngle);
    lastUpdate = millis();
  }
  
}

// Timer A CMP1 interrupt. Every 800us the program enters this interrupt. 
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
}


//The tracking function is based on the previous and current values of the optical sensor
//The three tracking variables trackAngle, trackCoordX and trackCoordY are updated each loop
//When the rover is turning by command, we have disabled the X/Y tracking because it turns along it's center
//Particularly useful function because when the rover moves forward, its tires sometimes twist
//Although the rover is only moving forward, its angle may change which means it won't reach the designated position if it weren't for tracking
void trackCoordinates(){
  //Calculate the difference between past and current value
  //x difference tracks the angle, y difference tracks the forward movement
  double x_difference = total_x - total_x_prev;
  double y_difference = total_y - total_y_prev;
  //When we finish a turning/moving function, the optical sensor values are resetted so they can be used to track the completion of the next turn/move
  //This resetting will create a big difference between past and current value, which is unwanted behaviour as it would drastically change the coordinates
  //So we check if the difference is very big, then we ignore it
  //On maximum speed the difference between past and current value of x/y does not exceed 3, so +-10 is an overestimation
  if(x_difference>10 || y_difference>10 || x_difference<-10 || y_difference<-10){
    x_difference = 0;
    y_difference = 0;
  }
  //Track the angle only when there is an x difference
  if(x_difference!=0){
    trackAngle-= x_difference*360/(rover_radius*2*PI);
  }
  //Track the coordinates using the angle and y difference
  //Only on when the rover is not turning
  //The if statement could be removed to track inaccuracies when turning but needs more testing
  if(stateTurned==2){
    trackCoordX += sin(trackAngle*PI/180)*y_difference;
    trackCoordY += cos(trackAngle*PI/180)*y_difference;
  }
  
  total_x_prev = total_x;
  total_y_prev = total_y;
}


//Simple move forward function
//Works only if the output voltage of the SMPS is higher than ~2
void moveforward() {
    DIRRstate = HIGH;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH); 
    digitalWrite(DIRR, DIRRstate);
    digitalWrite(DIRL, DIRLstate);}


//Stops the rover
void stopRover() {
   DIRRstate = LOW;
   DIRLstate = LOW;
   digitalWrite(pwmr, LOW);
   digitalWrite(pwml, LOW); 
   digitalWrite(DIRR, DIRRstate);
   digitalWrite(DIRL, DIRLstate);
}


//Make the rover move forward or backward depending on command (first, second, third and reverse)
//h is for halt/stop
void moveWithSpeed(){
  if(commandSpeed!=""){
    if(commandSpeed=="f"){
      vref=2;
      DIRRstate = HIGH;
      DIRLstate = LOW;
      digitalWrite(pwmr, HIGH);
      digitalWrite(pwml, HIGH); 
    }else if(commandSpeed=="s"){
      vref=3;
      DIRRstate = HIGH;
      DIRLstate = LOW;
      digitalWrite(pwmr, HIGH);
      digitalWrite(pwml, HIGH);
    }else if(commandSpeed=="t"){
      vref=4;
      DIRRstate = HIGH;
      DIRLstate = LOW;
      digitalWrite(pwmr, HIGH);
      digitalWrite(pwml, HIGH);
    }else if(commandSpeed=="r"){
      vref=2;
      DIRRstate = LOW;
      DIRLstate = HIGH;
      digitalWrite(pwmr, HIGH);
      digitalWrite(pwml, HIGH);
    }else if(commandSpeed=="h"){
      Serial.println("Stopped");
      digitalWrite(pwmr, LOW);
      digitalWrite(pwml, LOW);
    }
    
    digitalWrite(DIRR, DIRRstate);
    digitalWrite(DIRL, DIRLstate);
  }
}


//By tracking the optical sensor, we can compute the error each time this function is entered
//It is important to input a constant variable into this function, otherwise the function may not work as expected
//The PID controller gives an output voltage, which controls how fast the rover is moving
//This function was used to calibrate the optical sensor by telling the rover to move 1m
//After the final postion is reached, the rover stops moving
//Although a perfect PID controller should make it stop by itself, in practice there will always be some oscillations
//So stop the rover when it has reached the location and reset the optical sensor values
//The final state is 2. To begin moving a certain distance, the state needs to be reset to 0
void PID_move_fwd(double distance)
{
  if(total_y < distance && triggerOuterLoop>=4 && stateMoved==0)
  {    
    error_move = distance - total_y;
    vref = computePID(error_move);
    setUpSMPS();
    triggerOuterLoop=0;
    DIRRstate = HIGH;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH); 
    digitalWrite(DIRR, DIRRstate);
    digitalWrite(DIRL, DIRLstate);
  } else if(total_y >= distance && stateMoved==0){
    Serial.println("Location reached");
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW); 
    vref = 0;
    stateMoved=1;
    trackStoppingTime = millis();
  }else if(stateMoved==1 && trackStoppingTime+timeToStop<millis()){
    total_x1 = 0;
    total_y1 = 0;
    stateMoved=2;
    Serial.println("Done moving forward");
  }
}


//Turn left function using a PID controller
//The PID controller was tuned by making the rover turn 180 degrees because it's easiest to see the accuracy
//High integral gain to make the error small
//A bit of integral gain to smooth out the function
//The function makes a setpoint based on the angle input, that setpoint is calculated by noting that only the x value of the sensor changes when turning around its center
//This is due to sampling very fast and the y difference being close to 0
//Using the distance from the center of the rover to the optical sensor and calibrating it, we can calculate the setpoint knowing the circumference of the opt sensor when the rover turns 360 degrees
//Note that the rover cannot stop immediately but need ~250ms to stop. This means it will turn a bit more than wanted, so this is accounted for when comparing the current position with the setpoint
void PID_turn_left (double theta)
{
  double theta_rad= theta * PI/180;  
  Setpoint = theta_rad*rover_radius;//turning right makes the opt sensor move to negative x
  
  if(total_x <= (Setpoint - 16) && triggerOuterLoop>=4 && stateTurned==0)
  {    
    error_turn = abs(Setpoint) - abs(total_x);
    vref = computePID(error_turn);
    setUpSMPS();
    DIRRstate = HIGH;
    DIRLstate = HIGH;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH); 
    digitalWrite(DIRR, DIRRstate);
    digitalWrite(DIRL, DIRLstate);
    triggerOuterLoop=0;
  }else if(total_x > (Setpoint - 16) && stateTurned==0){
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
    vref = 0;
    stateTurned=1;
    trackStoppingTime = millis();
  }else if(stateTurned==1 && trackStoppingTime+timeToStop<millis()){
    total_x1 = 0;
    total_y1 = 0;
    stateTurned=2;
    Serial.println("Done turning");
  }
}


//Turn right using PID
//Detailed explanation before turn left function
void PID_turn_right (double theta)
{
  double theta_rad= theta * PI/180;  
  Setpoint = - theta_rad*rover_radius;//turning right makes the opt sensor move to negative x
  
  if(total_x >= (Setpoint + 16) && triggerOuterLoop>=4 && stateTurned==0)
  {    
    error_turn = abs(Setpoint) - abs(total_x);
    vref = computePID(error_turn);
    setUpSMPS();
    DIRRstate = LOW;
    DIRLstate = LOW;
    digitalWrite(pwmr, HIGH);
    digitalWrite(pwml, HIGH); 
    digitalWrite(DIRR, DIRRstate);
    digitalWrite(DIRL, DIRLstate);
    triggerOuterLoop=0;
  }else if(total_x < (Setpoint + 16) && stateTurned==0){
    digitalWrite(pwmr, LOW);
    digitalWrite(pwml, LOW);
    vref = 0;
    stateTurned=1;
    trackStoppingTime = millis();
  }else if(stateTurned==1 && trackStoppingTime+timeToStop<millis()){
    total_x1 = 0;
    total_y1 = 0;
    stateTurned=2;
    Serial.println("Done turning");
  }
}



// PID function for VREF
//The frequency of this PID is 4 times less than the frequency of the VREF function
//Although having the same frequency would still work, we apply the pricniples from control and inner/outer loop dynamics
float computePID(float inp){     
  float e_integration;
  error_turn_r = inp;
  e_integration = error_turn_r;
  
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(output_previous_r >= output_max) {
    e_integration = 0;
  } else if (output_previous_r <= output_min) {
    e_integration = 0;
  }
  
  delta_output_r = kp_turn_r*error_turn_r + ki_turn_r*(Ts*4)*e_integration + kd_turn_r/(Ts*4)*(error_turn_r-error_previous_r); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  output_turn_r = output_previous_r + delta_output_r;  //this time's control output
  
  //output limitation
  output_turn_r = saturation(output_turn_r,output_max,output_min);
  
  output_previous_r = output_turn_r; //update last time's control output
  error_previous_r = error_turn_r; // update last time's error
  
  return output_turn_r;
}


//Function that goes to the needed coordinate and avoids the obstacles in the way
//Function has 5 states, starting from 0 => initialising variables
void goToCoord(){
  if(stateCoord==0){
    double y_dif = finalCoordY - trackCoordY;
    double x_dif = finalCoordX - trackCoordX;
    //By computing the difference in the x/y direction, the needed angle can be calculated (the y-axis is 0 degrees)
    //Depending on the current position and final one, the equation changes slightly
    if(y_dif>=0){
      neededAngle = atan2(x_dif, y_dif)*180/PI;
    }else{
      if(x_dif>=0){
        neededAngle = (atan2(abs(y_dif),x_dif)*180/PI) + 90;
      }else{
        neededAngle = -(atan2(abs(y_dif),abs(x_dif))*180/PI) - 90;
      }
    }
    //Make states of turning and moving forward 0, so they can be used in the next parts of the function
    stateTurned=0;
    stateMoved=0;
    //remember the beginning angle, so we can have a constant input to the turning function
    initialAngle = trackAngle;
    
    Serial.print("Needed Angle: ");
    Serial.println(neededAngle);
    distToFinal = sqrt(sq(finalCoordX-trackCoordX)+sq(finalCoordY-trackCoordY));
    stateCoord++;
    Serial.print("Distance to finish: ");
    Serial.println(distToFinal);
  }else if(stateCoord==1){
    //In this state the rover turns
    if(stateTurned!=2){
      //Decide whether to turn left or right
      if(neededAngle>initialAngle){
        PID_turn_right(abs(neededAngle-initialAngle));
      }else if(neededAngle<initialAngle){
        PID_turn_left(abs(initialAngle-neededAngle));
      }
    }else{
      //When the rover is done turning, go to next state
      stateCoord++;
      Serial.println("Going to coordinate");
    }
  }else if(stateCoord==2){
    //In this state the rover moves to the wanted position and checks for obstacles in the way
    //Check obstacles function flags the obstacles that are in our path (the obstacles may be behind our path too, but are only avoided when the rover is close to them)
    checkObstacles();
    //Only check for avoiding the flagged obstacles and move forward if the rover is not currently avoiding an obstacle
    if(stateAvoid==4){
      if(stateMoved==2){
        stateCoord=3;
      }
      PID_move_fwd(distToFinal); 
      for(int i=0;i<obstArSize;i++){
        if(obstArray[i].flag==1){
          //If the obstacle is flagged,check how close the rover is
          double xToObst = (trackCoordX-obstArray[i].x)/10;//divide by 10 so it does not overflow
          double yToObst = (trackCoordY-obstArray[i].y)/10;
          double distToCrash = 10*sqrt(xToObst*xToObst+yToObst*yToObst);
          
          //if the distance to the flagged object is less than outerInflatedRadius, start avoiding procedure
          if(distToCrash<=outerInflatedRadius){
            digitalWrite(pwmr, LOW);
            digitalWrite(pwml, LOW);
            vref = 0;
            stateAvoid=0;
            //initialise the obstacle that is currently being avoided, so we can pass the obstacle to the avoid function
            currentObstAvoided.x = obstArray[i].x;
            currentObstAvoided.y = obstArray[i].y;
            currentObstAvoided.flag = 1;
            
          }
        }
      }
    }
    //Only avoid the obstacle if the currentObstAvoided is flagged
    if(currentObstAvoided.flag==1){
      avoidObst(currentObstAvoided);
    }
    
  }else if(stateCoord==3 && ((trackCoordX>(finalCoordX+15)||trackCoordX<(finalCoordX-15)) || (trackCoordY>(finalCoordY+15) || trackCoordY<(finalCoordY-15)))){
    //Sometimes the wheels of the rover turn slightly so the rover will not be able to reach the final position
    //This checks for this case and if so it calls the function again
    Serial.println("Starting again");
    Serial.print("Reached X pos: ");
    Serial.println(trackCoordX);
    Serial.print("Reached Y pos: ");
    Serial.println(trackCoordY);
    stateCoord = 0;
  }else if(stateCoord==3){
    //When the final position is reached, change the state
    stateCoord=4;
    Serial.println("Final position reached");
    Serial.print("Reached X pos: ");
    Serial.println(trackCoordX);
    Serial.print("Reached Y pos: ");
    Serial.println(trackCoordY);
    if(stateCoordReceived==1){
      stateCoordReceived=2;
    }
    //After reaching the final position, unflag all the obstacles
    for(int i=0;i<obstArSize;i++){
      obstArray[i].flag=0;
    }
  }
}

//Handles the commands received
//Priorities: coordinates, then turning, then moving
void commandRover(){
  if(commandSpeed=="" && commandTurn=="" && (commandCoordX=="" || commandCoordY=="")){
    //If there is no input, stop the rover
    stopRover();
  }else if(obstDist<=30){
    //if we are closer than 3cm to the obstacle, stop the rover
    //safety function
     
  }else if(commandCoordX!="" && commandCoordY!="" && stateCoordReceived==0){
    //start going to the coordinates by initialising stateCoord to 0
    stateCoord = 0;
    stateCoordReceived++;
  }else if(commandCoordX!="" && commandCoordY!="" && stateCoordReceived==1){
    if(stateCoord==4){
      //when the final position is reached, end the command
      stateCoordReceived=2;
    }
    goToCoord();
  }else if(commandTurn!="" && stateAngleReceived==0){
    stateTurned=0;
    stateAngleReceived++;
    initialAngleCommand = trackAngle;
    
    
  }else if(commandTurn!="" && stateAngleReceived==1){
    if(stateTurned==2){
      //When finished turning, end the command
      stateAngleReceived=2;
    }
    if(commandTurnAngle>initialAngleCommand){
      PID_turn_right(abs(commandTurnAngle-initialAngleCommand));
    }else{
      PID_turn_left(abs(initialAngleCommand-commandTurnAngle));
    }
  }else if(commandSpeed!=""){
    moveWithSpeed();    
  }
}

//Gets the coordinates of the obstacle based on the information received from vision and the current position of the rover
void getObstCoord(){
  if(commandObstDist!="" && commandObstAngle!=""){
    //Get the coordinates of the camera because the vision information is relative to it
    double CamX = total_x + distCenterToCamera*sin(trackAngle*PI/180);//calculate coordinate of camera, angle is relative to y-axis
    double CamY = total_y + distCenterToCamera*cos(trackAngle*PI/180);
    double ObstAngleRel = trackAngle + obstAngle; //get the angle of the obstacle relative to the y-axis
    obstCoordX = CamX + (obstDist+ballRadius)*sin(ObstAngleRel*PI/180);//calculate the position of the obstacle
    obstCoordY = CamY + (obstDist+ballRadius)*cos(ObstAngleRel*PI/180); 
    //After calculating the angle, check whether this obstacle was encounterd before (there is some leeway because the vision information is not that accurate)
    //If the obstacle has not been encountered, store it in the obstacle array
    for(int i=0;i<obstArSize;i++){
      if(!(obstArray[i].x==0&&obstArray[i].y==0)){
        if((obstArray[i].x<=(obstCoordX+30)&&obstArray[i].x>=(obstCoordX-30))&&(obstArray[i].y<=(obstCoordY+30)&&obstArray[i].y>=(obstCoordY-30))){
          //If we have encountered the obstacle, break. We do not need to check the rest of the obstacles
          obstCoordX = obstArray[i].x;
          obstCoordY = obstArray[i].y;
          break;
        }
      }else if(obstArray[i].x==0&&obstArray[i].y==0){
        //If the position in the array is empty and the obstacle has not been encountered, save the coordinates and break
        obstArray[i].x=obstCoordX;
        obstArray[i].y=obstCoordY;
        break;
      }
    }
  }
}


//function to check whether there are any obstacles in the way
//we inflate the obstacle by the rover radius plus some overapproximation because of inaccurate data from vision
//this way we can see the rover as a dot and just make the dot avoid the inflated obstacle
//equation of obstacle is (x-obst.x)^2 + (y-obst.y)^2=radius^2
//equation of the path to the final destination y=ax+b
//a is the slope, can find it by y1-y2/x1-x2
//we can find b by substitution

void checkObstacles(){
  for(int i=0;i<obstArSize;i++){
    double a_slope = (finalCoordY-trackCoordY)/(finalCoordX-trackCoordX);
    double b_offset = finalCoordY-finalCoordX*a_slope;
    //The determinant equation was done by hand, substituting y=ax+b in the circle equation of the obstacle
    double determinant = -2*a_slope*b_offset*obstArray[i].x + 2*a_slope*obstArray[i].x*obstArray[i].y - b_offset*b_offset - obstArray[i].y*obstArray[i].y + inflatedRadiusObst*inflatedRadiusObst + 2*b_offset*obstArray[i].y - a_slope*a_slope*obstArray[i].x*obstArray[i].x + a_slope*a_slope*inflatedRadiusObst*inflatedRadiusObst; //solving this equation shows if the circle of the obstacle and the path to the final coord intercept
    //If the determinant is positive, then the line to the final coordinates crosses the obstacle and it is flagged
    //Flagging an obstacle does not necassarily mean the rover needs to avoid it
    if(determinant>=0 && obstArray[i].flag!=1 && !(obstArray[i].x==0 && obstArray[i].y==0)){ //check whether the obstacle is initiated
      obstArray[i].flag = 1; //flag the obstacle that it's in the path, clear it once it's avoided
      Serial.print("Object was flagged: ");
      Serial.println(obstArray[i].x);
      
    }
  }
}


//When avoiding the obstacle, we choose two directions and this function checks whether the direction is safe (no obstacle)
//The rover only goes a certain distance forward when avoiding the obstacle, so only check the final coordinates of the avoiding state
bool checkOption(double cx, double cy){
  for(int i=0;i<obstArSize;i++){
    if(!(obstArray[i].x==0 && obstArray[i].y==0)){
      double cx_dif = obstArray[i].x-cx;
      double cy_dif = obstArray[i].y-cy;
      double c_dist = sq(cx_dif*cx_dif+cy_dif*cy_dif);
      if(c_dist<=inflatedRadiusObst){
        return false;
      }
    }
  }
  return true;
}


//This is the function responsible for avoiding the obstacle
//Final state is 4, initial is 0
void avoidObst(obstacle needsAvoiding){
  if(stateAvoid==0){
    initialAngleAvoiding=trackAngle;
    double angleRoverObst = atan2(abs(needsAvoiding.y-trackCoordY),abs(needsAvoiding.x-trackCoordX))*180/PI;
    if(needsAvoiding.x-trackCoordX<=0 && needsAvoiding.y-trackCoordY>0){
      angleRoverObst=180-angleRoverObst;//second quadrant
    }else if(needsAvoiding.x-trackCoordX<0 && needsAvoiding.y-trackCoordY<0){
       angleRoverObst+=180;//third quadrant
    }else if(needsAvoiding.x-trackCoordX>0 && needsAvoiding.y-trackCoordY<=0){
       angleRoverObst=-angleRoverObst;//fourth quadrant
    }
    //Calculation for the two possible routes, then compare them to choose the closest one to the final coordinates
    double avoidOptionOneX = cos((angleRoverObst-angleInAvoiding)*PI/180)*distToAvoided + trackCoordX;
    double avoidOptionOneY = sin((angleRoverObst-angleInAvoiding)*PI/180)*distToAvoided + trackCoordY;
    double avoidOptionTwoX = cos((angleRoverObst+angleInAvoiding)*PI/180)*distToAvoided + trackCoordX;
    double avoidOptionTwoY = sin((angleRoverObst+angleInAvoiding)*PI/180)*distToAvoided + trackCoordY;
    
    double difOpt1X = (finalCoordX-avoidOptionOneX)/10;
    double difOpt1Y = (finalCoordY-avoidOptionOneY)/10;
    double difOpt2X = (finalCoordX-avoidOptionTwoX)/10;
    double difOpt2Y = (finalCoordY-avoidOptionTwoY)/10;
    double opt1Dist = 10*sqrt(difOpt1X*difOpt1X + difOpt1Y*difOpt1Y);
    double opt2Dist = 10*sqrt(difOpt2X*difOpt2X + difOpt2Y*difOpt2Y);
    bool check1 = checkOption(avoidOptionOneX, avoidOptionOneY);
    bool check2 = checkOption(avoidOptionTwoX, avoidOptionTwoY);
    
    
    if(opt1Dist<=opt2Dist && check1){
      //The angle needed to avoid the obstacle, with respect ot y-axis
      Serial.println("Choosing option 1");
      avoidanceAngle=90-(angleRoverObst-angleInAvoiding);//previously angle was with respect to x-axis, so 90- makes it with respect to y-axis
    }else if(check2){
      Serial.println("Choosing option 2");
      avoidanceAngle=90-(angleRoverObst+angleInAvoiding);
    }
    stateTurned=0;
    stateAvoid=1;

    angleToAvoidObst = abs(avoidanceAngle-initialAngleAvoiding);
    if(angleToAvoidObst>=360){
      angleToAvoidObst-=360;
    }


    
  }else if(stateAvoid==1){
    //Decide whether to turn left or right
    if(stateTurned==2){
      //When done turning, move to the next state
      stateMoved=0;
      stateAvoid=2;
    }else{
      if(avoidanceAngle>=initialAngleAvoiding){
        PID_turn_right(angleToAvoidObst);
      }else{
        PID_turn_left(angleToAvoidObst);
      }
    }
  }else if(stateAvoid==2 && stateMoved==2){
    //Check if the moving forward command has finished, if so move to the next state
    stateAvoid=3;
  }else if(stateAvoid==2){
    PID_move_fwd(distToAvoided);
  }else if(stateAvoid==3){
    Serial.println("Done avoiding");
    //Unflag the obstacle that was avoided
    currentObstAvoided.flag=0;
    for(int i=0;i<obstArSize;i++){
      if(needsAvoiding.x==obstArray[i].x && needsAvoiding.y==obstArray[i].y){
        obstArray[i].flag=0;
      }
    }
    //Initialise the goToCoord function so it calculates the new route
    stateCoord=0;
    stateAvoid=4;
  }
}





//Reading from Serial and send back data
//The data format is:      /[speed][angle][finalX][finalY][obstDist][obstAngle]/
//Using commandNum the character read from serial is added to the right command
//After decoding the data, using the previous one, it is decided whether to change the command variables
//If there is new information, the new command is initialised by changing the specific state to 0
//At the end of this function, sendData function is called to send data to control
void serialToString(){
  while (Serial1.available() > 0) {
    int byteReceived = Serial1.read();
    char inp = (char)byteReceived;
    if(inp==beginEndTransm && commandNum==0){
      //Serial.println("Conversion started");
      commandSpeed = "";
      commandTurn = "";
      commandCoordX = "";
      commandCoordY = "";
      commandObstDist = "";
      commandObstAngle = "";
      commandColor = "";
    }else if(inp==beginEndTransm && (commandNum==7||commandNum==6)){
      //Serial.println("Conversion ended");
      commandNum=0;
      if(commandTurn!=commandTurnPrev){
        stateAngleReceived=0;
        commandTurnAngle = commandTurn.toDouble();
        Serial.println("Command Turn: " + commandTurn);
      }
      if(commandCoordX!=commandCoordXPrev){
        stateCoordReceived=0;
        finalCoordX = 10*commandCoordX.toDouble();
        Serial.println("Command CoordX: " + commandCoordX);
      }
      if(commandCoordY!=commandCoordYPrev){
        stateCoordReceived=0;
        finalCoordY = 10*commandCoordY.toDouble();
        Serial.println("Command CoordY: " + commandCoordY);
      }
      if(commandObstDist!=commandObstDistPrev){
        obstDist = 10*commandObstDist.toDouble();
        Serial.println("Command Distance: " + commandObstDist);
      }
      if(commandObstAngle!=commandObstAnglePrev){
        obstAngle = commandObstAngle.toDouble();
        Serial.println("Command Angle: " + commandObstAngle);
      }
      if(commandColor!=commandColorPrev){
        obstColor = commandColor.toInt();
        Serial.println("Command Color: " + commandColor);
      }
      //Serial.println("Command Speed: " + commandSpeed);
      //Serial.println("Sending started");
      commandTurnPrev=commandTurn;
      commandCoordXPrev=commandCoordX;
      commandCoordYPrev=commandCoordY;
      commandObstDistPrev=commandObstDist;
      commandObstAnglePrev=commandObstAngle;
      commandColorPrev=commandColor;
      sendData();
      return;
    }else if(inp==endSign||byteReceived==44){
      commandNum++;
    }else if(inp!=beginSign && inp!=endSign && commandNum==0 && inp!=beginEndTransm && inp!=","){
      commandSpeed+=inp;
    }else if(inp!=beginSign && inp!=endSign && commandNum==1 && inp!=beginEndTransm && inp!=","){
      commandTurn+=inp;
    }else if(inp!=beginSign && inp!=endSign && commandNum==2 && inp!=beginEndTransm && inp!=","){
      commandCoordX+=inp;
    }else if(inp!=beginSign && inp!=endSign && commandNum==3 && inp!=beginEndTransm && inp!=","){
      commandCoordY+=inp;
    }else if(inp!=beginSign && inp!=endSign && commandNum==4 && inp!=beginEndTransm && inp!=","){
      commandColor+=inp;
    }else if(inp!=beginSign && inp!=endSign && commandNum==5 && inp!=beginEndTransm && inp!=","){
      commandObstDist+=inp;
    }else if(inp!=beginSign && inp!=endSign && commandNum==6 && inp!=beginEndTransm && inp!=","){
      commandObstAngle+=inp;
    }
  }
}


//send data to ESP with format: /[currentX][currentY][currentAngle][obstCoordX][obstCoordY];
//Divide the coordinates by 10, so they're in cm
void sendData(){
  getObstCoord();
  String formattedSend = "/[" + String((int)(round(trackCoordX/10))) + "," + String((int)(round(trackCoordY/10))) + "][" + String((int)(round(trackAngle))) + "][" + String(obstColor) + "][" + String((int)(round(obstCoordX/10))) + "," + String((int)(round(obstCoordY/10))) + "]/";
  Serial1.write(formattedSend.c_str());
//  Serial.print("Sent to control: ");
//  Serial.println(formattedSend.c_str());
}



// This subroutine processes all of the analogue samples, creating the required values for the main loop

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensorValue0 = analogRead(A0); //sample Vb
  sensorValue2 = analogRead(A2); //sample Vref
  sensorValue3 = analogRead(A3); //sample Vpd
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  if (Boost_mode == 1){
    iL = -current_mA/1000.0;
    dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    iL = current_mA/1000.0;
    dutyref = sensorValue2 * (1.0 / 1023.0);
  }
  
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}

// This is a PID controller for the voltage

float pidv( float pid_input){
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;
 
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv*(e0v-e1v) + kiv*Ts*e_integration + kdv/Ts*(e0v-2*e1v+e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  u0v = saturation(u0v,uv_max,uv_min);
  
  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}

// This is a PID controller for the current

float pidi(float pid_input){
  float e_integration;
  e0i = pid_input;
  e_integration=e0i;
  
  //anti-windup
  if(u1i >= ui_max){
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }
  
  delta_ui = kpi*(e0i-e1i) + kii*Ts*e_integration + kdi/Ts*(e0i-2*e1i+e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  u0i = saturation(u0i,ui_max,ui_min);
  
  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}


void setUpSMPS(){
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt
    
    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    if (Boost_mode){
      if (CL_mode) { //Closed Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
      }else{ // Open Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
      }
    }else{      
      if (CL_mode) { // Closed Loop Buck
          // The closed loop path has a voltage controller cascaded with a current controller. The voltage controller
          // creates a current demand based upon the voltage error. This demand is saturated to give current limiting.
          // The current loop then gives a duty cycle demand based upon the error between demanded current and measured
          // current
          current_limit = 3; // Buck has a higher current limit
          ev = vref - vb;  //voltage error at this time
          cv=pidv(ev);  //voltage pid
          cv=saturation(cv, current_limit, 0); //current demand saturation
          ei=cv-iL; //current error
          closed_loop=pidi(ei);  //current pid
          closed_loop=saturation(closed_loop,0.99,0.01);  //duty_cycle saturation
          pwm_modulate(closed_loop); //pwm modulation
      }else{ // Open Loop Buck
          current_limit = 3; // Buck has a higher current limit
          oc = iL-current_limit; // Calculate the difference between current measurement and current limit
          if ( oc > 0) {
            open_loop=open_loop-0.001; // We are above the current limit so less duty cycle
          } else {
            open_loop=open_loop+0.001; // We are below the current limit so more duty cycle
          }
          open_loop=saturation(open_loop,dutyref,0.02); // saturate the duty cycle at the reference or a min of 0.01
          pwm_modulate(open_loop); // and send it out
      }
    }
    // closed loop control path

    digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
    triggerOuterLoop++;
  }
}


//*************Functions for the optical sensor**************
void mousecam_reset(){
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}


int mousecam_init(){
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  mousecam_reset();
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  return 0;
}

void mousecam_write_reg(int reg, int val){
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg){
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(1);
  return ret;
}

void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);
  
  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if(started==0){
      if(pix&0x40)started = 1;
      else {
        timeout++;
        if(timeout==100){
          ret = -1;
          break;
        }
      }
    }
    if(started==1){
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(14);
  return ret;
}




void setupOptSen(){
  #if 0
  
    /*
    if(movementflag){
    tdistance = tdistance + convTwosComp(xydat[0]);
    Serial.println("Distance = " + String(tdistance));
    movementflag=0;
    delay(3);
    }
    */
    // if enabled this section grabs frames and outputs them as ascii art
    
    if(mousecam_frame_capture(frame)==0)
    {
    int i,j,k;
    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++) 
    {
    for(j=0; j<ADNS3080_PIXELS_X; j++, k++) 
    {
    Serial.print(asciiart(frame[k]));
    Serial.print(' ');
    }
    Serial.println();
    }
    }
    Serial.println();
    delay(250);
  
  
  #else
  
    // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
    // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.
    int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
    MD md;
    mousecam_read_motion(&md);
    for(int i=0; i<md.squal/4; i++)
//    Serial.print('*');
//    Serial.print(' ');
//    Serial.print((val*100)/351); //value that shows the image quality pixels/smth
//    Serial.print(' ');
//    Serial.print(md.shutter); Serial.print(" (");
//    Serial.print((int)md.dx); Serial.print(',');
//    Serial.print((int)md.dy); Serial.println(')');
    
    // Serial.println(md.max_pix);
    
    distance_x = md.dx; //convTwosComp(md.dx);
    distance_y = md.dy; //convTwosComp(md.dy);
    
    total_x1 = (total_x1 + distance_x);
    total_y1 = (total_y1 + distance_y);
    
    total_x = 10*total_x1/205; //Conversion from counts per inch to mm (400 counts per inch)
    total_y = 10*total_y1/205; //Conversion from counts per inch to mm (400 counts per inch)
    
//    Serial.print('\n');
//    Serial.println("Distance_x = " + String(total_x));
//    
//    Serial.println("Distance_y = " + String(total_y));
//    Serial.print('\n');
    delay(10);
  
  #endif
}
