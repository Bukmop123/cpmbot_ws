  /*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

// 
//#define USE_USBCON      //this has to be active for Leonardo, for MEGA this line has to be off of on with the othere Arduino libraries above(for compiling)
#include <ACE128.h>  // Absolute Contact Encoder
#include <ACE128map12345678.h> // mapping for pin order 

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h> 

#include <cpmbot/cpmbot_detail.h> 


//encoder ACE128 settings  begin
ACE128 myACE(10,11,12,13,9,8,7,6, (uint8_t*)encoderMap_12345678);


float var_double;

uint8_t pinPos = 0; // pin values from MCP23008 or PCF8574
uint8_t rawPos = 0;
uint8_t upos = 0;
uint8_t oldPos = 255;
int8_t pos;
int16_t mpos;
uint8_t seen = 0;
//encoder ACE128 settings end

//base wheel left variables
int bw_left_cmd;                 //command value
const int bw_left_en = 3;          //motor enable
const int bw_left_lpin = 4;        //motor pins
const int bw_left_rpin = 5;        //motor pins
double bw_left_posr;               //join absolute position in radient 
//base wheel right variables
int bw_right_cmd;                //command value
const int bw_right_en = 0;          //motor enable
const int bw_right_lpin = 1;         //motor pins
const int bw_right_rpin = 2;        //motor pins


//programm variables
const int noise_ = 1;
const int motorMax_ = 255;
const int motorMin_ = 40;
double norm_ = 255;


ros::NodeHandle n;
//cpmMotor cpmMotor(ros::NodeHandle n );

std_msgs::Float64 _comm_Arduino;

cpmbot::cpmbot_detail _hardware_command;
cpmbot::cpmbot_detail _hardware_feedback;

ros::Publisher hwFeedbackPublisher("/controller/hardware/subscriber/detail/feedback", &_hardware_feedback);

ros::Publisher comm_ArduinoPublisher_("communication/arduino/feedback", &_comm_Arduino);

int checkMinMax( int value){
  
  if (abs( value) > motorMax_){value = motorMax_;}
  if (abs( value) < motorMin_){value = motorMin_;}
  return abs(value);  
}

void setMotor(int val, int motLeft, int motRight ){

int val_abs;
  if (val > noise_)
  {
    val_abs = checkMinMax( val );
    analogWrite(motLeft, val_abs);
    analogWrite(motRight, 0);
  }
  else if (val < -noise_)
  {
    val_abs = checkMinMax( val );
    analogWrite(motLeft, 0);
    analogWrite(motRight, val_abs);
  } 
  else
  {
    analogWrite(motLeft, 0);
    analogWrite(motRight, 0);
  }
}

//Subscriber / Callback Functions
void comm_ArduinoCallback( const std_msgs::Float64& cmd_msg){ 
  _comm_Arduino.data = cmd_msg.data;
  _comm_Arduino.data = _comm_Arduino.data + 1;
    comm_ArduinoPublisher_.publish( &_comm_Arduino );
}

void hwDetailCommandCallback( const cpmbot::cpmbot_detail& cmd_msg){ 
  
  _hardware_command.body_left = cmd_msg.body_left;
  _hardware_command.body_right = cmd_msg.body_right;

  _hardware_command.arm_front_left = cmd_msg.arm_front_left;
  _hardware_command.arm_front_right = cmd_msg.arm_front_right;
  _hardware_command.arm_back_left = cmd_msg.arm_back_left;
  _hardware_command.arm_back_right = cmd_msg.arm_back_right;

  _hardware_command.mecanum_front_left = cmd_msg.mecanum_front_left;
  _hardware_command.mecanum_front_right = cmd_msg.mecanum_front_right;
  _hardware_command.mecanum_back_left = cmd_msg.mecanum_back_left;
  _hardware_command.mecanum_back_right = cmd_msg.mecanum_back_right;
  
  }
void hwBaseWheelRightCallback( const std_msgs::Float64& cmd_msg){ bw_right_cmd =  cmd_msg.data;}





//Process Functions
void process(){
  //base 
  setMotor(bw_left_cmd, bw_left_lpin, bw_left_rpin );
  setMotor(bw_right_cmd, bw_right_rpin, bw_right_lpin );
  
}
//Publisher Functions
void pubfunc(){
  //ros::Time current_time = ros::Time::now();
  //_hardware_feedback.header.stamp = current_time;
  _hardware_feedback.header.frame_id = "feedback robot detail";

// connection test!! all command values are publiched as feedback
  _hardware_feedback.body_left = _hardware_command.body_left;
  _hardware_feedback.body_right = _hardware_command.body_right;

  _hardware_feedback.arm_front_left = _hardware_command.arm_front_left;
  _hardware_feedback.arm_front_right = _hardware_command.arm_front_right;
  _hardware_feedback.arm_back_left = _hardware_command.arm_back_left;
  _hardware_feedback.arm_back_right = _hardware_command.arm_back_right;

  _hardware_feedback.mecanum_front_left = _hardware_command.mecanum_front_left;
  _hardware_feedback.mecanum_front_right = _hardware_command.mecanum_front_right;
  _hardware_feedback.mecanum_back_left = _hardware_command.mecanum_back_left;
  _hardware_feedback.mecanum_back_right = _hardware_command.mecanum_back_right;

  
  hwFeedbackPublisher.publish( &_hardware_feedback );

}


ros::Subscriber<cpmbot::cpmbot_detail> hwDetailCommandSubscriber("/controller/hardware/publisher/detail/command",  hwDetailCommandCallback );

//Communication test with computer
ros::Subscriber<std_msgs::Float64> comm_ArduinoSubscriber_("communication/arduino/command",  comm_ArduinoCallback );

void setup()
{
  //set pin configuration for INPUT and OUTPUT 
  //motor enable
  pinMode(bw_left_en, OUTPUT);
  pinMode(bw_right_en, OUTPUT);

  
  //motor right and left direction
  //base
  pinMode(bw_left_rpin, OUTPUT);
  pinMode(bw_left_lpin, OUTPUT);
  pinMode(bw_right_rpin, OUTPUT);
  pinMode(bw_right_lpin, OUTPUT);


  //Enable / Disable the motor driver
  //base wheel
  digitalWrite(bw_left_en, HIGH);
  digitalWrite(bw_right_en, HIGH);

  myACE.begin();    // this is required for each instance, initializes the pins
//myACE.reverse(true);  // uncomment this for counter-clockwise operation
  pinPos = myACE.acePins();          // get IO expander pins
  oldPos = pinPos;                 // remember where we are
  

  //initiate  Node
  n.initNode();
  n.advertise(hwFeedbackPublisher);
  n.advertise(comm_ArduinoPublisher_);
  
  n.subscribe(hwDetailCommandSubscriber);
  n.subscribe(comm_ArduinoSubscriber_);
  
var_double = 0;
}

void loop()
{
  //reset encoder if needed 
  if (false) {      // check set-zero button
    // myACE.setMpos(6723);  // set current position to 6723
    myACE.setMpos(0);    // set logical multiturn zero to current position
    // myACE.setZero();     // set logical zero to current position
    // myACE.reverse(true);  // set reverse
    oldPos = 255;                   // force display update
  }
  mpos = myACE.mpos();               // get multiturn encoder position - signed
  bw_left_posr = mpos *PI/128;

  
  process();

  ros::Time current_time = n.now();
  _hardware_feedback.header.stamp = current_time;
  pubfunc();
 
  n.spinOnce();
  //delay(1000);

var_double =var_double+1;
}
