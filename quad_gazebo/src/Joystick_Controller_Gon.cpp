//ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>

//MOCAP
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"

//C++
#include <cstdlib>
#include <iostream>
//#include <quad_gazebo/PID_2.h>

using namespace std;


//----------------------------------------------------//
//                    PID LIBRARY                     //
//----------------------------------------------------//

class PID
{
public:

  //---------------------VARIABLES--------------------//

  //Name
  string name;

  //PID Gains 
  double kp;
  double ki;
  double kd;

  //Saturation Gains
  double minOutput;
  double maxOutput;

  //Error
  double error;
  double prev_error;

  //Components
  double proportional;
  double integral;
  double derivative;

  double integral_update;
  double output;

  //Time
  double time;
  double prev_time;
  double dt;

//---------------------FUNCTIONS--------------------// 

  PID(){};

//CONSTRUCTOR
  PID(double kp, double ki, double kd, double minOutput, double maxOutput, std::string name)
  {
    this->name = name;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->minOutput = minOutput;
    this->maxOutput = maxOutput;

    this->error = this->prev_error = 0.0;
    this->proportional = this->integral = this->derivative = 0.0;

    this->integral_update = 0.0;

    ros::Time::init();

    this->prev_time = ros::Time::now().toSec();
    this->dt = 0.0;

    //this->pid_pub=n.advertise<std_msgs::Float64>("/" + this->name, 1);
  }

  //GET OUTPUT
  double getOutput()
  {
    return this->output;
  }

  //RESET
  void Reset()
  {
    //this->error =
    this->integral_update = 0.0; 
    this->prev_error = 0.0;
    this->prev_time = ros::Time::now().toSec();
    //this->output = 0.0;
  }

//UPDATE PID ACTION
  double action(double goal, double pos)
  {
    //we save the actual time, and calculate the dt
    this->time = ros::Time::now().toSec();
    this->dt = this->time - this->prev_time;

    //Calculate the error
    this->error = goal-pos;

    //Calculate the integral change
    this->integral_update += this->error*this->dt;

    //Calculate the three actions  
    this->proportional = this->kp*this->error;
    this->integral = this->ki*this->integral_update;
    this->derivative = this->kd*((this->error-this->prev_error)/this->dt);

    //Now we save the actual values as old values to use it in the next iteration
    this->prev_time = this->time;
    this->prev_error = this->error;

    this->output = this->proportional + this->derivative + this->integral;
    //ROS_INFO_STREAM(this->name <<": " << proportional <<" + " << integral <<" + " << derivative);
    //ROS_INFO_STREAM("PID OUTPUT " << this->name <<" = "<< this->output);
    //this->pid_output.data = this->output;    
    //this->pid_pub.publish(this->pid_output);

    return max(min(this->output, this->maxOutput),this->minOutput);
  }

};




class Joystick{
 
public:
  Joystick();
  ~Joystick();


private:
  //=================================//
  //            VARIABLES            //
  //=================================//


  //*********************************//
  //        Common Variables         //
  //*********************************//

  //++Joystick State Variables  
      int btton[12];
      int btton_state[12];

  //++Quadcopter State Variables
      string flight_mode;
      double thrust; //Only for Take off step


  //*********************************//
  //   Control Position Variables    //
  //*********************************//

  //++Target Values
      double targetX;
      double targetY;
      double targetZ;
      double targetAngle;

  //++PID Objects
      PID *pid_x;
      PID *pid_y;
      PID *pid_z;
      PID *pid_yaw;


  //*********************************//
  //         MoCap Variables         //
  //*********************************//

  //++Quadcopter String
      string quad_name;


  //++TF transform variables
      tf::TransformListener listener;
      tf::StampedTransform transform;

      //Variables to save last possition/orientation
      tf::Vector3 mocap_position;
      tf::Quaternion mocap_orientation;
      double mocap_euler[3];

      //Variables to compute angular velocity
      tf::Quaternion mocap_angular_rate;
      tf::Quaternion mocap_last_quaternion;
      ros::Time last_quaternion_time;
      bool first_quaternion;

      tf::Quaternion average_quat;
      //tf::Quaternion *average_matrix[];
      bool first_average_vel;

  //*********************************//
  //          ROS Variables          //
  //*********************************//

  //++ROS messages
      geometry_msgs::Twist twist;
      geometry_msgs::Twist lastTwist;
      sensor_msgs::Joy lastJoy;

  //++ROS publishers/subscriber
      //ABOUT:
      //Publish the commands directly to the crazyflie
      //without knowing the origin of the commands
      ros::Publisher vel_pub;

      //ABOUT:
      //IN MANUAL,Take the commands from the logitech joystick
      ros::Subscriber vel_sub;
  
      //ABOUT:
      //IN HOLD POSITION, ...  
      ros::Subscriber joy_sub;

      //ABOUT:
      //Publish angular rate from the mocap
      ros::Publisher ang_vel_pub;    

  //++ROS starting node
      ros::NodeHandle node;
    
  
 
  //=================================//
  //             METHODS             //
  //=================================//


  //For manual update
  void cmdJoyChanged(geometry_msgs::Twist msg);

  //For change the flight_mode
  void changeMode(sensor_msgs::Joy joy);

  //Reset pid's
  void resetPID();

  //Get Position and Orientation from the MoCap
  void getPosQuatMoCap();  

  //Infinte loop
  void run();

  //Average filter
  //bool getAverage(tf::Quaternion quat);
};

Joystick::~Joystick()
{
  //Delete allocated memory by PID-objects
  delete pid_x;
  delete pid_y;
  delete pid_z;
  delete pid_yaw;
}

//Constructor, initializate the joystick class
Joystick::Joystick()
{
  //Initializate Variables in the same order as the definiton of the class

  //++Quadcopter State Variables   
  this->flight_mode = "Manual";
  ROS_INFO_STREAM("Flight Mode: Manual");
  thrust = 0.0; 

  //++Target Values
  targetX = 0.0;
  targetY = 0.0;
  targetZ = 1.0;
  targetAngle = 0.0;

  //++PID-Position objects
  pid_x  =  new PID(    20.0,    0.0,     12.0,    -30.0,    30.0,   "pidX");
  pid_y  =  new PID(   -20.0,    0.0,    -12.0,    -30.0,    30.0,   "pidY");
  pid_z  =  new PID( 15000.0, 1500.0,   3000.0,  10000.0, 60000.0,   "pidZ");
  pid_yaw = new PID(    50.0,    0.0,      0.0,   -100.0,   100.0, "pidYAW");

  //++Quadcopter String
  this->quad_name = "/Cf_Uni";

  //++TF transform variables
  first_quaternion = false;
  last_quaternion_time = ros::Time::now();
  first_average_vel = false;

  //average_matrix = new tf::Quaternion[5];

  //++ROS publisher/subscriber
  //Listen always the axes of the joystick 
  vel_sub = node.subscribe("/cmd_vel_telop",1,&Joystick::cmdJoyChanged,this);

  //Listen always the buttons of the joystick
  joy_sub = node.subscribe("/joy",1,&Joystick::changeMode,this);

  //Publish the commands (roll, pitch, yaw,thrust)
  vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  
  //Publish angular velocity estimated by the Mocap
  ang_vel_pub = node.advertise<geometry_msgs::Quaternion>("/angular_vel",1);
  

  //Call infinite loop
  this->run();

}


//Publish the new commands in Manual Mode
void Joystick::cmdJoyChanged(geometry_msgs::Twist  msg)
{
  //Only publish the message of the joystick if the Manual Mode is On.
  if (this->flight_mode == "Manual")
  {
    this->lastTwist = msg;
    vel_pub.publish(msg);
    //ROS_INFO_STREAM("Publicando Joystick");
  }
}


//Reset PID's
void Joystick::resetPID()
{
    pid_x->Reset();
    pid_y->Reset();
    pid_z->Reset();
    pid_yaw->Reset();
}

//Detect if the buttons are pressed
void Joystick::changeMode(sensor_msgs::Joy joy)
{
  //BUTTONS
  if ((joy.buttons).size() == (lastJoy.buttons).size())
  {
    //Check if some buttons have been pressed
    for(int i=0; i<12; i++)
    {
       btton[i] = joy.buttons[i] - lastJoy.buttons[i];
    }
    
    //Check button signals and messages    
    
    //======== Button 1 ==========//

    //Change to hold position if this mode isn't selected
    if (btton[4]==1 & this->flight_mode != "Hold_Position")
    {
      this->flight_mode = "Hold_Position";
      this->resetPID();
      ROS_INFO_STREAM("Hold Position");
    }
      
    //======== Button 2 ==========//
    
    //Change to Manual if this mode isn't selected
  if (btton[1] == 1 & this->flight_mode != "Manual")
    {
      this->flight_mode = "Manual";
      ROS_INFO_STREAM("Manual");
    }

    //======== Button 3 ==========//
    
    //Change to Take Off if the mode is not selected
    if (btton[2] == 1 & this->flight_mode != "Take_Off")
    {
      this->flight_mode = "Take_Off";
      ROS_INFO_STREAM("Take Off!");
    }
    //======== Button 4 ==========//
    /*
    if (btton[3] == 1 & this->flight_mode != "Landing")
    {
      this->flight_mode = "Landing";
      ROS_INFO_STREAM("Landing");
    }
    */
    //======== Button 5 ==========//
    
    //======== Button 6 ==========//
    

  }
  //Save the actual position of the joy for the next joyChange
  this->lastJoy = joy;
}

//Get the position and the orientation of the quadcopter,
//and save it in the private-variables availables in
//the joystick class.
void Joystick::getPosQuatMoCap()
{
  try
  {
    //local variables for the function
    ros::Time t;
    std::string err;

    //Search for the latest common time between frames
    listener.getLatestCommonTime("/mocap",quad_name,t,&err);

    //Look if it's possible to transform the two frames at time t
    //ROS_INFO_STREAM("Looking for availability of Transform in time: " << t);
    if (listener.canTransform(quad_name,"/mocap", t,NULL))
    {
      
      //ROS_INFO_STREAM("Transform available");
      //get the transform between the two frames
      listener.lookupTransform("/mocap",quad_name, t, transform);

      //and save the position and orientation(quaternion and euler angles) 
      mocap_position=transform.getOrigin();
      mocap_orientation=transform.getRotation();
      tf::Matrix3x3(mocap_orientation).getRPY(mocap_euler[0],mocap_euler[1],mocap_euler[2]);

      /*
      //DEBUG INFO--->Display Position and Orientation        
      ROS_INFO_STREAM("POS:" << mocap_position[0] <<" : "<<mocap_position[1] <<" : " <<mocap_position[2]);
      ROS_INFO_STREAM("YAW:" << mocap_euler[2]);
      ROS_INFO_STREAM("QUAT:" << mocap_orientation[0] <<" : "<<mocap_orientation[1] <<" : " <<mocap_orientation[2] <<" : " << mocap_orientation[3]);
      */
     
      //Compute angular rate
      //
      //  w(t) = 2*(dq/dt)*inv(q)
      //
      //if it's the first quaternion, wait until we have two 
      if (!first_quaternion)
      {
        mocap_last_quaternion = mocap_orientation;
        last_quaternion_time = t;
	first_quaternion = true;
      }
      //if not, compute the angular rate
      else
      {
        double k = 2.0/(t.toSec()-last_quaternion_time.toSec());
        mocap_angular_rate = (mocap_orientation-mocap_last_quaternion)*(mocap_orientation.inverse()); 
        mocap_angular_rate *= k;

        //Save values for next iteration
        mocap_last_quaternion = mocap_orientation;
        last_quaternion_time = t;
        
        if (!first_average_vel)
        {
          average_quat = mocap_angular_rate;
          first_average_vel = true;
        }
        else
        {
          geometry_msgs::Quaternion ang_q;
          ang_q.x = (mocap_angular_rate[0] + average_quat[0])/2;
          ang_q.y = (mocap_angular_rate[1] + average_quat[1])/2;
          ang_q.z = (mocap_angular_rate[2] + average_quat[2])/2;
          ang_q.w = (mocap_angular_rate[3] + average_quat[3])/2;

          ang_vel_pub.publish(ang_q);

          average_quat = mocap_angular_rate;
        }
      }
    }
    else
    {
      ROS_INFO_STREAM("Transform not available at this moment");
    }
  }
  catch(tf::TransformException& ex){
    ROS_INFO("Error trying to lookupTransform: %s", ex.what());
  }
}

//get average 
//bool getAverage(tf::Quaternion quat)


//Infinite loop
void Joystick::run()
{
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    //--------------------------//
    //       MANUAL MODE        //
    //--------------------------//
    if (this->flight_mode == "Manual")
    {
      this->getPosQuatMoCap();

    }
    
    //--------------------------//
    //      TAKE OFF MODE       //
    //--------------------------//
    if (this->flight_mode == "Take_Off")
    {
      this->getPosQuatMoCap();
      
      //TAKE OFF ACTION
      if (thrust > 46000)
      {
        this->resetPID();
        pid_z->integral_update = thrust / pid_z->ki;
        this->flight_mode = "Hold_Position";
        thrust = 0;
      }
      else
      {
        geometry_msgs::Twist takeOff;
        thrust += 300;
        takeOff.linear.z = thrust;
        vel_pub.publish(takeOff);
      }
    }    
    
    //--------------------------//
    //    HOLD POSITION MODE    //
    //--------------------------//
    if (this->flight_mode == "Hold_Position")
    { 
      this->getPosQuatMoCap();
  
      //PID ACTION
      geometry_msgs::Twist pid_twist;
      double x_prim, y_prim;
      
      //STRUCTURE PID ACTION METHOD         
      //pid_name.action(double goal, double pos); 
               
      //Descompose X and Y contributions taking care of the Yaw angle
      x_prim = pid_x->action(targetX-mocap_position[0],0.0);
      y_prim = pid_y->action(targetY-mocap_position[1],0.0);
               
      pid_twist.linear.x = x_prim*cos(mocap_euler[2])-y_prim*sin(mocap_euler[2]);
      pid_twist.linear.y = x_prim*sin(mocap_euler[2])+y_prim*cos(mocap_euler[2]);
      pid_twist.linear.z = pid_z->action(targetZ-mocap_position[2],0.0);
      pid_twist.angular.z = pid_yaw->action(targetAngle*(M_PI/180) + mocap_euler[2],0.0);
               
      vel_pub.publish(pid_twist);

    }  

    loop_rate.sleep();
    ros::spinOnce();
  }
}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"logitech_teleop");
  Joystick joy;
  
  return 0;

}

