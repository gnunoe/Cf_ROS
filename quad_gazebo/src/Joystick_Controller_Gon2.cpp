//ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//MOCAP
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"

//C++
#include <cstdlib>
#include <iostream>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>

//Include libraries
#include <quad_gazebo/PID.h>
#include "quad_gazebo/trajectory.hpp"
#include "quad_gazebo/t_circle.hpp"
#include "quad_gazebo/t_sparse.hpp"
#include "quad_gazebo/t_ompl.hpp"

//SRV test
#include "quad_gazebo/trajectory.h"
#include "std_msgs/String.h"


using namespace std;

//Global variables to hold the path
//of the files which contain the
//trajectories to follow
std::string path_sparse;
std::string path_ompl;

//Path Tracking errors for debug
std::vector<double> cross_error_x;
std::vector<double> cross_error_y;
std::vector<double> cross_error_z;
std::vector<double> along_error;

std::vector<double> actual_pos_x;
std::vector<double> actual_pos_y;
std::vector<double> actual_pos_z;

std::vector<double> desired_pos_x;
std::vector<double> desired_pos_y;
std::vector<double> desired_pos_z;

std::vector<double> plot_time;

std::vector<double> acc_cross;
std::vector<double> acc_along;

class Joystick{
 
public:
  Joystick();
  ~Joystick();

  std::string path_file_sparse;
  std::string path_file_ompl;

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
      int phase;
      double v_bat;

  //++Quadcopter/World Parameters
      double gravity;
      double quad_mass;  
      double quad_Klift;
      double quad_w_hover;
      int thrust_hover;

  //*********************************//
  //        Control Variables        //
  //*********************************//

  //++Target Values
      double targetX;
      double targetY;
      double targetZ;
      double targetAngle;

      double targetVX;
      double targetVY;
      double targetVZ;

      double targetAX;
      double targetAY;
      double targetAZ;    

  //++Desired Euler Angles
      double roll_des;
      double pitch_des;
      double yaw_des;
      double roll_rate_des;
      double pitch_rate_des;
      double yaw_rate_des;

  //++Desired Thrust Values
      //int thrust_hover;
      int delta_thrust;

  //++PID-Hover Objects
      //PID *pid_takeoff_x;
      //PID *pid_takeoff_y;
      //PID *pid_takeoff_z;

      PID *pid_hover_x;
      PID *pid_hover_y;
      PID *pid_hover_z;
    
      PID *pid_pathfoll_fake_x;
      PID *pid_pathfoll_fake_y;
      PID *pid_pathfoll_fake_z;

      PID *pid_pathfoll_x;
      PID *pid_pathfoll_y;
      PID *pid_pathfoll_z;

      PID *pid_pathfoll_vx;
      PID *pid_pathfoll_vy;
      PID *pid_pathfoll_vz;


      //PID *pid_rate_yaw;

  //++Timer variables
      int hover_time;
      int battery_time;
      int land_time;

  //++Trajectory Variables
      bool req_path;
      bool closed_path;
      bool start_path;
      int wait_path;
      int path_index;
      int freq_path;

      //Instance which will save the path 
      Trajectory * traj;

  //*********************************//
  //         MoCap Variables         //
  //*********************************//

  //++Quadcopter String
      string quad_name;
      bool quad_detected;

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
      //geometry_msgs::PoseStamped path_array[100];
      std::vector<geometry_msgs::PoseStamped> path_array;
      nav_msgs::Path quad_path;

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
      //ros::Publisher ang_vel_pub;    

      //ABOUT:
      //Publish trajectory checkpoints
      ros::Publisher trajpoints_pub; 

      //ABOUT:
      //Publish quadrotor path
      ros::Publisher path_pub;

      //ABOUT:
      //Publish Position-Target Values
      ros::Publisher tarX_pub;
      ros::Publisher tarY_pub;
      ros::Publisher tarZ_pub;      

      //ABOUT:
      //Publish Contributions Hover-PID
      //ros::Publisher h_x_prop_pub;
      //ros::Publisher h_y_prop_pub;
      //ros::Publisher h_z_prop_pub;
      //ros::Publisher h_x_integ_pub;
      //ros::Publisher h_y_integ_pub;
      //ros::Publisher h_z_integ_pub;
      //ros::Publisher h_x_der_pub;
      //ros::Publisher h_y_der_pub;
      //ros::Publisher h_z_der_pub;

      ros::Publisher thrust_pub;

      ros::ServiceServer traj_server;

      //std_msgs::float64 v_battery;
      ros::Subscriber v_bat_sub; 

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

  //Control loops
  void hoverLoopControl();
  void pathfollowLoopControl();

  //Service for trajectories
  bool getTrajectoryRequest(quad_gazebo::trajectory::Request &req,
                    	    quad_gazebo::trajectory::Response &res);

  //Read battery level
  void readBattery(std_msgs::Float32 v_battery);
  //void attitudeLoopControl();

};

Joystick::~Joystick()
{
  //Delete allocated memory by PID-objects
  //delete pid_takeoff_x;
  //delete pid_takeoff_y;
  //delete pid_takeoff_z;

  delete pid_hover_x;
  delete pid_hover_y;
  delete pid_hover_z;

  delete pid_pathfoll_fake_x;
  delete pid_pathfoll_fake_y;
  delete pid_pathfoll_fake_z;

  delete pid_pathfoll_x;
  delete pid_pathfoll_y;
  delete pid_pathfoll_z;

  delete pid_pathfoll_vx;
  delete pid_pathfoll_vy;
  delete pid_pathfoll_vz;

}


//Constructor, initializate the joystick class
Joystick::Joystick()
{
  //Initializate Variables in the same order as the definiton of the class

  //++Quadcopter State Variables   
  this->flight_mode = "Manual";
  ROS_INFO_STREAM("Flight Mode: Manual");
  thrust = 0.0; 
  phase = 0;
  v_bat = 0.0;
  path_file_sparse = path_sparse;
  path_file_ompl = path_ompl;

  //++Quadcopter Parameters
  gravity = 9.81;
  quad_mass = 0.021;
  quad_Klift = 0.00000000019407;
  quad_w_hover = sqrt((quad_mass*gravity)/(4*quad_Klift));
 
  quad_name = "/Cf_Uni3";
  thrust_hover = 43500;
  //quad_name = "/Cf_Gon";
  //thrust_hover = 47500; 

  //++Target Values
  targetX = 0.0;
  targetY = 0.0;
  targetZ = 1.0;
  targetAngle = 0.0;

  targetVX = 0.0;
  targetVY = 0.0;
  targetVZ = 0.0;

  targetAX = 0.0;
  targetAY = 0.0;
  targetAZ = 0.0;
  //++Desired Euler Angles
  roll_des = 0.0;
  pitch_des = 0.0;
  yaw_des = 0.0;

  //++Desired Thrust Values
  delta_thrust = 0;
  

  //++PID-Hover objects
  // 1.5 3.2 
/*
  pid_takeoff_x  =  new PID(     3.8,     0.0,      0.8,      -7.0,     7.0,   "pid_takeoff_X");
  pid_takeoff_y  =  new PID(    -3.8,    -0.0,     -0.8,      -7.0,     7.0,   "pid_takeoff_Y");
  pid_takeoff_z  =  new PID(     3.0,     0.1,      2.5,     -10.0,    10.0,   "pid_takeoff_Z");

  pid_hover_x  =  new PID(     5.2,     0.05,      1.1,      -7.0,     7.0,   "pid_hover_X");
  pid_hover_y  =  new PID(    -5.0,    -0.0,     -1.5,      -7.0,     7.0,   "pid_hover_Y");
  pid_hover_z  =  new PID(     9.0,     1.5,      3.2,     -6.0,    15.0,   "pid_hover_Z");
*/

  pid_hover_x  =  new PID(     3.8,     0.0,      0.8,      -7.0,     7.0,   "pid_hover_X");
  pid_hover_y  =  new PID(    -3.8,    -0.0,     -0.8,      -7.0,     7.0,   "pid_hover_Y");
  pid_hover_z  =  new PID(     9.0,     1.5,      2.5,     -15.0,    15.0,   "pid_hover_Z");


/*
  pid_hover_x  =  new PID(     5.2,     0.0,      1.5,      -7.0,     7.0,   "pid_hover_X");
  pid_hover_y  =  new PID(    -5.0,     0.0,     -1.5,      -7.0,     7.0,   "pid_hover_Y");
  pid_hover_z  =  new PID(     9.0,     2.5,      4.0,     -15.0,    15.0,   "pid_hover_Z");
*/

  pid_pathfoll_fake_x = new PID(     0.0,    0.0,      0.0,     -0.0,    0.0,   "pid_fake_x");
  pid_pathfoll_fake_y = new PID(     0.0,    0.0,      0.0,     -0.0,    0.0,   "pid_fake_y");
  pid_pathfoll_fake_z = new PID(     0.0,    0.0,      0.0,     -0.0,    0.0,   "pid_fake_z");

  // 9   -9    20
  pid_pathfoll_x= new PID(    6.0,     0.0,     0.0,      -50.0,     50.0,   "pid_pathfoll_X");
  pid_pathfoll_y= new PID(   -6.0,     0.0,     0.0,      -50.0,     50.0,   "pid_pathfoll_Y");
  pid_pathfoll_z= new PID(    20.0,     0.0,     0.0,      -6.0,     20.0,   "pid_pathfoll_Z");
  // minz -6 maxz +15

  pid_pathfoll_vx=new PID(    1.5,     0.0,     0.0,      -50.0,     50.0,   "pid_pathfoll_VX");
  pid_pathfoll_vy=new PID(   -1.5,     0.0,     0.0,      -50.0,     50.0,   "pid_pathfoll_VY");
  pid_pathfoll_vz=new PID(    6.0,     0.0,     0.0,      -3.0,     9.0,   "pid_pathfoll_VZ");
  // minz -3 maxz 9

  //pid_rate_yaw =  new PID(    50.0,    0.0,      0.0,    -100.0,  100.0, "pidYAW");

  //++Timer variables
  hover_time = 0;
  battery_time = 0;
  land_time = 0;

  //++Trajectory Variables
  req_path = false;
  closed_path = false;
  start_path = false;
  path_index = 0;
  wait_path = 0;
  freq_path = 0;

  //circle = new Circle_path();
  //traj = new t_circle();
  //circle->Generate_points();

  //++Quadcopter String
  //quad_name = "/Cf_Uni";
  //quad_name = "/Cf_Gon";
  quad_detected = false;

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
  //ang_vel_pub = node.advertise<geometry_msgs::Quaternion>("/angular_vel",1);
  
  //Publish Position-Target Values
  tarX_pub = node.advertise<std_msgs::Float64>("/target_X",1);
  tarY_pub = node.advertise<std_msgs::Float64>("/target_Y",1);
  tarZ_pub = node.advertise<std_msgs::Float64>("/target_Z",1);

  //Publish trajectory points
  //trajpoints_pub = node.advertise<visualization_msgs::Marker>("/trajectory_points",1);
  trajpoints_pub = node.advertise<visualization_msgs::MarkerArray>("/trajectory",10);


  //Publish path
  path_pub = node.advertise<nav_msgs::Path>("/Cf_Gon_Path",1);


  //h_x_prop_pub = node.advertise<std_msgs::Float64>("/pid_hover/x/P",1);
  //h_y_prop_pub = node.advertise<std_msgs::Float64>("/pid_hover/y/P",1);
  //h_z_prop_pub = node.advertise<std_msgs::Float64>("/pid_hover/z/P",1);
  //h_x_integ_pub = node.advertise<std_msgs::Float64>("/pid_hover/x/I",1);
  //h_y_integ_pub = node.advertise<std_msgs::Float64>("/pid_hover/y/I",1);
  //h_z_integ_pub = node.advertise<std_msgs::Float64>("/pid_hover/z/I",1);
  //h_x_der_pub = node.advertise<std_msgs::Float64>("/pid_hover/x/D",1);
  //h_y_der_pub = node.advertise<std_msgs::Float64>("/pid_hover/y/D",1);
  //h_z_der_pub = node.advertise<std_msgs::Float64>("/pid_hover/z/D",1);

  thrust_pub = node.advertise<std_msgs::Float64>("/thrust_hover",1);

  traj_server = node.advertiseService("trajectory_message", &Joystick::getTrajectoryRequest,this);

  v_bat_sub = node.subscribe("/battery",1,&Joystick::readBattery,this);


  //Call infinite loop
  this->run();

}


bool Joystick::getTrajectoryRequest(quad_gazebo::trajectory::Request &req,
                    		    quad_gazebo::trajectory::Response &res)
{
  ROS_INFO_STREAM("Processing request : "<< req.name);
  
  if (req.name == "circle")
  {
    ROS_INFO_STREAM("Creating circle trajectory object... \n");
    traj = new t_circle();
    ROS_INFO_STREAM("Trajectory planned, starting...\n");
    req_path = true;
    closed_path = true;
    //Flag to start with the taking off step
    //this->phase = 1;
    //this->flight_mode = "Take_Off";
    //targetX = mocap_position[0];
    //targetY = mocap_position[1];
    //targetZ = 1.0;
 
    //Answer the client
    res.done = "Done";
    return true;  
  }

  if (req.name == "rrt")
  {
    ROS_INFO_STREAM("Creating rrt trajectory object... \n");
    std::string file = path_file_sparse + "/trajectory_rrt.csv";
    traj = new t_sparse(file);
    //traj->show_points();
    ROS_INFO_STREAM("Trajectory planned, starting...\n");

    //Answer the client
    res.done = "Done";
    return true;
  }
  if (req.name == "rrt_search")
  {
    ROS_INFO_STREAM("Creating rrt_search trajectory object... \n");
    std::string file = path_file_sparse + "/trajectory_rrt_search.csv";
    traj = new t_sparse(file);
    //traj->show_points();
    ROS_INFO_STREAM("Trajectory planned, starting...\n");
    req_path = true;
    closed_path = false;

    //Answer the client
    res.done = "Done";
    return true;
  }

  if (req.name == "sst")
  {
    ROS_INFO_STREAM("Creating sst trajectory object... \n");
    std::string file = path_file_sparse + "/trajectory_sst.csv";
    traj = new t_sparse(file);
    //traj->show_points();
    ROS_INFO_STREAM("Trajectory planned, starting...\n");
 
    //Answer the client
    res.done = "Done";
    return true;
  }
  if (req.name == "sst_search")
  {
    ROS_INFO_STREAM("Creating sst_search trajectory object... \n");
    std::string file = path_file_sparse + "/trajectory_sst_search.csv";
    traj = new t_sparse(file);
    //traj->show_points();
    ROS_INFO_STREAM("Trajectory planned, starting...\n");
    req_path = true;
    closed_path = false;

    //Answer the client
    res.done = "Done";
    return true;
  }

  if (req.name == "ompl")
  {
    ROS_INFO_STREAM("Creating OMPL trajectory object... \n");
    std::string file = path_file_ompl + "/trajectory_ompl.csv";
    traj = new t_ompl(file);
    traj->show_points();
    ROS_INFO_STREAM("Trajectory planned, starting...\n");
    req_path = true;
    closed_path = false;

    //Answer the client
    res.done = "Done";
    return true;
  }
 
  //The trajectory is not saved in the library
  ROS_ERROR_STREAM("Impossible to make requested trajectory.\n");
  res.done = "Error";
  return false;
    
}


void Joystick::readBattery(std_msgs::Float32 v_battery)
{

  //Each 5 seconds, check the battery level
  if (battery_time >= 50)
  {
    //If we are following a path, keep the thrust until we
    //finish the tracking
    if(req_path == false)
    {
      //ROS_INFO_STREAM("batt" << v_bat);
      v_bat = v_battery.data;
    
      //Compute new hover thrust in discrete space
      if (quad_name == "/Cf_Gon")
      {
	if (v_bat <4.0 && v_bat >=3.75 && thrust_hover !=47500)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust: 47500");
          thrust_hover = 47500;
          this->resetPID();
        }
        else if (v_bat<3.75 && v_bat >= 3.5 && thrust_hover !=50000)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust:: 50000");
          thrust_hover = 50000;
          this->resetPID();
        }
        else if (v_bat<3.5 && thrust_hover !=52500)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust: 52500");
          thrust_hover = 52500;
          this->resetPID();
        }
      }
      else
      {
        if (v_bat <4.0 && v_bat >=3.75 && thrust_hover !=43500)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust: 43500");
          thrust_hover = 43500;
          this->resetPID();
        }
        else if (v_bat<3.75 && v_bat >= 3.5 && thrust_hover !=45000)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust:: 45000");
          thrust_hover = 45000;
          this->resetPID();
        }
        else if (v_bat<3.5 && v_bat >= 3.25 && thrust_hover !=47500)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust: 47500");
          thrust_hover = 47500;
          this->resetPID();
        }
        else if (v_bat<3.25 && thrust_hover !=50000)
        {
          ROS_INFO_STREAM("Battery level: " << v_bat <<", New thrust: 50000");
          thrust_hover = 50000;
          this->resetPID();
        }
      }
    }
    battery_time = 0;
  }
  else
  {
    battery_time ++;
    //std::cout << battery_time << "\n";
    //ROS_INFO_STREAM(battery_time);
  }

  //Compute new hover_thrust
  //thrust_hover = -21140.41*(v_bat*v_bat) + 135251.04*(v_bat) - 166535.79;
}

//Publish the new commands in Manual Mode
void Joystick::cmdJoyChanged(geometry_msgs::Twist  msg)
{
  //Only publish the message of the joystick if the Manual Mode is On.
  if (this->flight_mode == "Manual")
  {
    this->lastTwist = msg;
    ROS_INFO_STREAM("Pitch  " << msg.linear.x);
    ROS_INFO_STREAM("Roll  " << msg.linear.y);
    vel_pub.publish(msg);
    //ROS_INFO_STREAM("Publicando Joystick");
  }
}


//Reset PID's
void Joystick::resetPID()
{
    pid_hover_x->Reset();
    pid_hover_y->Reset();
    pid_hover_z->Reset();
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
    if (btton[0] == 1 & this->flight_mode != "Path_Follow")
    {
    /*
      this->flight_mode = "Path_Follow";
      ROS_INFO_STREAM("Path_Follow");
      if (start_path == false)
      {
        targetX = traj->path_points[0][0];
        targetY = traj->path_points[0][1];
        targetZ = traj->path_points[0][2];

        cout << targetX << targetY << targetZ;
      }
    */
    }
  
    //======== Button 2 ==========//
    
    //Change to Manual if this mode isn't selected
    if (btton[1] == 1 & this->flight_mode != "Manual")
    {
      this->flight_mode = "Manual";
      ROS_INFO_STREAM("Manual");
      phase = 0;
    }

    //======== Button 3 ==========//
    
    //Change to Take Off if the mode is not selected
    if (btton[2] == 1 & this->flight_mode != "Take_Off")
    {
     
      //this->flight_mode = "Take_Off";
      //ROS_INFO_STREAM("Take Off!");
   
      //targetX = -0.1;
      //targetY = -0.1;
      //targetZ = 1.0;
      //pid_hover_z->setMax(10.0);
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

    //Change to hold position if this mode isn't selected
    //if (btton[4]==1 & this->flight_mode != "Hold_Position")
    if (btton[4]==1)
    {
      for (int i=0; i<acc_cross.size(); i++)
      {
        std::cout << acc_cross[i] <<" "<<acc_along[i]<<", ";
      }


    /*
      this->flight_mode = "Hold_Position";
      this->resetPID();
      ROS_INFO_STREAM("Hold Position");
    */
    }

    //======== Button 6 ==========//

    //Only take off and hovering    
    if (btton[5]==1)
    {
      this->flight_mode = "Only_takeoff_hover";
      pid_hover_z->setMax(10.0);
      targetX = mocap_position[0];
      targetY = mocap_position[1];
      targetZ = 1.0;
    } 

    //======== Button 7 ==========//

    //Write file    

    if (btton[6]==1)
    {
      double avg_cerr_x = 0;
      double avg_cerr_y = 0;
      double avg_cerr_z = 0;
      double avg_aerr = 0;
      for (int i=0; i<cross_error_x.size(); i++)
      {
        avg_cerr_x += abs(cross_error_x[i]);
	avg_cerr_y += abs(cross_error_y[i]);
	avg_cerr_z += abs(cross_error_z[i]);
      }
      avg_cerr_x = avg_cerr_x/cross_error_x.size();
      avg_cerr_y = avg_cerr_y/cross_error_x.size();
      avg_cerr_z = avg_cerr_z/cross_error_x.size();
      for (int i=0; i<along_error.size(); i++)
      {
        avg_aerr += abs(along_error[i]);
      }
      avg_aerr += avg_aerr/along_error.size();
      std::cout << "Cross X: "<< avg_cerr_x << "\n";
      std::cout << "Cross Y: "<< avg_cerr_y << "\n";
      std::cout << "Cross Z: "<< avg_cerr_z << "\n";
      std::cout << "Along: " << avg_aerr << "\n";

      //Write File
      std::string dir;
      dir = path_sparse + "/tracking_error.csv";
      std::ofstream myFile;
      myFile.open(dir.c_str());
      for(int i=0; i<cross_error_x.size();i++)
      {
        myFile << plot_time[i] <<","
	       << cross_error_x[i] <<","
               << cross_error_y[i] <<","
               << cross_error_z[i] <<","
	       << actual_pos_x[i] <<","
               << actual_pos_y[i] <<","
               << actual_pos_z[i] <<","
               << desired_pos_x[i] <<","
               << desired_pos_y[i] <<","
               << desired_pos_z[i] <<"\n";
      }
      myFile.close();
    }

    //======== Button 8 ==========//

    //Only landing    

    if (btton[7]==1)
    {
      this->flight_mode = "Only_landing";
      ROS_INFO_STREAM("Only landing");
      thrust = thrust_hover;
      pid_hover_z->setMin(-10.0);
      //targetX = 0.0;
      //targetY = 0.0;
      //targetZ = 1.0;
    }
 

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
     
      quad_detected = true; 
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
      ROS_INFO_STREAM("EULER: " << mocap_euler[0]*(180/M_PI) <<" : "<<mocap_euler[1]*(180/M_PI) <<" : " << mocap_euler[2]*(180/M_PI));
      ROS_INFO_STREAM("QUAT:" << mocap_orientation[0] <<" : "<<mocap_orientation[1] <<" : " <<mocap_orientation[2] <<" : " << mocap_orientation[3]);
      */ 
      //ROS_INFO_STREAM("EULER: " << mocap_euler[0]*(180/M_PI) <<" : "<<mocap_euler[1]*(180/M_PI) <<" : " << mocap_euler[2]*(180/M_PI));

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


void Joystick::hoverLoopControl()
{    
  double rx_dd,ry_dd,rz_dd;
  double d2r = M_PI/180;

  //Acceleration in the three axis
  rx_dd = pid_hover_x->action(targetX-mocap_position[0],0.0);
  ry_dd = pid_hover_y->action(targetY-mocap_position[1],0.0);
  rz_dd = pid_hover_z->action(targetZ-mocap_position[2],0.0);

  /*
  std_msgs::Float64 P_x;
  std_msgs::Float64 P_y;
  std_msgs::Float64 P_z;
  std_msgs::Float64 I_x;
  std_msgs::Float64 I_y;
  std_msgs::Float64 I_z;  
  std_msgs::Float64 D_x;
  std_msgs::Float64 D_y;
  std_msgs::Float64 D_z;

  //P_x.data = pid_hover_x->proportional;
  //P_y.data = pid_hover_y->proportional;
  P_z.data = pid_hover_z->getP();
  //I_x.data = pid_hover_x->integral;
  //I_y.data = pid_hover_y->integral;
  I_z.data = pid_hover_z->getI();
  //D_x.data = pid_hover_x->derivative;
  //D_y.data = pid_hover_y->derivative;
  D_z.data = pid_hover_z->getD();

  //cout << P_z.data <<" "<< I_z.data <<" "<< D_z.data;

  //h_x_prop_pub.publish(P_x);
  //h_y_prop_pub.publish(P_y);
  h_z_prop_pub.publish(P_z);
  //h_x_integ_pub.publish(I_x);
  //h_y_integ_pub.publish(I_y);
  h_z_integ_pub.publish(I_z);
  //h_x_der_pub.publish(D_x);
  //h_y_der_pub.publish(D_y);
  h_z_der_pub.publish(D_z);
  */

  //Calculate Euler angles and thrust from the acceleration

  //Mellinger Approach
  //roll_des  = (rx_dd*sin(yaw_des*d2r) - ry_dd*cos(yaw_des*d2r))/gravity;
  //pitch_des = (rx_dd*cos(yaw_des*d2r) + ry_dd*sin(yaw_des*d2r))/gravity;

  //Mellinger Approach with real yaw angle
  //roll_des  = (rx_dd*sin(mocap_euler[2]) - ry_dd*cos(mocap_euler[2]))/gravity;
  //pitch_des = (rx_dd*cos(mocap_euler[2]) + ry_dd*sin(mocap_euler[2]))/gravity;

  //Whoenig Approach
  //Euler desired in rad
  //roll_des  = (rx_dd*cos(yaw_des*d2r) - ry_dd*sin(yaw_des*d2r))/gravity;
  //pitch_des = (rx_dd*sin(yaw_des*d2r) + ry_dd*cos(yaw_des*d2r))/gravity;
 
  roll_des  =     (rx_dd*cos(mocap_euler[2]) - ry_dd*sin(mocap_euler[2]))/gravity;
  pitch_des =     (rx_dd*sin(mocap_euler[2]) + ry_dd*cos(mocap_euler[2]))/gravity;

 
  delta_thrust = (quad_mass*rz_dd)/(4*quad_Klift*quad_w_hover);
  
 
  //DEBUG: Show acceleration, angle and delta_thrust
  /*
  ROS_INFO_STREAM("Linear_Acc: " << rx_dd << " , " << ry_dd << " , " << rz_dd);
  ROS_INFO_STREAM("Des_Euler: " << roll_des*(180/M_PI) << "," << pitch_des*(180/M_PI) << "," << yaw_des*(180/M_PI));
  ROS_INFO_STREAM("Thrust Desired: " << delta_thrust);
  ROS_INFO_STREAM("Act_Euler: " << mocap_euler[0]*(180/M_PI) <<" "<<mocap_euler[1]*(180/M_PI) <<" "<<mocap_euler[2]*(180/M_PI));
  */
}

void Joystick::pathfollowLoopControl()
{
  double rx_dd,ry_dd,rz_dd;
  //to get the error in position and the estimation of the velocity
  double fake_pid_x, fake_pid_y, fake_pid_z;
  double der_x, der_y, der_z;
  double err_x, err_y, err_z;
  //Real pid, one for position and the other for velocity
  double x_p, y_p, z_p;
  double x_d_p, y_d_p, z_d_p;
  double error_pos_normal;
  double error_pos_binormal;
  double error_pos [3] = { };
  double error_vel [3] = { };
  //variables to hold normal, binormial and tangent
  double tangent [3]= { };
  double normal [3] = { };
  double binormal [3] = { };
  
  double d2r = M_PI/180;

  //Get Error and estimate velocity using the PID structure
  fake_pid_x = pid_pathfoll_fake_x->action(targetX-mocap_position[0],0.0);
  fake_pid_y = pid_pathfoll_fake_y->action(targetY-mocap_position[1],0.0);
  fake_pid_z = pid_pathfoll_fake_z->action(targetZ-mocap_position[2],0.0);

  err_x = pid_pathfoll_fake_x->getError();
  err_y = pid_pathfoll_fake_y->getError();
  err_z = pid_pathfoll_fake_z->getError();

  der_x = (-1)*pid_pathfoll_fake_x->getDerivativeError();
  der_y = (-1)*pid_pathfoll_fake_y->getDerivativeError();
  der_z = (-1)*pid_pathfoll_fake_z->getDerivativeError();

  //Get tangent, normal and binormial from the trajectory
  tangent[0]  = traj->TNB[path_index-1][0];
  tangent[1]  = traj->TNB[path_index-1][1];
  tangent[2]  = traj->TNB[path_index-1][2];
  normal[0]   = traj->TNB[path_index-1][3];
  normal[1]   = traj->TNB[path_index-1][4];
  normal[2]   = traj->TNB[path_index-1][5];
  binormal[0] = traj->TNB[path_index-1][6];
  binormal[1] = traj->TNB[path_index-1][7];
  binormal[2] = traj->TNB[path_index-1][8];


  //Real pid for position
  error_pos_normal = err_x*normal[0] + err_y*normal[1] + err_z*normal[2];
  error_pos_binormal = err_x*binormal[0] + err_y*binormal[1] + err_z*binormal[2];

  error_pos[0] = error_pos_normal*normal[0] + error_pos_binormal*binormal[0];
  error_pos[1] = error_pos_normal*normal[1] + error_pos_binormal*binormal[1];
  error_pos[2] = error_pos_normal*normal[2] + error_pos_binormal*binormal[2];

  x_p = pid_pathfoll_x->action(error_pos[0],0.0);
  y_p = pid_pathfoll_y->action(error_pos[1],0.0);
  z_p = pid_pathfoll_z->action(error_pos[2],0.0);

  cross_error_x.push_back(error_pos[0]);
  cross_error_y.push_back(error_pos[1]);
  cross_error_z.push_back(error_pos[2]);
  
  //Real pid for velocity
  error_vel[0] = targetVX-der_x;
  error_vel[1] = targetVY-der_y;
  error_vel[2] = targetVZ-der_z;

  x_d_p = pid_pathfoll_vx->action(error_vel[0],0.0);
  y_d_p = pid_pathfoll_vy->action(error_vel[1],0.0);
  z_d_p = pid_pathfoll_vz->action(error_vel[2],0.0);

  along_error.push_back(error_vel[0]);
  along_error.push_back(error_vel[1]);
  along_error.push_back(error_vel[2]);

  acc_cross.push_back(x_p);
  acc_cross.push_back(y_p);
  acc_cross.push_back(z_p);
  acc_along.push_back(x_d_p);
  acc_along.push_back(y_d_p);
  acc_along.push_back(z_d_p);  

  actual_pos_x.push_back(mocap_position[0]);
  actual_pos_y.push_back(mocap_position[1]);
  actual_pos_z.push_back(mocap_position[2]);
  desired_pos_x.push_back(targetX);
  desired_pos_y.push_back(targetY);
  desired_pos_z.push_back(targetZ);
 
  ros::Time t = ros::Time();
  plot_time.push_back(t.toSec());

  //DEBUG
  //ROS_INFO_STREAM("acc: "<< x_p <<" "<< y_p <<" "<< z_p);
  //ROS_INFO_STREAM("acc_v: "<< x_d_p <<" "<< y_d_p <<" "<<z_d_p);

  //rx_dd = x_p + x_d_p + targetAX;
  //ry_dd = y_p + y_d_p + targetAY;
  //rz_dd = z_p + z_d_p + targetAZ;

  rx_dd = x_p + x_d_p;
  ry_dd = y_p + y_d_p;
  rz_dd = z_p + z_d_p;

  //roll_des  = (rx_dd*cos(yaw_des*d2r) - ry_dd*sin(yaw_des*d2r))/gravity;
  //pitch_des = (rx_dd*sin(yaw_des*d2r) + ry_dd*cos(yaw_des*d2r))/gravity;
  roll_des  =     (rx_dd*cos(mocap_euler[2]) - ry_dd*sin(mocap_euler[2]))/gravity;
  pitch_des =     (rx_dd*sin(mocap_euler[2]) + ry_dd*cos(mocap_euler[2]))/gravity;

  delta_thrust = (quad_mass*rz_dd)/(4*quad_Klift*quad_w_hover); 
  
}


//Infinite loop
void Joystick::run()
{
  //ros::Rate loop_rate(100);
	
  //double tic =ros::Time::now().toSec();

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
    /*
    if (this->flight_mode == "Take_Off" && this->phase == 1)
    {
      this->getPosQuatMoCap();
      
      //TAKE OFF ACTION
      //if (thrust > thrust_hover || mocap_position[2]>0.2)
      if ((targetZ - mocap_position[2])<0.05)
      {
        //this->resetPID();
        pid_hover_z->Reset();
        //pid_hover_z->integral_update = thrust / (pid_hover_z->ki*1000);
        this->phase = 2;
        this->flight_mode = "Hold_Position";
        thrust = 0;
        //targetX = 0.2;
        //targetY = 0.0;
        //targetZ = g;
        pid_hover_z->setMax(15.0);

      }
      else
      {
        //To control the take off step, we use the Hover Control, but
        //we increment the thrust in steps, to control avoid a quick step.
        this->hoverLoopControl(); 
        //this->attitudeLoopControl();
      
        geometry_msgs::Twist takeOff;
        thrust += 100;

        if (thrust > thrust_hover+1000) //45000
        {
	  thrust = thrust_hover+1000;
        }
        

	takeOff.linear.x =  roll_des*(180/M_PI);//0.0;//-10.0;//(0.0-mocap_euler[0])*(180/M_PI);//roll_des*(180/M_PI);
        //takeOff.linear.x =  roll_rate_des;
        //takeOff.linear.x =  (roll_des-mocap_euler[0])*(180/M_PI);
        takeOff.linear.y = pitch_des*(180/M_PI);//10.0;//(0.0-mocap_euler[1])*(180/M_PI);//pitch_des*(180/M_PI);
        takeOff.linear.z = thrust;
	takeOff.angular.z = 0.0;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

        vel_pub.publish(takeOff);
      }
    }    
   */

    //--------------------------//
    //         LANDING          //
    //--------------------------//
/*
    if (this->flight_mode == "Landing" && this->phase == 5)
    {
      this->getPosQuatMoCap();

      //TAKE OFF ACTION
      //if (thrust > thrust_hover || mocap_position[2]>0.2)
      if ((targetZ)<0.1)
      {
        this->resetPID();
        //pid_hover_z->integral_update = thrust / (pid_hover_z->ki*1000);
        this->phase = 0;
        this->flight_mode = "Manual";
        thrust = 0;

      }
      else
      {
        //To control the take off step, we use the Hover Control, but
        //we increment the thrust in steps, to control avoid a quick step.
        this->hoverLoopControl();
        //this->attitudeLoopControl();

        geometry_msgs::Twist landing;
        targetZ -= 0.0005;
        
        //thrust -= 1;

        //if (thrust < 40000)
        //{
         // thrust = 40000;
        //}
        

        landing.linear.x =  roll_des*(180/M_PI);//0.0;//-10.0;//(0.0-mocap_euler[0])*(180/M_PI);//roll_des*(180/M_PI);
        //takeOff.linear.x =  roll_rate_des;
        //takeOff.linear.x =  (roll_des-mocap_euler[0])*(180/M_PI);
        landing.linear.y = pitch_des*(180/M_PI);//10.0;//(0.0-mocap_euler[1])*(180/M_PI);//pitch_des*(180/M_PI);
        landing.linear.z = thrust;
        landing.angular.z = 0.0;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

        vel_pub.publish(landing);
      }
    }
*/

    ///////////////////////////////////////////////////
    //////////////////////////////////////////////////
    if (this->flight_mode == "Only_takeoff_hover")
    {
      //ROS_INFO_STREAM("Taking off");
      this->getPosQuatMoCap();

      //TAKE OFF ACTION
      //if (thrust > thrust_hover || mocap_position[2]>0.2)
      if ((targetZ - mocap_position[2])<0.05)
      {
        this->resetPID();
        //pid_hover_z->integral_update = thrust / (pid_hover_z->ki*1000);
        this->flight_mode = "Only_hover";
        thrust = 0;
        
        //pid_hover_z->setMax(15.0);

      }
      else
      {
        //To control the take off step, we use the Hover Control, but
        //we increment the thrust in steps, to control avoid a quick step.
        this->hoverLoopControl();
        //this->attitudeLoopControl();

        geometry_msgs::Twist takeOff;
        thrust += 100;

        if (thrust > thrust_hover+1000) //42500
        {
          thrust = thrust_hover+1000;
        }


        takeOff.linear.x =  roll_des*(180/M_PI);//0.0;//-10.0;//(0.0-mocap_euler[0])*(180/M_PI);//roll_des*(180/M_PI);
        //takeOff.linear.x =  roll_rate_des;
        //takeOff.linear.x =  (roll_des-mocap_euler[0])*(180/M_PI);
        takeOff.linear.y = pitch_des*(180/M_PI);//10.0;//(0.0-mocap_euler[1])*(180/M_PI);//pitch_des*(180/M_PI);
        takeOff.linear.z = thrust;
        takeOff.angular.z = 0.0;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

        vel_pub.publish(takeOff);
      }

    }

    if (this->flight_mode == "Only_hover")
    {
      //ROS_INFO_STREAM("Hovering");
      this->getPosQuatMoCap();
      //Only hover
      if (req_path == false)
      {	
        geometry_msgs::Twist pid_twist;
        this->hoverLoopControl();
        pid_twist.linear.x = roll_des*(180/M_PI);
        pid_twist.linear.y = pitch_des*(180/M_PI);
        pid_twist.linear.z = thrust_hover + delta_thrust/4;
        pid_twist.angular.z = yaw_des;
        vel_pub.publish(pid_twist);

	//Publish the total thrust sent to the copter
        std_msgs::Float64 t_z;
        t_z.data = pid_twist.linear.z/10000;
        thrust_pub.publish(t_z);
      }
      //We have received the order to perform a trajectory. Check
      //Our position regarding the beginning of the path.
      else
      {
	//Variables to hold the start of the path
        double startX = traj->path_points[0][0];
        double startY = traj->path_points[0][1];
        double startZ = traj->path_points[0][2]; 
	
	//We are in the beginning of the path. Keep the hovering during
	//a few seconds and change to the Path Following controller
	if (startX == targetX &&
	    startY == targetY &&
	    startZ == targetZ)
	{
	  //Publish the command
	  geometry_msgs::Twist pid_twist;
          this->hoverLoopControl();
          pid_twist.linear.x = roll_des*(180/M_PI);
          pid_twist.linear.y = pitch_des*(180/M_PI);
          pid_twist.linear.z = thrust_hover + delta_thrust/4;
          pid_twist.angular.z = yaw_des;
          vel_pub.publish(pid_twist);

	  //Wait 3 seconds hovering in the beginnig of the path
	  if (hover_time < 300)
	  {
	    //We increment the timer only if the copter is close to the
	    //start position
	    if (abs(mocap_position[0]-targetX)<0.1 &&
		abs(mocap_position[1]-targetY)<0.1 &&
		abs(mocap_position[2]-targetZ)<0.2)
	    {
	      hover_time ++;	
	    }

	    //Publish RVIZ markers for closed loop trajectories
	    if (closed_path == true && hover_time >100 && hover_time < 300)
	    {
            
              visualization_msgs::MarkerArray ma;
              visualization_msgs::Marker points;
              points.header.frame_id = "/mocap";
              points.header.stamp = ros::Time();
              points.ns = "trajectory_mocap";
              points.action = visualization_msgs::Marker::ADD;
              points.pose.orientation.w = 1.0;
              points.id = 0;
              points.type = visualization_msgs::Marker::POINTS;          
              points.scale.x = 0.05;
              points.scale.y = 0.05;
              points.color.g = 1.0f;
              points.color.a = 1.0;
          
              for (int i =0; i<traj->points-1; i++)
              {
                geometry_msgs::Point p;
                p.x = traj->path_points[i][0];
                p.y = traj->path_points[i][1];
                p.z = traj->path_points[i][2];
 
                points.points.push_back(p);
              }
              ma.markers.push_back(points); 
              trajpoints_pub.publish(ma);
	    }
	  }
	  //Give the signal to the Path Follow Controller
	  else
	  {
	    //req_path = false;
	    hover_time = 0;

	    //new values for the controller
	    cout << "Starting Path following\n";

            targetX = traj->path_points[1][0];
            targetY = traj->path_points[1][1];
            targetZ = traj->path_points[1][2];
          
            targetVX = traj->path_points[1][3];
            targetVY = traj->path_points[1][4];
            targetVZ = traj->path_points[1][5];

            targetAX = traj->path_points[1][6];
            targetAY = traj->path_points[1][7];
            targetAZ = traj->path_points[1][8];
            this->path_index = 1;

	    this->flight_mode = "Path_Follow";
	  }
	}
	//Select the new hover position
	else
	{
	  targetX = startX;
	  targetY = startY;
	  targetZ = startZ;

          geometry_msgs::Twist pid_twist;
          this->hoverLoopControl();
          pid_twist.linear.x = roll_des*(180/M_PI);
          pid_twist.linear.y = pitch_des*(180/M_PI);
          pid_twist.linear.z = thrust_hover + delta_thrust/4;
          pid_twist.angular.z = yaw_des;
          vel_pub.publish(pid_twist);
	}

      }

    }
   
    if (this->flight_mode =="Only_landing")
    {
      this->getPosQuatMoCap();

      //first time we entry, change the z command
      if (land_time==0)
      {
        targetZ -= 0.2;
	ROS_INFO_STREAM("Reducing landing height to: " << targetZ);
	land_time +=1;
	if (targetZ <=0.2)
	{
	  land_time +=150;
	}
      } 	

      //Increment land timer. being less strict when we know
      //we are close to the position
      if (abs(targetZ-mocap_position[2])<0.08)
      {
	land_time +=2;
      }
      else
      {
        land_time++;
      }
	

      if (land_time >300)
      {
	land_time = 0;
	if (targetZ <= 0.2)
	{
	  ROS_INFO_STREAM("Landing done");
          this->resetPID();
	  pid_hover_z->setMin(-15.0);
          this->flight_mode = "Manual";
          thrust = 0;
	  geometry_msgs::Twist landing;
	  landing.linear.z = 0.0;
          vel_pub.publish(landing);
	}
	else
        {
          this->hoverLoopControl();
          geometry_msgs::Twist landing;
          landing.linear.x =  roll_des*(180/M_PI);//0.0;//-10.0;//(0.0-mocap_euler[0])*(180/M_PI);//roll_des*(180/M_PI);
          landing.linear.y = pitch_des*(180/M_PI);//10.0;//(0.0-mocap_euler[1])*(180/M_PI);//pitch_des*(180/M_PI);
          landing.linear.z = thrust + delta_thrust/4;
          landing.angular.z = 0.0;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

          vel_pub.publish(landing);
        }
      }
      else
      {
	thrust -=2;
        this->hoverLoopControl();
        geometry_msgs::Twist landing;
        landing.linear.x =  roll_des*(180/M_PI);//0.0;//-10.0;//(0.0-mocap_euler[0])*(180/M_PI);//roll_des*(180/M_PI);
        landing.linear.y = pitch_des*(180/M_PI);//10.0;//(0.0-mocap_euler[1])*(180/M_PI);//pitch_des*(180/M_PI);
        landing.linear.z = thrust + delta_thrust/4;
        landing.angular.z = 0.0;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

        vel_pub.publish(landing);
      }
    }
 
              

    /////////////////////////////////////////////////////
    //////////////////////////////////////////////////// 



    //--------------------------//
    //    HOLD POSITION MODE    //
    //--------------------------//
/*
    if (this->flight_mode == "Hold_Position")
    { 
      this->getPosQuatMoCap();

      //If we are in phase 2 (Hovering after the path following)
      if (this->phase == 2)
      {      
        //we wait 10 seconds hovering
        if (hover_time < 1000)
        {
          //PID ACTION
          geometry_msgs::Twist pid_twist;
          //double x_prim, y_prim;
      
          //STRUCTURE PID ACTION METHOD         
          //pid_name.action(double goal, double pos); 


          this->hoverLoopControl();


          pid_twist.linear.x = roll_des*(180/M_PI);
          pid_twist.linear.y = pitch_des*(180/M_PI);
          pid_twist.linear.z = thrust_hover + delta_thrust/4;

          pid_twist.angular.z = yaw_des;// + mocap_euler[2]*(180/M_PI);//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

          vel_pub.publish(pid_twist);
          
          hover_time ++;
        }
        //and then start the path following step
        else
        {
          this->phase = 3;
          this->flight_mode = "Path_Follow";
          //Hovering in the beginning of the path
          targetX = traj->path_points[0][0];
          targetY = traj->path_points[0][1];
          targetZ = traj->path_points[0][2];
          start_path = false;
          hover_time = 0; 
        }
      }

      //If we are in phase 4 (Hovering before the path following)
      if (this->phase == 4)
      {
        //we wait 5 seconds hovering
        if (hover_time < 500)
        {
          //PID ACTION
          geometry_msgs::Twist pid_twist;
          //double x_prim, y_prim;

          //STRUCTURE PID ACTION METHOD         
          //pid_name.action(double goal, double pos); 


          this->hoverLoopControl();


          pid_twist.linear.x = roll_des*(180/M_PI);
          pid_twist.linear.y = pitch_des*(180/M_PI);
          pid_twist.linear.z = thrust_hover + delta_thrust/4;

          pid_twist.angular.z = yaw_des;// + mocap_euler[2]*(180/M_PI);//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

          vel_pub.publish(pid_twist);

          hover_time ++;
        }
        //and then start the landing step
        else
        {
          this->phase = 5;
          this->flight_mode = "Landing";
          thrust = thrust_hover;
          hover_time = 0;
        }
      }
 
    }  
*/
    //--------------------------//
    //      PATH FOLLOWING      //
    //--------------------------//
    if (this->flight_mode == "Path_Follow")
    {
      this->getPosQuatMoCap();

      //END of the trajectory
      if (path_index == traj->points-1)
      {
	//If we are performing a closed loop path
	if (closed_path == true)
	{
          targetX = traj->path_points[0][0];
          targetY = traj->path_points[0][1];
          targetZ = traj->path_points[0][2];
	}
	//If we are performing an open loop path
	else
	{
 	  targetX = traj->path_points[traj->points-1][0];
          targetY = traj->path_points[traj->points-1][1];
          targetZ = traj->path_points[traj->points-1][2];
	}

        this->resetPID();
	req_path = false;
        this->flight_mode = "Only_hover";
        
        geometry_msgs::Twist pid_twist;
        this->hoverLoopControl();
        pid_twist.linear.x = roll_des*(180/M_PI);
        pid_twist.linear.y = pitch_des*(180/M_PI);
        pid_twist.linear.z = thrust_hover + delta_thrust/4;
        pid_twist.angular.z = yaw_des;
        vel_pub.publish(pid_twist);

        cout << "End Path\n";
	delete traj;
	traj = NULL;
      }
      else
      {
	//Check if the point is achieved
        int i_tnb = path_index;
        double d = traj->TNB[i_tnb][0]*targetX +  traj->TNB[i_tnb][1]*targetY +  traj->TNB[i_tnb][2]*targetZ;

        if (abs(traj->TNB[i_tnb][0]*mocap_position[0] +  traj->TNB[i_tnb][1]*mocap_position[1] +  traj->TNB[i_tnb][2]*mocap_position[2]-d)<=0.05)
        {
	  std::cout <<"all: "<<traj->points << ", " << path_index << "\n";
          path_index += 1;

	  /*
	  if (closed_path == false && path_index == traj->points-2)
	  {
  	    path_index += 1;
	    std::cout << traj->points << ", " << path_index << "\n";
          }
	  */

          targetX = traj->path_points[path_index][0];
          targetY = traj->path_points[path_index][1];
          targetZ = traj->path_points[path_index][2];

          targetVX = traj->path_points[path_index][3];
          targetVY = traj->path_points[path_index][4];
          targetVZ = traj->path_points[path_index][5];

          targetAX = traj->path_points[path_index][6];
          targetAY = traj->path_points[path_index][7];
          targetAZ = traj->path_points[path_index][8];

          //index += 1;
        }

        this->pathfollowLoopControl();
        geometry_msgs::Twist pid_twist;

        pid_twist.linear.x = roll_des*(180/M_PI);
        pid_twist.linear.y = pitch_des*(180/M_PI);
        pid_twist.linear.z = thrust_hover + delta_thrust/4;
        pid_twist.angular.z = yaw_des;
        vel_pub.publish(pid_twist);

        //DEBUG
        //ROS_INFO_STREAM(pid_twist.linear.x << "," << pid_twist.linear.y << "," << pid_twist.linear.z << "," << pid_twist.angular.z);
        //ROS_INFO_STREAM("delta_thrust " << delta_thrust);
      }
    }
/*
    if (this->flight_mode == "Path_Follow")
    {
      this->getPosQuatMoCap();
      
      //If we are in the beginning of the path
      if (start_path == true)
      {

        //END of the trajectory
        if (path_index == traj->points-1)
        {
          targetX = traj->path_points[0][0];
          targetY = traj->path_points[0][1];
          targetZ = traj->path_points[0][2];

          this->resetPID();
          this->phase = 4;
          this->flight_mode = "Hold_Position";

          cout << "End Path\n"; 

          start_path = false;
          wait_path = 0;

        }
        else
        {
          //CHECK if the point is achieved
          int i_tnb = path_index;
          double d = traj->TNB[i_tnb][0]*targetX +  traj->TNB[i_tnb][1]*targetY +  traj->TNB[i_tnb][2]*targetZ;

          if (abs(traj->TNB[i_tnb][0]*mocap_position[0] +  traj->TNB[i_tnb][1]*mocap_position[1] +  traj->TNB[i_tnb][2]*mocap_position[2]-d)<=0.05)
          {
            path_index += 1;
            targetX = traj->path_points[path_index][0];
            targetY = traj->path_points[path_index][1];
            targetZ = traj->path_points[path_index][2];

            targetVX = traj->path_points[path_index][3];
            targetVY = traj->path_points[path_index][4];
            targetVZ = traj->path_points[path_index][5];

            targetAX = traj->path_points[path_index][6];
            targetAY = traj->path_points[path_index][7];
            targetAZ = traj->path_points[path_index][8];

            //index += 1;
          }

          this->pathfollowLoopControl();
          geometry_msgs::Twist pid_twist;

          pid_twist.linear.x = roll_des*(180/M_PI);
          pid_twist.linear.y = pitch_des*(180/M_PI);
          pid_twist.linear.z = thrust_hover + delta_thrust/4;

          pid_twist.angular.z = yaw_des;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);
          vel_pub.publish(pid_twist);
	  ROS_INFO_STREAM(pid_twist.linear.x << "," << pid_twist.linear.y << "," << pid_twist.linear.z << "," << pid_twist.angular.z);
          ROS_INFO_STREAM("delta_thrust " << delta_thrust);
        }
        
      }
      else
      {
        //if we are not in the beginning of the path
        this->hoverLoopControl();  
        geometry_msgs::Twist pid_twist;

        pid_twist.linear.x = roll_des*(180/M_PI);
        pid_twist.linear.y = pitch_des*(180/M_PI);
        pid_twist.linear.z = thrust_hover + delta_thrust/4;

        pid_twist.angular.z = yaw_des;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);
        vel_pub.publish(pid_twist); 

        //We wait 2 second hovering in the starting position
        if (wait_path > 200)
	{

          start_path = true;

          cout << "Starting Path following\n";

          targetX = traj->path_points[1][0];
          targetY = traj->path_points[1][1];
          targetZ = traj->path_points[1][2];

          
          targetVX = traj->path_points[1][3];
          targetVY = traj->path_points[1][4];
          targetVZ = traj->path_points[1][5];


          targetAX = traj->path_points[1][6];
          targetAY = traj->path_points[1][7];
          targetAZ = traj->path_points[1][8];
          this->path_index = 1;


        }
        else
        {
          if (abs(targetX-mocap_position[0])<=0.1 && abs(targetY-mocap_position[1])<=0.1)
          {
            wait_path +=1;
          }

          if (wait_path > 10 && wait_path <15)
          {
            //Publish the point of the trajectory as Markes
            //so they are available in RVIZi
            visualization_msgs::MarkerArray ma;
            visualization_msgs::Marker points;
            points.header.frame_id = "/mocap";
            points.header.stamp = ros::Time();
            points.ns = "trajectory_mocap";
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;
            points.id = 0;
  	    points.type = visualization_msgs::Marker::POINTS;          
            points.scale.x = 0.05;
            points.scale.y = 0.05;
  	    points.color.g = 1.0f;
            points.color.a = 1.0;
          
            for (int i =0; i<traj->points-1; i++)
            {
              geometry_msgs::Point p;
     	      p.x = traj->path_points[i][0];
              p.y = traj->path_points[i][1];
              p.z = traj->path_points[i][2];
 
 	      points.points.push_back(p);
            }
   	    ma.markers.push_back(points); 

            //trajpoints_pub.publish(points);
            trajpoints_pub.publish(ma);
          }

        }
      }
    }
  */ 
    //Publish Target Values 
    std_msgs::Float64 tX;
    std_msgs::Float64 tY;
    std_msgs::Float64 tZ;        

    tX.data = targetX;
    tY.data = targetY;
    tZ.data = targetZ;

    tarX_pub.publish(tX);
    tarY_pub.publish(tY);
    tarZ_pub.publish(tZ);


    freq_path +=1;
    //cout <<freq_path<<"\n";
    if (quad_detected == true & this->flight_mode == "Path_Follow" & freq_path >= 3)
    //if (quad_detected == true & freq_path >= 10)
    {
    //Publish path
    geometry_msgs::PoseStamped pos2path;

    pos2path.header.frame_id = "/mocap";
    pos2path.header.stamp = ros::Time::now();

    pos2path.pose.position.x = mocap_position[0];
    pos2path.pose.position.y = mocap_position[1];
    pos2path.pose.position.z = mocap_position[2];
    pos2path.pose.orientation.x = mocap_orientation[0];
    pos2path.pose.orientation.y = mocap_orientation[1];
    pos2path.pose.orientation.z = mocap_orientation[2];
    pos2path.pose.orientation.w = mocap_orientation[3];

    //int i = 1;
    path_array.push_back(pos2path);

    quad_path.header.frame_id = "/mocap";
    quad_path.poses = path_array;
    path_pub.publish(quad_path);

    //cout << "Path published\n";
    freq_path = 0;
    }

    //double tac =ros::Time::now().toSec() - tic;
    //std::cout << tac << "seconds \n";

    ros::Duration(0.01).sleep();
    //loop_rate.sleep();
    ros::spinOnce();

  }
}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"logitech_teleop");

  path_sparse = argv[1];
  path_ompl = argv[2];

  Joystick joy;
  
  return 0;

}

