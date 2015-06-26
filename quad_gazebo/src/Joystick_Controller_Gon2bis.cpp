//ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

//MOCAP
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"

//C++
#include <cstdlib>
#include <iostream>
#include <vector>
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
  double derivative_update;
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
    this->derivative_update = 0.0;

    ros::Time::init();

    this->prev_time = ros::Time::now().toSec();
    this->dt = 0.0;

    //this->pid_pub=n.advertise<std_msgs::Float64>("/" + this->name, 1);
  }

  //SET MAX OUTPUT
  void setMax(double max_out)
  {
    this->maxOutput = max_out;
  }

  //GET DERIVATIVE_UPDATE
  double getDerivativeError()
  {
    return this->derivative_update;
  }

  //PRINT CONTRIBUTIONS
  void printKID()
  {
    cout << this->proportional <<" "<<this->derivative <<" "<<this->integral<<"\n";
  }

  //RESET
  void Reset()
  {
    //this->error =
    this->integral_update = 0.0; 
    this->derivative_update = 0.0;
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
    this->derivative_update = ((this->error - this->prev_error)/this->dt);

    //Calculate the three actions  
    this->proportional = this->kp*this->error;
    this->integral = this->ki*this->integral_update;
    this->derivative = this->kd*this->derivative_update;

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

class Circle_path{

  public:
  int t0;
  double dt;
  int tf;
  int points;
  double lambda_v;
  double **path_points;
  double **TNB;

 Circle_path()
  {
    t0 = 0;
    dt = 0.3;
    tf = 3;
    points = 11;
    lambda_v = 0.3;
    
    //Allocate memory for points
    path_points = new double*[points];
    for (int i=0; i<points; i++)
    {
      path_points[i] = new double[6];
    }
 
    TNB = new double*[points];
    for (int i=0; i<points; i++)
    {
      TNB[i] = new double[9];
    }

    


    this->Generate_points();
  }

~Circle_path()
{
  for (int i=0; i<points; i++)
  {
    delete [] path_points[i];
  }
  delete [] path_points;

  for (int i=0; i<points; i++)
  {
    delete [] TNB[i];
  }
  delete [] TNB;
}


  void Generate_points()
  {
    double A = 2*M_PI/this->tf;
    double radio = 1.0;
    double z_fix = 1;
    double time = 0.0;
    //double path_points[points][6];

    for (int i=0; i<=points-1; i++)
    {
      this->path_points[i][0] = radio*cos(A*time);
      this->path_points[i][1] = radio*sin(A*time);
      this->path_points[i][2] = z_fix;
      this->path_points[i][3] = lambda_v*A*-sin(A*time);
      this->path_points[i][4] = lambda_v*A*cos(A*time);
      this->path_points[i][5] = 0;
      
      time += this->dt;
    }

    for (int i=0; i<points-1; i++)
    {
      this->TNB[i][0] = this->path_points[i+1][0] - this->path_points[i][0];
      this->TNB[i][1] = this->path_points[i+1][1] - this->path_points[i][1];
      this->TNB[i][2] = this->path_points[i+1][2] - this->path_points[i][2];
      this->TNB[i][3] = 0;
      this->TNB[i][4] = 0;
      this->TNB[i][5] = 1;

      this->TNB[i][6] = TNB[i][1]*TNB[i][5] - TNB[i][2]*TNB[i][4];
      this->TNB[i][7] = (TNB[i][0]*TNB[i][5] - TNB[i][2]*TNB[i][3])*(-1);
      this->TNB[i][8] = TNB[i][0]*TNB[i][4] - TNB[i][1]*TNB[i][3];

      //cout << TNB[i][0] <<" "<<TNB[i][1]<<" "<<TNB[i][2]<<" "<<TNB[i][6]<<" "<<TNB[i][7]<<" "<<TNB[i][8]<<"\n";

    }

    
    //this->Show_points();
  }

  void Show_points()
  {
    for (int i=0; i<=points-1; i++)
    {
      cout << this->path_points[i][0] <<" "<< this->path_points[i][1] <<" "<< this->path_points[i][2] <<" "<< this->path_points[i][3] <<" "<< this->path_points[i][4] <<" "<< this->path_points[i][5] << "\n";
    }
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
      PID *pid_hover_x;
      PID *pid_hover_y;
      PID *pid_hover_z;
    
      PID *pid_att_roll;
      PID *pid_att_pitch;
      PID *pid_att_yaw;

      PID *pid_pathfoll_x;
      PID *pid_pathfoll_y;
      PID *pid_pathfoll_z;

      PID *pid_pathfoll_vx;
      PID *pid_pathfoll_vy;
      PID *pid_pathfoll_vz;


      //PID *pid_rate_yaw;

  //++Trajectory Variables
      bool start_path;
      int wait_path;
      int index;
      int freq_path;

      Circle_path* circle;

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

  //void attitudeLoopControl();

  //Average filter
  //bool getAverage(tf::Quaternion quat);
};

Joystick::~Joystick()
{
  //Delete allocated memory by PID-objects
  delete pid_hover_x;
  delete pid_hover_y;
  delete pid_hover_z;
/*
  delete pid_att_roll;
  delete pid_att_pitch;
  delete pid_att_yaw;
*/
  delete pid_pathfoll_x;
  delete pid_pathfoll_y;
  delete pid_pathfoll_z;

  delete pid_pathfoll_vx;
  delete pid_pathfoll_vy;
  delete pid_pathfoll_vz;


  //delete pid_rate_yaw;
}

//Constructor, initializate the joystick class
Joystick::Joystick()
{
  //Initializate Variables in the same order as the definiton of the class

  //path_array = new geometry_msgs::PoseStamped [100];

  //++Quadcopter State Variables   
  this->flight_mode = "Manual";
  ROS_INFO_STREAM("Flight Mode: Manual");
  thrust = 0.0; 

  //++Quadcopter Parameters
  gravity = 9.81;
  quad_mass = 0.021;
  quad_Klift = 0.00000000019407;
  quad_w_hover = sqrt((quad_mass*gravity)/(4*quad_Klift));
  thrust_hover = 47500; 

  //++Target Values
  targetX = 0.0;
  targetY = 0.0;
  targetZ = 1.0;
  targetAngle = 0.0;

  targetVX = 0.0;
  targetVY = 0.0;
  targetVZ = 0.0;

  //++Desired Euler Angles
  roll_des = 0.0;
  pitch_des = 0.0;
  yaw_des = 0.0;

  //++Desired Thrust Values
  //thrust_hover = 48500;
  delta_thrust = 0;
  

  //++PID-Hover objects
/*
  pid_hover_x  =  new PID(     5.2,     0.05,      1.1,      -7.0,     7.0,   "pid_hover_X");
  pid_hover_y  =  new PID(    -5.0,     0.0,     -1.5,      -7.0,     7.0,   "pid_hover_Y");
  pid_hover_z  =  new PID(     9.0,     1.5,      3.2,     -6.0,    15.0,   "pid_hover_Z");
*/

  pid_hover_x  =  new PID(     3.8,     0.0,      0.8,      -7.0,     7.0,   "pid_hover_X");
  pid_hover_y  =  new PID(    -3.8,    -0.0,     -0.8,      -7.0,     7.0,   "pid_hover_Y");
  pid_hover_z  =  new PID(     9.0,     1.5,      2.5,     -15.0,    15.0,   "pid_hover_Z");

/*
  pid_att_roll =  new PID(     0.5,    0.0,       0.0,     -50.0,    50.0,   "pid_att_roll");
  pid_att_pitch = new PID(    14.0,    0.0,      0.95,     -1.0,    1.0,   "pid_att_pitch");
  pid_att_yaw =   new PID(    19.5,    0.0,      0.5,      -1.0,    1.0,   "pid_att_yaw");
*/
  pid_pathfoll_x= new PID(    3.8,     0.0,     0.0,      -3.0,     3.0,   "pid_pathfoll_X");
  pid_pathfoll_y= new PID(   -3.8,     0.0,     0.0,      -3.0,     3.0,   "pid_pathfoll_Y");
  pid_pathfoll_z= new PID(   15.0,     0.0,     0.0,      -6.0,     15.0,   "pid_pathfoll_Z");

  pid_pathfoll_vx=new PID(    2.5,     0.0,     0.0,      -1.5,     1.5,   "pid_pathfoll_VX");
  pid_pathfoll_vy=new PID(   -2.5,     0.0,     0.0,      -1.5,     1.5,   "pid_pathfoll_VY");
  pid_pathfoll_vz=new PID(    0.0,     0.0,     0.0,      -3.0,    3.0,   "pid_pathfoll_VZ");

  //pid_rate_yaw =  new PID(    50.0,    0.0,      0.0,    -100.0,  100.0, "pidYAW");

  //++Trajectory Variables
  start_path = false;
  index = 0;
  wait_path = 0;
  freq_path = 0;
  circle = new Circle_path();

  //++Quadcopter String
  quad_name = "/Cf_Uni";
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
  trajpoints_pub = node.advertise<visualization_msgs::Marker>("/trajectory_points",1);

  //Publish path
  path_pub = node.advertise<nav_msgs::Path>("/Cf_Gon_Path",1);

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
/*
    pid_att_roll->Reset();
    pid_att_pitch->Reset();
    pid_att_yaw->Reset();
*/
    //pid_rate_yaw->Reset();
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
      this->flight_mode = "Path_Follow";
      ROS_INFO_STREAM("Path_Follow");
      if (start_path == false)
      {
        targetX = circle->path_points[0][0];
        targetY = circle->path_points[0][1];
        targetZ = circle->path_points[0][2];

        cout << targetX << targetY << targetZ;
      }
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
      //targetX = -0.1;
      //targetY = -0.1;
      //targetZ = 1.0;
      pid_hover_z->setMax(10.0);
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
    if (btton[4]==1 & this->flight_mode != "Hold_Position")
    {
      this->flight_mode = "Hold_Position";
      this->resetPID();
      ROS_INFO_STREAM("Hold Position");
    }

    //======== Button 6 ==========//

    if (btton[5]==1)
    {
      yaw_des = 90.0;
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
 

/*
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
      }*/

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
  double der_x,der_y,der_z;
  double vx_p, vy_p,vz_p;
  double d2r = M_PI/180;

  //Acceleration in the three axis
  rx_dd = pid_pathfoll_x->action(targetX-mocap_position[0],0.0);
  ry_dd = pid_pathfoll_y->action(targetY-mocap_position[1],0.0);
  rz_dd = pid_pathfoll_z->action(targetZ-mocap_position[2],0.0);

  der_x = (-1)*pid_pathfoll_x->getDerivativeError();
  der_y = (-1)*pid_pathfoll_y->getDerivativeError();
  der_z = (-1)*pid_pathfoll_z->getDerivativeError();

  vx_p = pid_pathfoll_vx->action(targetVX-der_x,0.0);
  vy_p = pid_pathfoll_vy->action(targetVY-der_y,0.0);
  vz_p = pid_pathfoll_vz->action(targetVZ-der_z,0.0);

  ROS_INFO_STREAM("acc: "<< rx_dd <<" "<< ry_dd <<" "<<rz_dd);
  ROS_INFO_STREAM("acc_v: "<< vx_p <<" "<< vy_p <<" "<<vz_p);

  rx_dd += vx_p;
  ry_dd += vy_p;
  rz_dd += vz_p;

  //roll_des  = (rx_dd*cos(yaw_des*d2r) - ry_dd*sin(yaw_des*d2r))/gravity;
  //pitch_des = (rx_dd*sin(yaw_des*d2r) + ry_dd*cos(yaw_des*d2r))/gravity;
  roll_des  =     (rx_dd*cos(mocap_euler[2]) - ry_dd*sin(mocap_euler[2]))/gravity;
  pitch_des =     (rx_dd*sin(mocap_euler[2]) + ry_dd*cos(mocap_euler[2]))/gravity;

  delta_thrust = (quad_mass*rz_dd)/(4*quad_Klift*quad_w_hover); 
  
}

/*
void Joystick::attitudeLoopControl()
{
  double r2d = 180/M_PI;

  //Rate desired in deg/s
  roll_rate_des = pid_att_roll->action(roll_des*r2d-mocap_euler[0]*r2d,0.0); 
  //pitch_rate_des = pid_att_pitch->action(pitch_des-mocap_euler[1],0.0);
  //yaw_rate_des = pid_att_yaw->action(yaw_des*(180/M_PI)+mocap_euler[2],0.0);
  pid_att_roll->printKID();
  ROS_INFO_STREAM("Desired Rate "<< roll_rate_des);//<< "," << pitch_rate_des*(180/M_PI) << "," << yaw_rate_des*(180/M_PI));

}
*/
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
      if (thrust > thrust_hover || mocap_position[2]>0.2)
      {
        this->resetPID();
        //pid_hover_z->integral_update = thrust / (pid_hover_z->ki*1000);
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

	takeOff.linear.x =  roll_des*(180/M_PI);//0.0;//-10.0;//(0.0-mocap_euler[0])*(180/M_PI);//roll_des*(180/M_PI);
        //takeOff.linear.x =  roll_rate_des;
        //takeOff.linear.x =  (roll_des-mocap_euler[0])*(180/M_PI);
        takeOff.linear.y = pitch_des*(180/M_PI);//10.0;//(0.0-mocap_euler[1])*(180/M_PI);//pitch_des*(180/M_PI);
        takeOff.linear.z = thrust;
	takeOff.angular.z = 0.0;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);

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
      //double x_prim, y_prim;
      
      //STRUCTURE PID ACTION METHOD         
      //pid_name.action(double goal, double pos); 
      
      /*        
      // Whoening Approach 
      //Descompose X and Y contributions taking care of the Yaw angle
      x_prim = pid_x->action(targetX-mocap_position[0],0.0);
      y_prim = pid_y->action(targetY-mocap_position[1],0.0);
               
      pid_twist.linear.x = x_prim*cos(mocap_euler[2])-y_prim*sin(mocap_euler[2]);
      pid_twist.linear.y = x_prim*sin(mocap_euler[2])+y_prim*cos(mocap_euler[2]);
      pid_twist.linear.z = pid_z->action(targetZ-mocap_position[2],0.0);
      pid_twist.angular.z = pid_yaw->action(targetAngle*(M_PI/180) + mocap_euler[2],0.0);
               
      vel_pub.publish(pid_twist);
       */

      this->hoverLoopControl();

      //Mellinger Approach
      //pid_twist.linear.x = pitch_des*(180/M_PI);
      //pid_twist.linear.y = roll_des*(180/M_PI);

      //this->attitudeLoopControl();

      //Whoenig Approach
      //pid_twist.linear.x = roll_des*(180/M_PI);
      //pid_twist.linear.x = roll_rate_des;
      /*
      double roll_error = (roll_des-mocap_euler[0])*(180/M_PI);
      if (roll_error > 50.0)
      {
        roll_error = 50.0;
      }
      if (roll_error < -50.0)
      {
        roll_error = -50.0;
      }

      double pitch_error = (pitch_des+mocap_euler[1])*(180/M_PI);
      if (pitch_error > 50.0)
      {
        pitch_error = 50.0;
      }
      if (pitch_error < -50.0)
      {
        pitch_error = -50.0;
      }


      pid_twist.linear.x = roll_error;
      //pid_twist.linear.y = pitch_error;
      */

      pid_twist.linear.x = roll_des*(180/M_PI);
      pid_twist.linear.y = pitch_des*(180/M_PI);
      pid_twist.linear.z = thrust_hover + delta_thrust/4;

      pid_twist.angular.z = yaw_des;// + mocap_euler[2]*(180/M_PI);//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);
      //ROS_INFO_STREAM(pid_twist.linear.x << "," << pid_twist.linear.y << "," << pid_twist.linear.z << "," << pid_twist.angular.z);
      /*
      this->attitudeLoopControl();
      pid_twist.linear.x = roll_rate_des;
      pid_twist.linear.y = pitch_rate_des;
      pid_twist.linear.z = thrust_hover-1000 + delta_thrust/4; // + delta_thrust;

      pid_twist.angular.z = yaw_rate_des;
       */     
      vel_pub.publish(pid_twist);

    }  

    //--------------------------//
    //      PATH FOLLOWING      //
    //--------------------------//
    if (this->flight_mode == "Path_Follow")
    {
      this->getPosQuatMoCap();
      
      if (start_path == true)
      {

        //END of the trajectory
        if (index == circle->points-1)
        {
          targetX = circle->path_points[0][0];
          targetY = circle->path_points[0][1];
          targetZ = circle->path_points[0][2];

          this->resetPID();
          this->flight_mode = "Hold_Position";

          cout << "End Path\n"; 

          start_path = false;
          wait_path = 0;

        }
        else
        {
          //CHECK if the point is achieved
          int i_tnb = index-1;
          double d = circle->TNB[i_tnb][0]*targetX +  circle->TNB[i_tnb][1]*targetY +  circle->TNB[i_tnb][2]*targetZ;

          if (abs(circle->TNB[i_tnb][0]*mocap_position[0] +  circle->TNB[i_tnb][1]*mocap_position[1] +  circle->TNB[i_tnb][2]*mocap_position[2]-d)<=0.05)
          {
            index += 1;
            targetX = circle->path_points[index][0];
            targetY = circle->path_points[index][1];
            targetZ = circle->path_points[index][2];

            targetVX = circle->path_points[index][3];
            targetVY = circle->path_points[index][4];
            targetVZ = circle->path_points[index][5];
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
        this->hoverLoopControl();  
        geometry_msgs::Twist pid_twist;

        pid_twist.linear.x = roll_des*(180/M_PI);
        pid_twist.linear.y = pitch_des*(180/M_PI);
        pid_twist.linear.z = thrust_hover + delta_thrust/4;

        pid_twist.angular.z = yaw_des;//pid_rate_yaw->action(yaw_des*(M_PI/180) + mocap_euler[2],0.0);
        vel_pub.publish(pid_twist); 

        //We wait 1 second hovering in the starting position
        if (wait_path > 100)
	{

          start_path = true;

          cout << "Starting Path following\n";

          targetX = circle->path_points[1][0];
          targetY = circle->path_points[1][1];
          targetZ = circle->path_points[1][2];

          targetVX = circle->path_points[1][3];
          targetVY = circle->path_points[1][4];
          targetVZ = circle->path_points[1][5];

          this->index = 1;


        }
        else
        {
          if (abs(targetX-mocap_position[0])<=0.1 && abs(targetY-mocap_position[1])<=0.1)
          {
            wait_path +=1;
          }

          if (wait_path > 10 || wait_path <15)
          {
            //Publish the point of the trajectory as Markes
            //so they are available in RVIZ
            visualization_msgs::Marker points;
            points.header.frame_id = "/mocap";
            points.header.stamp = ros::Time();
            points.ns = "trajectory_points";
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;
            points.id = 0;
  	    points.type = visualization_msgs::Marker::POINTS;          
            points.scale.x = 0.05;
            points.scale.y = 0.05;
  	    points.color.g = 1.0f;
            points.color.a = 1.0;
          
            for (int i =0; i<circle->points-1; i++)
            {
              geometry_msgs::Point p;
     	      p.x = circle->path_points[i][0];
              p.y = circle->path_points[i][1];
              p.z = circle->path_points[i][2];
 
 	      points.points.push_back(p);
            }
 
            trajpoints_pub.publish(points);
          }

        }
      }
    }
   
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
    if (quad_detected == true & start_path == true & freq_path >= 3)
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

