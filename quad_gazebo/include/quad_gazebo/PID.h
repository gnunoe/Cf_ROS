#ifndef PID_H
#define PID_H

#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace std;


//----------------------------------------------------//
//                    PID LIBRARY                     //
//----------------------------------------------------//

class PID
{
public:

  //---------------------VARIABLES--------------------//

  //Name
  std::string name;

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

  //Derivative filter
  double min_d;
  double max_d;
  double new_derivative;
  double prev_derivative;
  double k_smooth;

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

    this->min_d = this->max_d = 0.0;
    this->new_derivative = 0.0;
    this->prev_derivative = 0.0;
    this->k_smooth = 0.1;

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

  //SET MIN OUTPUT
  void setMin(double min_out)
  {
    this->minOutput = min_out;
  }

  //GET DERIVATIVE_UPDATE
  double getDerivativeError()
  {
    return this->derivative_update;
  }

  //GET ERROR
  double getError()
  {
    return this->error;
  }


  double getP()
  {
    return this->proportional;
  }

  double getI()
  {
    return this->integral;
  }

  double getD()
  {
    return this->new_derivative;
  }


  //PRINT CONTRIBUTIONS
  void printKID()
  {
    std::cout << this->proportional <<" "<<this->derivative <<" "<<this->integral<<"\n";
  }

  //RESET
  void Reset()
  {
    //this->error =
    this->integral_update = 0.0;
    this->derivative_update = 0.0;
    this->prev_error = 0.0;
    this->min_d = this->max_d = 0.0;
    this->new_derivative = 0.0;
    this->prev_derivative = 0.0;
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
/*
    this->new_derivative = (1.0 - k_smooth)*this->prev_derivative + k_smooth*this->derivative_update;

    this->prev_derivative = this->new_derivative;

    if (this->derivative_update > min_d)
    {
      this->new_derivative = min_d;
    }
    else if (this->derivative_update < max_d)
    {
      this->new_derivative = max_d;
    }
    else
    {
      this->new_derivative = this->derivative_update;
    }

    if (this->derivative_update > this->prev_derivative)
    {
      this->min_d = this->derivative_update;
      this->max_d = this->prev_derivative;
    }
    else
    {
      this->max_d = this->derivative_update;
      this->min_d = this->prev_derivative;
    }
    this->prev_derivative = this->derivative;
*/
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


#endif /* PID_H */


