/**
 * @file rally_car.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */


#include "sparseRRT/systems/quadrotor.hpp"
#include "sparseRRT/utilities/random.hpp"
#include "sparseRRT/utilities/eulerquat.hpp"


#define _USE_MATH_DEFINES


#include <cmath>
/*
#define M 1450
#define IZ 2740
#define LF 1.3
#define LR 1.4
#define R .3
#define IF 1.8 
#define IR 1.8
#define H .4    
#define B 7
#define C 1.6
#define D .52

#define CRBRAKE 700
#define CRACC 0
#define CFBRAKE 700
#define CFACC 1000
*/

#define MASS 0.021
#define L 0.045
#define Kl 0.00000000019407  // [N / rpm^2]
#define Kd 0.000000000002033 //[N*m / rpm^2]
#define rpm2rad (M_PI^2/30^2)
#define Ixx 0.00000724
#define Iyy 0.00000724
#define Izz 0.000014017
#define w_hover 16291 // Rpm


#define STATE_X 0
#define STATE_Y 1
#define STATE_Z 2
#define STATE_VX 3
#define STATE_VY 4
#define STATE_VZ 5
#define STATE_ROLL 6
#define STATE_PITCH 7
#define STATE_YAW 8
#define STATE_WROLL 9
#define STATE_WPITCH 10
#define STATE_WYAW 11

#define CONTROL_F 0
#define CONTROL_MROLL 1
#define CONTROL_MPITCH 2
#define CONTROL_MYAW 3

//#define MIN_X -2.5
//#define MAX_X 2.5
//#define MIN_Y -3
//#define MAX_Y 3
//#define MIN_Z 0.8//0.5
//#define MAX_Z 1.2//1.5
#define V_XY 1.0 // //1.5
#define V_Z 1.0 //2.0
#define MAX_ROLL 60
#define MAX_PITCH 60
#define MAX_YAW 5
#define A_RP 0.3 //0.5
#define A_YAW 0.3 //0.5


bool quadrotor_cf_t::change_Map_Limits(double abs_x, double abs_y, double min_z, double max_z)
{
	MIN_X = (-1)*abs_x;
        MAX_X =      abs_x;
        MIN_Y = (-1)*abs_y;
        MAX_Y =      abs_y;
        MIN_Z =      min_z;
        MAX_Z =      max_z;
	std::cout << "Map limits changed succesfully\n";
}

bool quadrotor_cf_t::obstacles_in_goal()
{
	bool goal_in_obstacle = false;
        //std::cout << "Getting obstacles\n";
        double safety = 0.2;
        //any obstacles need to be checked here
        for(unsigned i=0;i<obstacles.size();i++)
        {
                if(     params::goal_state[0]>obstacles[i].low_3x-safety &&
                        params::goal_state[0]<obstacles[i].high_3x+safety &&
                        params::goal_state[1]>obstacles[i].low_3y-safety &&
                        params::goal_state[1]<obstacles[i].high_3y+safety &&
                        params::goal_state[2]>obstacles[i].low_3z-safety &&
                        params::goal_state[2]<obstacles[i].high_3z+safety)
                {
                        goal_in_obstacle = true;
                }
        }
        return goal_in_obstacle;
}


bool quadrotor_cf_t::obstacles_in_goal(double x, double y, double z)
{
        bool goal_in_obstacle = false;
        //std::cout << "Getting obstacles\n";
        double safety = 0.2;//0.2;
        double goal_point[3]={x,y,z};

        //any obstacles need to be checked here
        for(unsigned i=0;i<obstacles.size();i++)
        {
                if(     goal_point[0]>obstacles[i].low_3x-safety &&
                        goal_point[0]<obstacles[i].high_3x+safety &&
                        goal_point[1]>obstacles[i].low_3y-safety &&
                        goal_point[1]<obstacles[i].high_3y+safety &&
                        goal_point[2]>obstacles[i].low_3z-safety &&
                        goal_point[2]<obstacles[i].high_3z+safety)
                {
                        goal_in_obstacle = true;
                }
        }
        return goal_in_obstacle;
}



bool quadrotor_cf_t::obstacles_in_start()
{
        bool start_in_obstacle = false;
        //std::cout << "Getting obstacles\n";
        double safety = 0.2;
        //any obstacles need to be checked here
        for(unsigned i=0;i<obstacles.size();i++)
        {
                if(     params::start_state[0]>obstacles[i].low_3x-safety &&
                        params::start_state[0]<obstacles[i].high_3x+safety &&
                        params::start_state[1]>obstacles[i].low_3y-safety &&
                        params::start_state[1]<obstacles[i].high_3y+safety &&
                        params::start_state[2]>obstacles[i].low_3z-safety &&
                        params::start_state[2]<obstacles[i].high_3z+safety)
                {
                        start_in_obstacle = true;
                }
        }
        return start_in_obstacle;
}

bool quadrotor_cf_t::obstacles_in_start(double x, double y, double z)
{
        bool start_in_obstacle = false;
        //std::cout << "Getting obstacles\n";
        double safety = 0.2;
        double start_point[3] = {x,y,z};

        //any obstacles need to be checked here
        for(unsigned i=0;i<obstacles.size();i++)
        {
                if(     start_point[0]>obstacles[i].low_3x-safety &&
                        start_point[0]<obstacles[i].high_3x+safety &&
                        start_point[1]>obstacles[i].low_3y-safety &&
                        start_point[1]<obstacles[i].high_3y+safety &&
                        start_point[2]>obstacles[i].low_3z-safety &&
                        start_point[2]<obstacles[i].high_3z+safety)
                {
                        start_in_obstacle = true;
                }
        }
        return start_in_obstacle;
}


double quadrotor_cf_t::distance(double* point1,double* point2)
{
        //double val = fabs(point1[4]-point2[4]);
        //if(val > M_PI)
        //        val = 2*M_PI-val;
        return std::sqrt((point1[0]-point2[0]) * (point1[0]-point2[0]) + 
                         (point1[1]-point2[1]) * (point1[1]-point2[1]) +
                         (point1[2]-point2[2]) * (point1[2]-point2[2]) );
}

void quadrotor_cf_t::random_state(double* state)
{
        state[0] = uniform_random(MIN_X,MAX_X);
        state[1] = uniform_random(MIN_Y,MAX_Y);
        state[2] = uniform_random(MIN_Z,MAX_Z);

        state[8] = 0;
        // state[3] = uniform_random(-18,18);
        // state[5] = uniform_random(-17,17);
        // state[6] = uniform_random(-40,40);
        // state[7] = uniform_random(-40,40);

        /*
        //compute the angle that is created
        double theta = atan2(state[1],state[0]);
        theta+=M_PI/2;
        theta = uniform_random(theta-M_PI/6.0,theta+M_PI/6.0);
        if(theta > 2*M_PI)
            theta -= 2*M_PI;
        state[4]  = theta;
        // state[4] = uniform_random(-M_PI,M_PI);
         */
}

void quadrotor_cf_t::random_control(double* control)
{
// input_control_space:
//   min: [-1.0472, -700, -700]
//   max: [1.0472, 0, 1200]
    double rpm = 1500;
        double w_1 = w_hover + uniform_random(-rpm,rpm); // rpm
        double w_2 = w_hover + uniform_random(-rpm,rpm); // rpm
        double w_3 = w_hover + uniform_random(-rpm,rpm); // rpm
        double w_4 = w_hover + uniform_random(-rpm,rpm); // rpm
        
        /*
        if omega_motor < 0{
            control[0] = MASS*9.81 - *Kl*(omega_motor*omega_motor);
        }
        else{
            control[0] = MASS*9.81 + 4*Kl*(omega_motor*omega_motor);
        }
         */
        control[0] = Kl*(w_1*w_1 + w_2*w_2 + w_3*w_3 + w_4*w_4);
        control[1] = Kl*L*(w_4*w_4 - w_2*w_2);
        control[2] = Kl*L*(w_1*w_1 - w_3*w_3);
        control[3] = Kd*(w_1*w_1 - w_2*w_2 + w_3*w_3 - w_4*w_4);
}

bool quadrotor_cf_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{
        temp_state[0] = start_state[0]; 
        temp_state[1] = start_state[1];
        temp_state[2] = start_state[2];
        temp_state[3] = start_state[3];
        temp_state[4] = start_state[4];
        temp_state[5] = start_state[5];
        temp_state[6] = start_state[6];
        temp_state[7] = start_state[7];
        temp_state[8] = start_state[8];
        temp_state[9] = start_state[9];
        temp_state[10] = start_state[10];
        temp_state[11] = start_state[11];
        int num_steps = uniform_int_random(min_step,max_step);
        bool validity = true;
        for(int i=0;i<num_steps;i++)
        {
                update_derivative(control);
                temp_state[0] += params::integration_step*deriv[0];
                temp_state[1] += params::integration_step*deriv[1];
                temp_state[2] += params::integration_step*deriv[2];
                temp_state[3] += params::integration_step*deriv[3];
                temp_state[4] += params::integration_step*deriv[4];
                temp_state[5] += params::integration_step*deriv[5];
                temp_state[6] += params::integration_step*deriv[6];
                temp_state[7] += params::integration_step*deriv[7];
                temp_state[8] += params::integration_step*deriv[8];
                temp_state[9] += params::integration_step*deriv[9];
                temp_state[10] += params::integration_step*deriv[10];
                temp_state[11] += params::integration_step*deriv[11];
                enforce_bounds();
                validity = validity && valid_state();
        }
        result_state[0] = temp_state[0];
        result_state[1] = temp_state[1];
        result_state[2] = temp_state[2];
        result_state[3] = temp_state[3];
        result_state[4] = temp_state[4];
        result_state[5] = temp_state[5];
        result_state[6] = temp_state[6];
        result_state[7] = temp_state[7];
        result_state[8] = temp_state[8];
        result_state[9] = temp_state[9];
        result_state[10] = temp_state[10];
        result_state[11] = temp_state[11];
        duration = num_steps*params::integration_step;
        return validity;
}

void quadrotor_cf_t::enforce_bounds()
{
// #x y xdot ydot theta thetadot wf wr
// state_space: 
//   min: [-25, -40, -18, -18, -3.14, -17, -40, -40]
//   max: [25, 25, 18, 18, 3.14, 17, 40, 40]
        if(temp_state[0]<MIN_X)
                temp_state[0]=MIN_X;
        else if(temp_state[0]>MAX_X)
                temp_state[0]=MAX_X;

        if(temp_state[1]<MIN_Y)
                temp_state[1]=MIN_Y;
        else if(temp_state[1]>MAX_Y)
                temp_state[1]=MAX_Y;

        if(temp_state[2]<MIN_Z)
                temp_state[2]=MIN_Z;
        else if(temp_state[2]>MAX_Z)
                temp_state[2]=MAX_Z;

        if(temp_state[3]<-V_XY)
                temp_state[3]=-V_XY;
        else if(temp_state[3]>V_XY)
                temp_state[3]=V_XY;

        if(temp_state[4]<-V_XY)
                temp_state[4]= V_XY;
        else if(temp_state[4]>V_XY)
                temp_state[4]=V_XY;

        if(temp_state[5]<-V_Z)
                temp_state[5]=-V_Z;
        else if(temp_state[5]>V_Z)
                temp_state[5]=V_Z;

        if(temp_state[6]<-MAX_ROLL)
                temp_state[6]=-MAX_ROLL;
        else if(temp_state[6]>MAX_ROLL)
                temp_state[6]=MAX_ROLL;

        if(temp_state[7]<-MAX_PITCH)
                temp_state[7]=-MAX_PITCH;
        else if(temp_state[7]>MAX_PITCH)
                temp_state[7]=MAX_PITCH;
        
        if(temp_state[8]<-MAX_YAW)
                temp_state[8]=-MAX_YAW;
        else if(temp_state[8]>MAX_YAW)
                temp_state[8]=MAX_YAW;
        
        if(temp_state[9]<-A_RP)
                temp_state[9]=-A_RP;
        else if(temp_state[9]>A_RP)
                temp_state[9]=A_RP;
        
        if(temp_state[10]<-A_RP)
                temp_state[10]=-A_RP;
        else if(temp_state[10]>A_RP)
                temp_state[10]=A_RP;
        
        if(temp_state[11]<-A_YAW)
                temp_state[11]=-A_YAW;
        else if(temp_state[11]>A_YAW)
                temp_state[11]=A_YAW;
                
}


bool quadrotor_cf_t::valid_state()
{
        bool obstacle_collision = false;
  	double safety = 0.2;
        //any obstacles need to be checked here
        for(unsigned i=0;i<obstacles.size() && !obstacle_collision;i++)
        {
                if(     temp_state[0]>obstacles[i].low_3x-safety &&
                        temp_state[0]<obstacles[i].high_3x+safety &&
                        temp_state[1]>obstacles[i].low_3y-safety &&
                        temp_state[1]<obstacles[i].high_3y+safety &&
                        temp_state[2]>obstacles[i].low_3z-safety &&
                        temp_state[2]<obstacles[i].high_3z+safety)
		{
                        obstacle_collision = true;
                }
        }
        return !obstacle_collision;
}

svg::Point quadrotor_cf_t::visualize_point(double* state, svg::Dimensions dims)
{
        double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
        double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
        return svg::Point(x,y);
}

void quadrotor_cf_t::update_derivative(double* control)
{
        double _x = temp_state[0];
        double _y = temp_state[1];
        double _z = temp_state[2];
        double _vx = temp_state[3];
        double _vy = temp_state[4];
        double _vz = temp_state[5];
        double _roll = temp_state[6];
        double _pitch = temp_state[7];
        double _yaw = temp_state[8];
        double _wroll = temp_state[9];
        double _wpitch = temp_state[10];
        double _wyaw = temp_state[11];

        double _F = control[0];
        double _mroll = control[1];
        double _mpitch = control[2];
        double _myaw = control[3];

        deriv[STATE_X] = _vx;
        deriv[STATE_Y] = _vy;
        deriv[STATE_Z] = _vz;
        
        deriv[STATE_VX] = _F*(cos(_yaw)*sin(_pitch)+cos(_pitch)*sin(_roll)*sin(_yaw))/MASS;
        deriv[STATE_VY] = _F*(sin(_yaw)*sin(_pitch)-cos(_yaw)*sin(_pitch)*sin(_roll))/MASS;
        deriv[STATE_VZ] = (_F*(cos(_pitch)*cos(_roll))/MASS) - 9.81 ; 

        /*
        double *euler;
        euler = new double [3];
        double *quat;
        quat = new double [4];
        
        euler[0] = _roll;
        euler[1] = _pitch;
        euler[2] = _yaw;
        
        euler2quat(euler,quat);
        */
        deriv[STATE_ROLL] = cos(_pitch)*_wroll + sin(_pitch)*_wyaw;
        deriv[STATE_PITCH] = _wpitch;
        deriv[STATE_YAW] = cos(_pitch)*_wyaw/cos(_roll) - sin(_pitch)*_wroll/cos(_roll);
        deriv[STATE_WROLL] = _mroll/Ixx;
        deriv[STATE_WPITCH] = _mpitch/Iyy;
        deriv[STATE_WYAW] = _myaw/Izz;

        
        /*
        double V = sqrt(_vx*_vx+_vy*_vy);
        double beta = atan2(_vy,_vx) - _theta;
        double V_Fx = V*cos(beta-_sta) + _thetadot*LF*sin(_sta);
        double V_Fy = V*sin(beta-_sta) + _thetadot*LF*cos(_sta);
        double V_Rx = V*cos(beta);
        double V_Ry = V*sin(beta) - _thetadot*LR;

        double s_Fx = (V_Fx - _wf*R)/(_wf*R);
        double s_Fy = V_Fy/(_wf*R);
        double s_Rx = (V_Rx - _wr*R)/(_wr*R);
        double s_Ry = V_Ry/(_wr*R);

        double s_F = sqrt(s_Fx*s_Fx+s_Fy*s_Fy);
        double s_R = sqrt(s_Rx*s_Rx+s_Ry*s_Ry);

        double mu_F = D*sin(C*atan(B*s_F));
        double mu_R = D*sin(C*atan(B*s_R));
        double mu_Fx;
        double mu_Fy;
        if(std::isfinite(s_Fx))
                mu_Fx = -1*(s_Fx/s_F)*mu_F;
        else
                mu_Fx = -mu_F;
        if(std::isfinite(s_Fy))
                mu_Fy = -1*(s_Fy/s_F)*mu_F;
        else
                mu_Fy = -mu_F;
        double mu_Rx;
        double mu_Ry;
        if(std::isfinite(s_Rx))
                mu_Rx = -1*(s_Rx/s_R)*mu_R;
        else
                mu_Rx = -mu_R;
        if(std::isfinite(s_Ry))
                mu_Ry = -1*(s_Ry/s_R)*mu_R;
        else
                mu_Ry = -mu_R;

        double fFz = (LR*M*(9.8) - H*M*9.8*mu_Rx) / (LF+LR+H*(mu_Fx*cos(_sta)-mu_Fy*sin(_sta)-mu_Rx));
        double fRz = M*9.8 - fFz;

        double fFx = mu_Fx * fFz;
        double fFy = mu_Fy * fFz;
        double fRx = mu_Rx * fRz;
        double fRy = mu_Ry * fRz;;
        */


}
void quadrotor_cf_t::visualize_obstacles(svg::Document& doc ,svg::Dimensions dims)
{
        double temp[2];
        for(unsigned i=0;i<obstacles.size();i++)
        {
                temp[0] = obstacles[i].low_3x;
                temp[1] = obstacles[i].high_3y;

		if (params::map == "Map3_3D")
		{
		  //Do not visualize ground and first floor
		  if (i>=2 && i<53)
		  {
		    //std::cout << i << "\n";
		    doc<<svg::Rectangle(visualize_point(temp,dims), 
                                        (obstacles[i].high_3x-obstacles[i].low_3x)/(MAX_X-MIN_X) * dims.width,
                                        (obstacles[i].high_3y-obstacles[i].low_3y)/(MAX_Y-MIN_Y) * dims.height,
                                        svg::Color::Red);
		  }
		}
		else if (params::map == "Map4_3D")
                {
                  //Do not visualize ground and first floor
                  if (i>=2 && i<81)
                  {
                    //std::cout << i << "\n";
                    doc<<svg::Rectangle(visualize_point(temp,dims),
                                        (obstacles[i].high_3x-obstacles[i].low_3x)/(MAX_X-MIN_X) * dims.width,
                                        (obstacles[i].high_3y-obstacles[i].low_3y)/(MAX_Y-MIN_Y) * dims.height,
                                        svg::Color::Red);
                  }
                }
                else if (params::map == "Map5_3D")
                {
                  //Do not visualize ground and first floor
                  if (i>=2 && i<141)
                  {
                    //std::cout << i << "\n";
                    doc<<svg::Rectangle(visualize_point(temp,dims),
                                        (obstacles[i].high_3x-obstacles[i].low_3x)/(MAX_X-MIN_X) * dims.width,
                                        (obstacles[i].high_3y-obstacles[i].low_3y)/(MAX_Y-MIN_Y) * dims.height,
                                        svg::Color::Red);
                  }
                }

		else
		{
		  //Do not visualize ground floor
		  if (i>=2)
                  {
                    doc<<svg::Rectangle(visualize_point(temp,dims), 
                                        (obstacles[i].high_3x-obstacles[i].low_3x)/(MAX_X-MIN_X) * dims.width,
                                        (obstacles[i].high_3y-obstacles[i].low_3y)/(MAX_Y-MIN_Y) * dims.height,
                                        svg::Color::Red);
		  }  
		}
        }
}


