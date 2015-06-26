
/* 
 * File:   t_circle.hpp
 * Author: Gonzalo
 *
 * Created on 17 de abril de 2015, 11:20
 */

#ifndef T_CIRCLE_HPP
#define	T_CIRCLE_HPP

#include "quad_gazebo/trajectory.hpp"
#include <cmath>
#include <iostream>
#include <cstdlib>

class t_circle :public Trajectory
{
public:
    double t0;
    double tf;
    double dt;
    double v_max;
    double a_max;
    
    t_circle()
    {
        t0 = 0;
        dt = 0.3;
        tf = 6;
        setPoints(int(tf/dt)+1);
        v_max = 1.0;
        a_max = 0.5;
        
        allocateMemory();
        
        A = 2*M_PI/tf;
        radio = 1.0;
        z_fix = 1.0;

        Generate_points();
    }
    
    virtual void Generate_points();
    
private:
    double A;
    double radio;
    double z_fix;
};


#endif	/* T_CIRCLE_HPP */
