

#include "quad_gazebo/trajectory.hpp"
#include "quad_gazebo/t_circle.hpp"

#include <cmath>



void t_circle::Generate_points()
{
    double time = 0.0;
    
    //Position, velocity and acceleration of the points
    for (int i=0; i<points; i++)
    {
        path_points[i][0] = radio*cos(A*time);
        path_points[i][1] = radio*sin(A*time);
        path_points[i][2] = z_fix;
        path_points[i][3] = std::max(std::min(A*-sin(A*time),v_max),-v_max);
        path_points[i][4] = std::max(std::min(A*cos(A*time),v_max),-v_max);
        path_points[i][5] = 0;
        path_points[i][6] = std::max(std::min(A*A*-cos(A*time),a_max),-a_max);
        path_points[i][7] = std::max(std::min(A*A*-sin(A*time),a_max),-a_max);
        path_points[i][8] = 0;
        
        time+=dt;
    }
    Generate_TNB();
}
