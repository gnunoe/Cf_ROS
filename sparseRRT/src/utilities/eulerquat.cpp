/**
 * @file timer.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "sparseRRT/utilities/eulerquat.hpp"

#include <cmath>
#include <cstddef>

void euler2quat(const double *euler, double *quat){
    
    double phi = euler[0];
    double theta = euler[1];
    double psi = euler[2];
    
    double R [3][3] = {};
    
    R[0][0] = cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta);
    R[0][1] = cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta);
    R[0][2] = -cos(phi)*sin(theta);
    R[1][0] = -cos(phi)*sin(psi);
    R[1][1] = cos(phi)*cos(psi);
    R[1][2] = sin(phi);
    R[2][0] = cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
    R[2][1] = sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
    R[2][2] = cos(phi)*cos(theta);
    
    
    
    double tr = R[0][0] + R[1][1] + R[2][2];
    double S,qw,qx,qy,qz;
    
    if (tr > 0){
      S = sqrt(tr+1.0) * 2; // S=4*qw
      qw = 0.25 * S;
      qx = (R[2][1] - R[1][2]) / S;
      qy = (R[0][2] - R[2][0]) / S;
      qz = (R[1][0] - R[0][1]) / S;
    }
    else if ((R[0][0] > R[1][1])&(R[0][0] > R[2][2])){
      S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2; // S=4*qx
      qw = (R[2][1] - R[1][2]) / S;
      qx = 0.25 * S;
      qy = (R[0][1] + R[1][0]) / S;
      qz = (R[0][2] + R[2][0]) / S;
    }
    else if (R[1][1] > R[2][2]){
      S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2; // S=4*qy
      qw = (R[0][2] - R[2][0]) / S;
      qx = (R[0][1] + R[1][0]) / S;
      qy = 0.25 * S;
      qz = (R[1][2] + R[2][1]) / S;
    }
    else{
      S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2; // S=4*qz
      qw = (R[1][0] - R[0][1]) / S;
      qx = (R[0][2] + R[2][0]) / S;
      qy = (R[1][2] + R[2][1]) / S;
      qz = 0.25 * S;
    }
    
    if (qw < 0){
        qw *= -1;
        qx *= -1;
        qy *= -1;
        qz *= -1;
    }
    //cout << qw <<" "<<qx<<" "<<qy<<" "<<qz;
    
    quat[0] = qw;
    quat[1] = qx;
    quat[2] = qy;
    quat[3] = qz;
    
    
    /*
    R = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)
     cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), ...
     -cos(phi)*sin(theta); ...
     -cos(phi)*sin(psi),...
     cos(phi)*cos(psi), ...
     sin(phi);...
     cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),...
     sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),...
     cos(phi)*cos(theta)];
    */
}

void quat2euler(const double *quat, double *euler){
    
    double qw = quat[0];
    double qx = quat[1];
    double qy = quat[2];
    double qz = quat[3];
    
    double q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    
    qw /= q_norm;
    qx /= q_norm;
    qy /= q_norm;
    qz /= q_norm;
    
    double R[3][3];
    
    R[0][0] = - 2*qy*qy - 2*qz*qz + 1;
    R[0][1] = 2*qx*qy - 2*qw*qz;
    R[0][2] = 2*qw*qy + 2*qx*qz;
    R[1][0] = 2*qw*qz + 2*qx*qy;
    R[1][1] = - 2*qx*qx - 2*qz*qz + 1;
    R[1][2] = 2*qy*qz - 2*qw*qx;
    R[2][0] = 2*qx*qz - 2*qw*qy;
    R[2][1] = 2*qw*qx + 2*qy*qz;
    R[2][2] = - 2*qx*qx - 2*qy*qy + 1;
    
    euler[0] = asin(R[1][2]);
    euler[2] = atan2(-R[1][0]/cos(euler[0]),R[1][1]/cos(euler[0]));
    euler[1] = atan2(-R[0][2]/cos(euler[0]),R[2][2]/cos(euler[0]));
    
}
