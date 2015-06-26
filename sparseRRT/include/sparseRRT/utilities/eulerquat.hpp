/**
 * @file timer.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */
 
#ifndef EULER_QUAT_HPP
#define EULER_QUAT_HPP

void euler2quat(const double *euler, double *quat);

void quat2euler(const double *quat, double *euler);

#endif