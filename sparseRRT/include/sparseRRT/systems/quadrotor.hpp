/**
 * @file rally_car.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_QUADROTOR_CF_HPP
#define SPARSE_QUADROTOR_CF_HPP


#include "sparseRRT/systems/system.hpp"
#include "sparseRRT/systems/point.hpp"

class quadrotor_cf_t : public system_t
{
public:
	//Constructor
	quadrotor_cf_t()
	{
		state_dimension = 12;
		control_dimension = 4;
		temp_state = new double[state_dimension];
		deriv = new double[state_dimension];

		//Set default values for the mocap space
                MIN_X = -2.0;
                MAX_X =  2.0;
                MIN_Y = -2.5;
                MAX_Y =  2.5;
                MIN_Z =  0.5;
                MAX_Z =  1.5;
		
		/*
		MIN_X = -2.5;
        	MAX_X =  2.5;
        	MIN_Y = -3.0;
        	MAX_Y =  3.0;
        	MIN_Z =  0.5;
        	MAX_Z =  1.5;
		*/

                //A wall, to round it           
                //obstacles.push_back(Rectangle_3D(   1, 0 , 1, 0.05,1,2));

                //obstacles.push_back(Rectangle_3D(-1.25,    0, 1.5, 2,  0.5,   3));
		
		
		if (params::map != "Map2_3D" && 
		    params::map != "Map3_3D" &&
		    params::map != "Map4_3D" &&
		    params::map != "Map5_3D")
		{
                  //NOTE: 
		  //The first obstacle represents the floor, but a litle bit up, because the
		  //algorithm tend to take points in the low limit of the working space, and these
		  //points have strange Z velocities. To avoid this, the floor will be treated
		  //as an obstacle. The floor is calculate in such a way that its height, added with
		  //the safety distance coincide almost with the limit of working space.
		  //In the SVG file created with the nodes this obstacle won't be displayed, in order to
		  //avoid the oclusion of the other obstacles.
		  //The second obstacle represents the roof, with the same explanation.

		  //ground floor and roof
		  if (params::map == "Map5" ||
		      params::map == "Map6")
		  {
		    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 0.3, 5.0,  6.0,   0.05));
                    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 2.4, 5.0,  6.0,   0.05));
		
		    //Walls smaller
		    //obstacles.push_back(Rectangle_3D( 1.0,  0.0, 1.5,   0.05,  4.0,   3.0));                
                    //obstacles.push_back(Rectangle_3D( -1.0,  0.0, 1.5,   0.05,  4.0,  3.0));
                    //obstacles.push_back(Rectangle_3D( 0.0,  2.0, 1.5,  2.0,  0.05,   3.0));
                    //obstacles.push_back(Rectangle_3D( 0.0,  -2.0, 1.5, 2.0,  0.05,   3.0));
		    obstacles.push_back(Rectangle_3D( 2.2,   0.0, 1.5, 0.05, 6.0,   3));
                    obstacles.push_back(Rectangle_3D(-2.2,   0.0, 1.5, 0.05, 6.0,   3));
                    obstacles.push_back(Rectangle_3D( 0.0,   2.7, 1.5, 5.0,  0.05,  3));
                    obstacles.push_back(Rectangle_3D( 0.0,  -2.7, 1.5, 5.0,  0.05,  3));


		  }
		  else
		  {
		    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 0.3, 5.0,  6.0,   0.05));
		    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 1.7, 5.0,  6.0,   0.05));
	          

                    //Mocap walls, small maps, small mocap
                    obstacles.push_back(Rectangle_3D( 2.2,   0.0, 1.5, 0.05, 6.0,   3));
                    obstacles.push_back(Rectangle_3D(-2.2,   0.0, 1.5, 0.05, 6.0,   3));
                    obstacles.push_back(Rectangle_3D( 0.0,   2.7, 1.5, 5.0,  0.05,  3));
                    obstacles.push_back(Rectangle_3D( 0.0,  -2.7, 1.5, 5.0,  0.05,  3));
		  }

		  /*
		  //Mocap walls, small maps, big mocap
		  obstacles.push_back(Rectangle_3D( 2.7,   0.0, 1.5, 0.05, 6.0,   3));	
		  obstacles.push_back(Rectangle_3D(-2.7,   0.0, 1.5, 0.05, 6.0,   3));
		  obstacles.push_back(Rectangle_3D( 0.0,   3.2, 1.5, 5.0,  0.05,  3));
		  obstacles.push_back(Rectangle_3D( 0.0,  -3.2, 1.5, 5.0,  0.05,  3));
		  */
		}
		else
		{
		  //Because Map5_3D is three heights
		  if (params::map == "Map5_3D")
		  {
                    //ground floor and roof
                    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 0.3, 10.0, 12.0,   0.05));
                    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 4.3, 10.0, 12.0,   0.05));
                    //bigger walls, for simulation only         
                    obstacles.push_back(Rectangle_3D( 5.2,   0.0, 2.25, 0.05, 12.0,   4.5));
                    obstacles.push_back(Rectangle_3D(-5.2,   0.0, 2.25, 0.05, 12.0,   4.5));
                    obstacles.push_back(Rectangle_3D( 0.0,   6.2, 2.25, 10.0,  0.05,  4.5));
                    obstacles.push_back(Rectangle_3D( 0.0,  -6.2, 2.25, 10.0,  0.05,  4.5));
		  }
		  else
		  {
		    //ground floor and roof
		    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 0.3, 10.0, 12.0,   0.05));
		    obstacles.push_back(Rectangle_3D( 0.0,   0.0, 2.7, 10.0, 12.0,   0.05));
		    //bigger walls, for simulation only		
		    obstacles.push_back(Rectangle_3D( 5.2,   0.0, 1.5, 0.05, 12.0,   3));  
                    obstacles.push_back(Rectangle_3D(-5.2,   0.0, 1.5, 0.05, 12.0,   3));
                    obstacles.push_back(Rectangle_3D( 0.0,   6.2, 1.5, 10.0,  0.05,  3));
                    obstacles.push_back(Rectangle_3D( 0.0,  -6.2, 1.5, 10.0,  0.05,  3));
  		  }
		}

		if (params::map == "Map1")
                {
                obstacles.push_back(Rectangle_3D( 1.25,    1, 1.5, 2.5, 0.05,   3));
		obstacles.push_back(Rectangle_3D(-1.25,    1, 1.5, 0.5,  0.5,   3));
		obstacles.push_back(Rectangle_3D(    0,   -1, 1.5, 2.5, 0.05,   3));
		}
                
		 
                if (params::map == "Map2")
  		{
                obstacles.push_back(Rectangle_3D( 1.25,    1, 1.5, 2.5, 0.2,   3));
                obstacles.push_back(Rectangle_3D(-1.25,    1, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D(-1.25,   -1, 1.5, 2.5, 0.2,   3));
                obstacles.push_back(Rectangle_3D( 1.25,   -1, 1.5, 0.5,  0.5,   3));
		}

                if (params::map == "Map3")
                {
                obstacles.push_back(Rectangle_3D( 1.25,    1, 1.5, 2.5, 0.2,   3));
                obstacles.push_back(Rectangle_3D(-1.25,    1, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D(-1.25,   -1, 1.5, 2.5, 0.2,   3));
                obstacles.push_back(Rectangle_3D( 1.25,   -1, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D( 0,       0, 1.5, 1.0, 0.05,   3));
                }

                if (params::map == "Map4")
                {
                obstacles.push_back(Rectangle_3D( 0,       0, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D( 0,       2, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D( 0,      -2, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D( 1.25,    1, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D(-1.25,    1, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D( 1.25,   -1, 1.5, 0.5,  0.5,   3));
                obstacles.push_back(Rectangle_3D(-1.25,   -1, 1.5, 0.5,  0.5,   3));
                }
                
		if (params::map == "Map5")
                {
		//Vertical windows
                obstacles.push_back(Rectangle_3D( 1.3,  0.0, 1.5,   1.4,  0.05,   3));
                obstacles.push_back(Rectangle_3D(-1.3,  0.0, 1.5,   1.4,  0.05,   3));
                obstacles.push_back(Rectangle_3D(-0.3,  0.0, 0.4,  0.6,  0.05,   0.8)); 
                obstacles.push_back(Rectangle_3D(-0.3,  0.0, 2.2,  0.6,  0.05,   1.6));
		obstacles.push_back(Rectangle_3D(0.3,  0.0, 0.8,  0.6,  0.05,   1.6));
                obstacles.push_back(Rectangle_3D(0.3,  0.0, 2.6,  0.6,  0.05,   0.8));

		//Floor + horizontal window
		obstacles.push_back(Rectangle_3D(0.0,  0.55, 1.5,   4.0,  3.9,   0.05));
		obstacles.push_back(Rectangle_3D(1.15, -1.7, 1.5,   1.7,  0.6,   0.05));
		obstacles.push_back(Rectangle_3D(-1.15, -1.7, 1.5,  1.7,  0.6,   0.05));
                }

                if (params::map == "Map6")
                {
                //Vertical windows
                obstacles.push_back(Rectangle_3D( 1.3,  0.0, 1.5,   1.4,  0.05,   3));
                obstacles.push_back(Rectangle_3D(-1.3,  0.0, 1.5,   1.4,  0.05,   3));
                obstacles.push_back(Rectangle_3D(-0.3,  0.0, 0.4,  0.6,  0.05,   0.8));
                obstacles.push_back(Rectangle_3D(-0.3,  0.0, 2.2,  0.6,  0.05,   1.6));
                obstacles.push_back(Rectangle_3D(0.3,  0.0, 0.8,  0.6,  0.05,   1.6));
                obstacles.push_back(Rectangle_3D(0.3,  0.0, 2.6,  0.6,  0.05,   0.8));

                //Floor + horizontal window
                obstacles.push_back(Rectangle_3D(0.0,  0.55, 1.5,   4.0,  3.9,   0.05));
                obstacles.push_back(Rectangle_3D(1.15, -1.7, 1.5,   1.7,  0.6,   0.05));
                obstacles.push_back(Rectangle_3D(-1.15, -1.7, 1.5,  1.7,  0.6,   0.05));

		//First Room Obstacles
		obstacles.push_back(Rectangle_3D(-1.0,  0.5, 0.75,  0.05,  1.0,   1.5));
                obstacles.push_back(Rectangle_3D(0.0, 1.0, 0.75,   2.0,  0.05,   1.5));

                //Second Room Obstacles
                obstacles.push_back(Rectangle_3D(1.0,  -1.0, 1.25,  0.05,  0.05,   2.5));
                obstacles.push_back(Rectangle_3D(-1.0,  -1.0, 1.25,  0.05,  0.05,   2.5));
		obstacles.push_back(Rectangle_3D(0.0,  -1.0, 1.25,  0.05,  0.05,   2.5));
		obstacles.push_back(Rectangle_3D(0.5,  -1.0, 0.5,  0.35,  0.25,   1.0));
		obstacles.push_back(Rectangle_3D(-0.5,  -1.0, 1.1,  1.0,  0.05,   0.05));

		//Third Room Obstacles
		obstacles.push_back(Rectangle_3D(0.0,  -1.0, 2.0,  2.0,  0.05,   1.0));
                }

		if (params::map == "Map1_3D")
                {
		//a
                obstacles.push_back(Rectangle_3D( 1.6,   1.5, 1.5, 1.8,  0.05,  3));
		//b
                obstacles.push_back(Rectangle_3D( 0.7,   1.75,1.5,0.05,  0.5,   3));
		//c
                obstacles.push_back(Rectangle_3D(-0.9,   1.5, 1.5, 1.8,  0.05,  3));
   		//d
                obstacles.push_back(Rectangle_3D(-1.8,  0.5, 1.5,0.05,  2.0,   3));
		//e
                obstacles.push_back(Rectangle_3D(-1.8, -1.35, 1.5,0.05,  0.3,   3));
		//f
                obstacles.push_back(Rectangle_3D(-1.3, -1.9, 1.5, 0.05,  0.8,   3));
		//g
                obstacles.push_back(Rectangle_3D(-1.55, -1.5, 1.5, 0.5, 0.05,   3));
		//h
		obstacles.push_back(Rectangle_3D(-0.4, -2.3, 1.5, 1.8, 0.05,   3));
                //i
                obstacles.push_back(Rectangle_3D(2.25, 0.5, 1.5, 0.5, 0.05,   3));
                //j
                obstacles.push_back(Rectangle_3D(0.35, 0.5, 1.5, 1.9, 0.05,   3));
                //k
                obstacles.push_back(Rectangle_3D(-1.55, 0.5, 1.5, 0.5, 0.05,   3));
                //l
                obstacles.push_back(Rectangle_3D(-0.6, 0.25, 1.5, 0.05, 0.5,   3));
                //m
                obstacles.push_back(Rectangle_3D(-0.6, -1.1, 1.5, 0.05, 0.8,   3));
                //n
                obstacles.push_back(Rectangle_3D(1.2, -2.25, 1.5, 0.05, 1.5,   3));
                //o
                obstacles.push_back(Rectangle_3D(0.3, -1.5, 1.5, 1.8, 0.05,   3));
                //p
                obstacles.push_back(Rectangle_3D(0.5, -0.5, 1.5, 0.05, 0.6,   3));
                //q
                obstacles.push_back(Rectangle_3D(1.5, -0.5, 1.5, 2.0, 0.05,   3));
                }

		if (params::map == "Map2_3D")
		{
		//10
		obstacles.push_back(Rectangle_3D(0, 4.5, 1.5, 2.0, 0.05,   3));
                //11
                obstacles.push_back(Rectangle_3D(3, 4.5, 1.5, 2.0, 0.05,   3));
                //12
                obstacles.push_back(Rectangle_3D(1.5, 4.5, 0.5, 1.0, 0.05,   1));
                //13
                obstacles.push_back(Rectangle_3D(1.5, 4.5, 2.5, 1.0, 0.05,   1));
                //14
                obstacles.push_back(Rectangle_3D(-3, 4.5, 1.5, 2.0, 0.05,   3));
                //15
                obstacles.push_back(Rectangle_3D(-1.5, 4.5, 0.8, 1.0, 0.05, 1.6));
                //16
                obstacles.push_back(Rectangle_3D(-1.5, 4.5, 2.8, 1.0, 0.05,   0.4));
                //17
                obstacles.push_back(Rectangle_3D(4.0, 3.5, 1.5, 0.05, 2.0,  3));
                //18
                obstacles.push_back(Rectangle_3D(4.25, 0.5, 1.5, 1.5, 0.05,   3));
                //19
                obstacles.push_back(Rectangle_3D(1.5, 0.5, 1.5, 2.0, 0.05,   3));
                //20
                obstacles.push_back(Rectangle_3D(3.0, 0.5, 0.8, 1.0, 0.05,  1.6));
                //21
                obstacles.push_back(Rectangle_3D(3.0, 0.5, 2.8, 1.0, 0.05,  0.4));
                //22
                obstacles.push_back(Rectangle_3D(0.5, 1.0, 1.5, 0.05, 1.0,  3.0));
                //23
                obstacles.push_back(Rectangle_3D(1.25, 1.5, 1.5, 1.5, 0.05,  3.0));
                //24
                obstacles.push_back(Rectangle_3D(1.25, 3.5, 1.5, 1.5, 0.05,  3.0));
                //25
                obstacles.push_back(Rectangle_3D(2.0, 1.75, 1.5, 0.05, 0.5,  3.0));
                //26
                obstacles.push_back(Rectangle_3D(2.0, 3.25, 1.5, 0.05, 0.5,  3.0));
		//27
                obstacles.push_back(Rectangle_3D(2.0, 2.5, 2.2, 0.05, 1.0,  1.6));
                //28
                obstacles.push_back(Rectangle_3D(2.0, 2.5, 0.2, 0.05, 1.0,  0.4));
                //29
                obstacles.push_back(Rectangle_3D(-1.25, 3.5, 1.5, 1.5, 0.05,  3.0));
                //30
                obstacles.push_back(Rectangle_3D(0, 3.5, 0.5, 1.0, 0.05,  1.0));
                //31
                obstacles.push_back(Rectangle_3D(0, 3.5, 2.5, 1.0, 0.05,  1.0));
                //32
                obstacles.push_back(Rectangle_3D(-2.0, 2.5, 1.5, 0.05, 2.0,  3.0));
                //33
                obstacles.push_back(Rectangle_3D(-1.25, 1.5, 1.5, 1.5, 0.05,  3.0));
                //34
                obstacles.push_back(Rectangle_3D(-0.5, 1.0, 1.5, 0.05, 1.0,  3.0));
                //35
                obstacles.push_back(Rectangle_3D(-1.5, 0.5, 1.5, 2.0, 0.05,  3.0));
                //36
                obstacles.push_back(Rectangle_3D(-4.5, 3.5, 1.5, 1.0, 0.05,  3.0));
                //37
                obstacles.push_back(Rectangle_3D(-3.0, 3.0, 1.5, 0.05, 1.0,  3.0));
                //38
                obstacles.push_back(Rectangle_3D(-4.0, 1.5, 1.5, 0.05, 2.0,  3.0));
                //39
                obstacles.push_back(Rectangle_3D(-3.5, 2.5, 0.2, 1.0, 0.05,  0.4));
                //40
                obstacles.push_back(Rectangle_3D(-3.5, 2.5, 2.2, 1.0, 0.05,  1.6));
                //41
                obstacles.push_back(Rectangle_3D(3.0, -1.0, 1.5, 4.0, 0.05,  3.0));
                //42
                obstacles.push_back(Rectangle_3D(-3.0, -1.0, 1.5, 4.0, 0.05,  3.0));
		//43
                obstacles.push_back(Rectangle_3D(1.0, -1.5, 1.5, 0.05, 1.0,  3.0));
                //44
                obstacles.push_back(Rectangle_3D(-1.0, -1.5, 1.5, 0.05, 1.0,  3.0));
                //45
                obstacles.push_back(Rectangle_3D(1.0, -4.5, 1.5, 0.05, 3.0,  3.0));
                //46
                obstacles.push_back(Rectangle_3D(-1.0, -4.5, 1.5, 0.05, 3.0,  3.0));
                //47
                obstacles.push_back(Rectangle_3D(-2.5, -2.5, 1.5, 0.05, 3.0,  3.0));
                //48
                obstacles.push_back(Rectangle_3D(-3.5, -3.0, 1.5, 0.05, 2.0,  3.0));
                //49
                obstacles.push_back(Rectangle_3D(-3.5, -5.5, 1.5, 0.05, 1.0,  3.0));
                //50
                obstacles.push_back(Rectangle_3D(-3.5, -4.5, 2.8, 0.05, 1.0,  0.4));
                //51
                obstacles.push_back(Rectangle_3D(-3.5, -4.5, 0.8, 0.05, 1.0,  1.6));
                //52
                obstacles.push_back(Rectangle_3D(1.5, -3.0, 1.5, 1.0, 0.05,  3.0));
                //53
                obstacles.push_back(Rectangle_3D(4.5, -3.0, 1.5, 1.0, 0.05,  3.0));
                //54
                obstacles.push_back(Rectangle_3D(3.0, -2.0, 1.5, 2.0, 0.05,  3.0));
                //55
                obstacles.push_back(Rectangle_3D(3.5, -4.0, 1.5, 3.0, 0.05,  3.0));
                //56
                obstacles.push_back(Rectangle_3D(3.0, -2.5, 1.5, 0.05, 1.0,  3.0));
		}


		if (params::map == "Map3_3D")
		{
		//GROUND OBSTACLES 1--

		//100
                obstacles.push_back(Rectangle_3D(-1.0, 5.5, 0.75, 0.05, 1.0,  1.5));
                //101
                obstacles.push_back(Rectangle_3D(-1.0, 3.0, 0.75, 0.05, 2.0,  1.5));
		//102
                obstacles.push_back(Rectangle_3D(-2.0, 2.0, 0.75, 2.0, 0.05,  1.5));
                //103
                obstacles.push_back(Rectangle_3D(-4.5, 2.0, 0.75, 1.0, 0.05,  1.5));
                //104
                obstacles.push_back(Rectangle_3D(1.0, 5.0, 0.75, 4.0, 0.05,  1.5));
                //105
                obstacles.push_back(Rectangle_3D(-0.5, 4.0, 0.75, 1.0, 0.05,  1.5));
                //106
                obstacles.push_back(Rectangle_3D(1.5, 4.0, 0.75, 1.0, 0.05,  1.5));
                //107
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 0.75, 1.0, 0.05,  1.5));
                //108
                obstacles.push_back(Rectangle_3D(0.5, 3.0, 0.75, 1.0, 0.05,  1.5));
                //109
                obstacles.push_back(Rectangle_3D(3.0, 4.0, 0.75, 0.05, 2.0,  1.5));
                //110
                obstacles.push_back(Rectangle_3D(0.0, 2.5, 0.75, 0.05, 1.0,  1.5));
                //111
                obstacles.push_back(Rectangle_3D(3.0, 2.0, 0.75, 4.0, 0.05,  1.5));
                //112
                obstacles.push_back(Rectangle_3D(-4.0, 1.5, 0.75, 0.05, 1.0,  1.5));
                //113
                obstacles.push_back(Rectangle_3D(-3.0, 1.5, 0.75, 0.05, 1.0,  1.5));
                //114
                obstacles.push_back(Rectangle_3D(-4.5, 1.0, 0.75, 1.0, 0.05,  1.5));
                //115
                obstacles.push_back(Rectangle_3D(-1.5, 1.0, 0.75, 3.0, 0.05,  1.5));
                //116
                obstacles.push_back(Rectangle_3D(-1.0, -2.0, 0.75, 2.0, 0.05,  1.5));
                //117
                obstacles.push_back(Rectangle_3D(-4.0, -2.0, 0.75, 2.0, 0.05,  1.5));
                //118
                obstacles.push_back(Rectangle_3D(0.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //119
                obstacles.push_back(Rectangle_3D(-1.0, 0.0, 0.75, 0.5, 0.5,  1.5));
                //120
                obstacles.push_back(Rectangle_3D(-3.0, 0.0, 0.75, 0.5, 0.5,  1.5));
                //121
                obstacles.push_back(Rectangle_3D(-2.0, -1.0, 0.75, 0.5, 0.5,  1.5));
                //122
                obstacles.push_back(Rectangle_3D(-4.0, -1.0, 0.75, 0.5, 0.5,  1.5));
                //123
                obstacles.push_back(Rectangle_3D(-1.0, -3.0, 0.75, 8.0, 0.05,  1.5));
                //124
                obstacles.push_back(Rectangle_3D(3.0, -3.5, 0.75, 0.05, 1.0,  1.5));
                //125
                obstacles.push_back(Rectangle_3D(3.0, -5.5, 0.75, 0.05, 1.0,  1.5));
                //126
                obstacles.push_back(Rectangle_3D(2.0, -5.0, 0.75, 0.05, 2.0,  1.5));
                //127
                obstacles.push_back(Rectangle_3D(1.0, -4.0, 0.75, 0.05, 2.0,  1.5));
                //128
                obstacles.push_back(Rectangle_3D(-3.0, -5.0, 0.75, 0.05, 2.0,  1.5));
                //129
                obstacles.push_back(Rectangle_3D(0.0, -5.0, 0.75, 2.0, 0.05,  1.5));
                //130
                obstacles.push_back(Rectangle_3D(-2.0, -4.0, 0.75, 2.0, 0.05,  1.5));
                //131
                obstacles.push_back(Rectangle_3D(1.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //132
                obstacles.push_back(Rectangle_3D(4.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //133
                obstacles.push_back(Rectangle_3D(3.0, 0.0, 0.75, 0.05, 2.0,  1.5));
                //134
                obstacles.push_back(Rectangle_3D(2.0, -0.5, 0.75, 0.05, 1.0,  1.5));
                //135
                obstacles.push_back(Rectangle_3D(2.0, 1.0, 0.75, 2.0, 0.05,  1.5));
                //136
                obstacles.push_back(Rectangle_3D(2.5, -2.0, 0.75, 3.0, 0.05,  1.5));
                //137
                obstacles.push_back(Rectangle_3D(2.5, -1.0, 0.75, 1.0, 0.05,  1.5));

		//FIRST FLOOR OBSTACLES 2--

		//201
                obstacles.push_back(Rectangle_3D(-1.5, 4.0, 2.25, 7.0, 0.05,  1.5));
                //202
                obstacles.push_back(Rectangle_3D(-1.5, -3.0, 2.25, 7.0, 0.05,  1.5));
                //203
                obstacles.push_back(Rectangle_3D(1.5, 2.0, 2.25, 7.0, 0.05,  1.5));
                //204
                obstacles.push_back(Rectangle_3D(-2.0, 0.0, 2.25, 0.05, 4.0,  1.5));
                //205
                obstacles.push_back(Rectangle_3D(0.0, -1.0, 2.25, 0.05, 4.0,  1.5));
                //206
                obstacles.push_back(Rectangle_3D(-1.0, -5.0, 2.25, 0.05, 2.0,  1.5));
                //207
                obstacles.push_back(Rectangle_3D(0.0, 4.0, 2.25, 0.05, 2.0,  1.5));
                //208
                obstacles.push_back(Rectangle_3D(-4.0, 0.0, 2.25, 2.0, 0.05,  1.5));
                //209
                obstacles.push_back(Rectangle_3D(4.0, -3.0, 2.25, 2.0, 0.05,  1.5));


                //FIRST FLOOR           

                //F1
                obstacles.push_back(Rectangle_3D(-4.75, 0.0, 1.5, 0.5, 12.0,  0.05));
                //F2
                obstacles.push_back(Rectangle_3D(-3.0, 0.0, 1.5, 1.0, 12.0,  0.05));
                //F3
                obstacles.push_back(Rectangle_3D(0.25, 0.0, 1.5, 3.5, 12.0,  0.05));
                //F4
                obstacles.push_back(Rectangle_3D(3.25, 0.0, 1.5, 0.5, 12.0,  0.05));
                //F5
                obstacles.push_back(Rectangle_3D(4.75, 0.0, 1.5, 0.5, 12.0,  0.05));
                //C1_A
                obstacles.push_back(Rectangle_3D(-4.0, 5.75, 1.5, 1.0, 0.5,  0.05));
                //C1_B
                obstacles.push_back(Rectangle_3D(-4.0, 0.25, 1.5, 1.0, 8.5,  0.05));
                //C1_C
                obstacles.push_back(Rectangle_3D(-4.0, -5.55, 1.5, 1.0, 1.0,  0.05));
                //C2_A
                obstacles.push_back(Rectangle_3D(-2.0, 4.75, 1.5, 1.0, 2.5,  0.05));
                //C2_B
                obstacles.push_back(Rectangle_3D(-2.0, -1.75, 1.5, 1.0, 8.5,  0.05));
                //C3_A
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 1.5, 1.0, 6.0,  0.05));
                //C3_B
                obstacles.push_back(Rectangle_3D(2.5, -2.5, 1.5, 1.0, 3.0,  0.05));
                //C3_C
                obstacles.push_back(Rectangle_3D(2.5, -5.5, 1.5, 1.0, 1.0,  0.05));
                //C4_A
                obstacles.push_back(Rectangle_3D(4.0, 5.25, 1.5, 1.0, 1.5,  0.05));
                //C3_B
                obstacles.push_back(Rectangle_3D(4.0, -1.25, 1.5, 1.0, 9.5,  0.05));
	
		}

		if (params::map == "Map4_3D")
		{
		//GROUND OBSTACLES 1--

		//100
                obstacles.push_back(Rectangle_3D(-1.0, 5.5, 0.75, 0.05, 1.0,  1.5));
                //101
                obstacles.push_back(Rectangle_3D(-1.0, 3.0, 0.75, 0.05, 2.0,  1.5));
		//102
                obstacles.push_back(Rectangle_3D(-2.0, 2.0, 0.75, 2.0, 0.05,  1.5));
                //103
                obstacles.push_back(Rectangle_3D(-4.5, 2.0, 0.75, 1.0, 0.05,  1.5));
                //104
                obstacles.push_back(Rectangle_3D(1.0, 5.0, 0.75, 4.0, 0.05,  1.5));
                //105
                obstacles.push_back(Rectangle_3D(-0.5, 4.0, 0.75, 1.0, 0.05,  1.5));
                //106
                obstacles.push_back(Rectangle_3D(1.5, 4.0, 0.75, 1.0, 0.05,  1.5));
                //107
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 0.75, 1.0, 0.05,  1.5));
                //108
                obstacles.push_back(Rectangle_3D(0.5, 3.0, 0.75, 1.0, 0.05,  1.5));
                //109
                obstacles.push_back(Rectangle_3D(3.0, 4.0, 0.75, 0.05, 2.0,  1.5));
                //110
                obstacles.push_back(Rectangle_3D(0.0, 2.5, 0.75, 0.05, 1.0,  1.5));
                //111
                obstacles.push_back(Rectangle_3D(3.0, 2.0, 0.75, 4.0, 0.05,  1.5));
                //112
                obstacles.push_back(Rectangle_3D(-4.0, 1.5, 0.75, 0.05, 1.0,  1.5));
                //113
                obstacles.push_back(Rectangle_3D(-3.0, 1.5, 0.75, 0.05, 1.0,  1.5));
                //114
                obstacles.push_back(Rectangle_3D(-4.5, 1.0, 0.75, 1.0, 0.05,  1.5));
                //115
                obstacles.push_back(Rectangle_3D(-1.5, 1.0, 0.75, 3.0, 0.05,  1.5));
                //116
                obstacles.push_back(Rectangle_3D(-1.0, -2.0, 0.75, 2.0, 0.05,  1.5));
                //117
                obstacles.push_back(Rectangle_3D(-4.0, -2.0, 0.75, 2.0, 0.05,  1.5));
                //118
                obstacles.push_back(Rectangle_3D(0.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //119
                obstacles.push_back(Rectangle_3D(-1.0, 0.0, 0.75, 0.5, 0.5,  1.5));
                //120
                obstacles.push_back(Rectangle_3D(-3.0, 0.0, 0.75, 0.5, 0.5,  1.5));
                //121
                obstacles.push_back(Rectangle_3D(-2.0, -1.0, 0.75, 0.5, 0.5,  1.5));
                //122
                obstacles.push_back(Rectangle_3D(-4.0, -1.0, 0.75, 0.5, 0.5,  1.5));
                //123
                obstacles.push_back(Rectangle_3D(-1.0, -3.0, 0.75, 8.0, 0.05,  1.5));
                //124
                obstacles.push_back(Rectangle_3D(3.0, -3.5, 0.75, 0.05, 1.0,  1.5));
                //125
                obstacles.push_back(Rectangle_3D(3.0, -5.5, 0.75, 0.05, 1.0,  1.5));
                //126
                obstacles.push_back(Rectangle_3D(2.0, -5.0, 0.75, 0.05, 2.0,  1.5));
                //127
                obstacles.push_back(Rectangle_3D(1.0, -4.0, 0.75, 0.05, 2.0,  1.5));
                //128
                obstacles.push_back(Rectangle_3D(-3.0, -5.0, 0.75, 0.05, 2.0,  1.5));
                //129
                obstacles.push_back(Rectangle_3D(0.0, -5.0, 0.75, 2.0, 0.05,  1.5));
                //130
                obstacles.push_back(Rectangle_3D(-2.0, -4.0, 0.75, 2.0, 0.05,  1.5));
                //131
                obstacles.push_back(Rectangle_3D(1.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //132
                obstacles.push_back(Rectangle_3D(4.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //133
                obstacles.push_back(Rectangle_3D(3.0, -0.5, 0.75, 0.05, 1.0,  1.5));
                //134
                obstacles.push_back(Rectangle_3D(2.0, -0.5, 0.75, 0.05, 1.0,  1.5));
                //135
                obstacles.push_back(Rectangle_3D(1.5, 1.0, 0.75, 1.0, 0.05,  1.5));
                //136
                obstacles.push_back(Rectangle_3D(2.5, -2.0, 0.75, 3.0, 0.05,  1.5));
                //137
                obstacles.push_back(Rectangle_3D(2.5, -1.0, 0.75, 1.0, 0.05,  1.5));


		//FIRST FLOOR OBSTACLES 2--


		//201
                obstacles.push_back(Rectangle_3D(-0.5, 5.5, 2.25, 8.0, 0.05,  1.5));
                //202
                obstacles.push_back(Rectangle_3D(-2.25, 4.5, 2.25, 4.5, 0.05,  1.5));
                //203
                obstacles.push_back(Rectangle_3D(-4.5, 5.0, 2.25, 0.05, 1.0,  1.5));
                //204
                obstacles.push_back(Rectangle_3D(0.0, 4.0, 2.25, 0.05, 1.0,  1.5));
                //205
                obstacles.push_back(Rectangle_3D(-1.25, 3.5, 2.25, 2.5, 0.05,  1.5));
                //206
                obstacles.push_back(Rectangle_3D(-2.25, 2.5, 2.25, 4.5, 0.05,  1.5));
                //207
                obstacles.push_back(Rectangle_3D(-2.5, 3.0, 2.25, 0.05, 1.0,  1.5));
                //208
                obstacles.push_back(Rectangle_3D(-1.25, 1.5, 2.25, 4.5, 0.05,  1.5));
                //209
                obstacles.push_back(Rectangle_3D(-1.25, 0.5, 2.25, 2.5, 0.05,  1.5));
                //210
                obstacles.push_back(Rectangle_3D(-2.25, -2.0, 2.25, 4.5, 0.05,  1.5));
                //211
                obstacles.push_back(Rectangle_3D(-1.75, -3.0, 2.25, 3.5, 0.05,  1.5));
                //212
                obstacles.push_back(Rectangle_3D(-4.0, -5.0, 2.25, 1.0, 0.05,  1.5));
                //213
                obstacles.push_back(Rectangle_3D(-4.5, -1.25, 2.25, 0.05, 7.5,  1.5));
                //214
                obstacles.push_back(Rectangle_3D(-3.5, 0.25, 2.25, 0.05, 2.5,  1.5));
                //215
                obstacles.push_back(Rectangle_3D(-2.5, -0.75, 2.25, 0.05, 2.5,  1.5));
                //216
                obstacles.push_back(Rectangle_3D(0.0, -0.75, 2.25, 0.05, 2.5,  1.5));
                //217
                obstacles.push_back(Rectangle_3D(-3.5, -4.0, 2.25, 0.05, 2.0,  1.5));
                //218
                obstacles.push_back(Rectangle_3D(0.0, -4.5, 2.25, 0.05, 3.0,  1.5));
                //219
                obstacles.push_back(Rectangle_3D(1.0, -3.0, 2.25, 0.05, 4.0,  1.5));
                //220
                obstacles.push_back(Rectangle_3D(2.0, -4.0, 2.25, 0.05, 2.0,  1.5));
                //221
                obstacles.push_back(Rectangle_3D(4.0, -4.5, 2.25, 0.05, 3.0,  1.5));
                //222
                obstacles.push_back(Rectangle_3D(3.0, -4.5, 2.25, 0.05, 1.0,  1.5));
                //223
                obstacles.push_back(Rectangle_3D(2.0, -5.0, 2.25, 2.0, 0.05,  1.5));
                //224
                obstacles.push_back(Rectangle_3D(3.0, -3.0, 2.25, 2.0, 0.05,  1.5));
                //225
                obstacles.push_back(Rectangle_3D(2.0, -1.0, 2.25, 2.0, 0.05,  1.5));
                //226
                obstacles.push_back(Rectangle_3D(2.0, 0.0, 2.25, 2.0, 0.05,  1.5));
                //227
                obstacles.push_back(Rectangle_3D(3.0, -0.5, 2.25, 0.05, 1.0,  1.5));
                //228
                obstacles.push_back(Rectangle_3D(1.0, 1.75, 2.25, 0.05, 3.5,  1.5));
                //229
                obstacles.push_back(Rectangle_3D(1.0, 5.0, 2.25, 0.05, 1.0,  1.5));
                //230
                obstacles.push_back(Rectangle_3D(2.0, 5.0, 2.25, 0.05, 1.0,  1.5));
                //231
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 2.25, 0.05, 1.0,  1.5));
                //232
                obstacles.push_back(Rectangle_3D(3.5, 4.5, 2.25, 0.05, 2.0,  1.5));
                //233
                obstacles.push_back(Rectangle_3D(4.5, 3.5, 2.25, 0.05, 2.0,  1.5));
                //234
                obstacles.push_back(Rectangle_3D(4.0, 4.5, 2.25, 1.0, 0.05,  1.5));
                //235
                obstacles.push_back(Rectangle_3D(1.5, 4.5, 2.25, 1.0, 0.05,  1.5));
                //236
                obstacles.push_back(Rectangle_3D(3.5, 2.5, 2.25, 2.0, 0.05,  1.5));
                //237
                obstacles.push_back(Rectangle_3D(1.75, 3.5, 2.25, 1.5, 0.05,  1.5));


                //FIRST FLOOR           

                //F1
                obstacles.push_back(Rectangle_3D(-4.75, 0.0, 1.5, 0.5, 12.0,  0.05));
                //F2
                obstacles.push_back(Rectangle_3D(-3.0, 0.0, 1.5, 1.0, 12.0,  0.05));
                //F3
                obstacles.push_back(Rectangle_3D(0.25, 0.0, 1.5, 3.5, 12.0,  0.05));
                //F4
                obstacles.push_back(Rectangle_3D(3.25, 0.0, 1.5, 0.5, 12.0,  0.05));
                //F5
                obstacles.push_back(Rectangle_3D(4.75, 0.0, 1.5, 0.5, 12.0,  0.05));
                //C1_A
                obstacles.push_back(Rectangle_3D(-4.0, 5.75, 1.5, 1.0, 0.5,  0.05));
                //C1_B
                obstacles.push_back(Rectangle_3D(-4.0, 0.25, 1.5, 1.0, 8.5,  0.05));
                //C1_C
                obstacles.push_back(Rectangle_3D(-4.0, -5.55, 1.5, 1.0, 1.0,  0.05));
                //C2_A
                obstacles.push_back(Rectangle_3D(-2.0, 4.75, 1.5, 1.0, 2.5,  0.05));
                //C2_B
                obstacles.push_back(Rectangle_3D(-2.0, -1.75, 1.5, 1.0, 8.5,  0.05));
                //C3_A
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 1.5, 1.0, 6.0,  0.05));
                //C3_B
                obstacles.push_back(Rectangle_3D(2.5, -2.5, 1.5, 1.0, 3.0,  0.05));
                //C3_C
                obstacles.push_back(Rectangle_3D(2.5, -5.5, 1.5, 1.0, 1.0,  0.05));
                //C4_A
                obstacles.push_back(Rectangle_3D(4.0, 5.25, 1.5, 1.0, 1.5,  0.05));
                //C3_B
                obstacles.push_back(Rectangle_3D(4.0, -1.25, 1.5, 1.0, 9.5,  0.05));
	
		}

		if (params::map == "Map5_3D")
		{
		//GROUND OBSTACLES 1--

		//100
                obstacles.push_back(Rectangle_3D(-1.0, 5.5, 0.75, 0.05, 1.0,  1.5));
                //101
                obstacles.push_back(Rectangle_3D(-1.0, 3.0, 0.75, 0.05, 2.0,  1.5));
		//102
                obstacles.push_back(Rectangle_3D(-2.0, 2.0, 0.75, 2.0, 0.05,  1.5));
                //103
                obstacles.push_back(Rectangle_3D(-4.5, 2.0, 0.75, 1.0, 0.05,  1.5));
                //104
                obstacles.push_back(Rectangle_3D(1.0, 5.0, 0.75, 4.0, 0.05,  1.5));
                //105
                obstacles.push_back(Rectangle_3D(-0.5, 4.0, 0.75, 1.0, 0.05,  1.5));
                //106
                obstacles.push_back(Rectangle_3D(1.5, 4.0, 0.75, 1.0, 0.05,  1.5));
                //107
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 0.75, 1.0, 0.05,  1.5));
                //108
                obstacles.push_back(Rectangle_3D(0.5, 3.0, 0.75, 1.0, 0.05,  1.5));
                //109
                obstacles.push_back(Rectangle_3D(3.0, 4.0, 0.75, 0.05, 2.0,  1.5));
                //110
                obstacles.push_back(Rectangle_3D(0.0, 2.5, 0.75, 0.05, 1.0,  1.5));
                //111
                obstacles.push_back(Rectangle_3D(3.0, 2.0, 0.75, 4.0, 0.05,  1.5));
                //112
                obstacles.push_back(Rectangle_3D(-4.0, 1.5, 0.75, 0.05, 1.0,  1.5));
                //113
                obstacles.push_back(Rectangle_3D(-3.0, 1.5, 0.75, 0.05, 1.0,  1.5));
                //114
                obstacles.push_back(Rectangle_3D(-4.5, 1.0, 0.75, 1.0, 0.05,  1.5));
                //115
                obstacles.push_back(Rectangle_3D(-1.5, 1.0, 0.75, 3.0, 0.05,  1.5));
                //116
                obstacles.push_back(Rectangle_3D(-1.0, -2.0, 0.75, 2.0, 0.05,  1.5));
                //117
                obstacles.push_back(Rectangle_3D(-4.0, -2.0, 0.75, 2.0, 0.05,  1.5));
                //118
                obstacles.push_back(Rectangle_3D(0.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //119
                obstacles.push_back(Rectangle_3D(-1.0, 0.0, 0.75, 0.5, 0.5,  1.5));
                //120
                obstacles.push_back(Rectangle_3D(-3.0, 0.0, 0.75, 0.5, 0.5,  1.5));
                //121
                obstacles.push_back(Rectangle_3D(-2.0, -1.0, 0.75, 0.5, 0.5,  1.5));
                //122
                obstacles.push_back(Rectangle_3D(-4.0, -1.0, 0.75, 0.5, 0.5,  1.5));
                //123
                obstacles.push_back(Rectangle_3D(-1.0, -3.0, 0.75, 8.0, 0.05,  1.5));
                //124
                obstacles.push_back(Rectangle_3D(3.0, -3.5, 0.75, 0.05, 1.0,  1.5));
                //125
                obstacles.push_back(Rectangle_3D(3.0, -5.5, 0.75, 0.05, 1.0,  1.5));
                //126
                obstacles.push_back(Rectangle_3D(2.0, -5.0, 0.75, 0.05, 2.0,  1.5));
                //127
                obstacles.push_back(Rectangle_3D(1.0, -4.0, 0.75, 0.05, 2.0,  1.5));
                //128
                obstacles.push_back(Rectangle_3D(-3.0, -5.0, 0.75, 0.05, 2.0,  1.5));
                //129
                obstacles.push_back(Rectangle_3D(0.0, -5.0, 0.75, 2.0, 0.05,  1.5));
                //130
                obstacles.push_back(Rectangle_3D(-2.0, -4.0, 0.75, 2.0, 0.05,  1.5));
                //131
                obstacles.push_back(Rectangle_3D(1.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //132
                obstacles.push_back(Rectangle_3D(4.0, -0.5, 0.75, 0.05, 3.0,  1.5));
                //133
                obstacles.push_back(Rectangle_3D(3.0, -0.5, 0.75, 0.05, 1.0,  1.5));
                //134
                obstacles.push_back(Rectangle_3D(2.0, -0.5, 0.75, 0.05, 1.0,  1.5));
                //135
                obstacles.push_back(Rectangle_3D(1.5, 1.0, 0.75, 1.0, 0.05,  1.5));
                //136
                obstacles.push_back(Rectangle_3D(2.5, -2.0, 0.75, 3.0, 0.05,  1.5));
                //137
                obstacles.push_back(Rectangle_3D(2.5, -1.0, 0.75, 1.0, 0.05,  1.5));


		//FIRST FLOOR OBSTACLES 2--


		//201
                obstacles.push_back(Rectangle_3D(-2.25, 5.5, 2.25, 4.5, 0.05,  1.5));
                //202
                obstacles.push_back(Rectangle_3D(-2.75, 1.5, 2.25, 3.5, 0.05,  1.5));
                //203
                obstacles.push_back(Rectangle_3D(-2.75, -3.0, 2.25, 3.5, 0.05,  1.5));
                //204
                obstacles.push_back(Rectangle_3D(0.25, -5.0, 2.25, 9.5, 0.05,  1.5));
                //205
                obstacles.push_back(Rectangle_3D(-3.25, -4.0, 2.25, 2.5, 0.05,  1.5));
                //206
                obstacles.push_back(Rectangle_3D(1.25, 2.5, 2.25, 5.5, 0.05,  1.5));
                //207
                obstacles.push_back(Rectangle_3D(1.0, 0.0, 2.25, 4.0, 0.05,  1.5));
                //208
                obstacles.push_back(Rectangle_3D(1.0, -1.0, 2.25, 4.0, 0.05,  1.5));
                //209
                obstacles.push_back(Rectangle_3D(-4.5, 3.5, 2.25, 0.05, 4.0,  1.5));
                //210
                obstacles.push_back(Rectangle_3D(-4.5, -4.0, 2.25, 0.05, 2.0,  1.5));
                //211
                obstacles.push_back(Rectangle_3D(-1.0, 0.75, 2.25, 0.05, 1.5,  1.5));
                //212
                obstacles.push_back(Rectangle_3D(-1.0, -3.0, 2.25, 0.05, 4.0,  1.5));
                //213
                obstacles.push_back(Rectangle_3D(0.0, 5.75, 2.25, 0.05, 0.5,  1.5));
                //214
                obstacles.push_back(Rectangle_3D(3.0, 0.25, 2.25, 0.05, 4.5,  1.5));
                //215
                obstacles.push_back(Rectangle_3D(0.0, 2.0, 2.25, 0.05, 1.0,  1.5));
                //216
                obstacles.push_back(Rectangle_3D(0.0, 0.25, 2.25, 0.05, 0.5,  1.5));
                //217
                obstacles.push_back(Rectangle_3D(-2.5, 2.5, 2.25, 0.05, 2.0,  1.5));
                //218
                obstacles.push_back(Rectangle_3D(-1.5, 3.0, 2.25, 0.05, 1.0,  1.5));
                //219
                obstacles.push_back(Rectangle_3D(-3.5, 5.0, 2.25, 0.05, 1.0,  1.5));
                //220
                obstacles.push_back(Rectangle_3D(-4.0, 4.5, 2.25, 1.0, 0.05,  1.5));
                //221
                obstacles.push_back(Rectangle_3D(-2.0, 3.5, 2.25, 1.0, 0.05,  1.5));
                //222
                obstacles.push_back(Rectangle_3D(4.5, 1.5, 2.25, 1.0, 0.05,  1.5));
                //223
                obstacles.push_back(Rectangle_3D(3.5, 0.5, 2.25, 1.0, 0.05,  1.5));
                //224
                obstacles.push_back(Rectangle_3D(4.5, -0.5, 2.25, 1.0, 0.05,  1.5));
                //225
                obstacles.push_back(Rectangle_3D(3.5, -1.5, 2.25, 1.0, 0.05,  1.5));
                //226
                obstacles.push_back(Rectangle_3D(4.5, -2.5, 2.25, 1.0, 0.05,  1.5));
                //227
                obstacles.push_back(Rectangle_3D(2.5, -4.0, 2.25, 3.0, 0.05,  1.5));
                //228
                obstacles.push_back(Rectangle_3D(1.0, -3.5, 2.25, 0.05, 3.0,  1.5));
                //229
                obstacles.push_back(Rectangle_3D(0.0, -2.5, 2.25, 0.05, 3.0,  1.5));
                //230
                obstacles.push_back(Rectangle_3D(2.5, 4.5, 2.25, 1.0, 2.0,  1.5));
                //231
                obstacles.push_back(Rectangle_3D(-0.25, 3.0, 2.25, 2.5, 1.0,  1.5));
                //232
                obstacles.push_back(Rectangle_3D(-1.25, 5.0, 2.55, 2.5, 0.05,  1.5));
                //233
                obstacles.push_back(Rectangle_3D(-1.25, 4.0, 2.25, 2.5, 0.05,  1.5));


		//SECOND FLOOR OBSTACLES 3--

		//301
                obstacles.push_back(Rectangle_3D(-4.5, 3.5, 3.75, 0.05, 4.0,  1.5));
                //302
                obstacles.push_back(Rectangle_3D(-4.5, -4.0, 3.75, 0.05, 2.0,  1.5));
                //303
                obstacles.push_back(Rectangle_3D(0.0, 5.75, 3.75, 0.05, 0.5,  1.5));
                //304
                obstacles.push_back(Rectangle_3D(-2.25, 5.5, 3.75, 4.5, 0.05,  1.5));
                //305
                obstacles.push_back(Rectangle_3D(0.25, -5.0, 3.75, 9.5, 0.05,  1.5));
                //306
                obstacles.push_back(Rectangle_3D(1.5, 2.5, 3.75, 3.0, 0.05,  1.5));
                //307
                obstacles.push_back(Rectangle_3D(0.0, 2.0, 3.75, 0.05, 1.0,  1.5));
                //308
                obstacles.push_back(Rectangle_3D(-1.0, 1.5, 3.75, 2.0, 0.05,  1.5));
                //309
                obstacles.push_back(Rectangle_3D(-3.75, 1.5, 3.75, 1.5, 0.05,  1.5));
                //310
                obstacles.push_back(Rectangle_3D(2.0, 0.0, 3.75, 2.0, 0.05,  1.5));
                //311
                obstacles.push_back(Rectangle_3D(-0.5, 0.0, 3.75, 1.0, 0.05,  1.5));
                //312
                obstacles.push_back(Rectangle_3D(-1.5, -3.0, 3.75, 1.0, 0.05,  1.5));
                //313
                obstacles.push_back(Rectangle_3D(-3.75, -3.0, 3.75, 1.5, 0.05,  1.5));
                //314
                obstacles.push_back(Rectangle_3D(3.0, 2.25, 3.75, 0.05, 0.5,  1.5));
                //315
                obstacles.push_back(Rectangle_3D(3.0, 0.5, 3.75, 0.05, 1.0,  1.5));
                //316
                obstacles.push_back(Rectangle_3D(-1.0, -1.5, 3.75, 0.05, 3.0,  1.5));
                //317
                obstacles.push_back(Rectangle_3D(1.5, -3.0, 3.75, 5.0, 0.05,  1.5));
                //318
                obstacles.push_back(Rectangle_3D(1.5, -4.0, 3.75, 5.0, 0.05,  1.5));
                //319
                obstacles.push_back(Rectangle_3D(-0.5, -1.0, 3.75, 1.0, 0.05,  1.5));
                //320
                obstacles.push_back(Rectangle_3D(3.0, -2.0, 3.75, 2.0, 0.05,  1.5));
                //321
                obstacles.push_back(Rectangle_3D(2.0, -1.0, 3.75, 2.0, 0.05,  1.5));
                //322
                obstacles.push_back(Rectangle_3D(3.5, 0.0, 3.75, 1.0, 0.05,  1.5));
                //323
                obstacles.push_back(Rectangle_3D(4.0, -1.0, 3.75, 0.05, 2.0,  1.5));
                //324
                obstacles.push_back(Rectangle_3D(1.0, -2.0, 3.75, 0.05, 2.0,  1.5));
                //325
                obstacles.push_back(Rectangle_3D(0.0, -1.5, 3.75, 0.05, 1.0,  1.5));

                //326
                obstacles.push_back(Rectangle_3D(4.0, 5.5, 3.75, 0.1, 0.1,  1.5));
                //327
                obstacles.push_back(Rectangle_3D(4.0, 4.5, 3.75, 0.1, 0.1,  1.5));
                //328
                obstacles.push_back(Rectangle_3D(4.0, 3.5, 3.75, 0.1, 0.1,  1.5));
                //329
                obstacles.push_back(Rectangle_3D(4.0, 2.5, 3.75, 0.1, 0.1,  1.5));

                //330
                obstacles.push_back(Rectangle_3D(2.0, 5.5, 3.75, 0.1, 0.1,  1.5));
                //331
                obstacles.push_back(Rectangle_3D(2.0, 4.5, 3.75, 0.1, 0.1,  1.5));
                //332
                obstacles.push_back(Rectangle_3D(2.0, 3.5, 3.75, 0.1, 0.1,  1.5));
                //333
                obstacles.push_back(Rectangle_3D(2.0, 2.5, 3.75, 0.1, 0.1,  1.5));

                //334
                obstacles.push_back(Rectangle_3D(0.0, 5.5, 3.75, 0.1, 0.1,  1.5));
                //335
                obstacles.push_back(Rectangle_3D(0.0, 4.5, 3.75, 0.1, 0.1,  1.5));
                //336
                obstacles.push_back(Rectangle_3D(0.0, 3.5, 3.75, 0.1, 0.1,  1.5));
                //337
                obstacles.push_back(Rectangle_3D(0.0, 2.5, 3.75, 0.1, 0.1,  1.5));

                //338
                obstacles.push_back(Rectangle_3D(-2.0, 5.5, 3.75, 0.1, 0.1,  1.5));
                //339
                obstacles.push_back(Rectangle_3D(-2.0, 4.5, 3.75, 0.1, 0.1,  1.5));
                //340
                obstacles.push_back(Rectangle_3D(-2.0, 3.5, 3.75, 0.1, 0.1,  1.5));
                //341
                obstacles.push_back(Rectangle_3D(-2.0, 2.5, 3.75, 0.1, 0.1,  1.5));

                //342
                obstacles.push_back(Rectangle_3D(3.0, 5.0, 3.75, 0.1, 0.1,  1.5));
                //343
                obstacles.push_back(Rectangle_3D(3.0, 4.0, 3.75, 0.1, 0.1,  1.5));
                //344
                obstacles.push_back(Rectangle_3D(3.0, 3.0, 3.75, 0.1, 0.1,  1.5));
             
                //345
                obstacles.push_back(Rectangle_3D(1.0, 5.0, 3.75, 0.1, 0.1,  1.5));
                //346
                obstacles.push_back(Rectangle_3D(1.0, 4.0, 3.75, 0.1, 0.1,  1.5));
                //347
                obstacles.push_back(Rectangle_3D(1.0, 3.0, 3.75, 0.1, 0.1,  1.5));

                //348
                obstacles.push_back(Rectangle_3D(-1.0, 5.0, 3.75, 0.1, 0.1,  1.5));
                //349
                obstacles.push_back(Rectangle_3D(-1.0, 4.0, 3.75, 0.1, 0.1,  1.5));
                //350
                obstacles.push_back(Rectangle_3D(-1.0, 3.0, 3.75, 0.1, 0.1,  1.5));


                //351
                obstacles.push_back(Rectangle_3D(4.0, 4.0, 3.75, 0.05, 1.0,  1.5));
                //352
                obstacles.push_back(Rectangle_3D(-2.0, 5.0, 3.75, 0.05, 1.0,  1.5));
                //353
                obstacles.push_back(Rectangle_3D(-2.0, 3.0, 3.75, 0.05, 1.0,  1.5));
                //354
                obstacles.push_back(Rectangle_3D(0.0, 5.0, 3.75, 0.05, 1.0,  1.5));
                //355
                obstacles.push_back(Rectangle_3D(3.0, 3.5, 4.3, 0.1, 1.0,  0.4));
                //356
                obstacles.push_back(Rectangle_3D(0.0, 3.0, 4.2, 0.1, 1.0,  0.6));
                //357
                obstacles.push_back(Rectangle_3D(2.0, 4.0, 3.3, 0.1, 1.0,  0.6));
                //358
                obstacles.push_back(Rectangle_3D(-2.0, 4.0, 3.2, 0.1, 1.0,  0.4));
                //359
                obstacles.push_back(Rectangle_3D(1.0, 4.5, 3.75, 0.1, 1.0,  0.4));
                //360
                obstacles.push_back(Rectangle_3D(-1.0, 4.0, 3.75, 0.1, 2.0,  0.4));
		//361
                obstacles.push_back(Rectangle_3D(-2.0, -0.75, 3.75, 0.05, 4.5,  1.5));
                //362
                obstacles.push_back(Rectangle_3D(-3.0, -0.75, 3.75, 0.05, 4.5,  1.5));
                //363
                obstacles.push_back(Rectangle_3D(-2.5, 0.75, 3.0, 1.0, 1.5,  0.05));
                //364
                obstacles.push_back(Rectangle_3D(-2.5, -2.0, 3.0, 1.0, 2.0,  0.05));



                //FIRST FLOOR           

                //F1
                obstacles.push_back(Rectangle_3D(-4.75, 0.0, 1.5, 0.5, 12.0,  0.05));
                //F2
                obstacles.push_back(Rectangle_3D(-3.0, 0.0, 1.5, 1.0, 12.0,  0.05));
                //F3
                obstacles.push_back(Rectangle_3D(0.25, 0.0, 1.5, 3.5, 12.0,  0.05));
                //F4
                obstacles.push_back(Rectangle_3D(3.25, 0.0, 1.5, 0.5, 12.0,  0.05));
                //F5
                obstacles.push_back(Rectangle_3D(4.75, 0.0, 1.5, 0.5, 12.0,  0.05));
                //C1_A
                obstacles.push_back(Rectangle_3D(-4.0, 5.75, 1.5, 1.0, 0.5,  0.05));
                //C1_B
                obstacles.push_back(Rectangle_3D(-4.0, 0.25, 1.5, 1.0, 8.5,  0.05));
                //C1_C
                obstacles.push_back(Rectangle_3D(-4.0, -5.55, 1.5, 1.0, 1.0,  0.05));
                //C2_A
                obstacles.push_back(Rectangle_3D(-2.0, 4.75, 1.5, 1.0, 2.5,  0.05));
                //C2_B
                obstacles.push_back(Rectangle_3D(-2.0, -1.75, 1.5, 1.0, 8.5,  0.05));
                //C3_A
                obstacles.push_back(Rectangle_3D(2.5, 3.0, 1.5, 1.0, 6.0,  0.05));
                //C3_B
                obstacles.push_back(Rectangle_3D(2.5, -2.5, 1.5, 1.0, 3.0,  0.05));
                //C3_C
                obstacles.push_back(Rectangle_3D(2.5, -5.5, 1.5, 1.0, 1.0,  0.05));
                //C4_A
                obstacles.push_back(Rectangle_3D(4.0, 5.25, 1.5, 1.0, 1.5,  0.05));
                //C3_B
                obstacles.push_back(Rectangle_3D(4.0, -1.25, 1.5, 1.0, 9.5,  0.05));
	
	
		//SECOND FLOOR           

                //S1
                obstacles.push_back(Rectangle_3D(4.0, 0.5, 3.0, 2.0, 11.0,  0.05));
                //S2
                obstacles.push_back(Rectangle_3D(1.5, 4.25, 3.0, 3.0, 3.5,  0.05));
                //S3
                obstacles.push_back(Rectangle_3D(-1.75, 3.5, 3.0, 3.5, 4.0,  0.05));
                //S4
                obstacles.push_back(Rectangle_3D(1.5, -2.5, 3.0, 3.0, 5.0,  0.05));
                //S5
                obstacles.push_back(Rectangle_3D(-2.25, -4.0, 3.0, 2.5, 2.0,  0.05));
                //SC1
                obstacles.push_back(Rectangle_3D(-0.5, 0.25, 3.0, 1.0, 2.5,  0.05));
                //SC2
                obstacles.push_back(Rectangle_3D(-0.5, -3.5, 3.0, 1.0, 3.0,  0.05));
                //SC3
                obstacles.push_back(Rectangle_3D(-4.0, 3.5, 3.0, 1.0, 2.0,  0.05));
                //SC4
                obstacles.push_back(Rectangle_3D(-4.0, -4.5, 3.0, 1.0, 1.0,  0.05));


		}


	}
	virtual ~quadrotor_cf_t(){}

	/**
	 * @copydoc system_t::distance(double*, double*)
	 */
	virtual double distance(double* point1, double* point2);

	/**
	 * @copydoc system_t::random_state(double*)
	 */
	virtual void random_state(double* state);

	/**
	 * @copydoc system_t::random_control(double*)
	 */
	virtual void random_control(double* control);

	/**
	 * @copydoc system_t::propagate(double*, double*, int, int, double*, double& )
	 */
	virtual bool propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration );

	/**
	 * @copydoc system_t::enforce_bounds()
	 */
	virtual void enforce_bounds();
	
	/**
	 * @copydoc system_t::valid_state()
	 */
	virtual bool valid_state();

	/**
	 * @copydoc system_t::visualize_point(double*, svg::Dimensions)
	 */
	svg::Point visualize_point(double* state, svg::Dimensions dims);
	
	/**
	 * @copydoc system_t::visualize_obstacles(svg::Document&, svg::Dimensions)
	 */
        virtual void visualize_obstacles(svg::Document& doc ,svg::Dimensions dims);

	/**
         * @brief Check if the point supplied as goal is part of an obstacle.
         * @details Check if the point supplied as goal is part of an obstacle.
	 *
         * @return true if goal is in obstacle, false if not.
         */
        bool obstacles_in_goal();
	bool obstacles_in_goal(double x, double y, double z);
        /**
         * @brief Check if the point supplied as start is part of an obstacle.
         * @details Check if the point supplied as start is part of an obstacle.
         *
         * @return true if start is in obstacle, false if not.
         */
        bool obstacles_in_start();
        bool obstacles_in_start(double x, double y, double z);

	bool change_Map_Limits(double abs_x, double abs_y, double min_z, double max_z);

protected:
	double* deriv;
	void update_derivative(double* control);
	//std::vector<Rectangle_t> obstacles;
	std::vector<Rectangle_3D> obstacles;

	//Map limits
        double MIN_X;
 	double MAX_X;
 	double MIN_Y;
 	double MAX_Y;
	double MIN_Z;
	double MAX_Z;

};


#endif
