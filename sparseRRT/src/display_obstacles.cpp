#include <ros/ros.h>
//#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <cstdlib>
#include <cmath>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <geometry_msgs/PoseStamped.h>


using namespace std;


int main( int argc, char** argv )
{
  ros::init(argc,argv, "display_obstacles");

  std::string map_name ="";


  
  if (argc != 2)
  {
    ROS_INFO_STREAM("Usage: display_obstacles <Map>");
    ROS_INFO_STREAM("Supported Maps: Map1, Map2, Map3, Map4");
    return 1;
  }
  else
  {
    map_name = argv[1];
    if (map_name != "Map1" &&
        map_name != "Map2" &&
        map_name != "Map3" &&
        map_name != "Map4" &&
	map_name != "Map5" &&
	map_name != "Map6" &&
	map_name != "Map1_3D" &&
	map_name != "Map2_3D" &&
	map_name != "Map3_3D" &&
	map_name != "Map4_3D" &&
	map_name != "Map5_3D")
    {
      ROS_INFO_STREAM(map_name << " is not a supported map");
      return 1;
    }
  }


  ros::NodeHandle n;
  /*
  //n.getParam("map_name", map_name);
  if (n.getParam("map", map_name))
  {
    ROS_INFO_STREAM("Got param:))"); //" << map_name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'map_name'");
  }
  */

  //-----------------------ROS STUFF-----------------------------//

  //Create Publisher for RVIZ
  ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("obstacles", 1); 

  bool publish = false;  
  int count = 0;
  int n_wall = 4;
  int num_obs = n_wall;
  
  if (map_name == "Map1")
  {
    num_obs += 3;
  }   
  if (map_name == "Map2")
  {
    num_obs += 4;
  }
  if (map_name == "Map3")
  {
    num_obs += 5;
  }
  if (map_name == "Map4")
  {
    num_obs += 7;
  }
  if (map_name == "Map5")
  {
    num_obs += 14;
  }
  if (map_name == "Map6")
  {
    num_obs += 34;
  }
  if (map_name == "Map1_3D")
  {
    num_obs += 17;
  }
  if (map_name == "Map2_3D")
  {
    num_obs += 47;
  }
  if (map_name == "Map3_3D")
  {
    num_obs += 62;
  }
  if (map_name == "Map4_3D")
  {
    num_obs += 90;
  }
  if (map_name == "Map5_3D")
  {
    num_obs += 159;
  }

  //position & lenght
  double cube_array [num_obs][6];
  /*
  for (int i=0; i<num_obs; i++)
  {
    for (int j=0; j<6; j++)
    {
      cube_array[i][j] = 0.0;
    }
  }  
  */
 
   
  if (map_name != "Map2_3D" && 
      map_name != "Map3_3D" &&
      map_name != "Map4_3D" &&
      map_name != "Map5_3D")
  {
    //Limit mocap space, small space
    cube_array[0][0] = 2.0; 
    cube_array[0][1] = 0.0;
    cube_array[0][2] = 1.5;
    cube_array[0][3] = 0.05;
    cube_array[0][4] = 5.0;
    cube_array[0][5] = 3.0;

    cube_array[1][0] = -2.0;  
    cube_array[1][1] = 0.0;
    cube_array[1][2] = 1.5;
    cube_array[1][3] = 0.05;
    cube_array[1][4] = 5.0;
    cube_array[1][5] = 3.0;

    cube_array[2][0] = 0.0;  
    cube_array[2][1] = 2.5;
    cube_array[2][2] = 1.5;
    cube_array[2][3] = 4.0;
    cube_array[2][4] = 0.05;
    cube_array[2][5] = 3.0;

    cube_array[3][0] = 0.0;  
    cube_array[3][1] = -2.5;
    cube_array[3][2] = 1.5;
    cube_array[3][3] = 4.0;
    cube_array[3][4] = 0.05;
    cube_array[3][5] = 3.0;
  }
  else
  {
    
    if (map_name == "Map5_3D")
    {
      //Biger square for simulation purpose,
      //3 heights
      cube_array[0][0] = 5;
      cube_array[0][1] = 0.0;
      cube_array[0][2] = 2.25;
      cube_array[0][3] = 0.05;
      cube_array[0][4] = 12.0;
      cube_array[0][5] = 4.5;

      cube_array[1][0] = -5;
      cube_array[1][1] = 0.0;
      cube_array[1][2] = 2.25;
      cube_array[1][3] = 0.05;
      cube_array[1][4] = 12.0;
      cube_array[1][5] = 4.5;

      cube_array[2][0] = 0.0;
      cube_array[2][1] = 6.0;
      cube_array[2][2] = 2.25;
      cube_array[2][3] = 10.0;
      cube_array[2][4] = 0.05;
      cube_array[2][5] = 4.5;

      cube_array[3][0] = 0.0;
      cube_array[3][1] = -6.0;
      cube_array[3][2] = 2.25;
      cube_array[3][3] = 10.0;
      cube_array[3][4] = 0.05;
      cube_array[3][5] = 4.5;

    }
    else
    {
      //Biger square for simulation purpose,
      //1 or 2 heights
      cube_array[0][0] = 5;
      cube_array[0][1] = 0.0;
      cube_array[0][2] = 1.5;
      cube_array[0][3] = 0.05;
      cube_array[0][4] = 12.0;
      cube_array[0][5] = 3.0;

      cube_array[1][0] = -5; 
      cube_array[1][1] = 0.0;
      cube_array[1][2] = 1.5;
      cube_array[1][3] = 0.05;
      cube_array[1][4] = 12.0;
      cube_array[1][5] = 3.0;

      cube_array[2][0] = 0.0;
      cube_array[2][1] = 6.0;
      cube_array[2][2] = 1.5;
      cube_array[2][3] = 10.0;
      cube_array[2][4] = 0.05;
      cube_array[2][5] = 3.0;

      cube_array[3][0] = 0.0;
      cube_array[3][1] = -6.0;
      cube_array[3][2] = 1.5;
      cube_array[3][3] = 10.0;
      cube_array[3][4] = 0.05;
      cube_array[3][5] = 3.0;
    }
  }

  int i_obs = 4;

  
  //map 1
  if (map_name == "Map1")
  {
    cube_array[i_obs][0] = 1.25; 
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3; 
 
    i_obs ++;
    cube_array[i_obs][0] = -1.25;  
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;
  }

  
  //map 2
  if (map_name == "Map2")
  {
    cube_array[i_obs][0] = 1.25; 
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3; 
 
    i_obs ++;
    cube_array[i_obs][0] = -1.25;  
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -1.25; 
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3; 
 
    i_obs ++;
    cube_array[i_obs][0] = 1.25;  
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;
  }


  //Map3
  if (map_name == "Map3")
  {
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = 0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

  }
  
  //Map 4
  if (map_name == "Map4")
  {
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = 0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;
  }

  //Map 5
  if (map_name == "Map5")
  {
    //Horizontal windows
    cube_array[i_obs][0] = 1.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.4;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;
  
    i_obs ++;
    cube_array[i_obs][0] = -1.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.4;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.4;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.8;

    i_obs ++;
    cube_array[i_obs][0] = -0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.2;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    i_obs ++;
    cube_array[i_obs][0] = 0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.8;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    i_obs ++;
    cube_array[i_obs][0] = 0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.6;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.8;

    
    //first floor + horizontal window
    i_obs ++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 0.55;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 3.9;
    cube_array[i_obs][5] = 0.05;
 
    i_obs ++;
    cube_array[i_obs][0] = 1.15;
    cube_array[i_obs][1] = -1.7;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.7;
    cube_array[i_obs][4] = 0.6;
    cube_array[i_obs][5] = 0.05;

    i_obs ++;
    cube_array[i_obs][0] = -1.15;
    cube_array[i_obs][1] = -1.7;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.7;
    cube_array[i_obs][4] = 0.6;
    cube_array[i_obs][5] = 0.05;

  }
 
  //Map 6
  if (map_name == "Map6")
  {
    //Horizontal windows
    cube_array[i_obs][0] = 1.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.4;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;
  
    i_obs ++;
    cube_array[i_obs][0] = -1.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.4;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.4;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.8;

    i_obs ++;
    cube_array[i_obs][0] = -0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.2;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    i_obs ++;
    cube_array[i_obs][0] = 0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.8;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    i_obs ++;
    cube_array[i_obs][0] = 0.3;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.6;
    cube_array[i_obs][3] = 0.6;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.8;

    
    //first floor + horizontal window
    i_obs ++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 0.55;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 3.9;
    cube_array[i_obs][5] = 0.05;
 
    i_obs ++;
    cube_array[i_obs][0] = 1.15;
    cube_array[i_obs][1] = -1.7;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.7;
    cube_array[i_obs][4] = 0.6;
    cube_array[i_obs][5] = 0.05;

    i_obs ++;
    cube_array[i_obs][0] = -1.15;
    cube_array[i_obs][1] = -1.7;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.7;
    cube_array[i_obs][4] = 0.6;
    cube_array[i_obs][5] = 0.05;

    //First room obstacles
    i_obs ++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    i_obs ++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //Second room obstacle
    i_obs ++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 2.5;

    i_obs ++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 2.5;

    i_obs ++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 2.5;

    i_obs ++;
    cube_array[i_obs][0] = 0.5;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.5;
    cube_array[i_obs][3] = 0.35;
    cube_array[i_obs][4] = 0.25;
    cube_array[i_obs][5] = 1.0;

    i_obs ++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.1;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.05;

    //Third room obstacle
    i_obs ++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 2.0;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.0;



  }

 
  if (map_name == "Map1_3D")
  {
    //a 
    cube_array[i_obs][0] = 1.6; 
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.8;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //b
    i_obs++;
    cube_array[i_obs][0] = 0.7;
    cube_array[i_obs][1] = 1.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    //c
    i_obs++;
    cube_array[i_obs][0] = -0.9;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.8;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //d
    i_obs++;
    cube_array[i_obs][0] = -1.8;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 3;

    //e
    i_obs++;
    cube_array[i_obs][0] = -1.8;
    cube_array[i_obs][1] = -1.35;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.3;
    cube_array[i_obs][5] = 3;

    //f
    i_obs++;
    cube_array[i_obs][0] = -1.3;
    cube_array[i_obs][1] = -1.9;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.8;
    cube_array[i_obs][5] = 3;

    //g
    i_obs++;
    cube_array[i_obs][0] = -1.55;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //h
    i_obs++;
    cube_array[i_obs][0] = -0.4;
    cube_array[i_obs][1] = -2.3;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.8;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //i
    i_obs++;
    cube_array[i_obs][0] = 2.25;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //j
    i_obs++;
    cube_array[i_obs][0] = 0.35;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.9;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //k
    i_obs++;
    cube_array[i_obs][0] = -1.55;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //l
    i_obs++;
    cube_array[i_obs][0] = -0.6;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3;

    //m
    i_obs++;
    cube_array[i_obs][0] = -0.6;
    cube_array[i_obs][1] = -1.1;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.8;
    cube_array[i_obs][5] = 3;

    //n
    i_obs++;
    cube_array[i_obs][0] = 1.2;
    cube_array[i_obs][1] = -2.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.5;
    cube_array[i_obs][5] = 3;

    //o
    i_obs++;
    cube_array[i_obs][0] = 0.3;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.8;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //p
    i_obs++;
    cube_array[i_obs][0] = 0.5;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.6;
    cube_array[i_obs][5] = 3;

    //q
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

	
  }


  if (map_name == "Map2_3D")
  {
    //10 
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //11
    i_obs++;
    cube_array[i_obs][0] = 3;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //12
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 0.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1;

    //13
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1;


    //14
    i_obs++;
    cube_array[i_obs][0] = -3;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    //15
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 0.8;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    //16
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.8;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.4;

    //17
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 3.0;

    //18
    i_obs++;
    cube_array[i_obs][0] = 4.25;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //19
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //20
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 0.8;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    //21
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 2.8;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.4;

    //22
    i_obs++;
    cube_array[i_obs][0] = 0.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

    //23
    i_obs++;
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //24
    i_obs++;
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //25
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 1.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3.0;

    //26
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 3.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 3.0;

    //27
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 2.2;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.6;

    //28
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 0.2;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.4;

    //29
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;


    //30
    i_obs++;
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 0.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1;

    //31
    i_obs++;
    cube_array[i_obs][0] = 0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 2.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1;

    //32
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 3.0;

    //33
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;


    //34
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

    //35
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //36
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //37
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

    //38
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 3.0;


    //39
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 0.2;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 0.4;

    //40
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 2.2;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.6;

    //41
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //42
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //43
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

    //44
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

    //45
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 3.0;

    //46
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 3.0;

    //47
    i_obs++;
    cube_array[i_obs][0] = -2.5;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 3.0;

    //48
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 3.0;

    //49
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

    //50
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 2.8;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.4;

    //51
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 0.8;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.6;

    //52
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //53
    i_obs++;
    cube_array[i_obs][0] = 4.5;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //54
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //55
    i_obs++;
    cube_array[i_obs][0] = 3.5;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3.0;

    //56
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 3.0;

}

if (map_name == "Map3_3D")
  {
    //GROUND OBSTACLES 1--

    //100
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //101
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //102
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //103
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //104
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //105
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //106
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //107
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //108
    i_obs++;
    cube_array[i_obs][0] = 0.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //109
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //110
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //111
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //112
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //113
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //114
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //115
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //116
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //117
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //118
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //119
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //120
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //121
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //122
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //123
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 8.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //124
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -3.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //125
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //126
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //127
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //128
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //129
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //130
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //131
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //132
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //133
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //134
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //135
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //136
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //137
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    
    //FIRST FLOOR OBSTACLES 2--
    
    //201
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 7.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //202
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 7.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //203
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 7.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //204
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 1.5;

    //205
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 1.5;

    //206
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //207
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //208
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //209
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //FIRST FLOOR

    //F1
    i_obs++;
    cube_array[i_obs][0] = -4.75;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F2
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F3
    i_obs++;
    cube_array[i_obs][0] = 0.25;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F4
    i_obs++;
    cube_array[i_obs][0] = 3.25;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F5
    i_obs++;
    cube_array[i_obs][0] = 4.75;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //C1_A
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 5.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 0.05;

    //C1_B
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 8.5;
    cube_array[i_obs][5] = 0.05;

    //C1_C
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;

    //C2_A
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 4.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 0.05;

    //C2_B
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -1.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 8.5;
    cube_array[i_obs][5] = 0.05;

    //C3_A
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 6.0;
    cube_array[i_obs][5] = 0.05;

    //C3_B
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 0.05;

    //C3_C
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;

    //C4_A
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 5.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.5;
    cube_array[i_obs][5] = 0.05;

    //C4_B
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -1.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 9.5;
    cube_array[i_obs][5] = 0.05;

}


if (map_name == "Map4_3D")
{
    //GROUND OBSTACLES 1--

    //100
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //101
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //102
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //103
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //104
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //105
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //106
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //107
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //108
    i_obs++;
    cube_array[i_obs][0] = 0.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //109
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //110
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //111
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //112
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //113
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //114
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //115
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //116
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //117
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //118
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //119
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //120
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //121
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //122
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //123
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 8.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //124
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -3.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //125
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //126
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //127
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //128
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //129
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //130
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //131
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //132
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //133
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //134
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //135
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //136
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //137
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    
    //FIRST FLOOR OBSTACLES 2--
    
    //201
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 8.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //202
    i_obs++;
    cube_array[i_obs][0] = -2.25;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //203
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //204
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //205
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //206
    i_obs++;
    cube_array[i_obs][0] = -2.25;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //207
    i_obs++;
    cube_array[i_obs][0] = -2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //208
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //209
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //210
    i_obs++;
    cube_array[i_obs][0] = -2.25;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //211
    i_obs++;
    cube_array[i_obs][0] = -1.75;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //212
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //213
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = -1.25;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 7.5;
    cube_array[i_obs][5] = 1.5;

    //214
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 1.5;

    //215
    i_obs++;
    cube_array[i_obs][0] = -2.5;
    cube_array[i_obs][1] = -0.75;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 1.5;

    //216
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -0.75;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 1.5;

    //217
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //218
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //219
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 1.5;

    //220
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //221
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //222
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //223
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //224
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //225
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //226
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //227
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //228
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 1.75;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.5;
    cube_array[i_obs][5] = 1.5;

    //229
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //230
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //231
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //232
    i_obs++;
    cube_array[i_obs][0] = 3.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //233
    i_obs++;
    cube_array[i_obs][0] = 4.5;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //234
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //235
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //236
    i_obs++;
    cube_array[i_obs][0] = 3.5;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //237
    i_obs++;
    cube_array[i_obs][0] = 1.75;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;



    //FIRST FLOOR

    //F1
    i_obs++;
    cube_array[i_obs][0] = -4.75;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F2
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F3
    i_obs++;
    cube_array[i_obs][0] = 0.25;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F4
    i_obs++;
    cube_array[i_obs][0] = 3.25;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F5
    i_obs++;
    cube_array[i_obs][0] = 4.75;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //C1_A
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 5.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 0.05;

    //C1_B
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 8.5;
    cube_array[i_obs][5] = 0.05;

    //C1_C
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;

    //C2_A
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 4.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 0.05;

    //C2_B
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -1.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 8.5;
    cube_array[i_obs][5] = 0.05;

    //C3_A
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 6.0;
    cube_array[i_obs][5] = 0.05;

    //C3_B
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 0.05;

    //C3_C
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;

    //C4_A
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 5.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.5;
    cube_array[i_obs][5] = 0.05;

    //C4_B
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -1.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 9.5;
    cube_array[i_obs][5] = 0.05;

}

if (map_name == "Map5_3D")
  {
    //GROUND OBSTACLES 1--

    //100
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //101
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //102
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //103
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //104
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //105
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //106
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //107
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //108
    i_obs++;
    cube_array[i_obs][0] = 0.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //109
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //110
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //111
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //112
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //113
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //114
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //115
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //116
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //117
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //118
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //119
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //120
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //121
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //122
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //123
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 8.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //124
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -3.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //125
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //126
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //127
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //128
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //129
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //130
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //131
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //132
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //133
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //134
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //135
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //136
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //137
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 0.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

  
    //FIRST FLOOR OBSTACLES 2--
    
    //201
    i_obs++;
    cube_array[i_obs][0] = -2.25;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //202
    i_obs++;
    cube_array[i_obs][0] = -2.75;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //203
    i_obs++;
    cube_array[i_obs][0] = -2.75;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //204
    i_obs++;
    cube_array[i_obs][0] = 0.25;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 9.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //205
    i_obs++;
    cube_array[i_obs][0] = -3.25;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //206
    i_obs++;
    cube_array[i_obs][0] = 1.25;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 5.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //207
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //208
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //209
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 1.5;

    //210
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //211
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 0.75;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.5;
    cube_array[i_obs][5] = 1.5;

    //212
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 1.5;

    //213
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 5.75;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //214
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.5;
    cube_array[i_obs][5] = 1.5;

    //215
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //216
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //217
    i_obs++;
    cube_array[i_obs][0] = -2.5;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //218
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //219
    i_obs++;
    cube_array[i_obs][0] = -3.5;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //220
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //221
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //222
    i_obs++;
    cube_array[i_obs][0] = 4.5;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //223
    i_obs++;
    cube_array[i_obs][0] = 3.5;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //224
    i_obs++;
    cube_array[i_obs][0] = 4.5;
    cube_array[i_obs][1] = -0.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //225
    i_obs++;
    cube_array[i_obs][0] = 3.5;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //226
    i_obs++;
    cube_array[i_obs][0] = 4.5;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //227
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //228
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -3.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //229
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //230
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //231
    i_obs++;
    cube_array[i_obs][0] = -0.25;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //232
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //233
    i_obs++;
    cube_array[i_obs][0] = -1.25;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 2.25;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;


    //SECOND FLOOR OBSTACLES 3--

    //301
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 1.5;

    //302
    i_obs++;
    cube_array[i_obs][0] = -4.5;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //303
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 5.75;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;


    //304
    i_obs++;
    cube_array[i_obs][0] = -2.25;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 4.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //305
    i_obs++;
    cube_array[i_obs][0] = 0.25;
    cube_array[i_obs][1] = -5.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 9.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //306
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //307
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 2.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //308
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //309
    i_obs++;
    cube_array[i_obs][0] = -3.75;
    cube_array[i_obs][1] = 1.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //310
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //311
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //312
    i_obs++;
    cube_array[i_obs][0] = -1.5;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //313
    i_obs++;
    cube_array[i_obs][0] = -3.75;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 1.5;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //314
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 2.25;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 1.5;

    //315
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //316
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 1.5;

    //317
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = -3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 5.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //318
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 5.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //319
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //320
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //321
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //322
    i_obs++;
    cube_array[i_obs][0] = 3.5;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 1.5;

    //323
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //324
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 1.5;

    //325
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = -1.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //326
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //327
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //328
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //329
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //330
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //331
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //332
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //333
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;


    //334
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //335
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //336
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //337
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //338
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 5.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //339
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //340
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //341
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 2.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //342
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //343
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //344
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //345
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //346
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //347
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //348
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //349
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //350
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 0.1;
    cube_array[i_obs][5] = 1.5;

    //351
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;
	
    //352
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //353
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //354
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 5.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 1.5;

    //355
    i_obs++;
    cube_array[i_obs][0] = 3.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 4.3;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.4;

    //356
    i_obs++;
    cube_array[i_obs][0] = 0.0;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 4.2;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.6;

    //357
    i_obs++;
    cube_array[i_obs][0] = 2.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.3;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.6;

    //358
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.2;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.4;

    //359
    i_obs++;
    cube_array[i_obs][0] = 1.0;
    cube_array[i_obs][1] = 4.5;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.4;

    //360
    i_obs++;
    cube_array[i_obs][0] = -1.0;
    cube_array[i_obs][1] = 4.0;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.1;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 0.4;

    //361
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -0.75;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.5;
    cube_array[i_obs][5] = 1.5;

    //362
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = -0.75;
    cube_array[i_obs][2] = 3.75;
    cube_array[i_obs][3] = 0.05;
    cube_array[i_obs][4] = 4.5;
    cube_array[i_obs][5] = 1.5;

    //363
    i_obs++;
    cube_array[i_obs][0] = -2.5;
    cube_array[i_obs][1] = 0.75;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.5;
    cube_array[i_obs][5] = 0.05;

    //364
    i_obs++;
    cube_array[i_obs][0] = -2.5;
    cube_array[i_obs][1] = -2.0;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 0.05;

    //FIRST FLOOR

    //F1
    i_obs++;
    cube_array[i_obs][0] = -4.75;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F2
    i_obs++;
    cube_array[i_obs][0] = -3.0;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F3
    i_obs++;
    cube_array[i_obs][0] = 0.25;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F4
    i_obs++;
    cube_array[i_obs][0] = 3.25;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //F5
    i_obs++;
    cube_array[i_obs][0] = 4.75;
    cube_array[i_obs][1] = 0.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.5;
    cube_array[i_obs][4] = 12;
    cube_array[i_obs][5] = 0.05;

    //C1_A
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 5.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 0.5;
    cube_array[i_obs][5] = 0.05;

    //C1_B
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 8.5;
    cube_array[i_obs][5] = 0.05;

    //C1_C
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;

    //C2_A
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = 4.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 0.05;

    //C2_B
    i_obs++;
    cube_array[i_obs][0] = -2.0;
    cube_array[i_obs][1] = -1.75;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 8.5;
    cube_array[i_obs][5] = 0.05;

    //C3_A
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = 3.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 6.0;
    cube_array[i_obs][5] = 0.05;

    //C3_B
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 0.05;

    //C3_C
    i_obs++;
    cube_array[i_obs][0] = 2.5;
    cube_array[i_obs][1] = -5.5;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;

    //C4_A
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 5.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.5;
    cube_array[i_obs][5] = 0.05;

    //C4_B
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = -1.25;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 9.5;
    cube_array[i_obs][5] = 0.05;


    //SECOND FLOOR

    //S1
    i_obs++;
    cube_array[i_obs][0] = 4.0;
    cube_array[i_obs][1] = 0.5;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 2.0;
    cube_array[i_obs][4] = 11.0;
    cube_array[i_obs][5] = 0.05;

    //S2
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = 4.25;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 3.5;
    cube_array[i_obs][5] = 0.05;

    //S3
    i_obs++;
    cube_array[i_obs][0] = -1.75;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 3.5;
    cube_array[i_obs][4] = 4.0;
    cube_array[i_obs][5] = 0.05;

    //S4
    i_obs++;
    cube_array[i_obs][0] = 1.5;
    cube_array[i_obs][1] = -2.5;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 3.0;
    cube_array[i_obs][4] = 5.0;
    cube_array[i_obs][5] = 0.05;

    //S5
    i_obs++;
    cube_array[i_obs][0] = -2.25;
    cube_array[i_obs][1] = -4.0;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 2.5;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 0.05;

    //SC1
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = 0.25;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.5;
    cube_array[i_obs][5] = 0.05;

    //SC2
    i_obs++;
    cube_array[i_obs][0] = -0.5;
    cube_array[i_obs][1] = -3.5;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 3.0;
    cube_array[i_obs][5] = 0.05;

    //SC3
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = 3.5;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 2.0;
    cube_array[i_obs][5] = 0.05;

    //SC4
    i_obs++;
    cube_array[i_obs][0] = -4.0;
    cube_array[i_obs][1] = -4.5;
    cube_array[i_obs][2] = 3.0;
    cube_array[i_obs][3] = 1.0;
    cube_array[i_obs][4] = 1.0;
    cube_array[i_obs][5] = 0.05;


}



    /* 
    //First Window
    cube_array[i_obs][0] = 0.5; 
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;

    i_obs ++;
    cube_array[i_obs][0] = -2.35;  
    cube_array[i_obs][1] = -1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.3;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;


    //Second Window

  
    i_obs ++;
    cube_array[i_obs][0] = -0.5; 
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 4.0;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;
 
    i_obs ++;
    cube_array[i_obs][0] = 2.35;
    cube_array[i_obs][1] = 1.0;
    cube_array[i_obs][2] = 1.5;
    cube_array[i_obs][3] = 0.3;
    cube_array[i_obs][4] = 0.05;
    cube_array[i_obs][5] = 3;
    */


 /*
  i_obs ++;
  cube_array[i_obs][0] = 1.0;
  cube_array[i_obs][1] = 0.75;
  cube_array[i_obs][2] = 1.0;
  cube_array[i_obs][3] = 0.05;
  cube_array[i_obs][4] = 0.6;
  cube_array[i_obs][5] = 2.0;

  i_obs ++;
  cube_array[i_obs][0] = 1.0;
  cube_array[i_obs][1] = -0.75;
  cube_array[i_obs][2] = 1.0;
  cube_array[i_obs][3] = 0.05;
  cube_array[i_obs][4] = 0.6;
  cube_array[i_obs][5] = 2.0;
  */

  ros::Rate r(1);

  while (ros::ok() && !publish){

    //Create messages for Publishers
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker cubes;

    //VISUALIZATION CUBES
    //Fill marker_msgs fields necessary for the representation
    cubes.header.frame_id = "/mocap";
    cubes.header.stamp = ros::Time::now();
    cubes.ns = "obstacles_mocap";
    cubes.action = visualization_msgs::Marker::ADD;
    
    //each figure one unique id
    //cubes.id = 0;
    
    cubes.type = visualization_msgs::Marker::CUBE;

    //and fixed orientation    
    cubes.pose.orientation.x = 0.0;
    cubes.pose.orientation.y = 0.0;
    cubes.pose.orientation.z = 0.0;
    cubes.pose.orientation.w = 1.0;


    for (int i=0;i<num_obs;i++)
    {
      cubes.id = i;
      //Mocap wall color
      if (i < 4)
      {
        cubes.color.r = 0.3;
        cubes.color.g = 0.3;
        cubes.color.b = 0.1;
        cubes.color.a = 0.2;
      }
      //In Map3_3D
      else if (map_name == "Map3_3D") 
      {
        //Color of the first floor
        if (i>=51)
        {
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.1;
        }
	//Obstacle color first floor
	else if (i>=42 && i<51)
        {
          cubes.color.r = 0.8;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.5;
        }
	//Normal color obstacles
	else
        {
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.5;
        }
      }
      //In Map4_3D
      else if (map_name == "Map4_3D")
      {
        //Color of the first floor
        if (i>=79)
        {
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.1;
        }
        //Obstacle color first floor
        else if (i>=42 && i<79)
        {
          cubes.color.r = 0.8;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.5;
        }
        //Normal color obstacles
        else
        {
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.5;
        }
      }	
      //In Map5_3D
      else if (map_name == "Map5_3D")
      {
        //Color of the first floor
        if (i>=139 && i<154)
        {
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.3;
        }
        //Color of the second floor
        else if (i>=154 && i<163)
        {
          cubes.color.r = 0.3;
          cubes.color.g = 0.8;
          cubes.color.b = 0.3;
          cubes.color.a = 0.1;
        }
        //Obstacle color first floor
        else if (i>=42 && i<75)
        {
          cubes.color.r = 0.8;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.5;
        }
	//Obstacles color second floor
	else if (i>=75 && i<139)
	{
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.8;
          cubes.color.a = 0.5;
	}
        //Normal color obstacles
        else
        {
          cubes.color.r = 0.5;
          cubes.color.g = 0.5;
          cubes.color.b = 0.5;
          cubes.color.a = 0.5;
        }
      }


      //Obstacle color
      else
      {
	cubes.color.r = 0.5;
        cubes.color.g = 0.5;
        cubes.color.b = 0.5;
        cubes.color.a = 0.5;

      }
      cubes.pose.position.x = cube_array[i][0];
      cubes.pose.position.y = cube_array[i][1];
      cubes.pose.position.z = cube_array[i][2];
      cubes.scale.x = cube_array[i][3];
      cubes.scale.y = cube_array[i][4];
      cubes.scale.z = cube_array[i][5];
   
      ma.markers.push_back(cubes);
    }

    marker_array_pub.publish(ma);  

    count ++;
    if (count == 3)
    {
      publish = true;
    }


    r.sleep();

  }

}
   
