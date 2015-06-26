#include <ros/ros.h>
//#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <geometry_msgs/PoseStamped.h>


using namespace std;


int main( int argc, char** argv )
{
  ros::init(argc, argv, "display_trajectory");
 
  ros::NodeHandle n;
  
  std::string path_file = argv[1];
  std::string name_file = argv[2];

  //-----------------------ROS STUFF-----------------------------//

  //Create Publisher for RVIZ
  ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("trajectory", 10); 

  bool publish = false;  
  int count = 0;
  double ** traj;
  int n_elements = 12;
  int n_rows = 0;
  int array_index = 0;
    
  //Open the file
  //ROS_INFO_STREAM("opening file...");
  std::string command = path_file + name_file;
  ifstream myFile(command.c_str());
  //ifstream myFile ("/home/estevez/catkin_ws/src/sparseRRT/bin/trajectory.csv");
  if (! myFile.is_open())
  {
    ROS_INFO_STREAM("Could not open the file!");
    return 1;
  }

  //get number of lines 
  std::string line;
  while (std::getline(myFile,line))
  {
    ++ n_rows;
  }
  
  //Reset buffer and point to the beginning of the document
  myFile.clear();
  myFile.seekg(0,ios::beg);
  
  //Save all the values in a vector
  std::vector<double> array;
  std::string val;
  double d_val;
  while (myFile.good())
  {
    getline(myFile,val,',');
    d_val = atof(val.c_str());
    array.push_back(d_val);
  }

  //And close file
  //ROS_INFO_STREAM("closing file...");
  myFile.close();

  //Allocate matrix to sort the numbers
  traj = new double*[n_rows];
  for (int i=0; i<n_rows; i++)
  {
    traj[i] = new double [n_elements];
  }
 
  //And sort the vector in a 2D array
  for (int i=0; i<n_rows; i++)
  {
    for (int j=0; j<n_elements; j++)
    {
      traj[i][j] = array[array_index];
      array_index ++;
    }
  }

  /*
  //Finally display the array
  for (int i=0; i<n_rows; i++)
  {
    for (int j=0; j<n_elements; j++)
    {
      cout << traj[i][j] <<" ";
    }
    cout << "\n";
  }
  */


  ros::Rate r(1);

  while (ros::ok() && !publish){

    //Create messages for Publishers
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker points;
    //visualization_msgs::Marker arrow;

    //VISUALIZATION POINTS
    //Fill marker_msgs fields necessary for the representation
    points.header.frame_id = "/mocap";
    points.header.stamp = ros::Time::now();
    points.ns = "trajectory_mocap";
    points.action = visualization_msgs::Marker::ADD;
    
    //each figure one unique id
    points.id = 2;
    
    points.type = visualization_msgs::Marker::POINTS;
   
    // Cubes are grey
    points.color.r = 1.0f;
    points.color.g = 0.5;
    points.color.b = 0.5;
    points.color.a = 1.0f;

    //Fixed orientation    
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = 0.0;
    points.pose.orientation.w = 1.0;

    //and fixed scale
    points.scale.x = 0.05;
    points.scale.y = 0.05;

/*
    //VISUALIZATION ARROW
    //Fill marker_msgs fields necessary for the representation
    arrow.header.frame_id = "/mocap";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "trajectory_mocap";
    arrow.action = visualization_msgs::Marker::ADD;

    //each figure one unique id
    arrow.id = 3;

    arrow.type = visualization_msgs::Marker::ARROW;

    // Cubes are grey
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    //Fixed orientation    
    //arrow.pose.orientation.x = 0.0;
    //arrow.pose.orientation.y = 0.0;
    //arrow.pose.orientation.z = 0.0;
    //arrow.pose.orientation.w = 1.0;

    //and fixed scale
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;

*/
   
    for (int i=0;i<n_rows;i++)
    {

      geometry_msgs::Point p;
      p.x = traj[i][0];
      p.y = traj[i][1];
      p.z = traj[i][2];

      points.points.push_back(p);

      //arrow.points.push_back(p);

      //geometry_msgs::Point pp;
      //pp.x = traj[i][0] + traj[i][3];
      //pp.y = traj[i][1] + traj[i][4];
      //pp.z = traj[i][2] + traj[i][5];

      //arrow.points.push_back(pp);
      //ma.markers.push_back(arrow);
      //marker_array_pub.publish(ma);
      //arrow.id ++;
    }
    ma.markers.push_back(points);
    //ma.markers.push_back(arrow);
    marker_array_pub.publish(ma);  



    for (int i=1;i<n_rows-1;i++)
    {
    visualization_msgs::Marker arrow;

    //VISUALIZATION ARROW
    //Fill marker_msgs fields necessary for the representation
    arrow.header.frame_id = "/mocap";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "trajectory_mocap";
    arrow.action = visualization_msgs::Marker::ADD;

    //each figure one unique id
    arrow.id =i+ 3;

    arrow.type = visualization_msgs::Marker::ARROW;

    // Cubes are grey
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    //Fixed orientation    
    //arrow.pose.orientation.x = 0.0;
    //arrow.pose.orientation.y = 0.0;
    //arrow.pose.orientation.z = 0.0;
    //arrow.pose.orientation.w = 1.0;

    //and fixed scale
    arrow.scale.x = 0.01;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.08;


      geometry_msgs::Point ps;
      geometry_msgs::Point pg;
      ps.x = traj[i][0];
      ps.y = traj[i][1];
      ps.z = traj[i][2];      
      pg.x = (traj[i][0]+traj[i][3]/6);
      pg.y = (traj[i][1]+traj[i][4]/6);
      pg.z = (traj[i][2]+traj[i][5]/6);

      arrow.points.push_back(ps);
      arrow.points.push_back(pg);

      ma.markers.push_back(arrow);
      marker_array_pub.publish(ma);

    }
    //ma.markers.push_back(arrow);
    //ma.markers.push_back(arrow);
    //marker_array_pub.publish(ma);




    count ++;
    if (count == 3)
    {
      publish = true;
    }


    r.sleep();

  }

}
   
