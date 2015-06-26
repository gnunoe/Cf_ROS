#include "ros/ros.h"
#include "quad_gazebo/trajectory.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <boost/lexical_cast.hpp>

#include "sparseRRT/sparse_search.h"
#include "sparseRRT/rrt_search.h"

/*
void publish_temp(double value)
{
  ros::init(argc,argv, "pub")
  ros::NodeHandle n;
  ros::Publisher pub2 = n.advertise<prueba_1::temperature>("temperatura",1000);
  pub2.publish(value);
}

*/

double act [3] = {0,0,0};
bool act_ok [3] = {false, false, false};

void getActualX(std_msgs::Float64 tX)
{
  act[0] = tX.data;
  act_ok[0] = true;
}
void getActualY(std_msgs::Float64 tY)
{
  act[1] = tY.data;
  act_ok[1] = true;
}
void getActualZ(std_msgs::Float64 tZ)
{
  act[2] = tZ.data;
  act_ok[2] = true;
}

static void show_usage()
{
  std::cerr << "\n"
            << "Usage: trajectory_client <name_of_trajectory> <args> \n"
            << "\n"
            << "Closed loop trajectories: \n"
	    << "\t Example: trajectory_client <name of trajectory>\n"
            << "\t\t Supported trajectories: circle, lissajous\n"
 	    << "\n"
            << "Open loop trajectories with RRT/SST planner: \n"
	    << "\t OPTION 1: build a new tree from scratch\n"
            << "\t Example: trajectory_client rrt/sst -m <map name> -optional args <arg name>\n"
            << "\t Mandatory arguments:\n"
            << "\t -m,\t\t Map name <map> (Supported: Map1, Map2, Map3, Map4)\n"
            << "\t Optional arguments:\n"
            << "\t -t,\t\t Iteration time <time> (Default = 30s)\n"
            << "\t -g,\t\t Goal position <X Y Z> (Default = <-1.5 -2 1>)\n"
            << "\t -s,\t\t Start position <X Y Z> (Default = <1.5 2 1>)\n"
            << "\n"
	    << "\t OPTION 2: search in the tree the best path between nodes (need quad_sst/rrt_service running)\n"
	    << "\t Example: trajectory_client sst/rrt_search -g <goal>\n"
            << "\t Mandatory arguments:\n"
	    << "\t -g,\t\t Goal position <X Y Z>\n"
	    << "\t Optional arguments:\n"
            << "\t -s,\t\t Start position <X Y Z>\n"
            << "--------------------------------\n"
            << "To show this help message: trajectory_client -h\n"
            << std::endl;
}

int main(int argc, char **argv)
{
  std::string traj_type;
  std::string map_opt;
  std::string sparse_map;

  //the mocap space {abs(max_x), abs(max_y), abs(min_z), abs(max_z)}
  //small mocap space
  double mocap_limits [4] = {2.0, 2.5, 0.5, 1.5};

  //big mocap space
  //double mocap_limits [4] = {2.5, 3.0, 0.5, 1.5};

  //For receive the actual position of the quad by the MoCap
  //double act [3] = {0,0,0};
  //bool act_ok [3] = {false, false, false};
  bool start_by_mocap = false;

  //Start the ros node
  ros::init(argc, argv, "trajectory_client");


  /***********************************************************************
   ***********************************************************************
   ***                                                                 ***
   ***                   START OF CHECK ARGUMENTS                      ***
   ***                                                                 ***
   ***********************************************************************
   ***********************************************************************/



  /////////////////////////////////////////////////////////////////////////
  //                 Check common aguments for planners                  //
  /////////////////////////////////////////////////////////////////////////


  //Show help
  if (argc == 2)
  {
   std::string help = argv[1];
   if (help == "-h")
   {
     show_usage();
     return 1;
   }
  }


  //Not enough arguments
  if (argc < 2)
  {
    show_usage();
    return 1; 
  }
  traj_type = argv[1];

  if (traj_type != "circle" &&
      traj_type != "rrt" &&
      traj_type != "rrt_search" &&
      traj_type != "sst"   &&
      traj_type != "sst_search" &&
      traj_type != "ompl" &&
      traj_type != "send_last")
  {
    ROS_ERROR_STREAM("Wrong trajectory or planner name, please try again");
    return 1;
  }



  /////////////////////////////////////////////////////////////////////////
  //                   Check arguments for RRT planner                   //
  /////////////////////////////////////////////////////////////////////////


  bool check_rrt_optional = false;

  //---------------------------------------------------------------------//
  //                 Mandatory Options for RRT trajectory                //
  //---------------------------------------------------------------------//

  if (traj_type == "rrt") 
  {
    if (argc<4)
    {  
      ROS_ERROR_STREAM("Wrong number arguments, do you forget to add the mandatory argumetn -m <map name>?");
      show_usage();
      return 1; 
    }
    else
    {
      map_opt = argv[2];
      sparse_map = argv[3];
    
      if (map_opt == "-m")
      {
        if (sparse_map != "Map1" &&
            sparse_map != "Map2" &&
            sparse_map != "Map3" &&
            sparse_map != "Map4" &&
	    sparse_map != "Map5" &&
	    sparse_map != "Map6" &&
	    sparse_map != "Map1_3D" &&
	    sparse_map != "Map2_3D" &&
	    sparse_map != "Map3_3D" &&
	    sparse_map != "Map4_3D" &&
	    sparse_map != "Map5_3D")
        {
          ROS_ERROR_STREAM("Unsupported map, please use one of the supported maps: Map1, Map2, Map3, Map4");
          return 1;
        }
        //mandatory argument was correct, let's check the optional
	check_rrt_optional = true;
      }
      else
      {
        ROS_ERROR_STREAM("Please check -m is well written");
        return 1;
      }
    }
  }

  //---------------------------------------------------------------------//
  //               Optional Arguments for RRT Trajectory                 //
  //---------------------------------------------------------------------//

  bool rrt_time_flag = false;
  int rrt_index_time = 0;
  int rrt_time = 30;  

  bool rrt_goal_flag = false;
  int rrt_index_goal = 0;
  double rrt_goal[3] = {-1.5,-2.0,1.0}; 

  bool rrt_start_flag = false;
  int rrt_index_start = 0;
  double rrt_start[3] = {1.5, 2.0, 1.0};

  //Change the default goal and start for bigger maps
  if(sparse_map == "Map2_3D" || 
     sparse_map == "Map3_3D" ||
     sparse_map == "Map4_3D" ||
     sparse_map == "Map5_3D")
  {
    rrt_goal[0] = -4.5;
    rrt_goal[1] = -5.0;
    rrt_goal[2] = 1.0;

    rrt_start[0] = 4.5;
    rrt_start[1] = 5.0;
    rrt_start[2] = 1.0;
  }

  if (check_rrt_optional)
  {
  //Check the arguments used by the user
  std::string rrt_opt;
  if (argc>4)
  {
    int i=4;
    //for (int i=4;i<argc;i++)
    while (i<argc)
    {
      rrt_opt = argv[i];

      //Check time option
      if (rrt_opt == "-t")
      {
        rrt_time_flag = true;
        i ++;
        rrt_index_time = i;
      }

      //Check goal option
      if (rrt_opt == "-g")
      {
        rrt_goal_flag = true;
        i++;
        rrt_index_goal = i;
        i++;
        i++;
      }

      //Check start option
      if (rrt_opt == "-s")
      {
        rrt_start_flag = true;
        i++;
        rrt_index_start = i;
        i++;
        i++;
      }

     i++;
    }
    if (!rrt_time_flag && !rrt_goal_flag && !rrt_start_flag)
    {
      ROS_ERROR_STREAM("Invalid argument, please use -h to see posible arguments");
      return 1;
    }
  }

  //Check validity of time option
  if (rrt_time_flag)
  {
    if (rrt_index_time < argc)
    {
      rrt_time = atoi(argv[rrt_index_time]);
      if (rrt_time <30)
      {
        ROS_ERROR_STREAM("<iteration_time> should be equal or greater than 30");
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("<iteration time> changed to: " << rrt_time << " seconds");
      }
    }
    else
    {
      ROS_ERROR_STREAM("Argument -t <iteration time> was written wrong, please check it");
      return 1;
    }
  }

  //Check validity of goal option
  if (rrt_goal_flag)
  {
    if (rrt_index_goal+2 < argc)
    {
      rrt_goal[0] = atof(argv[rrt_index_goal]);
      rrt_goal[1] = atof(argv[rrt_index_goal+1]);
      rrt_goal[2] = atof(argv[rrt_index_goal+2]);

      //if the big maps for simulation were selected, change the mocap limits
      if (sparse_map == "Map2_3D" || 
	  sparse_map == "Map3_3D" ||
	  sparse_map == "Map4_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 2.5;
      }
      if (sparse_map == "Map5_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 4.0;
      }
      if (sparse_map == "Map5" ||
	  sparse_map == "Map6")
      {
        mocap_limits[3] = 2.4;
      }


      if (abs(rrt_goal[0])>mocap_limits[0] | abs(rrt_goal[1]>mocap_limits[1]) |
          rrt_goal[2]<mocap_limits[2] | rrt_goal[2]>mocap_limits[3])
      {
        ROS_ERROR_STREAM("Space limits exceeded");
        ROS_ERROR_STREAM("X-->   -" << mocap_limits[0] << " < X < " << mocap_limits[0]);
        ROS_ERROR_STREAM("Y-->   -" << mocap_limits[1] << " < Y < " << mocap_limits[1]);
        ROS_ERROR_STREAM("Z-->    " << mocap_limits[2] << " < Z < " << mocap_limits[3]);
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("<goal> changed to: X= " << rrt_goal[0] 
                                      << ", Y= " << rrt_goal[1] 
                                      << ", Z= " << rrt_goal[2]);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Argument -g <x y z> was written wrong, please check it");
      return 1;
    }
  }

  //Check validity of start option
  if (rrt_start_flag)
  {
    if (rrt_index_start+2 < argc)
    {
      rrt_start[0] = atof(argv[rrt_index_start]);
      rrt_start[1] = atof(argv[rrt_index_start+1]);
      rrt_start[2] = atof(argv[rrt_index_start+2]);

      //if the big maps for simulation were selected, change the mocap limits
      if (sparse_map == "Map2_3D" || 
	  sparse_map == "Map3_3D" ||
	  sparse_map == "Map4_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 2.5;
      }
      if (sparse_map == "Map5_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 4.0;
      }
      if (sparse_map == "Map5" 	||
	  sparse_map == "Map6")
      {
        mocap_limits[3] = 2.4;
      }

      if (abs(rrt_start[0])>mocap_limits[0] | abs(rrt_start[1]>mocap_limits[1]) |
          rrt_start[2]<mocap_limits[2] | rrt_start[2]>mocap_limits[3])
      {
        ROS_ERROR_STREAM("Space limits exceeded");
        ROS_ERROR_STREAM("X-->   -" << mocap_limits[0] << " < X < " << mocap_limits[0]);
        ROS_ERROR_STREAM("Y-->   -" << mocap_limits[1] << " < Y < " << mocap_limits[1]);
        ROS_ERROR_STREAM("Z-->    " << mocap_limits[2] << " < Z < " << mocap_limits[3]);
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("<start> changed to: X= " << rrt_start[0]
                                       << ", Y= " << rrt_start[1]
                                       << ", Z= " << rrt_start[2]);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Argument -s <x y z> was written wrong, please check it");
      return 1;
    }
  }
  }
 

  /////////////////////////////////////////////////////////////////////////
  //                  Check arguments for RRT__search                    //
  /////////////////////////////////////////////////////////////////////////

  bool check_rrt_search_optional = false;

  //---------------------------------------------------------------------//
  //                 Mandatory arguments for RRT_search                  //
  //---------------------------------------------------------------------//

  std::string rrt_goal_opt_search;
  double rrt_goal_search[3]={0,0,0};  

  if (traj_type == "rrt_search")
  {
    if (argc<6)
    {
      ROS_ERROR_STREAM("Wrong number arguments, do you forget to add the mandatory argument -g <X Y Z>?");
      show_usage();
      return 1;
    }
    else
    {
      rrt_goal_opt_search = argv[2];
       
      rrt_goal_search[0] = atof(argv[3]);
      rrt_goal_search[1] = atof(argv[4]);
      rrt_goal_search[2] = atof(argv[5]);

      if (rrt_goal_opt_search == "-g")
      {
        //ROS_INFO_STREAM("Calling the sparse tree server, please wait...");
	check_rrt_search_optional = true;
      }
      else
      {
        ROS_ERROR_STREAM("Please check -g is well written");
        return 1;
      }
    }

  }

  //---------------------------------------------------------------------//
  //                  Optional arguments for RRT_search                  //
  //---------------------------------------------------------------------//
  bool rrt_start_flag_search = false;
  int rrt_index_start_search = 0;

  double rrt_search_start[3] = {-1.5,-2.0,1.0};  
  
  if (check_rrt_search_optional)
  {
    //Check the arguments used by the user
    std::string rrt_search_opt;
    if (argc>6)
    {
      int i=6;
     
      while (i<argc)
      {
        rrt_search_opt = argv[i];

        //Check start option
        if (rrt_search_opt == "-s")
        {
          rrt_start_flag_search = true;
          i++;
          rrt_index_start_search = i;
          i++;
          i++;
        }

        i++;
      }
      if (!rrt_start_flag_search)
      {
        ROS_ERROR_STREAM("Invalid argument, please use -h to see posible arguments");
        return 1;
      }
    }

    //Validity start option
    if (rrt_start_flag_search)
    {
      if (rrt_index_start_search+2 < argc)
      {
        rrt_search_start[0] = atof(argv[rrt_index_start_search]);
        rrt_search_start[1] = atof(argv[rrt_index_start_search+1]);
        rrt_search_start[2] = atof(argv[rrt_index_start_search+2]);
       
        ROS_INFO_STREAM("<start> changed to: X= " << rrt_search_start[0]
                                       << ", Y= " << rrt_search_start[1]
                                       << ", Z= " << rrt_search_start[2]);
      }
      else
      {
        ROS_ERROR_STREAM("Argument -s <x y z> was written wrong, please check it");
        return 1;
      }
    }
    else
    {
      start_by_mocap = true;
    }
  }



  /////////////////////////////////////////////////////////////////////////
  //                   Check arguments for SST planner                   //
  /////////////////////////////////////////////////////////////////////////


  bool check_sst_optional = false;

  //---------------------------------------------------------------------//
  //                 Mandatory Options for SST trajectory                //
  //---------------------------------------------------------------------//

  if (traj_type == "sst") 
  {
    if (argc<4)
    {  
      ROS_ERROR_STREAM("Wrong number arguments, do you forget to add the mandatory argumetn -m <map name>?");
      show_usage();
      return 1; 
    }
    else
    {
      map_opt = argv[2];
      sparse_map = argv[3];
    
      if (map_opt == "-m")
      {
        if (sparse_map != "Map1" &&
            sparse_map != "Map2" &&
            sparse_map != "Map3" &&
            sparse_map != "Map4" &&
	    sparse_map != "Map5" &&
	    sparse_map != "Map6" &&
	    sparse_map != "Map1_3D" &&
	    sparse_map != "Map2_3D" &&
	    sparse_map != "Map3_3D" &&
	    sparse_map != "Map4_3D" &&
	    sparse_map != "Map5_3D")
        {
          ROS_ERROR_STREAM("Unsupported map, please use one of the supported maps: Map1, Map2, Map3, Map4");
          return 1;
        }
        //mandatory argument was correct, let's check the optional
	check_sst_optional = true;
      }
      else
      {
        ROS_ERROR_STREAM("Please check -m is well written");
        return 1;
      }
    }
  }

  //---------------------------------------------------------------------//
  //             Optional Arguments for Sparse Trajectory                //
  //---------------------------------------------------------------------//

  bool sst_time_flag = false;
  int sst_index_time = 0;
  int sst_time = 30;  

  bool sst_goal_flag = false;
  int sst_index_goal = 0;
  double sst_goal[3] = {-1.5,-2.0,1.0}; 

  bool sst_start_flag = false;
  int sst_index_start = 0;
  double sst_start[3] = {1.5, 2.0, 1.0};

  //Change the default goal and start for bigger maps
  if(sparse_map == "Map2_3D" || 
     sparse_map == "Map3_3D" ||
     sparse_map == "Map4_3D" ||
     sparse_map == "Map5_3D")
  {
    sst_goal[0] = -4.5;
    sst_goal[1] = -5.0;
    sst_goal[2] = 1.0;

    sst_start[0] = 4.5;
    sst_start[1] = 5.0;
    sst_start[2] = 1.0;
  }

  if (check_sst_optional)
  {
  //Check the arguments used by the user
  std::string sst_opt;
  if (argc>4)
  {
    int i=4;
    //for (int i=4;i<argc;i++)
    while (i<argc)
    {
      sst_opt = argv[i];

      //Check time option
      if (sst_opt == "-t")
      {
        sst_time_flag = true;
        i ++;
        sst_index_time = i;
      }

      //Check goal option
      if (sst_opt == "-g")
      {
        sst_goal_flag = true;
        i++;
        sst_index_goal = i;
        i++;
        i++;
      }

      //Check start option
      if (sst_opt == "-s")
      {
        sst_start_flag = true;
        i++;
        sst_index_start = i;
        i++;
        i++;
      }

     i++;
    }
    if (!sst_time_flag && !sst_goal_flag && !sst_start_flag)
    {
      ROS_ERROR_STREAM("Invalid argument, please use -h to see posible arguments");
      return 1;
    }
  }

  //Check validity of time option
  if (sst_time_flag)
  {
    if (sst_index_time < argc)
    {
      sst_time = atoi(argv[sst_index_time]);
      if (sst_time <30)
      {
        ROS_ERROR_STREAM("<iteration_time> should be equal or greater than 30");
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("<iteration time> changed to: " << sst_time << " seconds");
      }
    }
    else
    {
      ROS_ERROR_STREAM("Argument -t <iteration time> was written wrong, please check it");
      return 1;
    }
  }

  //Check validity of goal option
  if (sst_goal_flag)
  {
    if (sst_index_goal+2 < argc)
    {
      sst_goal[0] = atof(argv[sst_index_goal]);
      sst_goal[1] = atof(argv[sst_index_goal+1]);
      sst_goal[2] = atof(argv[sst_index_goal+2]);

      //if the big maps for simulation were selected, change the mocap limits
      if (sparse_map == "Map2_3D" || 
	  sparse_map == "Map3_3D" ||
	  sparse_map == "Map4_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 2.5;
      }
      if (sparse_map == "Map5_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 4.0;
      }
      if (sparse_map == "Map5" ||
	  sparse_map == "Map6")
      {
        mocap_limits[3] = 2.4;
      }

      if (abs(sst_goal[0])>mocap_limits[0] | abs(sst_goal[1]>mocap_limits[1]) |
          sst_goal[2]<mocap_limits[2] | sst_goal[2]>mocap_limits[3])
      {
        ROS_ERROR_STREAM("Space limits exceeded");
        ROS_ERROR_STREAM("X-->   -" << mocap_limits[0] << " < X < " << mocap_limits[0]);
        ROS_ERROR_STREAM("Y-->   -" << mocap_limits[1] << " < Y < " << mocap_limits[1]);
        ROS_ERROR_STREAM("Z-->    " << mocap_limits[2] << " < Z < " << mocap_limits[3]);
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("<goal> changed to: X= " << sst_goal[0] 
                                      << ", Y= " << sst_goal[1] 
                                      << ", Z= " << sst_goal[2]);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Argument -g <x y z> was written wrong, please check it");
      return 1;
    }
  }

  //Check validity of start option
  if (sst_start_flag)
  {
    if (sst_index_start+2 < argc)
    {
      sst_start[0] = atof(argv[sst_index_start]);
      sst_start[1] = atof(argv[sst_index_start+1]);
      sst_start[2] = atof(argv[sst_index_start+2]);

      //if the big maps for simulation were selected, change the mocap limits
      if (sparse_map == "Map2_3D" || 
	  sparse_map == "Map3_3D" ||
	  sparse_map == "Map4_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 2.5;
      }
      if (sparse_map == "Map5_3D")
      {
        mocap_limits[0] = 5.0;
        mocap_limits[1] = 6.0;
        mocap_limits[2] = 0.5;
        mocap_limits[3] = 4.0;
      }
      if (sparse_map == "Map5" || 
	  sparse_map == "Map6")
      {
        mocap_limits[3] = 2.4;
      }

      if (abs(sst_start[0])>mocap_limits[0] | abs(sst_start[1]>mocap_limits[1]) |
          sst_start[2]<mocap_limits[2] | sst_start[2]>mocap_limits[3])
      {
        ROS_ERROR_STREAM("Space limits exceeded");
        ROS_ERROR_STREAM("X-->   -" << mocap_limits[0] << " < X < " << mocap_limits[0]);
        ROS_ERROR_STREAM("Y-->   -" << mocap_limits[1] << " < Y < " << mocap_limits[1]);
        ROS_ERROR_STREAM("Z-->    " << mocap_limits[2] << " < Z < " << mocap_limits[3]);
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("<start> changed to: X= " << sst_start[0]
                                       << ", Y= " << sst_start[1]
                                       << ", Z= " << sst_start[2]);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Argument -s <x y z> was written wrong, please check it");
      return 1;
    }
  }
  }
 


  /////////////////////////////////////////////////////////////////////////
  //                  Check arguments for SST__search                    //
  /////////////////////////////////////////////////////////////////////////

  bool check_sparse_search_optional = false;

  //---------------------------------------------------------------------//
  //                 Mandatory arguments for SST_search                  //
  //---------------------------------------------------------------------//

  std::string goal_opt_search;
  double goal_search[3]={0,0,0};  

  if (traj_type == "sst_search")
  {
    if (argc<6)
    {
      ROS_ERROR_STREAM("Wrong number arguments, do you forget to add the mandatory argument -g <X Y Z>?");
      show_usage();
      return 1;
    }
    else
    {
      goal_opt_search = argv[2];
       
      goal_search[0] = atof(argv[3]);
      goal_search[1] = atof(argv[4]);
      goal_search[2] = atof(argv[5]);

      if (goal_opt_search == "-g")
      {
        //ROS_INFO_STREAM("Calling the sparse tree server, please wait...");
	check_sparse_search_optional = true;
      }
      else
      {
        ROS_ERROR_STREAM("Please check -g is well written");
        return 1;
      }
    }

  }

  //---------------------------------------------------------------------//
  //                  Optional arguments for SST_search                  //
  //---------------------------------------------------------------------//
  bool start_flag_search = false;
  int index_start_search = 0;

  double search_start[3] = {-1.5,-2.0,1.0};  
  
  if (check_sparse_search_optional)
  {
    //Check the arguments used by the user
    std::string sparse_search_opt;
    if (argc>6)
    {
      int i=6;
     
      while (i<argc)
      {
        sparse_search_opt = argv[i];

        //Check start option
        if (sparse_search_opt == "-s")
        {
          start_flag_search = true;
          i++;
          index_start_search = i;
          i++;
          i++;
        }

        i++;
      }
      if (!start_flag_search)
      {
        ROS_ERROR_STREAM("Invalid argument, please use -h to see posible arguments");
        return 1;
      }
    }

    //Validity start option
    if (start_flag_search)
    {
      if (index_start_search+2 < argc)
      {
        search_start[0] = atof(argv[index_start_search]);
        search_start[1] = atof(argv[index_start_search+1]);
        search_start[2] = atof(argv[index_start_search+2]);
       
        ROS_INFO_STREAM("<start> changed to: X= " << search_start[0]
                                       << ", Y= " << search_start[1]
                                       << ", Z= " << search_start[2]);
      }
      else
      {
        ROS_ERROR_STREAM("Argument -s <x y z> was written wrong, please check it");
        return 1;
      }
    }
    else
    {
      start_by_mocap = true;
    }
  }


  /////////////////////////////////////////////////////////////////////////
  //                 Check arguments for OMPL planners                   //
  /////////////////////////////////////////////////////////////////////////
  if (traj_type == "ompl")
  {
    ROS_INFO_STREAM("OMPL trajectory received, sending it to the controller...");
  }




  /***********************************************************************
   ***********************************************************************
   ***                                                                 ***
   ***                    END OF CHECK ARGUMENTS                       ***
   ***                                                                 ***
   ***********************************************************************
   ***********************************************************************/




  //Create ROS threads.
  ros::NodeHandle n;

  //Subscriber to get start values
  ros::Subscriber actualX = n.subscribe("/target_X",1,getActualX);
  ros::Subscriber actualY = n.subscribe("/target_Y",1,getActualY);
  ros::Subscriber actualZ = n.subscribe("/target_Z",1,getActualZ);

  //Create the client objects for the two servers needed
  ros::ServiceClient traj_client = n.serviceClient<quad_gazebo::trajectory>("trajectory_message");
  ros::ServiceClient sst_search_client = n.serviceClient<sparseRRT::sparse_search>("sparse_tree_search");  
  ros::ServiceClient rrt_search_client = n.serviceClient<sparseRRT::rrt_search>("rrt_tree_search");

  //Create the server clients
  quad_gazebo::trajectory srv;
  sparseRRT::sparse_search srv_sst_search;
  sparseRRT::rrt_search srv_rrt_search;

  //ros::Duration(5).sleep();

  //Save the actual start position of the quadrotors 
  //Only for real trajectories
  bool done = false;
  if (start_by_mocap == true)
  {
    ROS_INFO_STREAM("Getting the actual position of the crazyflie...");
    while (!done)
    {
      if (!ros::ok())
      {
        break;
	return 1;
      }
      if (act_ok[0] == true &&
  	  act_ok[1] == true &&
          act_ok[2] == true)
      {
        done = true;
      }
      ros::spinOnce();
    }
    ROS_INFO_STREAM("Start position: " << act[0]<<" "<<act[1] <<" "<< act[2]);
  }

  //--------------------------------------------------------------------//
  //                          RRT from scratch                          //
  //--------------------------------------------------------------------//


  if (traj_type == "rrt")
  {
    //Execute sparse planner with map as arg
    ROS_INFO_STREAM("Launching rrt planner");
    n.setParam("available_rrt_traj_scratch", 0);
    //std::string time = boost::lexical_cast<std::string>(sparse_time);
    std::string command = "roslaunch sparseRRT quad_rrt.launch map:=" + sparse_map + 
		          " time:=" + boost::lexical_cast<std::string>(rrt_time) +
                          " goalX:=" + boost::lexical_cast<std::string>(rrt_goal[0]) + 
 			  " goalY:=" + boost::lexical_cast<std::string>(rrt_goal[1]) + 
                          " goalZ:=" + boost::lexical_cast<std::string>(rrt_goal[2]) +
			  " startX:=" + boost::lexical_cast<std::string>(rrt_start[0]) +
                          " startY:=" + boost::lexical_cast<std::string>(rrt_start[1]) +
                          " startZ:=" + boost::lexical_cast<std::string>(rrt_start[2]);

    //std::cout << command << "\n";
    //command.append(sparse_time);
    system(command.c_str());

    //When return from the planner, we check
    // the parameter available_sparse_traj:
    //   1 --> planner found path
    //   2 --> planner did not found path
    int param_t;
    n.getParam("available_rrt_traj_scratch", param_t);
    if (param_t == 1)
    {
      ROS_INFO_STREAM("Path found, connecting with the server...");
    }     
    else if (param_t == 2)
    {
      ROS_ERROR_STREAM("Path not found, closing client");
      return 1;
    }
    else
    {
      ROS_ERROR_STREAM("Error with parameter, closing client...");
      return 1;
    }
  }


  //--------------------------------------------------------------------//
  //                   RRT search from existing tree                    //
  //--------------------------------------------------------------------//


  if (traj_type =="rrt_search")
  {
    int param_service_rrt;
    n.getParam("available_rrt_service", param_service_rrt);
    if (param_service_rrt==1)
    {
      if (start_by_mocap == false)
      {
        srv_rrt_search.request.x_s = rrt_search_start[0];
        srv_rrt_search.request.y_s = rrt_search_start[1];
        srv_rrt_search.request.z_s = rrt_search_start[2];
      }
      else
      {
        srv_rrt_search.request.x_s = act[0];
        srv_rrt_search.request.y_s = act[1];
        srv_rrt_search.request.z_s = act[2];
      }
      srv_rrt_search.request.x_g = rrt_goal_search[0];
      srv_rrt_search.request.y_g = rrt_goal_search[1];
      srv_rrt_search.request.z_g = rrt_goal_search[2];
      if(rrt_search_client.call(srv_rrt_search))
      {
        switch(srv_rrt_search.response.done)
        {
          case 1: ROS_INFO_STREAM("Path found, calling the server...");
	          break;
	  case 2: ROS_ERROR_STREAM("The start point given collide with an obstacle. Please select another point");
		  return 1;
 	  case 3: ROS_ERROR_STREAM("The goal point given collide with an obstacle. Please select another point");
		  return 1;
          case 4: ROS_ERROR_STREAM("Path not found, closing client...");
                  return 1;
          case 5: ROS_ERROR_STREAM("Start exceeded map limits. Please see service output for more information");
                  return 1;
          case 6: ROS_ERROR_STREAM("Goal exceeded map limits. Please see service output for more information");
                  return 1;
	  default:ROS_ERROR_STREAM("Unnespecified error, closing the client...");
		  return 1;
        }
 	
	//wait a second to give enough time to the server to
	//create the path. If not, the client will crash	
	ros::Duration(1.0).sleep();
        ros::spinOnce();
      }
      else
      {
        ROS_ERROR_STREAM("Connecting problems with the server");
        return 1;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unable to reach the server.");
      ROS_ERROR_STREAM("Check if the process is alive and if the tree has been built succesfully");
      return 1;
    }
  }


  //--------------------------------------------------------------------//
  //                          SST from scratch                          //
  //--------------------------------------------------------------------//


  if (traj_type == "sst")
  {
    //Execute sparse planner with map as arg
    ROS_INFO_STREAM("Launching sst planner");
    n.setParam("available_sst_traj_scratch", 0);
    //std::string time = boost::lexical_cast<std::string>(sparse_time);
    std::string command = "roslaunch sparseRRT quad_sst.launch map:=" + sparse_map + 
		          " time:=" + boost::lexical_cast<std::string>(sst_time) +
                          " goalX:=" + boost::lexical_cast<std::string>(sst_goal[0]) + 
 			  " goalY:=" + boost::lexical_cast<std::string>(sst_goal[1]) + 
                          " goalZ:=" + boost::lexical_cast<std::string>(sst_goal[2]) +
			  " startX:=" + boost::lexical_cast<std::string>(sst_start[0]) +
                          " startY:=" + boost::lexical_cast<std::string>(sst_start[1]) +
                          " startZ:=" + boost::lexical_cast<std::string>(sst_start[2]);

    //std::cout << command << "\n";
    //command.append(sparse_time);
    system(command.c_str());

    //When return from the planner, we check
    // the parameter available_sparse_traj:
    //   1 --> planner found path
    //   2 --> planner did not found path
    int param_t;
    n.getParam("available_sst_traj_scratch", param_t);
    if (param_t == 1)
    {
      ROS_INFO_STREAM("Path found, connecting with the server...");
    }     
    else if (param_t == 2)
    {
      ROS_ERROR_STREAM("Path not found, closing client");
      return 1;
    }
    else
    {
      ROS_ERROR_STREAM("Error with parameter, closing client...");
      return 1;
    }
  }


  //--------------------------------------------------------------------//
  //                 SPARSE search from existing tree                   //
  //--------------------------------------------------------------------//


  if (traj_type =="sst_search")
  {
    int param_service_sst;
    n.getParam("available_sparse_service", param_service_sst);
    if (param_service_sst==1)
    {
      if (start_by_mocap == false)
      {
        srv_sst_search.request.x_s = search_start[0];
        srv_sst_search.request.y_s = search_start[1];
        srv_sst_search.request.z_s = search_start[2];
      }
      else
      {
        srv_sst_search.request.x_s = act[0];
        srv_sst_search.request.y_s = act[1];
        srv_sst_search.request.z_s = act[2];
      }
      srv_sst_search.request.x_g = goal_search[0];
      srv_sst_search.request.y_g = goal_search[1];
      srv_sst_search.request.z_g = goal_search[2];
      if(sst_search_client.call(srv_sst_search))
      {
        switch(srv_sst_search.response.done)
        {
          case 1: ROS_INFO_STREAM("Path found, calling the server...");
	          break;
	  case 2: ROS_ERROR_STREAM("The start point given collide with an obstacle. Please select another point");
		  return 1;
 	  case 3: ROS_ERROR_STREAM("The goal point given collide with an obstacle. Please select another point");
		  return 1;
          case 4: ROS_ERROR_STREAM("Path not found, closing client...");
                  return 1;
          case 5: ROS_ERROR_STREAM("Start exceeded map limits. Please see service output for more information");
                  return 1;
          case 6: ROS_ERROR_STREAM("Goal exceeded map limits. Please see service output for more information");
                  return 1;
	  default:ROS_ERROR_STREAM("Unnespecified error, closing the client...");
		  return 1;
        }
 	
	//wait a second to give enough time to the server to
	//create the path. If not, the client will crash	
	ros::Duration(1.0).sleep();
        ros::spinOnce();
      }
      else
      {
        ROS_ERROR_STREAM("Connecting problems with the server");
        return 1;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unable to reach the server.");
      ROS_ERROR_STREAM("Check if the process is alive and if the tree has been built succesfully");
      return 1;
    }
  }

 
  //--------------------------------------------------------------------//
  //               DEBUG FUNCTION: Send last trajectory                 //
  //--------------------------------------------------------------------//
   
  if (traj_type == "send_last")
  {
    if (argc !=3)
    {
      ROS_ERROR_STREAM("Select the trajectory to send");
      return 1;
    }
    std::string path_opt = argv[2];
    if (path_opt == "rrt")
    {
      traj_type = "rrt";
    }
    else if (path_opt == "rrt_search")
    {
      traj_type = "rrt_search";
    }
    else if (path_opt == "sst")
    {
      traj_type = "sst";
    }
    else if (path_opt == "sst_search")
    {
      traj_type = "sst_search";
    }
    else if (path_opt == "ompl")
    {
      traj_type = "ompl";
    }
    else
    {
      ROS_ERROR_STREAM("Wrong use of send_last command");
      return 1;
    }
  }


 
  //--------------------------------------------------------------------//
  //              Server to contact with the controller                 //
  //--------------------------------------------------------------------//


  //ros::Duration(5.0).sleep();
  //if the program reach this point, whichever the path is, has been done succesfully
  //so the last step is to send it to the crazyflie controller.
  srv.request.name = traj_type;
  
  //Check que connection
  if(traj_client.call(srv))
  { 
    ROS_INFO_STREAM("Planning "<< srv.response.done);    
    
    ros::spinOnce();
  }

  else
  {
    ROS_ERROR_STREAM("Unable to reach the server, closing client...");
    return 1;
  }

  return 0;

}

