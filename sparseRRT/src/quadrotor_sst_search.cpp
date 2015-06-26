#include <ros/ros.h>
#include <ros/package.h>
#include "sparseRRT/utilities/parameter_reader.hpp"
#include "sparseRRT/utilities/condition_check.hpp"
#include "sparseRRT/utilities/random.hpp"
#include "sparseRRT/utilities/timer.hpp"

#include "sparseRRT/systems/point.hpp"
#include "sparseRRT/systems/quadrotor.hpp"

#include "sparseRRT/motion_planners/sst.hpp"
#include "sparseRRT/motion_planners/rrt.hpp"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>

//SRV test
#include "sparseRRT/sparse_search.h"
#include "std_msgs/String.h"


using namespace std;


//Call ROS nodes to visualize result in RVIZ
void visualize_path()
{
  std::string command = "roslaunch sparseRRT disp_obst_traj.launch map:=" + params::map
			+ " file_name:=trajectory_sst_search.csv";
  //command.append(params::map);
  system(command.c_str());
}




class tree{

public:
  
  //=================================//
  //            VARIABLES            //
  //=================================//
  
  //Pointer to hold the quadrotor system
  system_t* system;
  
  //Pointer to hold the planner (sst)
  planner_t* planner;

  //Service to process the requests
  ros::ServiceServer sparse_search;

  //Ros Node Handle
  ros::NodeHandle n;

  //bool to check the tree has been built
  bool tree_built;


  //=================================//
  //             METHODS             //
  //=================================//

  //Constructor
  tree(std::string path_file, std::string map, int iter_time, double start[3], double goal[3])
  {
    ROS_INFO_STREAM("creating tree object ...");

    //Initializate variables
    sparse_search = n.advertiseService("sparse_tree_search",&tree::search_nodes_in_tree, this);
    tree_built = false;    

    //Read default paramentes and complete them with
    //the args received 
    read_parameters();
    params::path_file = path_file;
    params::map = map;
    params::stopping_check = iter_time;

    params::start_state[0] = start[0];
    params::start_state[1] = start[1];
    params::start_state[2] = start[2];

    params::goal_state[0] = goal[0];
    params::goal_state[1] = goal[1];
    params::goal_state[2] = goal[2];

    //Create the system and the planner desired
    init_random(std::time(NULL));//params::random_seed);
    if(params::system=="quadrotor_cf")
    {
      system = new quadrotor_cf_t();
    }
    if(params::planner=="rrt")
    {
      planner = new rrt_t(system);
    }
    else if(params::planner=="sst")
    {
      planner = new sst_t(system);
    }

    //Change the limits of the map if the selected
    //is a only simulation map
    if (params::map == "Map2_3D" || 
	params::map == "Map3_3D" ||
	params::map == "Map4_3D")
    {
      system->change_Map_Limits(5.0, 6.0, 0.5, 2.5); 
    }
    if (params::map == "Map5_3D")
    {
      system->change_Map_Limits(5.0, 6.0, 0.5, 4.0);
    }
    if (params::map == "Map5" ||
	params::map == "Map6")
    {
      system->change_Map_Limits(2.0, 2.5, 0.5, 2.4);
    }


  }    

  //Service for search two nodes in the tree
  //Return code message
  //  -> 1 = Success
  //  -> 2 = Start in collision
  //  -> 3 = Goal in collision
  //  -> 4 = Path not found
  //  -> 5 = Start exceeds map limits
  //  -> 6 = Goal exceeds map limits
  bool search_nodes_in_tree(sparseRRT::sparse_search::Request &req,
                            sparseRRT::sparse_search::Response &res)
  {
    //space map limits
    double mocap_limits[4]= {2.0, 2.5, 0.5, 1.5};

    std::string answer;
    bool smooth = false;

    while(true)
    {
      ROS_INFO_STREAM("Apply smoothing to the trajectory? (y/n)");
      std::cin >> answer;
      if (answer == "y" || answer == "Y") 
      {
	smooth = true;
	break;
      }	
      else if (answer == "n" || answer == "N")
      {
        smooth = false;
	break;
      }
    }	

    double sec = ros::Time::now().toSec();
    ROS_INFO_STREAM("Processing request");
    double start[3]={req.x_s, req.y_s, req.z_s};
    double goal[3] ={req.x_g, req.y_g, req.z_g};

    //if the big maps for simulation were selected,
    //change the mocap limits
    if (params::map == "Map2_3D" || 
	params::map == "Map3_3D" ||
	params::map == "Map4_3D")
    {
      mocap_limits[0] = 5.0;
      mocap_limits[1] = 6.0;
      mocap_limits[2] = 0.5;
      mocap_limits[3] = 2.5;
    }
    if (params::map == "Map5_3D")
    {
      mocap_limits[0] = 5.0;
      mocap_limits[1] = 6.0;
      mocap_limits[2] = 0.5;
      mocap_limits[3] = 4.0;
    }
    if (params::map == "Map5" ||
	params::map == "Map6")
    {
      mocap_limits[0] = 2.0;
      mocap_limits[1] = 2.5;
      mocap_limits[2] = 0.5;
      mocap_limits[3] = 2.4;
    }



    if (abs(start[0])>mocap_limits[0] | abs(start[1])>mocap_limits[1] |
        start[2]<mocap_limits[2] | start[2]>mocap_limits[3])
    {
      ROS_ERROR_STREAM("Start exceeded map limits");
      ROS_ERROR_STREAM("X-->   -" << mocap_limits[0] << " < X < " << mocap_limits[0]);
      ROS_ERROR_STREAM("Y-->   -" << mocap_limits[1] << " < Y < " << mocap_limits[1]);
      ROS_ERROR_STREAM("Z-->    " << mocap_limits[2] << " < Z < " << mocap_limits[3]);
      res.done = 5;
      return true;
    }

    if (abs(goal[0])>mocap_limits[0] | abs(goal[1])>mocap_limits[1] |
        goal[2]<mocap_limits[2] | goal[2]>mocap_limits[3])
    {
      ROS_ERROR_STREAM("Goal exceeded map limits");
      ROS_ERROR_STREAM("X-->   -" << mocap_limits[0] << " < X < " << mocap_limits[0]);
      ROS_ERROR_STREAM("Y-->   -" << mocap_limits[1] << " < Y < " << mocap_limits[1]);
      ROS_ERROR_STREAM("Z-->    " << mocap_limits[2] << " < Z < " << mocap_limits[3]);
      res.done = 6;
      return true;
    }


    bool start_ok = system->obstacles_in_start(start[0],start[1],start[2]);
    if(start_ok)
    {
      res.done = 2;
      return true;
    }
   
    bool goal_ok = system->obstacles_in_start(goal[0],goal[1],goal[2]);
    if(goal_ok)
    {
      res.done = 3;
      return true;
    }

    bool found_path = planner->path_between_nodes(start[0], start[1], start[2],
				 	 	  goal[0], goal[1], goal[2],
						  smooth);
    if (!found_path)
    {
      res.done = 4;
      return true;
    }

    ROS_INFO_STREAM("Processed in: " << ros::Time::now().toSec() - sec << " seconds");
    ROS_INFO_STREAM("Visualizing in RVIZ...");
    visualize_path();
    res.done = 1;
    return true;
  }

  /*
  bool search(double x_s, double y_s, double z_s, double x_g, double y_g, double z_g)
  {
    bool start_ok = system->obstacles_in_start(x_s, y_s,z_s);
    if(start_ok)
    {
      return false;
    }

    bool goal_ok = system->obstacles_in_start(x_g, y_g, z_g);
    if(goal_ok)
    {
      return false;
    }

    bool found_path = planner->path_between_nodes(x_s, y_s, z_s, x_g, y_g, z_g);
    if (!found_path)
    {
      return false;
    }

    //ROS_INFO_STREAM(ros::Time::now().toSec() - sec);
    return true;
  }
  */

  //Create the tree
  void create_tree()
  {
    planner->set_start_state(params::start_state);	
    planner->set_goal_state(params::goal_state,params::goal_radius);
    planner->setup_planning();

    condition_check_t checker(params::stopping_type,params::stopping_check);
    condition_check_t* stats_check=NULL;
    if(params::stats_check!=0)
    {
      stats_check = new condition_check_t(params::stats_type,params::stats_check);
    }

    checker.reset();
	
    std::cout<<"Building the tree for the planner: "<<params::planner<<" for the system: "<<params::system<<" in: "<< params::map <<std::endl;
    if(stats_check==NULL)
    {
      do
      {
        planner->step();
      }
      while(!checker.check());

      std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<< "\n";
	
    }
    else
    {
      int count = 0;
      bool execution_done = false;	
      bool stats_print = false;
      while(true && ros::ok())
      {
        do
	{
	  planner->step();
	  execution_done = checker.check();
	  stats_print = stats_check->check();
        }
	while(!execution_done && !stats_print);
			
        if(stats_print)
	{
	  std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<< "\n";
	  stats_print = false;
							
	  stats_check->reset();
	}
			
        if (execution_done)
	{
	  std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<"\n";
	  break;
	}
      }
    }
	
    ROS_INFO_STREAM("Tree built, waiting for request");
    tree_built = true;
    n.setParam("available_sparse_service", 1);
    infinite_loop();
  }
  
  //Infinite loop to keep the node alive and process client requests
  void infinite_loop()
  {
    while(ros::ok())
    {
      /*
      //std::cout << ".";
      double start[3];
      double goal[3];
      std::cin >> start[0];
      std::cin >> start[1];
      std::cin >> start[2];
      std::cin >> goal[0];
      std::cin >> goal[1];
      std::cin >> goal[2];

      bool done = search(start[0], start[1], start[2], goal[0], goal[1], goal[2]);
      
      std::cout << done << "\n";
      */

      ros::spinOnce();
    }
  }

};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "quadrotor_SST_tree_service");
  
  std::string path_file ="";
  std::string map_name = "";
  int iter_time = 0;
  double sparse_start[3]={0,0,0};
  double sparse_goal[3]={0,0,0};

  if (argc !=10)
  {
    ROS_INFO_STREAM("Usage: quadrotor_SST <Map> <Iteration Time> <startX> <startY> <startZ> <goalX> <goalY> <goalZ>");
    ROS_INFO_STREAM("Supported Maps: Map1, Map2, Map3, Map4");
    return 1;
  }
  else
  {
    path_file = argv[1];
    map_name = argv[2];
    iter_time = atoi(argv[3]);

    //fill start position
    sparse_start[0] = atof(argv[4]);
    sparse_start[1] = atof(argv[5]);
    sparse_start[2] = atof(argv[6]);

    //and goal position
    sparse_goal[0] = atof(argv[7]);
    sparse_goal[1] = atof(argv[8]);
    sparse_goal[2] = atof(argv[9]);

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

  tree *t = new tree(path_file, map_name, iter_time, sparse_start, sparse_goal);
  
  //Check if there is obstacles in goal or start
  bool goal_in_obstacle = t->system->obstacles_in_goal();
  if (goal_in_obstacle)
  {
    ROS_ERROR_STREAM("Abort planning, obstacles in goal");
  }    
  bool start_in_obstacle = t->system->obstacles_in_start();
  if (start_in_obstacle)
  {
    ROS_ERROR_STREAM("Abort planning, obstacles in start");
  }       

  if(start_in_obstacle || goal_in_obstacle)
  {
    ROS_ERROR_STREAM("Closing tree server...");
    n.setParam("available_sparse_service", 2);
  }
  else
  {
    //n.setParam("available_sparse_service", 1);
    t->create_tree();
  }
  return 1;
}
   
