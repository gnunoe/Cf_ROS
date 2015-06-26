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

using namespace std;

//Call ROS nodes to visualize result in RVIZ
void visualize_path()
{
  std::string command = "roslaunch sparseRRT disp_obst_traj.launch map:=" + params::map
                        + " file_name:=trajectory_rrt.csv";
  //command.append(params::map);
  system(command.c_str());
}

bool path_done = false;



int main( int argc, char** argv )
{
  ros::init(argc, argv, "quadrotor_RRT");
  
  std::string map_name = "";
  int iter_time = 0;
  double rrt_goal[3]={0,0,0};
  double rrt_start[3]={0,0,0};

  if (argc !=10)
  {
    ROS_INFO_STREAM("Usage: quadrotor_SST <Map> <Iteration Time> <goalX> <goalY> <goalZ> <startX> <startY> <startZ>");
    ROS_INFO_STREAM("Supported Maps: Map1, Map2, Map3, Map4, Map1_3D, Map2_3D");
    return 1;
  }
  else
  {
    map_name = argv[2];
    iter_time = atoi(argv[3]);
 
    rrt_goal[0] = atof(argv[4]);
    rrt_goal[1] = atof(argv[5]);
    rrt_goal[2] = atof(argv[6]);

    rrt_start[0] = atof(argv[7]);
    rrt_start[1] = atof(argv[8]);
    rrt_start[2] = atof(argv[9]);

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


  while(ros::ok() && !path_done){
  
	read_parameters();
        //Read map from command line
        params::map = map_name;
	params::path_file = argv[1];                
        params::stopping_check = iter_time;
	params::planner = "rrt";
   
        params::goal_state[0] = rrt_goal[0];
        params::goal_state[1] = rrt_goal[1];
        params::goal_state[2] = rrt_goal[2];
        
        params::start_state[0] = rrt_start[0];
        params::start_state[1] = rrt_start[1];
        params::start_state[2] = rrt_start[2];


	//****************After reading in from input, we need to instantiate classes
	init_random(std::time(NULL));//params::random_seed);
	system_t* system;
        if(params::system=="quadrotor_cf")
        {
          system = new quadrotor_cf_t();
        }


	planner_t* planner;
	if(params::planner=="rrt")
	{
		planner = new rrt_t(system);
	}
	else if(params::planner=="sst")
	{
		planner = new sst_t(system);
	}

	//Check if the mocap limits are correct
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

	
        //Check if there is obstacles in goal
        bool goal_in_obstacle = system->obstacles_in_goal();
        if (goal_in_obstacle)
        {
          ROS_ERROR_STREAM("Abort planning, obstacles in goal");
          n.setParam("available_rrt_traj_scratch", 2);
          ros::Duration(1).sleep();
          //ros::shutdown();
          return 1;
        }
        
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
	
	std::cout<<"Starting the planner: "<<params::planner<<" for the system: "<<params::system<<" in: "<< params::map <<std::endl;
	if(stats_check==NULL)
	{
		do
		{
			planner->step();
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		planner->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
		std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
		planner->visualize_tree(0);
		planner->visualize_nodes(0);
	}
	else
	{
		int count = 0;
		bool execution_done = false;
		bool stats_print = false;
		while(true)
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
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
				stats_print = false;
				if(params::intermediate_visualization)
				{
					planner->visualize_tree(count);
					planner->visualize_nodes(count);
					count++;
				}				
				stats_check->reset();
			}
			if (execution_done)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
				planner->visualize_tree(count);
				planner->visualize_nodes(count);
				break;
			}
		}
	}
	//std::cout<<"Done planning."<<std::endl;
	//std::cout<<"Visualize results in RVIZ" << std::endl;
	path_done = true;
	//Only visualize if there is solution        
	if (params::availableSolution == true)
        {
          ROS_INFO_STREAM("Planning Done. Visualizing results in RVIZ");
          //std::cout<<"Done planning."<<std::endl;
          //std::cout<<"Visualize results in RVIZ" << std::endl;
	  visualize_path();
          n.setParam("available_rrt_traj_scratch", 1);
	}
	else
	{
          ROS_ERROR_STREAM("Impossible to find a path. Please, give more time to the planner in the next execution");
          //std::cout<<"Impossible to find a path."<<std::endl;
          //std::cout<<"Please, check the map or give more time to the planner in the next execution."<<std::endl;
          n.setParam("available_rrt_traj_scratch", 2);
        }
}
 /*
  ros::Rate r(30);

  while (ros::ok()){
  
    r.sleep();

  }
  */

}
   
