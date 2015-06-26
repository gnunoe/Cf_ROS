#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <sstream>
#include <iostream>
#include <unistd.h>
#include <limits>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <math.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <cf_action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

/*
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
*/

//Global variable to save the path where the
//trajectory will be saved
std::string path;

class Controller{
private:
	typedef actionlib::ActionServer<cf_action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	Controller(ros::NodeHandle &n) :
		node_(n),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
				false),
				has_active_goal_(false)
{
		creato=0;
		empty.linear.x=0;
		empty.linear.y=0;
		empty.linear.z=0;
		empty.angular.z=0;
		empty.angular.y=0;
		empty.angular.x=0;
		pub_topic = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		action_server_.start();
		ROS_INFO_STREAM("Node ready!");
}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Publisher pub_topic;
	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
	geometry_msgs::Twist cmd;
	pthread_t trajectoryExecutor;


	int creato;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(creato){
				ROS_INFO_STREAM("Stop thread");
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(creato){
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;
		//moveit::planning_interface::MoveGroup group_quad("quad_group");

		ROS_INFO_STREAM("Trajectory received, processing"); 
		int n_points = toExecute.points.size();
		double meters = 0.0;
		double vel_cte = 0.5;
		double t = 0.0;
		double dt = 0.0;
		double traj [n_points][6];
		
		//Get Position of Waypoints
		for (int k=0; k<toExecute.points.size(); k++)
		{
                  geometry_msgs::Transform_<std::allocator<void> > punto=toExecute.points[k].transforms[0];
		  traj[k][0] = punto.translation.x;
		  traj[k][1] = punto.translation.y;
		  traj[k][2] = punto.translation.z;
		}

		//Determine distance between consecutive Waypoints
		//to get aproximate path lenght
                for (int k=0; k<toExecute.points.size()-1; k++)
                {
                  traj[k][3] = traj[k+1][0] - traj[k][0];
                  traj[k][4] = traj[k+1][1] - traj[k][1];
                  traj[k][5] = traj[k+1][2] - traj[k][2];
		  
		  meters += sqrt(traj[k][3]*traj[k][3]+traj[k][4]*traj[k][4]+traj[k][5]*traj[k][5]);
          	}
		ROS_INFO_STREAM("Aprox distance: "<< meters << " meters");

		//Determine time in cte velocity profile
		t = meters/vel_cte;

		//We suppose equal separation between waypoint, so we
		//can get the time we have between each waypoint
		dt = t/(toExecute.points.size()-1);

		//Determine the velocity between waypoints
		double max_vel = 0.3;
		double min_vel = -0.3;
		for (int k=0; k<toExecute.points.size()-1; k++)
                {
		  //X-Velocity 
                  traj[k][3] /= dt;
		  if (traj[k][3]>max_vel)
		  {
		    traj[k][3] = max_vel;
		  }
		  else if (traj[k][3]<min_vel)
		  {
		    traj[k][3] = min_vel;
		  }

		  //Y-Velocity
                  traj[k][4] /= dt;
		  if (traj[k][4]>max_vel)
                  {
                    traj[k][4] = max_vel;
                  }
                  else if (traj[k][4]<min_vel)
                  {
                    traj[k][4] = min_vel;
                  }
		
		  //Z-Velocity
                  traj[k][5] /= dt;
                  if (traj[k][5]>max_vel)
                  {
                    traj[k][5] = max_vel;
                  }
                  else if (traj[k][5]<min_vel)
                  {
                    traj[k][5] = min_vel;
                  }

                }


		//Post process in narrow edges
		std::vector<double> edges;
		for(unsigned i=1; i<n_points-1;i++)
		{
		  //Get the angles between two consecutives velocity vectors
		  //Source:http://de.mathworks.com/matlabcentral/answers/16243-angle-between-two-vectors-in-3d
		  double cross_i = traj[i][4]*traj[i-1][5] - traj[i][5]*traj[i-1][4];
		  double cross_j = (-1)*(traj[i][3]*traj[i-1][5] - traj[i][5]*traj[i-1][3]);
		  double cross_k = traj[i][3]*traj[i-1][4] - traj[i][4]*traj[i-1][3];
		  double dot = traj[i][3]*traj[i-1][3] + traj[i][4]*traj[i-1][4] + traj[i][5]*traj[i-1][5];
		  double norm_cross = sqrt(cross_i*cross_i + cross_j*cross_j + cross_k*cross_k);

		  double angle = atan2(norm_cross,dot)*180/M_PI;		 
		  //ROS_INFO_STREAM(angle); 
		  if (angle >= 25)
		  {
		    edges.push_back(i);
		  }
		}		

		for(int i=0; i<edges.size(); i++)
		{
		  ROS_INFO_STREAM(edges[i]);
		}


		std::string dir = path + "/trajectory_ompl.csv";
                //Open file to write trajectory
                std::ofstream myFile;
                myFile.open(dir.c_str());
                //Save all the values separated by comas except the last
                for(unsigned i=0; i<n_points-2;i++)
                {
                  myFile << traj[i][0] <<","
                         << traj[i][1] <<","
                         << traj[i][2] <<","
			  /*
			 << 0.0 <<","
                         << 0.0 <<","
                         << 0.0 <<",\n";
			 */
                         << traj[i][3] <<","
                         << traj[i][4] <<","
                         << traj[i][5] <<",\n";
			 

                }
                //Which is saved here, but without a coma in the last element.
                int i = n_points-1;

                        myFile    << traj[i][0] <<","
                                  << traj[i][1] <<","
                                  << traj[i][2] <<","
                                  << 0.0 <<","
                                  << 0.0 <<","
                                  << 0.0 <<",\n";
                myFile.close();

 		/*
   		//Show trajectory
 		for (int k=0; k<toExecute.points.size(); k++)
                {
                  ROS_INFO_STREAM(traj[k][0]<<" "<<traj[k][1] <<" "<<traj[k][2] <<" "<<
		  		  traj[k][3]<<" "<<traj[k][4] <<" "<<traj[k][5]);
                }
		*/

		//std::string command = "rosrun quad_gazebo trajectory_client ompl";
	        //system(command.c_str());
                active_goal_.setSucceeded();
                has_active_goal_=false;
                creato=0;
  		ROS_INFO_STREAM("Calling the client to execute the trajectory, please wait...");
                std::string command = "rosrun quad_gazebo trajectory_client ompl";
                system(command.c_str());



	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_controller_node");
	ros::NodeHandle node;//("~");

	path = argv[1];
	ROS_INFO_STREAM(path);

	Controller control(node);

	ros::spin();

	return 0;
}
