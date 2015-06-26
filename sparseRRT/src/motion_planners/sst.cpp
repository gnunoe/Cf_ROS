/**
 * @file sst.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include <ros/ros.h>
#include "sparseRRT/motion_planners/sst.hpp"
#include "sparseRRT/nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>


void sst_t::setup_planning()
{
	best_goal = NULL;
	//init internal variables
	sample_state = system->alloc_state_point();
	sample_control = system->alloc_control_point();
	metric_query = new sst_node_t();
	metric_query->point = system->alloc_state_point();

    close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
    distances = (double*)malloc(MAX_KK * sizeof (double));

	//initialize the metrics
	metric = new graph_nearest_neighbors_t();
	metric->set_system(system);
	//create the root of the tree
	root = new sst_node_t();
	root->point = system->alloc_state_point();
	system->copy_state_point(root->point,start_state);
	add_point_to_metric(root);
	number_of_nodes++;

	samples = new graph_nearest_neighbors_t();
	samples->set_system(system);
	witness_sample = new sample_node_t();
	witness_sample->point = system->alloc_state_point();
	system->copy_state_point(witness_sample->point,start_state);
	add_point_to_samples(witness_sample);

	witness_sample->rep = (sst_node_t*)root;

}
void sst_t::get_solution(std::vector<std::pair<double*,double> >& controls)
{
	last_solution_path.clear();
	if(best_goal==NULL)
		return;
	nearest = best_goal;
	
	//now nearest should be the closest node to the goal state
	std::deque<tree_node_t*> path;
	while(nearest->parent!=NULL)
	{
		path.push_front(nearest);
		nearest = (sst_node_t*)nearest->parent;
	}
	last_solution_path.push_back(root);
	for(unsigned i=0;i<path.size();i++)
	{
		last_solution_path.push_back(path[i]);
		controls.push_back(std::pair<double*,double>(NULL,0));
		controls.back().first = system->alloc_control_point();
		system->copy_control_point(controls.back().first,path[i]->parent_edge->control);
		controls.back().second = path[i]->parent_edge->duration;
	}
}
void sst_t::step()
{
	random_sample();
	nearest_vertex();
	if(propagate())
	{
		add_to_tree();
	}
}

void sst_t::add_point_to_metric(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	metric->add_node(new_node);
}

void sst_t::add_point_to_samples(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	samples->add_node(new_node);
}


void sst_t::random_sample()
{
	system->random_state(sample_state);
	system->random_control(sample_control);
}
void sst_t::nearest_vertex()
{
	//performs the best near query
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);

    double length = 999999999;
    for(unsigned i=0;i<val;i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = (sst_node_t*)v;
        }
    }
}
bool sst_t::propagate()
{
	return system->propagate(nearest->point,sample_control,params::min_time_steps,params::max_time_steps,sample_state,duration);
}
void sst_t::add_to_tree()
{
	//check to see if a sample exists within the vicinity of the new node
	check_for_witness();

	if(witness_sample->rep==NULL || witness_sample->rep->cost > nearest->cost + duration)
	{
		//create a new tree node
		sst_node_t* new_node = new sst_node_t();
		new_node->point = system->alloc_state_point();
		system->copy_state_point(new_node->point,sample_state);
		//create the link to the parent node
		new_node->parent_edge = new tree_edge_t();
		new_node->parent_edge->control = system->alloc_control_point();
		system->copy_control_point(new_node->parent_edge->control,sample_control);
		new_node->parent_edge->duration = duration;
		//set this node's parent
		new_node->parent = nearest;
		new_node->cost = nearest->cost + duration;
		//set parent's child
		nearest->children.insert(nearest->children.begin(),new_node);
		number_of_nodes++;

        if(best_goal==NULL && system->distance(new_node->point,goal_state)<goal_radius)
        	best_goal = new_node;
        else if(best_goal!=NULL && best_goal->cost > new_node->cost && system->distance(new_node->point,goal_state)<goal_radius)
        	best_goal = new_node;


		if(witness_sample->rep!=NULL)
		{
			//optimization for sparsity
			if(!(witness_sample->rep->inactive))
			{
				remove_point_from_metric(witness_sample->rep);
				witness_sample->rep->inactive = true;
			}

            sst_node_t* iter = witness_sample->rep;
            while( is_leaf(iter) && iter->inactive && !is_best_goal(iter))
            {
                sst_node_t* next = (sst_node_t*)iter->parent;
                remove_leaf(iter);
                iter = next;
            } 

		}
		witness_sample->rep = new_node;
		add_point_to_metric(new_node);
	}	

}

void sst_t::check_for_witness()
{
	system->copy_state_point(metric_query->point,sample_state);
	double distance;
	witness_sample = (sample_node_t*)samples->find_closest(metric_query,&distance)->get_state();
	if(distance > params::sst_delta_drain)
	{
		//create a new sample
		witness_sample = new sample_node_t();
		witness_sample->point = system->alloc_state_point();
		system->copy_state_point(witness_sample->point,sample_state);
		add_point_to_samples(witness_sample);
	}
}

void sst_t::remove_point_from_metric(tree_node_t* node)
{
	proximity_node_t* old_node = node->prox_node;
	metric->remove_node(old_node);
	delete old_node;
}

bool sst_t::is_leaf(tree_node_t* node)
{
	return node->children.size()==0;
}

void sst_t::remove_leaf(tree_node_t* node)
{
	if(node->parent != NULL)
	{
		tree_edge_t* edge = node->parent_edge;
		node->parent->children.remove(node);
		number_of_nodes--;
		delete edge->control;
		delete node->point;
		delete node;
	}
}

bool sst_t::is_best_goal(tree_node_t* v)
{
	if(best_goal==NULL)
		return false;
    tree_node_t* new_v = best_goal;

    while(new_v->parent!=NULL)
    {
        if(new_v == v)
            return true;

        new_v = new_v->parent;
    }
    return false;

}


bool sst_t::path_between_nodes(double x_s, double y_s, double z_s,
                               double x_g, double y_g, double z_g,
			       bool smooth)
{

	//ROS_INFO_STREAM("Getting start and goal neighbordhood");

	std::vector<double> start_goal;
	start_goal.push_back(x_s);
	start_goal.push_back(y_s);
	start_goal.push_back(z_s);
	start_goal.push_back(x_g);
	start_goal.push_back(y_g);
	start_goal.push_back(z_g);
	//START POSITION
	//fill the sample_state with the values of the start position
	sample_state[0] = x_s;
	sample_state[1] = y_s;
	sample_state[2] = z_s;

	//find the nearest neighbords of the start point
        system->copy_state_point(metric_query->point,sample_state);
        int num_start_neighbors = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);

	std::deque<tree_node_t*> start_neighbors;
	double d_start_root = system->distance(metric_query->point,root->point);
        for(int i=0;i<num_start_neighbors;i++)
        {
                tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
		start_neighbors.push_back(v);
        }

	//GOAL POSITION
	sample_state[0] = x_g;
        sample_state[1] = y_g;
        sample_state[2] = z_g;

	system->copy_state_point(metric_query->point,sample_state);
        int num_goal_neighbors = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);

	std::deque<tree_node_t*> goal_neighbors;
	double d_goal_root = system->distance(metric_query->point,root->point);
	for(int i=0;i<num_goal_neighbors;i++)
        {      
                tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
                goal_neighbors.push_back(v);
        }

	//Check if the positions have neighbors
	if(start_neighbors.size() == 0 || goal_neighbors.size() ==0)
	{
		//ROS_ERROR_STREAM("We couldn't find any neighbord");
		return false;
	}

	//Find the trajectory
	//bool smooth = true;
	double cost; 
	bool close_root;
	//bool start_is_root = false; 
	//bool goal_is_root = false;
	bool found = false;
	std::deque<tree_node_t> trajectory;
	std::deque<tree_node_t*> goal_trajectory;
	std::deque<tree_node_t*> start_trajectory;
	std::vector<tree_node_t*> fake_shared_node;

	goal_trajectory.clear();
	start_trajectory.clear();
	trajectory.clear();
	
	std::cout << "trajectory is empty: " << trajectory.empty() << "\n";

	//TODO:Crash when you supply the root point
	//if goal is closer to root than start we should spand the start neighbors
	if (d_start_root > d_goal_root)
	{
		//std::cout << "Expanding start neighbords \n";
	 	found = search_shared_node(num_goal_neighbors, goal_neighbors, goal_trajectory,
					   num_start_neighbors, start_neighbors, start_trajectory,
					   &cost, &close_root);	


		std::cout << "goal is root: " << close_root << "\n";
		if (found && cost != 0)
		{
			if (smooth && !close_root)
			{
				ROS_INFO_STREAM("Smoothing around the shared node in process");
				smooth_path(start_trajectory, goal_trajectory, start_goal, cost);
			}
			else if (close_root)
			{
				compose_trajectory(start_trajectory, goal_trajectory, start_goal, false, fake_shared_node, false, true);	
			}
			else
			{
				compose_trajectory(start_trajectory, goal_trajectory, start_goal, false, fake_shared_node, false, false);
			}
		}
		else
		{
			//ROS_ERROR_STREAM("We couldn't find any shared node");
			ROS_ERROR_STREAM("Start and Goal are very close, imposible to create a trajectory");
			return false;
		}

	}
	//if start is closer to root than goal we should spand the goal neighbors
	else
	{

		//std::cout << "Expanding goal neighbords \n";
		found = search_shared_node(num_start_neighbors, start_neighbors, start_trajectory,
					   num_goal_neighbors, goal_neighbors, goal_trajectory,
					   &cost, &close_root);


		std::cout << "start is root: " << close_root << "\n";
		if (found && cost != 0)
                {
			if (smooth && !close_root)
			{
				ROS_INFO_STREAM("Smoothing around the shared node in process");
				smooth_path(start_trajectory, goal_trajectory, start_goal,cost);
			}
			else if (close_root)
			{
				compose_trajectory(start_trajectory, goal_trajectory, start_goal, false, fake_shared_node, true, false);
			}
			else
			{
				compose_trajectory(start_trajectory, goal_trajectory, start_goal, false, fake_shared_node, false, false);
			}
		}
                else
                {
			//ROS_ERROR_STREAM("We couldn't find any shared node");
			ROS_ERROR_STREAM("Start and Goal are very close, imposible to create a trajectory");
                        return false;
                }

	}
	/*	
        //DEBUG
	std::cout << "FINAL TRAJECTORY \n";
	for (int i=0; i<trajectory.size(); i++)
        {
        	std::cout << trajectory[i]->point[0] << " "
                          << trajectory[i]->point[1] << " "
                          << trajectory[i]->point[2] << " "
                          << trajectory[i]->point[3] << " "
                          << trajectory[i]->point[4] << " "
                          << trajectory[i]->point[5] << "\n";
        }
	*/

	/*
	//Write path in file
	std::string dir = params::path_file + "/trajectory_sparse_search.csv";
        //Open file to write trajectory
        std::ofstream myFile;
        myFile.open(dir.c_str());
	//Save the start position
	myFile << x_s <<","<< y_s <<","<< z_s <<","<< 0 <<","<< 0 <<","<< 0 <<","
	       << 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<",\n";	

        //Save all the intermediate values except the last
        for(unsigned i=0; i<trajectory.size();i++)
        {
        	myFile << trajectory[i].point[0] <<","
                       << trajectory[i].point[1] <<","
                       << trajectory[i].point[2] <<","
                       << trajectory[i].point[3] <<","
                       << trajectory[i].point[4] <<","
                       << trajectory[i].point[5] <<","
                       << trajectory[i].point[6] <<","
                       << trajectory[i].point[7] <<","
                       << trajectory[i].point[8] <<","
                       << trajectory[i].point[9] <<","
                       << trajectory[i].point[10] <<","
                       << trajectory[i].point[11] <<",\n";

	}
        //Which is saved here, but without a coma in the last element.
        myFile << x_g <<","<< y_g <<","<< z_g <<","<< 0 <<","<< 0 <<","<< 0 <<","
               << 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<"\n";
 
        myFile.close();
	*/

	return true;
}


void sst_t::smooth_path(std::deque<tree_node_t*> &start_path, std::deque<tree_node_t*> &goal_path, std::vector<double> &sg,double old_cost)
{
	int s_size = start_path.size();
	int g_size = goal_path.size();
	std::vector <tree_node_t*> neighbor;
	int less_cost_element;
	double cost = 0;

	if (s_size<g_size)
	{
		//std::cout << "start path smaller \n";	
		less_cost_element = search_points_to_smooth(start_path, goal_path, neighbor, old_cost, true);
		if (less_cost_element == -1)
		{
			ROS_INFO_STREAM("Impossible to smooth the trajectory");
			compose_trajectory(start_path, goal_path, sg, false, neighbor, false, false);
		}
		else
		{
			compose_trajectory(start_path, goal_path, sg, true, neighbor, false, false);
		}

        }
	else
	{
		//std::cout << "goal path smaller \n";
		less_cost_element = search_points_to_smooth(goal_path, start_path, neighbor, old_cost, false);
		if (less_cost_element == -1)
		{
			ROS_INFO_STREAM("Impossible to smooth the trajectory");
			compose_trajectory(start_path, goal_path, sg, false, neighbor, false, false);
		}
		else
		{
			compose_trajectory(start_path, goal_path, sg, true, neighbor, false, false);
		}
		//compose_trajectory(start_path, goal_path, traj, true, neighbor);

	}
	
}


int sst_t::search_points_to_smooth(std::deque<tree_node_t*> &small_segment, std::deque<tree_node_t*> &big_segment,
				   std::vector <tree_node_t*> &neighbor, double old_cost, bool start_is_smaller)
{

	std::vector <std::pair<int,int> > nodes_sharing_neighbors;
	std::vector <tree_node_t*> shared_neighbor;

	nodes_sharing_neighbors.clear();
	shared_neighbor.clear();
	
	std::cout << "Starting search_points_to_smooth \n";
	//if (params::map == "Map2_3D" || params::map == "Map3_3D")
	if(true)
	{
	
	//restrict the search nodes in 5 nodes from the shared node in big
	//maps, to avoid a long and unpractical search
	int small_size = small_segment.size();
	int big_size = big_segment.size();
	if (small_size > 10)
	{
		small_size = 10;
	}
	if (big_size>10)
	{
		big_size = 10;
	}
	
	for (int i=small_segment.size()-1; i>small_segment.size()-small_size; i--)
	{
		//Copy the new node
		system->copy_state_point(metric_query->point,small_segment[i]->point);
		
		//and get the list of neighbors
        	int num_neighbors_1 = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);

		std::deque<tree_node_t*> smooth_neighbors;

		smooth_neighbors.clear();       
 
		//Save them in the list
        	for (int j=0;j<num_neighbors_1;j++)
        	{
                	tree_node_t* v = (tree_node_t*)(close_nodes[j]->get_state());
                	smooth_neighbors.push_back(v);
        
		}
		
		for (int j=0; j<num_neighbors_1; j++)
		{
			//We select a node of the list
			tree_node_t* check_node = smooth_neighbors[j];
			
			//And expand the other branch to get the neighbors of each node
			for (int k=big_segment.size()-1; k>big_segment.size()-big_size; k--)
			{
				system->copy_state_point(metric_query->point,big_segment[k]->point);
				int num_neighbors_2 = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);
				
				std::deque<tree_node_t*> smooth_neighbors_temp;
		
				smooth_neighbors_temp.clear();

		                //Save them in the list
                		for (int l=0;l<num_neighbors_2;l++)
                		{
                       			tree_node_t* v = (tree_node_t*)(close_nodes[l]->get_state());
                       			smooth_neighbors_temp.push_back(v);

                		}
				
				//and check if the neighbors are shared
				for (int l=0; l<num_neighbors_2;l++)
				{
					tree_node_t* temp_2_node = smooth_neighbors_temp[l];
					if (temp_2_node == check_node)
					{
						//ROS_INFO_STREAM("PARTY LOCA MADAFACKAA");
						nodes_sharing_neighbors.push_back(std::pair<int,int>(-1,-1));
                                                nodes_sharing_neighbors.back().first = i;
                                                nodes_sharing_neighbors.back().second = k;
						shared_neighbor.push_back(check_node);

					}
				}					

			}

		}
	}

	}
	/*
	else
	{
	for (int i=0; i<small_segment.size(); i++)
	{
		//Copy the new node
		system->copy_state_point(metric_query->point,small_segment[i]->point);
		
		//and get the list of neighbors
        	int num_neighbors_1 = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);

		std::deque<tree_node_t*> smooth_neighbors;

		smooth_neighbors.clear();       
 
		//Save them in the list
        	for (int j=0;j<num_neighbors_1;j++)
        	{
                	tree_node_t* v = (tree_node_t*)(close_nodes[j]->get_state());
                	smooth_neighbors.push_back(v);
        
		}
		
		for (int j=0; j<num_neighbors_1; j++)
		{
			//We select a node of the list
			tree_node_t* check_node = smooth_neighbors[j];
			
			//And expand the other branch to get the neighbors of each node
			for (int k=0; k<big_segment.size();k++)
			{
				system->copy_state_point(metric_query->point,big_segment[k]->point);
				int num_neighbors_2 = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,params::sst_delta_near);
				
				std::deque<tree_node_t*> smooth_neighbors_temp;
		
				smooth_neighbors_temp.clear();

		                //Save them in the list
                		for (int l=0;l<num_neighbors_2;l++)
                		{
                       			tree_node_t* v = (tree_node_t*)(close_nodes[l]->get_state());
                       			smooth_neighbors_temp.push_back(v);

                		}
				
				//and check if the neighbors are shared
				for (int l=0; l<num_neighbors_2;l++)
				{
					tree_node_t* temp_2_node = smooth_neighbors_temp[l];
					if (temp_2_node == check_node)
					{
						//ROS_INFO_STREAM("PARTY LOCA MADAFACKAA");
						nodes_sharing_neighbors.push_back(std::pair<int,int>(-1,-1));
                                                nodes_sharing_neighbors.back().first = i;
                                                nodes_sharing_neighbors.back().second = k;
						shared_neighbor.push_back(check_node);

					}
				}					

			}

		}
	}
	}
	*/	

	std::cout << "getting cost \n";
	std::cout <<"shared neighbors: "  <<shared_neighbor.size() << "\n";
	
	if (shared_neighbor.size()==0)
	{
		return -1;
	}
	//Get the total cost of all the combinations and save the index of the lower one
	double total_cost = 1000;
	int index_min_cost = -1;
	for (int i=0; i<shared_neighbor.size(); i++)
	{
		//get total cost of the path passing through the new node and erasing the
		// two closest neighbords
		double local_cost = 0;
		
		//get the cost from the start of the small segment to the node which 
                //is in a less position in the tree that the one who shares the neighbord
		//and the same from start of the big segment.
		
		
		//Small segment
		int i_less_position = nodes_sharing_neighbors[i].first;
		
		if (i_less_position == 0)
		{
			local_cost += system->distance(shared_neighbor[i]->point,small_segment[i_less_position]->point);
		}		
		else if (i_less_position == 1)
		{
			local_cost += system->distance(shared_neighbor[i]->point,small_segment[i_less_position-1]->point);
		}
		else
		{
			local_cost += small_segment[0]->cost;
	                local_cost -= small_segment[i_less_position -1]->cost;
			local_cost += system->distance(shared_neighbor[i]->point,small_segment[i_less_position-1]->point);
		}

		//Big segment
		i_less_position = nodes_sharing_neighbors[i].second;

                if (i_less_position == 0)       
                {
                        local_cost += system->distance(shared_neighbor[i]->point,big_segment[i_less_position]->point);
                } 
                else if (i_less_position == 1)
                {
                        local_cost += system->distance(shared_neighbor[i]->point,big_segment[i_less_position-1]->point);
                }
                else
                {
                        local_cost += big_segment[0]->cost;
                        local_cost -= big_segment[i_less_position -1]->cost;
                        local_cost += system->distance(shared_neighbor[i]->point,big_segment[i_less_position-1]->point);
                }
		

		/*
		local_cost = small_segment[0]->cost;
		local_cost -= small_segment[nodes_sharing_neighbors[i].first -1]->cost;
		local_cost += big_segment[0]->cost;
                local_cost -= big_segment[nodes_sharing_neighbors[i].second -1]->cost;		

		local_cost += system->distance(shared_neighbor[i]->point,small_segment[nodes_sharing_neighbors[i].first-1]->point);

		local_cost += system->distance(shared_neighbor[i]->point,big_segment[nodes_sharing_neighbors[i].second-1]->point);
		*/
		
		if (local_cost < total_cost)
		{
			total_cost = local_cost;
			index_min_cost = i;
		} 
	
	}
	
	if (old_cost > total_cost)
	{
		ROS_INFO_STREAM("Path smoothing found lower cost: " << total_cost);
	}
	else
	{
		ROS_INFO_STREAM("Path smoothing didnt found a lower cost");
		return -1;
	}

	//Save the neighbor in the pointer 
	neighbor.push_back(shared_neighbor[index_min_cost]);

	
	//DEBUG: Information of the best shared node
	std::cout << "node " << nodes_sharing_neighbors[index_min_cost].first << " in smaller list "
		  << "share with " << nodes_sharing_neighbors[index_min_cost].second << "\n";
	/*
	std::cout << "node " << small_segment[nodes_sharing_neighbors[index_min_cost].first]->point[0]<< " " 
			     << small_segment[nodes_sharing_neighbors[index_min_cost].first]->point[1]<< " "
			     << small_segment[nodes_sharing_neighbors[index_min_cost].first]->point[2]<< " "
		  << "share with node " << big_segment[nodes_sharing_neighbors[index_min_cost].second]->point[0]<< " "
					<< big_segment[nodes_sharing_neighbors[index_min_cost].second]->point[1]<< " "
					<< big_segment[nodes_sharing_neighbors[index_min_cost].second]->point[2]<< " "
		  << "the neighbor " << shared_neighbor[index_min_cost]->point[0] << " "
				     << shared_neighbor[index_min_cost]->point[1] << " "
				     << shared_neighbor[index_min_cost]->point[2] << "\n"; 
	*/

	std::cout << "Erasing step \n";
	//Erase the elements to leave in each path the node which shares
	//neighbor as the last element

	while(small_segment.size() != nodes_sharing_neighbors[index_min_cost].first + 1)
	{
		small_segment.pop_back();
	}

	while(big_segment.size() != nodes_sharing_neighbors[index_min_cost].second + 1)
        {
                big_segment.pop_back();
        }
	/*	
	for (int i=0; i<small_segment.size() - nodes_sharing_neighbors[index_min_cost].first; i++)
	{
		small_segment.pop_back();
        	std::cout << "erase last node in small segment \n";
	}

	//std::cout << "-------------\n";
        for (int i=0; i<big_segment.size() - nodes_sharing_neighbors[index_min_cost].second; i++)
        {
                big_segment.pop_back();
                std::cout << "erase last node in big segment \n";
        }
	*/
	for (int i=0; i<small_segment.size(); i++)
        {
                std::cout <<small_segment[i]->point[0] << "\n";
                //std::cout << "erase last node in small segment \n";
        }
        for (int i=0; i<big_segment.size(); i++)
        {
                std::cout <<big_segment[i]->point[0] << "\n";
                //std::cout << "erase last node in small segment \n";
        }


	std::cout << "Erased finished\n";	
	
	return index_min_cost;
}

void sst_t::compose_trajectory(const std::deque<tree_node_t*> &start_traj, const std::deque<tree_node_t*> &goal_traj, std::vector<double> &sg, 
			       bool smooth, const std::vector<tree_node_t*> &shared_neighbor, bool start_is_root, bool goal_is_root)
{


	std::cout << "Starting deep copying \n";
	
	//std::vector<double> start_local;
	//std::vector<double> goal_local;

	std::cout << "size: " << shared_neighbor.size() << "\n";

	int rows_start = start_traj.size();
        int rows_goal = goal_traj.size();
	int rows_traj;

	double ** start_local_array;
	double ** goal_local_array;
	double ** traj_local_array;

	start_local_array = new double *[rows_start];
	for (int i=0; i<rows_start; i++)
	{
		start_local_array[i] = new double [12];
	}
	
	goal_local_array = new double *[rows_goal];
        for (int i=0; i<rows_goal; i++)
        {
                goal_local_array[i] = new double [12];
        }

	for (int i=0;i<rows_start; i++)
	{
		for (int j=0; j<12; j++)
		{
			start_local_array[i][j] = start_traj[i]->point[j];
		}
	}
	
	for (int i=0;i<rows_goal; i++)
        {
                for (int j=0; j<12; j++)
                {
                        goal_local_array[i][j] = goal_traj[i]->point[j];
                }
        }


	if (smooth)
	{
	       /*
                * The start_traj come in order, but we should reverse the velocity, as the tree has been
                * built to reach start. We will only reverse the velocity as is the only value we need 
                * for our controller.
                * Then, we change the last node with the new shared node (path with lower cost), and determine
                * the velocity of this new node to make it consisten with the path
		* Finally, we delete the las node of the goal trajectory, and add the segment to the final trajectory 
		*
                */


		//invert the velocities in the start path
                for (int i=0; i<rows_start; i++)
                {
			start_local_array[i][3] *=-1;
			start_local_array[i][4] *=-1;
			start_local_array[i][5] *=-1;
                        //tree_node_t v; // start_traj[i];
			//v.point = start_traj[i]->point;
                        //v.point[3] = (-1)*v.point[3];
                        //v.point[4] = (-1)*v.point[4];
                        //v.point[5] = (-1)*v.point[5];
			
			//start_local_pp[i]->point[3] *= -1;
			//start_local_pp[i]->point[4] *= -1;
			//start_local_pp[i]->point[5] *= -1;
                        //trajj.push_back(start_local_pp[i]);
                }
		
		//Show start path inverted	
		for (int i=0; i<rows_start; i++)
		{
			for (int j=0; j<6; j++)
			{
				std::cout << start_local_array[i][j] << " ";
			}
			std::cout << "\n";
		}
		std::cout << "\n";

		/*
		//DEBUG
		std::cout << "old velocities: " << shared_neighbor[0]->point[3] << " "          
                                                << shared_neighbor[0]->point[4] << " "
                                                << shared_neighbor[0]->point[5] << "\n"; 
		*/

		//Determine the velocity of the shared node by evaluating
		//the velocity of its neighbors. We take half of the velocity
		//to avoid a quick change

		double shared_local[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
		for (int i=0; i<6;i++)
		{
			shared_local[i] = shared_neighbor[0]->point[i];
		}

		//start_local_array[rows_start-1][3] /=2;
		//start_local_array[rows_start-1][4] /=2;
		//start_local_array[rows_start-1][5] /=2;

		//tree_node_t *shared;
		//shared = new tree_node_t();
		//shared->point = shared_neighbor[0]->point;
		shared_local[0] = (goal_local_array[rows_goal-1][0] - start_local_array[rows_start-1][0])/2.0 + start_local_array[rows_start-1][0];
		shared_local[1] = (goal_local_array[rows_goal-1][1] - start_local_array[rows_start-1][1])/2.0 + start_local_array[rows_start-1][1];
		shared_local[2] = (goal_local_array[rows_goal-1][2] - start_local_array[rows_start-1][2])/2.0 + start_local_array[rows_start-1][2];

		shared_local[3] = (goal_local_array[rows_goal-1][0] - shared_local[0])/0.25;
		shared_local[4] = (goal_local_array[rows_goal-1][1] - shared_local[1])/0.25;
		shared_local[5] = (goal_local_array[rows_goal-1][2] - shared_local[2])/0.25;

		//It don't break if after smoothing start is root
		//if (rows_start >1)
		//{
		  start_local_array[rows_start-1][3] = (shared_local[0] - start_local_array[rows_start-1][0])/0.25;
                  start_local_array[rows_start-1][4] = (shared_local[1] - start_local_array[rows_start-1][1])/0.25;
                  start_local_array[rows_start-1][5] = (shared_local[2] - start_local_array[rows_start-1][2])/0.25;
		//}

		//shared_neighbor[0]->point[3] = (traj[traj.size()-1]->point[3] + goal_traj[goal_traj.size()-1]->point[3])/2.0;
		//shared_neighbor[0]->point[4] = (traj[traj.size()-1]->point[4] + goal_traj[goal_traj.size()-1]->point[4])/2.0;
		//shared_neighbor[0]->point[5] = (traj[traj.size()-1]->point[5] + goal_traj[goal_traj.size()-1]->point[5])/2.0;	

		/*
		//DEBUG
		std::cout << "new velocities: " << shared_neighbor[0]->point[3] << " " 
						<< shared_neighbor[0]->point[4] << " "
					        << shared_neighbor[0]->point[5] << "\n";		
		*/

		//change the last element (node which share neighbor)
		//with the actual shared neighbor
		//trajj[trajj.size()-1]->point = shared->point;
		//start_local_array[rows_start-1][3] = shared_local[3];
		//start_local_array[rows_start-1][4] = shared_local[4];
		//start_local_array[rows_start-1][5] = shared_local[5];

		//Erase the last element of the goal path
		//goal_traj.pop_back();
		//goal_local_pp.pop_back();
		
		//And add the goal path changing the order
		//for (int i=goal_local.size(); i>0; i--)
                //{
                        //tree_node_t v; //= goal_traj[i-1];
			//v.point = goal_traj[i-1]->point;
                        //trajj.push_back(goal_local_pp[i-1]);
                //}
		


                for (int i=0; i<rows_goal; i++)
                {
                        for (int j=0; j<6; j++)
                        {
                                std::cout << goal_local_array[i][j] << " ";
                        }
                        std::cout << "\n";
                }
                std::cout << "\n";




		rows_traj = rows_start+rows_goal+1;
			
		traj_local_array = new double *[rows_traj];
        	for (int i=0; i<rows_traj; i++)
    	        {
               		traj_local_array[i] = new double [12];
        	}

		//Start fill
		for (int i=0; i<rows_start; i++)
		{
			for (int j=0; j<12; j++)
			{
				traj_local_array[i][j] = start_local_array[i][j];
			}
		}

		//Shared node
		traj_local_array[rows_start][0] = shared_local[0];
		traj_local_array[rows_start][1] = shared_local[1];
		traj_local_array[rows_start][2] = shared_local[2];
		traj_local_array[rows_start][3] = shared_local[3];
		traj_local_array[rows_start][4] = shared_local[4];
		traj_local_array[rows_start][5] = shared_local[5];

		//Goal fill
		for (int i=0; i<rows_goal; i++)
		{
			for (int j=0; j<12; j++)
			{
				traj_local_array[rows_traj-1-i][j] = goal_local_array[i][j];
			}
		}
	

                for (int i=0; i<rows_traj; i++)
                {
                        for (int j=0; j<6; j++)
                        {
                                std::cout << traj_local_array[i][j] << " ";
                        }
                        std::cout << "\n";
                }
                std::cout << "\n";



	
	}
	else if(start_is_root)
	{
		//means all the trajectory is stored in goal but with the
		//wrong direction
		
		rows_traj = rows_goal;
		traj_local_array = new double *[rows_traj];
                for (int i=0; i<rows_traj; i++)
                {
                        traj_local_array[i] = new double [12];
                }

		/*
		for (int i=0; i<rows_goal; i++)
                {
                        goal_local_array[i][3] *=-1;
                        goal_local_array[i][4] *=-1;
                        goal_local_array[i][5] *=-1;
		}
		*/
		
		for (int i=0; i<rows_traj; i++)
		{
			for (int j=0; j<12; j++)
			{
				traj_local_array[i][j] = goal_local_array[rows_goal-1-i][j];
			}
		} 

		//for (int i=0; i<goal_local.size(); i++)
                //{
                        //tree_node_t v;// = goal_traj[i];
			//v.point = goal_traj[i]->point;
                        //v->point[3] = (-1)*v->point[3];
                        //v->point[4] = (-1)*v->point[4];
                        //v->point[5] = (-1)*v->point[5];
                        //trajj.push_front(goal_local_pp[i]);
                //}

	}
	else if(goal_is_root)
	{
		//means all the trajectory saved in start is in the right 
		//direction but the velocities are inverted

                rows_traj = rows_start;
                traj_local_array = new double *[rows_traj];
                for (int i=0; i<rows_traj; i++)
                {
                        traj_local_array[i] = new double [12];
                }

                
                for (int i=0; i<rows_start; i++)
                {
                        start_local_array[i][3] *=-1;
                        start_local_array[i][4] *=-1;
                        start_local_array[i][5] *=-1;
                }
                
                
                for (int i=0; i<rows_traj; i++)
                {
                        for (int j=0; j<12; j++)
                        {
                                traj_local_array[i][j] = start_local_array[i][j];
                        }
                }

		//for (int i=0; i<start_local.size(); i++)
                //{
                        //tree_node_t v; //= start_traj[i];
			//v.point = start_traj[i]->point;
                        //v.point[3] = (-1)*v.point[3];
                        //v.point[4] = (-1)*v.point[4];
                        //v.point[5] = (-1)*v.point[5];
                        //traj.push_front(v);

			//start_local_pp[i]->point[3] *= -1;
                        //start_local_pp[i]->point[4] *= -1;
                        //start_local_pp[i]->point[5] *= -1;
                        //trajj.push_back(start_local_pp[i]);

                //}
	}
	else 
	{
		/*
	 	* The start_traj come in order, but we should reverse the velocity, as the tree has been
	 	* built to reach start. We will only reverse the velocity as is the only value we need 
	 	* for our controller.
	 	* We loop .size()-1 as the shared node is repeated in both parts, so we ignore it here
	 	* and include it in the next part.
	 	*/

		rows_traj = rows_start + rows_goal - 1;
                traj_local_array = new double *[rows_traj];
                for (int i=0; i<rows_traj; i++)
                {
                        traj_local_array[i] = new double [12];
                }

		for (int i=0; i<rows_start; i++)
                {
                        start_local_array[i][3] *=-1;
                        start_local_array[i][4] *=-1;
                        start_local_array[i][5] *=-1;
                }

 		//Start fill
		std::cout << "filling start \n";
                for (int i=0; i<rows_start-1; i++)
                {
                        for (int j=0; j<12; j++)
                        {
                                traj_local_array[i][j] = start_local_array[i][j];
                        }
                }

		//Goal fill
		std::cout << "filling goal \n";
                for (int i=0; i<rows_goal; i++)
                {
                        for (int j=0; j<12; j++)
                        {
                                traj_local_array[rows_traj-1-i][j] = goal_local_array[i][j];
                        }
                }


		//for (int i=0; i<start_local.size()-1; i++)
		//{
			/*
			start_traj[i].point[3] = (-1)*start_traj[i].point[3];
			start_traj[i].point[3] = (-1)*start_traj[i].point[4];
			start_traj[i].point[3] = (-1)*start_traj[i].point[3];
			traj.push_back(start_traj[i]);
			*/
			//tree_node_t v;// = start_traj[i];
			//v.point = start_traj[i]->point;
			//v.point[3] = (-1)*v.point[3];
			//v.point[4] = (-1)*v.point[4];
			//v.point[5] = (-1)*v.point[5];
			//traj.push_back(v);

			//start_local_pp[i]->point[3] *= -1;
                        //start_local_pp[i]->point[4] *= -1;
                        //start_local_pp[i]->point[5] *= -1;
                        //trajj.push_back(start_local_pp[i]);

			
		//}	

		//the goal_traj has the correct values, but it's not in order.
		//for (int i=goal_local.size(); i>0; i--)
		//{
			//tree_node_t v;// = goal_traj[i-1];
			//v.point = goal_traj[i-1]->point;
			//trajj.push_back(goal_local_pp[i-1]);
		//}
	}

	//Change the position and velocity of the start and end point
	//so the trajectory will be smoother

	//Start point
 	//position in the middle of start and 2nd point
	traj_local_array[0][0] = (traj_local_array[1][0] - sg[0])/2 + sg[0];
	traj_local_array[0][1] = (traj_local_array[1][1] - sg[1])/2 + sg[1];
	traj_local_array[0][2] = (traj_local_array[1][2] - sg[2])/2 + sg[2];

	traj_local_array[0][3] = (traj_local_array[1][0] - sg[0])/0.5;
        traj_local_array[0][4] = (traj_local_array[1][1] - sg[1])/0.5;
        traj_local_array[0][5] = (traj_local_array[1][2] - sg[2])/0.5;


	//End point
        traj_local_array[rows_traj-1][0] = (sg[3] - traj_local_array[rows_traj-2][0])/2 + traj_local_array[rows_traj-2][0];
        traj_local_array[rows_traj-1][1] = (sg[4] - traj_local_array[rows_traj-2][1])/2 + traj_local_array[rows_traj-2][1];;
        traj_local_array[rows_traj-1][2] = (sg[5] - traj_local_array[rows_traj-2][2])/2 + traj_local_array[rows_traj-2][2];;
	
        traj_local_array[rows_traj-1][3] = (sg[3] - traj_local_array[rows_traj-1][0])/0.5;
        traj_local_array[rows_traj-1][4] = (sg[4] - traj_local_array[rows_traj-1][1])/0.5;
        traj_local_array[rows_traj-1][5] = (sg[5] - traj_local_array[rows_traj-1][2])/0.5;


        //Write path in file
        std::string dir = params::path_file + "/trajectory_sst_search.csv";
        //Open file to write trajectory
        std::ofstream myFile;
        myFile.open(dir.c_str());
        //Save the start position
        myFile << sg[0] <<","<< sg[1] <<","<< sg[2] <<","<< 0 <<","<< 0 <<","<< 0 <<","
               << 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<",\n";

        //Save all the intermediate values except the last
        for(unsigned i=0; i<rows_traj;i++)
        {
                myFile << traj_local_array[i][0] <<","
                       << traj_local_array[i][1] <<","
                       << traj_local_array[i][2] <<","
                       << traj_local_array[i][3]*0.5 <<","
                       << traj_local_array[i][4]*0.5 <<","
                       << traj_local_array[i][5]*0.5 <<","
                       << traj_local_array[i][6] <<","
                       << traj_local_array[i][7] <<","
                       << traj_local_array[i][8] <<","
                       << traj_local_array[i][9] <<","
                       << traj_local_array[i][10] <<","
                       << traj_local_array[i][11] <<",\n";

        }
        //Which is saved here, but without a coma in the last element.
        myFile << sg[3] <<","<< sg[4] <<","<< sg[5] <<","<< 0 <<","<< 0 <<","<< 0 <<","
               << 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<","<< 0 <<"\n";

        myFile.close();

	for (int i=0; i<rows_start; i++)
  	{
    		delete [] start_local_array[i];
  	}
  	delete [] start_local_array;

	for (int i=0; i<rows_goal; i++)
        {
                delete [] goal_local_array[i];
        }
        delete [] goal_local_array;
	
	for (int i=0; i<rows_traj; i++)
        {
                delete [] traj_local_array[i];
        }
        delete [] traj_local_array;



}


void sst_t::get_parent(int jumps, tree_node_t * node)
{
	tree_node_t * n = node;
	for (int i=0; i<jumps; i++)
	{
		n = (tree_node_t*) n->parent;
	}
	node = (tree_node_t*) n;
}
	
bool sst_t::search_shared_node(int num_close_neighbors, std::deque<tree_node_t*> &close_neighbors, std::deque<tree_node_t*> &close_to_shared, 
                               int num_far_neighbors, std::deque<tree_node_t*> &far_neighbors, std::deque<tree_node_t*> &far_to_shared,
			       double * cost, bool * close_is_root)
{

	//std::cout << "searching shared node with lower cost \n";

	//clear solution vectors
	close_to_shared.clear();
	far_to_shared.clear();

	//std::deque<tree_node_t*> &close_too_shared = close_to_shared;
        //std::deque<tree_node_t*> &start_trajectory = far_to_shared;

	//tree_node_t * shared_node;
	std::vector <std::pair<int,int> > shared_table;
	std::vector <std::pair<int,int> > level_shared_table;

	bool finish = false;
	
	//for each node in the far_neighbord list
	do
	{
		for(int i=0;i<num_far_neighbors;i++)
                {
			//exapand it until we reach root, and save 
			//the nodes in the far_to_root list
			tree_node_t * v = (tree_node_t*)far_neighbors[i];
			std::deque<tree_node_t*> far_to_root;
			far_to_root.clear();
			while(v->parent!=NULL)
                	{
                		far_to_root.push_back(v);
                		//v = (sst_node_t*)v->parent;
				v = (tree_node_t*)v->parent;
                	}
			far_to_root.push_back(root);		
			//for each node in the far_to_root list
			for(int j=0; j<far_to_root.size(); j++)
			{
				tree_node_t * is_start = far_to_root[j];

				//check if it's equal to one of the neighbors
				//from the close_neighbord list
				
				for(int k=0;k<num_close_neighbors; k++)
				{
					//Expand the node until we reach root
					tree_node_t * temp_node = (tree_node_t*)close_neighbors[k];
					std::deque<tree_node_t*> temp_list;
					temp_list.clear();
		                        while(temp_node->parent!=NULL)
                        		{
                                		temp_list.push_back(temp_node);
                                		//temp_node = (sst_node_t*)temp_node->parent;
						temp_node = (tree_node_t*)temp_node->parent;
                        		}
					temp_list.push_back(root);	
									
					//l represent the tree level
					for (int l=0; l<temp_list.size(); l++)
					{
						tree_node_t * collapse_node = (tree_node_t*)temp_list[l];					
									
						if (is_start->point[0] == collapse_node->point[0] &&
				    		    is_start->point[1] == collapse_node->point[1] &&
 					    	    is_start->point[2] == collapse_node->point[2])
						{
							shared_table.push_back(std::pair<int,int>(-1,-1));
                					shared_table.back().first = i;
                					shared_table.back().second = k;
							level_shared_table.push_back(std::pair<int,int>(-1,-1));
							level_shared_table.back().first = j;
							level_shared_table.back().second = l;
						}
					}		
								
				
				}
					
			}
			
		}
		finish = true;
	}
	while(!finish);
	ROS_INFO_STREAM("Search finished");


	/*	
	//DEBUG
	//show all the shared nodes between the close_neighborhood
	//and the far_neighborhood	
	for (int i=0;i<level_shared_table.size();i++)
	{
		std::cout << i << "\n";
		std::cout << "the node "<< level_shared_table[i].first  
			  << " in the far_node_list " << shared_table[i].first 
			  << " is shared by the node in the close_node_list "
			  << shared_table[i].second << " located in the level "
			  << level_shared_table[i].second << "\n"; 
	}
 	*/	
	
	//check if any shared node has been found
	if (level_shared_table.size() == 0)
	{
		std::cout << "No shared node found \n";
		return false;
	}

	/* TODO:NOT CLEAR YET
         * for each shared node we have found in the step below 
	 * (to do it quicker, we will use only the first half of the solutions
	 * found. This is robust because since a shared node is found, then all
	 * succesive nodes will be shared too, and it's not neccesary to check 
	 * them because the cost will be greater)
	 */
	double total_cost = 600.0;
	int index_less_cost = -1;
	for(int i=0; i<(level_shared_table.size()); i++)
	{
		tree_node_t * v = (tree_node_t*)far_neighbors[shared_table[i].first];
        	std::deque<tree_node_t*> far_to_shared_temp;
		far_to_shared_temp.clear();
		double cost_far = far_neighbors[shared_table[i].first]->cost;
		int shared_index = 0;
	        while(shared_index <=level_shared_table[i].first)
       		{
        		far_to_shared_temp.push_back(v);
			//std::cout << v->point[0] << " "
			//	  << v->point[1] << " "
			//  	<< v->point[2] << "\n";
			//std::cout << "cost: " << v->cost << "\n";
			
                	//v = (sst_node_t*)v->parent;
			v = (tree_node_t*)v->parent;
			shared_index ++;
 	       	}
	        cost_far -= far_to_shared_temp.back()->cost;

		tree_node_t * w = (tree_node_t*)close_neighbors[shared_table[i].second];
		std::deque<tree_node_t*> close_to_shared_temp;
		close_to_shared_temp.clear();
		double cost_close = close_neighbors[shared_table[i].second]->cost;
		shared_index = 0;	
		while(shared_index <= level_shared_table[i].second)
		{
			close_to_shared_temp.push_back(w);
			//std::cout << w->point[0] << " "
                	//          << w->point[1] << " "
                	//          << w->point[2] << "\n";
			//std::cout << "cost: " << w->cost << "\n";

			//w = (sst_node_t*)w->parent;
			w = (tree_node_t*)w->parent;
                	shared_index ++;
		}
	
		cost_close -= close_to_shared_temp.back()->cost;

		if (total_cost > cost_far + cost_close)
		{
			total_cost = cost_far + cost_close;
			index_less_cost = i;
		}

	}
	ROS_INFO_STREAM("Less cost founded: " << total_cost);
	//std::cout << "less_cost " << total_cost << " in the element nº " << index_less_cost <<  "\n";
	* cost = total_cost;
	
	//fill the vector to return the better path
	
	//first the far_path
	tree_node_t * v = (tree_node_t*)far_neighbors[shared_table[index_less_cost].first];
	int fill_index = 0;
	std::cout << "-----------\n";
	while(fill_index <= level_shared_table[index_less_cost].first)
        {
        	far_to_shared.push_back(v);
			
		//DEBUG
		std::cout << v->point[0] << " "
                          << v->point[1] << " "
                          << v->point[2] << " "
			  << v->point[3] << " "
                          << v->point[4] << " "
                          << v->point[5] << "\n";
		
                v = (sst_node_t*)v->parent;
                fill_index ++;
        }

	//and then the close_path
	bool close_root = false;
	tree_node_t * w = (tree_node_t*)close_neighbors[shared_table[index_less_cost].second];
        fill_index = 0;
	std::cout << "-----------\n";
        while(fill_index <= level_shared_table[index_less_cost].second)
        {
                close_to_shared.push_back(w);
		
		//DEUG
		std::cout << w->point[0] << " "
                          << w->point[1] << " "
                          << w->point[2] << " "
			  << w->point[3] << " "
                          << w->point[4] << " "
                          << w->point[5] << "\n";
		
                w = (sst_node_t*)w->parent;
                fill_index ++;
        }
	std::cout << "-----------\n";


	

	//TODO:check if it's in the neighboord of root
	bool is_almost_root = false;
	
	if (close_to_shared.size()==1)
	{
		for(int k=0;k<num_close_neighbors; k++)
        	{
        		//Expand the node until we reach root
                	tree_node_t * temp_node_2 = (tree_node_t*)close_neighbors[k];
			if (temp_node_2 == close_to_shared[close_to_shared.size()-1])
			{
				is_almost_root = true;
			}
		}
        }	
	
	std::cout << "almost rooot: " << is_almost_root << "\n";	
	
	//     and if they are the same
	if (close_to_shared.size()==1) 
	{
		if (close_to_shared[close_to_shared.size()-1] == root || is_almost_root)
		{
			close_root = true;
		}
			
	}

	* close_is_root = close_root;

	return true;
	
}
