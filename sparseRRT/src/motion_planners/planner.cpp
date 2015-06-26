/**
 * @file planner.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "sparseRRT/motion_planners/planner.hpp"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <limits>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>


//#include "ros/ros.h"

void planner_t::set_start_state(double* in_start)
{
	if(start_state==NULL)
		start_state = system->alloc_state_point();
	system->copy_state_point(start_state,in_start);
}

void planner_t::set_goal_state(double* in_goal,double in_radius)
{
	if(goal_state==NULL)
		goal_state = system->alloc_state_point();
	system->copy_state_point(goal_state,in_goal);
	goal_radius = in_radius;
}

void planner_t::visualize_tree(int image_counter)
{
    /*
    std::stringstream s;
    s<<"tree_"<<image_counter<<".svg";
    std::string dir = "/home/estevez/catkin_ws/src/sparseRRT/bin/tree_0.svg";
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));
	
    
    visualize_edge(root,doc,dimensions);

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_path(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);
    doc.save();
    */
}

void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->cost > x->cost)
		{
			nodes[j] = nodes[j-1];
			j--;
		}
		nodes[j] = x;
	}
}

void planner_t::visualize_nodes(int image_counter)
{
    /*
    char buff[PATH_MAX];
    //getcwd( buff, PATH_MAX );
    
    ssize_t len = ::readlink(-d "sparseRRT", buff, sizeof(buff)-1);
    if (len != -1) 
    {	
      buff[len] = '\0';
    }
    
    std::cout << buff;    
    */
    //system("beep");
    //std::stringstream s;
    //std::cout <<"Path: " <<s.str();
    //params::path_file<<"/nodes_"<<image_counter<<".svg";
    std::string dir;
    if (params::planner == "rrt")
    {
      dir = params::path_file + "/nodes_rrt.svg";
    }
    else
    {
      dir = params::path_file + "/nodes_sst.svg";
    }
    //std::cout <<"Path: " <<dir;
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

    //-------------------
    sorted_nodes.clear();
    get_max_cost();
    sort(sorted_nodes);

    
    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    visualize_node(sorted_nodes[i],doc,dimensions);
    }
    //------------------


	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_nodes(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}
void planner_t::visualize_edge(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));

		traj_line<<system->visualize_point(node->point,dim);
		traj_line<<system->visualize_point((*i)->point,dim);
		doc<<traj_line;

		visualize_edge(*i,doc,dim);

	}

}

void planner_t::visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	svg::Circle circle(system->visualize_point(node->point,dim),params::node_diameter,svg::Fill( svg::Color((node->cost/max_cost)*255,(node->cost/max_cost)*255,(node->cost/max_cost)*255) ) );
	doc<<circle;
	// for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	// {
	// 	visualize_node(*i,doc,dim);
	// }

}

void planner_t::visualize_solution_path( svg::Document& doc, svg::Dimensions& dim)
{
	if(last_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(params::solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			traj_line<<system->visualize_point(last_solution_path[i]->point,dim);
		}
		doc<<traj_line;
	}
}


void planner_t::visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim)
{
	std::string dir;
	if (params::planner == "rrt")
	{
	  dir = params::path_file + "/trajectory_rrt.csv";
	}
	else
	{
          dir = params::path_file + "/trajectory_sst.csv";
        }
        //std::cout << system->get_state_dimension();
        if(last_solution_path.size()!=0)
        {
        //Flag to tell there is a solution
        params::availableSolution = true;
        //Open file to write trajectory
        std::ofstream myFile;
        myFile.open(dir.c_str());
        //myFile.open("/home/estevez/catkin_ws/src/sparseRRT/bin/trajectory.csv");
                //Save all the values separated by comas except the last
                for(unsigned i=0; i<last_solution_path.size()-1;i++)
                {
                        myFile << last_solution_path[i]->point[0] <<","
                                  << last_solution_path[i]->point[1] <<","
                                  << last_solution_path[i]->point[2] <<","
                                  << last_solution_path[i]->point[3] <<","
                                  << last_solution_path[i]->point[4] <<","
                                  << last_solution_path[i]->point[5] <<","
                                  << last_solution_path[i]->point[6] <<","
                                  << last_solution_path[i]->point[7] <<","
                                  << last_solution_path[i]->point[8] <<","
                                  << last_solution_path[i]->point[9] <<","
                                  << last_solution_path[i]->point[10] <<","
                                  << last_solution_path[i]->point[11] <<",\n";

                        svg::Circle circle(system->visualize_point(last_solution_path[i]->point,dim),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
                        doc<<circle;
                }
                //Which is saved here, but without a coma in the last element.
                int i = last_solution_path.size()-1;
                myFile << last_solution_path[i]->point[0] <<","
                       << last_solution_path[i]->point[1] <<","
                       << last_solution_path[i]->point[2] <<","
                       << last_solution_path[i]->point[3] <<","
                       << last_solution_path[i]->point[4] <<","
                       << last_solution_path[i]->point[5] <<","
                       << last_solution_path[i]->point[6] <<","
                       << last_solution_path[i]->point[7] <<","
                       << last_solution_path[i]->point[8] <<","
                       << last_solution_path[i]->point[9] <<","
                       << last_solution_path[i]->point[10] <<","
                       << last_solution_path[i]->point[11] <<"\n";

                myFile.close();

        }
}

/*
void planner_t::visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim)
{

    //std::cout << system->get_state_dimension();
	if(last_solution_path.size()!=0)
	{
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
                        std::cout << last_solution_path[i]->point[0] <<" " 
                                  << last_solution_path[i]->point[1] <<" " 
                                  << last_solution_path[i]->point[2] <<" " 
                                  << last_solution_path[i]->point[3] <<" " 
                                  << last_solution_path[i]->point[4] <<" " 
                                  << last_solution_path[i]->point[5] <<" " 
                                  << last_solution_path[i]->point[6] <<" " 
                                  << last_solution_path[i]->point[7] <<" " 
                                  << last_solution_path[i]->point[8] <<" " 
                                  << last_solution_path[i]->point[9] <<" " 
                                  << last_solution_path[i]->point[10] <<" " 
                                  << last_solution_path[i]->point[11] <<"\n";
			svg::Circle circle(system->visualize_point(last_solution_path[i]->point,dim),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}
*/


void planner_t::get_max_cost()
{
	max_cost = 0;
	get_max_cost(root);
}

void planner_t::get_max_cost(tree_node_t* node)
{
	sorted_nodes.push_back(node);
	if(node->cost > max_cost)
		max_cost = node->cost;
	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		get_max_cost(*i);
	}
}

