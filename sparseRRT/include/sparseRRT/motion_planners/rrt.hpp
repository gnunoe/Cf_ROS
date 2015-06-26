/**
 * @file rrt.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_PLANNER_RRT_HPP
#define SPARSE_PLANNER_RRT_HPP

#include "sparseRRT/systems/system.hpp"
#include "sparseRRT/motion_planners/planner.hpp"

/**
 * @brief The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 * @details The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 */
class rrt_t : public planner_t
{
public:
	/**
	 * @copydoc planner_t::planner_t(system_t*)
	 */
	rrt_t(system_t* in_system) : planner_t(in_system)
	{

	}
	virtual ~rrt_t(){}

	/**
	 * @copydoc planner_t::setup_planning()
	 */
	virtual void setup_planning();

	/**
	 * @copydoc planner_t::get_solution(std::vector<std::pair<double*,double> >&)
	 */
	virtual void get_solution(std::vector<std::pair<double*,double> >& controls);

	/**
	 * @copydoc planner_t::step()
	 */
	virtual void step();


	/**
         * @copydoc planner_t::path_between_nodes(double x_s, double y_s, double z_s,
         *                                        double x_g, double y_g, double z_g)
         */
	//We must define it in rrt, but we only want to use it
	//with sst, so we implement it as above.
	virtual bool path_between_nodes(double x_s, double y_s, double z_s,
                                        double x_g, double y_g, double z_g,
					bool smooth);

        void get_parent(int jumps, tree_node_t * node);

protected:

        int search_points_to_smooth(std::deque<tree_node_t*> &small_segment, std::deque<tree_node_t*> &big_segment,
                                    std::vector <tree_node_t*> &neighbor, double old_cost);
                                    //std::vector <std::pair<int,int> > &nodes_sharing_neighbors, std::vector <tree_node_t*> &shared_neighbor);

        void smooth_path(std::deque<tree_node_t*> &start_path, std::deque<tree_node_t*> &goal_path, std::vector<double> &sg, double old_cost);

        /**
         * @brief Search the shared node between two nodes.
         * @details Search the shared node between two nodes. Get the tree start-root 
         * and goal-root, and search the closest common node, to get the lowest cost
         * 
         * @param num_close_neighbors number of neighbors of the point closer to root.
         * @param close_neighbors list with the neighbors nodes of the closer point.
         * @param close_to_shared result list with the nodes from the closer point to the shared node.
         * @param num_far_neighbors number of neighbors of the point father to root.
         * @param far_neighbors list with the neighbors nodes of the father point.
         * @param far_to_shared result list with the nodes from the farther point to the shared node.
         *
         * @return true if a shared node has been found, false if not.
         */
        bool search_shared_node(int num_close_neighbors, std::deque<tree_node_t*> &close_neighbors, std::deque<tree_node_t*> &close_to_shared,
                                int num_far_neighbors,  std::deque<tree_node_t*> &far_neighbors, std::deque<tree_node_t*> &far_to_shared,
                                double * cost, bool * close_is_root);

        /**
         * @brief Compose the trajectory between two nodes.
         * @details Compose the trajectory between two nodes, joining the paths segments 
         * start - shared_node and shared_node - goal.
         * 
         * @param start_traj vector which holds the nodes between start and shared_node.
         * @param goal_traj vector which holds the nodes between goal and shared_node.
         * @param traj result vector holding the sorted nodes between start and goal.
         *
         */
        void compose_trajectory(const std::deque<tree_node_t*> &start_traj, const std::deque<tree_node_t*> &goal_traj, std::vector<double> &sg,
                                bool smooth, const std::vector<tree_node_t*> &shared_neighbor, bool start_is_root, bool goal_is_root);

	
	/**
	 * @brief A randomly sampled state.
	 */
	double* sample_state;

	/**
	 * @brief A randomly sampled control.
	 */
	double* sample_control;


	/**
	 * @brief A resulting duration of a propagation step.
	 */
	double duration;

	/**
	 * @brief Storage used to query the nearest neighbor structure.
	 */
	tree_node_t* metric_query;

	/**
	 * @brief The result of a query in the nearest neighbor structure.
	 */
	tree_node_t* nearest;
	
	/**
	 * @brief A set of nodes used to get solutions.
	 */
	proximity_node_t** close_nodes;
	/**
	 * @brief A set of distances used to get solutions.
	 */
	double* distances;

	/**
	 * @brief Perform the random sampling step of RRT.
	 * @details Perform the random sampling step of RRT.
	 */
	void random_sample();

	/**
	 * @brief Find the nearest node to the randomly sampled state.
	 * @details Find the nearest node to the randomly sampled state.
	 */
	void nearest_vertex();

	/**
	 * @brief Perform a local propagation from the nearest state.
	 * @details Perform a local propagation from the nearest state.
	 * @return If the trajectory is collision free or not.
	 */
	bool propagate();

	/**
	 * @brief If propagation was successful, add the new state to the tree.
	 * @details If propagation was successful, add the new state to the tree.
	 */
	void add_to_tree();

	/**
	 * @brief Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * @details Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * 
	 * @param node The node to add.
	 */
	void add_point_to_metric(tree_node_t* node);


};

#endif
