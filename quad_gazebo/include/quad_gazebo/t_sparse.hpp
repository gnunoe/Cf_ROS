
/* 
 * File:   t_sparse.hpp
 * Author: Gonzalo
 *
 * Created on 17 de abril de 2015, 11:20
 */

#ifndef T_SPARSE_HPP
#define	T_SPARSE_HPP

#include "quad_gazebo/trajectory.hpp"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

class t_sparse :public Trajectory
{
public:

    //Constructor with the path of the file    
    t_sparse(std::string path_file)
    {
        n_elements = 12;
        n_rows = 0;
        temp_array_index = 0;
        
        readCSV(path_file);       
        allocateTempTraj();

        setPoints(n_rows);
        allocateMemory(); 
        Generate_points();

        freeTempTraj();
    }
    
    virtual void Generate_points();
    
private:
    int n_rows;    
    int n_elements;

    std::vector <double> temp_array;
    int temp_array_index;

    double **temp_traj;
    
    void allocateTempTraj();
      
    void freeTempTraj();
   
    void readCSV(std::string path_file);
  
};


#endif	/* T_SPARSE_HPP */
