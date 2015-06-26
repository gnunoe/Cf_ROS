
/* 
 * File:   trajecory.hpp
 * Author: Gonzalo
 *
 * Created on 17 de abril de 2015, 11:01
 */

#ifndef TRAJECTORY_HPP
#define	TRAJECTORY_HPP

#include <iostream>
#include <cstdlib>
#include <cmath>

class Trajectory{
    
public:

    double ** path_points;
    double ** TNB;
    int points;

    //Constructor
    Trajectory(){};
    
    //Destructor
    virtual ~Trajectory()
    {
        std::cout << "Destructor superclass called \n";
        for (int i=0; i<points; i++)
        {
            delete [] path_points[i];
        }
        for (int i=0; i<points-1; i++)
        {
        delete [] TNB[i];
        } 
        
        delete [] path_points;
        delete [] TNB;
    };
    
    //Norm function
    double getNorm (double p1, double p2, double p3)
    {
        return sqrt(p1*p1 + p2*p2 + p3*p3);
    }
    
    //Show path
    void show_points()
    {
        for (int i=0; i<points;i++)
        {
            std::cout << path_points[i][0] <<" "<< path_points[i][1] <<" "<< 
                         path_points[i][2] <<" "<< path_points[i][3] <<" "<< 
                         path_points[i][4] <<" "<< path_points[i][5] <<" "<< 
                         path_points[i][6] <<" "<< path_points[i][7] <<" "<< 
                         path_points[i][8] <<"\n";
        }
    }

    //Show TNB
    void show_TNB()
    {
        for (int i=0; i<points-1;i++)
        {
            std::cout << TNB[i][0] <<" "<< TNB[i][1] <<" "<< 
                         TNB[i][2] <<" "<< TNB[i][3] <<" "<< 
                         TNB[i][4] <<" "<< TNB[i][5] <<" "<< 
                         TNB[i][6] <<" "<< TNB[i][7] <<" "<< 
                         TNB[i][8] <<"\n";
        }
    }
    
    //Generate points
    virtual void Generate_points(){std::cout<<"I don't have points\n";};
    
    //Generate Tangent-Normal-Binormial Array
    void Generate_TNB()
    {
        double norm_t;
        //Normal, binormal and tanget vectors
        for (int i=0; i<points-1;i++)
        {
            //Tangent
            TNB[i][0] = path_points[i+1][0] - path_points[i][0];
            TNB[i][1] = path_points[i+1][1] - path_points[i][1];
            TNB[i][2] = path_points[i+1][2] - path_points[i][2];
            //And normalize it
            norm_t = getNorm(TNB[i][0],TNB[i][1],TNB[i][2]);
            TNB[i][0] = TNB[i][0]/norm_t;
            TNB[i][1] = TNB[i][1]/norm_t;
            TNB[i][2] = TNB[i][2]/norm_t;

            //Normal
            TNB[i][3] = 0;
            TNB[i][4] = 0;
            TNB[i][5] = 1;

            //Binormal
            TNB[i][6] = TNB[i][1]*TNB[i][5] -TNB[i][2]*TNB[i][4];
            TNB[i][7] = (TNB[i][0]*TNB[i][5] -TNB[i][2]*TNB[i][3])*(-1);
            TNB[i][8] = TNB[i][0]*TNB[i][4] -TNB[i][1]*TNB[i][3];
        }
    }

protected:    
    int getPoints()
    {
        return points;
    }
    
    void setPoints(int n_points)
    {
        points = n_points;
    }
    
    void allocateMemory()
    {
        path_points = new double*[points];
        TNB = new double*[points-1];
        
        for (int i=0; i<points; i++)
        {
            path_points[i] = new double [9];
        }
        
        for (int i=0; i<points-1; i++)
        {
            TNB[i] = new double [9];
        }
    }
};


#endif	/* TRAJECTORY_HPP */

