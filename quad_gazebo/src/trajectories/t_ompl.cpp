

#include "quad_gazebo/trajectory.hpp"
#include "quad_gazebo/t_ompl.hpp"

#include <cmath>



void t_ompl::Generate_points()
{
    
    //Position, velocity and acceleration of the points
    for (int i=0; i<points; i++)
    {
        path_points[i][0] = temp_traj[i][0];
        path_points[i][1] = temp_traj[i][1];
        path_points[i][2] = temp_traj[i][2];
        path_points[i][3] = temp_traj[i][3];
        path_points[i][4] = temp_traj[i][4];
        path_points[i][5] = temp_traj[i][5];
        path_points[i][6] = 0;
        path_points[i][7] = 0;
        path_points[i][8] = 0;
    }
    Generate_TNB();
}


void t_ompl::allocateTempTraj()
{
  //Allocate matrix to sort the numbers
  temp_traj = new double*[n_rows];
  for (int i=0; i<n_rows; i++)  
  {
    temp_traj[i] = new double [n_elements];
  }

  //And sort the vector in a 2D array
  for (int i=0; i<n_rows; i++)
  {
    for (int j=0; j<n_elements; j++)
    {
      temp_traj[i][j] = temp_array[temp_array_index];
      temp_array_index ++;
    }
  }
}

void t_ompl::freeTempTraj()
{
  for (int i=0; i<n_rows; i++)
  {
    delete [] temp_traj[i];
  }
  delete [] temp_traj;
}

void t_ompl::readCSV(std::string path_file)
{
  //Open the file
  std::ifstream myFile(path_file.c_str());

  if (! myFile.is_open())
  {
    std::cout << "Could not open the file!\n";
  }

  //get number of lines 
  std::string line;
  while (std::getline(myFile,line))
  {
    ++ n_rows;
  }

  //Reset buffer and point to the beginning of the document
  myFile.clear();
  myFile.seekg(0,std::ios::beg);

  //Save all the values in a vector
  std::string val;
  double d_val;
  while (myFile.good())
  {
    getline(myFile,val,',');
    d_val = atof(val.c_str());
    temp_array.push_back(d_val);
  }

  //And close file
  //ROS_INFO_STREAM("closing file...");
  myFile.close();
}

