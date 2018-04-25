#ifndef POTENTIALFIELD_H // To make sure you don't declare the function more than once by including the header multiple times.
#define POTENTIALFIELD_H

#include <ros/ros.h>
#include <iostream>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <vector>
#include <math.h>

#define PI 3.14159265
// temp values
#define RHO_0 10
#define ETA 10
#define RANGE 5

struct Pixel
{
  float x;
  float y;
  int occupancyValue;
};

struct Potential
{
  float x;
  float y;
};

// define a class, including a constructor, member variables and member functions
class PotentialField
{
public:
  PotentialField(ros::NodeHandle *nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
  void GetSpaceInRange(geometry_msgs::PoseWithCovariance, nav_msgs::OccupancyGrid);
  Pixel GetFreePose();

private:
  // put private member data here;  "private" data will only be available to member functions of this class;
  ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

  //list of pixels
  std::vector<Pixel> obstacles;
  std::vector<Pixel> frees;
  std::vector<Pixel> allSpaces;

  void GetObstacles();
  Potential RepulsivePotential(Pixel pixel, Pixel obstacle);
  std::vector<std::vector<Pixel>> ConvertTo2DArray(int, int, int[]);
};
#endif