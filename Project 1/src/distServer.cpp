#include "ros/ros.h"
#include "project_1/distServ.h"
#include "math.h"

bool calcDist(project_1::distServ::Request  &req,
         project_1::distServ::Response &res)
{	
  ROS_INFO("car pos: %f,%f,%f",req.car_x,req.car_y,req.car_z);
  ROS_INFO("car pos: %f,%f,%f",req.obs_x,req.obs_y,req.obs_z);
  res.dist = sqrt(pow((req.car_x-req.obs_x),2)+pow((req.car_y-req.obs_y),2)+pow((req.car_z-req.obs_z),2));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distServer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("distServ", calcDist);
  ROS_INFO("Ready to calc dist");
  ros::spin();

  return 0;
}
