#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/String.h"
#include <math.h>  
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include "project_1/distServ.h"
#include <cmath>
#include <cfloat>
#include "project_1/Status.h"
#include <dynamic_reconfigure/server.h>
#include <project_1/dynaParamConfig.h>
float thresh1;
float thresh2;
class Pub_sub
{


private:
 
  ros::Publisher pub;
  project_1::distServ srv;
  ros::ServiceClient client;
  ros::NodeHandle n; 
  project_1::Status statMsg;
  
public:
  	Pub_sub(ros::NodeHandle nin){
		ROS_INFO("In constructor");
		n=nin;
		client = n.serviceClient<project_1::distServ>("distServ");
		pub=n.advertise<project_1::Status>("status",1);
		
		
	}


void callbackSub(const nav_msgs::Odometry::ConstPtr& carMsg,const  nav_msgs::Odometry::ConstPtr& obsMsg){
	ROS_INFO("In here");
	double car_x=carMsg->pose.pose.position.x;
	double car_y=carMsg->pose.pose.position.y;
	double car_z=carMsg->pose.pose.position.z;
	double obs_x=obsMsg->pose.pose.position.x;
	double obs_y=obsMsg->pose.pose.position.y;
	double obs_z=obsMsg->pose.pose.position.z;
	if(!(std::isnan(car_x) || std::isnan(car_y) || std::isnan(car_z) || std::isnan(obs_x) || std::isnan(obs_y) || std::isnan(obs_z))){
		srv.request.car_x = car_x;
		srv.request.car_y = car_y;
		srv.request.car_z = car_z;
		srv.request.obs_x = obs_x;
		srv.request.obs_y = obs_y;
		srv.request.obs_z = obs_z;
		if (client.call(srv))
  		{
    			ROS_INFO("Dist: %f", srv.response.dist);
			statMsg.dist=srv.response.dist;
			if(srv.response.dist<thresh1){
				statMsg.flag="Crash";
			}
			else if(srv.response.dist<thresh2){
				statMsg.flag="Unsafe";
			}
			else{
				statMsg.flag="Safe";
			}
			pub.publish(statMsg);
				
  		}
	}
	else{
		statMsg.dist=std::numeric_limits<double>::quiet_NaN();	
		statMsg.flag="GPS signal lost";
		pub.publish(statMsg);
	}
}

};

void paramCallback(project_1::dynaParamConfig &config, uint32_t level) {

	thresh1=config.threshold_1_param;
	thresh2=config.threshold_2_param;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "customMessagePub");
  ROS_INFO("Ready to start");
  ros::NodeHandle n; 
  n.getParam("/thresh1", thresh1);
  n.getParam("/thresh2", thresh2);
  message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "car_convert/front/odom", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "obs_convert/obs/odom", 1);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  Pub_sub my_pub_sub(n);
  sync.registerCallback(boost::bind(&Pub_sub::callbackSub,&my_pub_sub,_1,_2));
  dynamic_reconfigure::Server<project_1::dynaParamConfig> paramServer;
  dynamic_reconfigure::Server<project_1::dynaParamConfig>::CallbackType cb;
  cb=boost::bind(&paramCallback, _1, _2);
  paramServer.setCallback(cb);
  ros::spin();

  return 0;
}
