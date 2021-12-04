#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>  
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string.h>

class pub_sub
{


private:
ros::NodeHandle n; 
ros::Subscriber sub;
ros::Publisher pub1; 
std::string frame;
tf::TransformBroadcaster odom_broadcaster;
	
public:
  	pub_sub(char* type){
		if(strcmp(type,"0")==0){
  			sub = n.subscribe("/swiftnav/front/gps_pose", 1, &pub_sub::callback1, this);
			pub1 = n.advertise<nav_msgs::Odometry>("front/odom", 1);
			frame="car";
		}
		else{
			sub = n.subscribe("/swiftnav/obs/gps_pose", 1, &pub_sub::callback1, this);
			pub1 = n.advertise<nav_msgs::Odometry>("obs/odom", 1);
			frame="obs";
		}
	}

void callback1(const sensor_msgs::NavSatFix::ConstPtr& msg){
  ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude,msg->altitude);

  // fixed values

  double a = 6378137;
  double b = 6356752.3142;
  double f = (a - b) / a;
  double e_sq = f * (2-f);
  float deg_to_rad = 0.0174533;
  
  // input data from msg
  float latitude = msg->latitude;
  float longitude = msg->longitude;
  float h = msg->altitude;

  // fixed position
  
  float latitude_init;
  float longitude_init;
  float h0;
  n.getParam("/origin_long", longitude_init);
  n.getParam("/origin_lat", latitude_init);
  n.getParam("/origin_alt", h0);
  ROS_INFO("%s", frame.c_str());

  //lla to ecef
  float lamb = deg_to_rad*(latitude);
  float phi = deg_to_rad*(longitude);
  float s = sin(lamb);
  float N = a / sqrt(1 - e_sq * s * s);

  float sin_lambda = sin(lamb);
  float  cos_lambda = cos(lamb);
  float  sin_phi = sin(phi);
  float  cos_phi = cos(phi);

  float  x = (h + N) * cos_lambda * cos_phi;
  float  y = (h + N) * cos_lambda * sin_phi;
  float  z = (h + (1 - e_sq) * N) * sin_lambda;
  


  // ecef to enu
 
  lamb = deg_to_rad*(latitude_init);
  phi = deg_to_rad*(longitude_init);
  s = sin(lamb);
  N = a / sqrt(1 - e_sq * s * s);

  sin_lambda = sin(lamb);
  cos_lambda = cos(lamb);
  sin_phi = sin(phi);
  cos_phi = cos(phi);

  float  x0 = (h0 + N) * cos_lambda * cos_phi;
  float  y0 = (h0 + N) * cos_lambda * sin_phi;
  float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

  float xd = x - x0;
  float  yd = y - y0;
  float  zd = z - z0;

  float  xEast = -sin_phi * xd + cos_phi * yd;
  float  yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
  float  zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;


  ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth,zUp);
  
  //fill & publish transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = frame;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  if(msg->latitude==0 && msg->longitude==0 && msg->altitude==0){
	double nan=std::numeric_limits<double>::quiet_NaN();	
	odom_trans.transform.translation.x = nan;
  	odom_trans.transform.translation.y = nan;
  	odom_trans.transform.translation.z = nan;
  }
  else{
  	odom_trans.transform.translation.x = xEast;
  	odom_trans.transform.translation.y = yNorth;
  	odom_trans.transform.translation.z = zUp;
  }
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);

  // fill & publish odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp= ros::Time::now();
  odom.header.frame_id=frame;
  if(msg->latitude==0 && msg->longitude==0 && msg->altitude==0){
	double nan=std::numeric_limits<double>::quiet_NaN();	
	odom.pose.pose.position.x = nan;
  	odom.pose.pose.position.y = nan;
  	odom.pose.pose.position.z = nan;
  }
  else{
  	odom.pose.pose.position.x = xEast;
  	odom.pose.pose.position.y = yNorth;
  	odom.pose.pose.position.z = zUp;
  }
  pub1.publish(odom);
}

};


int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");
	pub_sub my_pub_sub(argv[1]);
  	ros::spin();

  return 0;
}


