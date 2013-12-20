#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

#define RATE 20

int main(int argc, char** argv){
	ros::init(argc, argv, "vel_control");

	std::string goal_tf_name, proj_tf_frame;
	double p_lin, p_ang, vmax_lin,vmax_ang;

	ros::NodeHandle node;

	node.param<std::string>("goal_frame", goal_tf_name, "goal");
	node.param<std::string>("projection_frame", proj_tf_frame, "fixed_goal");
	node.param("p_lin", p_lin, 0.5);
	node.param("p_ang", p_ang, 1.0);
	node.param("vmax_lin", vmax_lin, 0.3);
	node.param("vmax_ang", vmax_ang, 0.5);

	ros::Publisher vel_msg_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	tf::TransformListener listener;

	ros::Rate rate(RATE);
	bool foundOnce = false;
	
	while (node.ok()){

	  tf::StampedTransform transform;
	  geometry_msgs::Twist vel_msg;
	  
	  try {
	    if (!foundOnce)
	    {
	    	ros::Time now = ros::Time::now();
		listener.waitForTransform("base_footprint", proj_tf_frame, now, ros::Duration(5.0) );
	    }
	    
	    foundOnce = true;
	    listener.lookupTransform("base_footprint", proj_tf_frame, ros::Time(0), transform);
	    
	    if(fabs(transform.getOrigin().x()) > 0.15)
	    {
	      vel_msg.linear.x = p_lin * transform.getOrigin().x();
	    }
	    else
	    {
	      vel_msg.linear.x = 0.0;
	    }
	    tfScalar roll, pitch, yaw;
	    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	    vel_msg.angular.z = p_ang * yaw;

	    if(fabs(vel_msg.linear.x) > vmax_lin) vel_msg.linear.x = vmax_lin * vel_msg.linear.x/fabs(vel_msg.linear.x);
	    if(fabs(vel_msg.angular.z) > vmax_ang) vel_msg.angular.z = vmax_ang * vel_msg.angular.z/fabs(vel_msg.angular.z);
	    
	  } catch (tf::TransformException ex) {
	    ROS_INFO("No pattern detected yet!");
	    vel_msg.linear.x = 0.0;
	    vel_msg.angular.z = 0.0;
	  }	  

	  vel_msg_pub.publish(vel_msg);
	  
	  ros::spinOnce();
	  rate.sleep();
	}
	
	return 0;
	
};
