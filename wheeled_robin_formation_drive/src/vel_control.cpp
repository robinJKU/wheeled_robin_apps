#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "vel_control");

	std::string goal_tf_name;
	double p_lin, p_ang;

	ros::NodeHandle node;

	node.param<std::string>("goal_frame", goal_tf_name, "goal");
	node.param("p_lin", p_lin, 4.0);
	node.param("p_ang", p_ang, 0.5);

	ros::Publisher vel_msg_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	tf::TransformListener listener;

	ros::Rate rate(100.0);

	while (node.ok()){

		tf::StampedTransform transform;

		try {
			listener.waitForTransform("base_footprint", goal_tf_name, ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("base_footprint", goal_tf_name, ros::Time(0), transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}


		geometry_msgs::Twist vel_msg;

		vel_msg.angular.z = p_lin * atan2(transform.getOrigin().y(), transform.getOrigin().x());
		vel_msg.linear.x = p_ang * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
		ROS_INFO("Speed: %3.2f", vel_msg.linear.x);
		vel_msg_pub.publish(vel_msg);

		rate.sleep();
	}

	return 0;
};
