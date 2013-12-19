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

	ros::Rate rate(20.0);

	while (node.ok()){

	  tf::StampedTransform transform;
	  geometry_msgs::Twist vel_msg;
	  
	  try {
	    ros::Time now = ros::Time::now();
	    //listener.waitForTransform("base_footprint", goal_tf_name, now, ros::Duration(1.0) );
	    //listener.lookupTransform("base_footprint", goal_tf_name, now, transform);
	    listener.waitForTransform("base_footprint", "fixed_goal", now, ros::Duration(1.0) );
	    listener.lookupTransform("base_footprint", "fixed_goal", now, transform);
	    
	    double r_euklid = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

	    //if(r_euklid < 0.05){
	      
	    // vel_msg.linear.x = 0;
	      //vel_msg.angular.z = 4 * transform.getRotation().z();
	      
	    //} else {
	      if(fabs(transform.getOrigin().x()) > 0.1)
	      {
		vel_msg.linear.x = 2 * transform.getOrigin().x();
	      }
	      else
	      {
		vel_msg.linear.x = 0;
	      }
	      tfScalar roll, pitch, yaw;
	      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	      vel_msg.angular.z = yaw;
	      ROS_INFO("x off=%3.2f; y off=%3.2f; speed=%3.2f", transform.getOrigin().x(), transform.getOrigin().y(), r_euklid);
	    //} //if(r_euklid)
	    
	    if(fabs(vel_msg.linear.x) > 0.3) vel_msg.linear.x = 0.3*vel_msg.linear.x/fabs(vel_msg.linear.x);
	    if(fabs(vel_msg.angular.z) > 0.5) vel_msg.angular.z = 0.5*vel_msg.angular.z/fabs(vel_msg.angular.z);
	    
	  } catch (tf::TransformException ex) {
	    ROS_INFO("%s",ex.what());
	    vel_msg.linear.x = 0;
	    vel_msg.angular.z = 0;
	  }
	 // vel_msg.linear.x = 0;
	  

	  vel_msg_pub.publish(vel_msg);
	  ros::spinOnce();
	  rate.sleep();
	}
	
	return 0;
	
};
