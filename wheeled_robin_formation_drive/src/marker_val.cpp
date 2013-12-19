#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <math.h>

#define _USE_MATH_DEFINES
#define WAITING_TIME 3.0f
#define MAX_DIST 0.1f
#define SEARCH_TIME_SEC 60
#define RATE 10


int main(int argc, char** argv){
  ros::init(argc, argv, "marker_validation");
  if(argc != 1){
    ROS_ERROR("Error in argument count");
  }
  
  std::string marker_tf_name, goal_tf_name, dist_x_str, dist_z_str;
  double dist_x, dist_z;
 
  ros::NodeHandle node;
  
  node.param<std::string>("marker_frame", marker_tf_name, "master_pattern");
  node.param<std::string>("goal_frame", goal_tf_name, "goal");
  node.param("x_offset", dist_z, 1.0);
  node.param("y_offset", dist_x, 0.0);
    
  tf::TransformBroadcaster broadcast;
  tf::TransformListener listener;
  tf::StampedTransform last_o2m;
  ros::Publisher pos_msg_pub = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  
  ros::Rate rate(RATE);
  bool searching = false , initialized = false;
  ros::Time search_start;
  
  while (node.ok())
  {
    try
    {
      tf::StampedTransform odom2marker;
      
      ros::Time now = ros::Time::now();
      listener.waitForTransform("odom", marker_tf_name, now, ros::Duration(WAITING_TIME));
      listener.lookupTransform("odom", marker_tf_name, now, odom2marker);

      tf::Transform marker2goal;     
      marker2goal.setOrigin( tf::Vector3(dist_x, 0.0, dist_z) );
      //marker2goal.setOrigin( tf::Vector3(-1,0,0));
      marker2goal.setRotation( tf::Quaternion(tf::Vector3(0,0,1), 0) );
      
      tf::Transform odom2goal;
      odom2goal = odom2marker*marker2goal;
      
      last_o2m = odom2marker;
      broadcast.sendTransform(tf::StampedTransform(marker2goal, ros::Time::now(), marker_tf_name, goal_tf_name));
      //broadcast.sendTransform(tf::StampedTransform(odom2goal, ros::Time::now(), "odom", goal_tf_name));
            
      searching = false;
      initialized = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("Master Robot was not found! Recovery Behaviour started!");
      tf::StampedTransform odom2base;
      listener.lookupTransform("odom", "base_footprint", ros::Time(0), odom2base);
      tf::Vector3 diff_origin = odom2base.getOrigin()-last_o2m.getOrigin();

      if (initialized)
      {
	if(true)//diff_origin.length() >= MAX_DIST)
	{
	    broadcast.sendTransform(tf::StampedTransform(last_o2m, ros::Time::now(), "odom", goal_tf_name));
	} 
	else
	{
	    if(!searching)
	    {
		searching = true;
		search_start = ros::Time::now();
		
	    }
	    
	    if((ros::Time::now() - search_start).sec < SEARCH_TIME_SEC)
	    {
		tf::Transform rot_z;     
		rot_z.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		rot_z.setRotation( tf::Quaternion(tf::Vector3(0,0,1), M_PI/2) );
		
		broadcast.sendTransform(tf::StampedTransform(odom2base*rot_z, ros::Time::now(), "odom", goal_tf_name));
	    }
	    else
	    {
		broadcast.sendTransform(tf::StampedTransform(odom2base, ros::Time::now(), "odom", goal_tf_name));
	    } // else
	} // else
      } // if(initialized)
    } // catch
    
    geometry_msgs::PoseStamped msg;
    try
    {
      tf::StampedTransform trans;
      listener.waitForTransform("map", goal_tf_name, ros::Time(0), ros::Duration(WAITING_TIME));
      listener.lookupTransform("map", goal_tf_name, ros::Time(0), trans);
      
      tfScalar roll, pitch, yaw;
      tf::Matrix3x3(trans.getRotation()).getRPY(roll, pitch, yaw);
      tf::Quaternion quat(tf::Vector3(0,0,1), yaw - M_PI / 2.0);
      
      tf::StampedTransform fixedTrans;
      fixedTrans.setOrigin(tf::Vector3(trans.getOrigin().x(),trans.getOrigin().y(),0));
      fixedTrans.setRotation(quat);
      broadcast.sendTransform(tf::StampedTransform(fixedTrans, ros::Time::now(), "map", "fixed_goal"));
      
      msg.pose.position.x = trans.getOrigin().x();
      msg.pose.position.y = trans.getOrigin().y();
      msg.pose.position.z = 0;
      
      msg.pose.orientation.w = quat.w();
      msg.pose.orientation.x = quat.x();
      msg.pose.orientation.y = quat.y();
      msg.pose.orientation.z = quat.z();
      
      msg.header.frame_id = "map";
      msg.header.stamp = ros::Time::now();
      
      //pos_msg_pub.publish(msg);
    }
    catch (tf::TransformException ex)
    {

    }

    ros::spinOnce();
    rate.sleep();
    
  }

  
  return 0;
};
