#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <math.h>

#define _USE_MATH_DEFINES
#define WAITING_TIME 0.4f
#define MAX_DIST 0.2f
#define SEARCH_TIME_SEC 60
#define RATE 20


int main(int argc, char** argv){
  ros::init(argc, argv, "marker_validation");
  if(argc != 1){
    ROS_ERROR("Error in argument count");
  }
  
  std::string marker_tf_name, goal_tf_name, proj_tf_frame;
  double dist_z;
 
  ros::NodeHandle node;
  
  node.param<std::string>("marker_frame", marker_tf_name, "master_pattern");
  node.param<std::string>("goal_frame", goal_tf_name, "goal");
  node.param<std::string>("projection_frame", proj_tf_frame, "fixed_goal");
  node.param("x_offset", dist_z, 1.0);
  
  tf::TransformBroadcaster broadcast;
  tf::TransformListener listener;
  tf::StampedTransform last_o2m;
  ros::Publisher pos_msg_pub = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  
  ros::Rate rate(RATE);
  
  bool searching = false , initialized = false, turn = false;
  ros::Time search_start;
  int turnCnt = 0;
  sleep(5);
  
  while (node.ok())
  {
    try
    {
      tf::StampedTransform odom2marker;
      
      ros::Time now = ros::Time::now();
      listener.waitForTransform("odom", marker_tf_name, now, ros::Duration(WAITING_TIME));
      listener.lookupTransform("odom", marker_tf_name, now, odom2marker);
      if(searching) ROS_INFO("Found again?!");

      tf::Transform marker2goal;     
      marker2goal.setOrigin( tf::Vector3(0.0, 0.0, dist_z) );
      marker2goal.setRotation( tf::Quaternion(tf::Vector3(0,0,1), 0) );
      
      tf::Transform odom2goal;
      odom2goal = odom2marker*marker2goal;
      
      last_o2m = odom2marker;
      broadcast.sendTransform(tf::StampedTransform(marker2goal, ros::Time::now(), marker_tf_name, goal_tf_name));
            
      searching = false;
      initialized = true;
      turn = false;
      turnCnt = 0;
    }
    catch (tf::TransformException ex)
    {
      if (initialized)
      {
	tf::StampedTransform odom2base;
	listener.lookupTransform("odom", "base_footprint", ros::Time(0), odom2base);
	double diff_origin = odom2base.getOrigin().x()-last_o2m.getOrigin().x();
      	searching = true;
	ROS_INFO("Master pattern was not found! Recovery Behaviour started!");

	if(fabs(diff_origin) >= MAX_DIST && !turn)
	{
	    ROS_INFO("Old marker position published");
	} 
	else
	{
	    turn = true; 	    
	    ROS_INFO("Turning robot to search master pattern!");
	} // else
      } // if(initialized)
    } // catch
    
    //geometry_msgs::PoseStamped msg;
    ros::spinOnce();
    
    try
    {
      tf::StampedTransform trans;
      if(!searching)
      {
	listener.waitForTransform("map", goal_tf_name, ros::Time(0), ros::Duration(WAITING_TIME));
	listener.lookupTransform("map", goal_tf_name, ros::Time(0), trans);
      }
      else 
      {
	trans = last_o2m;
      }
      
      tfScalar roll, pitch, yaw;
      tf::Matrix3x3(trans.getRotation()).getRPY(roll, pitch, yaw);
      tf::Quaternion quat(tf::Vector3(0,0,1), yaw - M_PI / 2.0);
      tf::StampedTransform fixedTrans;
      fixedTrans.setOrigin(tf::Vector3(trans.getOrigin().x(),trans.getOrigin().y(),0));
      fixedTrans.setRotation(quat);
      if(!searching)	broadcast.sendTransform(tf::StampedTransform(fixedTrans, ros::Time::now(), "map", proj_tf_frame));
      else if(!turn)	broadcast.sendTransform(tf::StampedTransform(fixedTrans, ros::Time::now(), "odom", proj_tf_frame));
      else
      {
	tf::Transform rot_z;     
	rot_z.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	rot_z.setRotation( tf::Quaternion(tf::Vector3(0,0,1), M_PI/10 ) );
	broadcast.sendTransform(tf::StampedTransform(rot_z, ros::Time::now(), "base_footprint", proj_tf_frame));
      }
      

      /*
      msg.pose.position.x = trans.getOrigin().x();
      msg.pose.position.y = trans.getOrigin().y();
      msg.pose.position.z = 0;
      
      msg.pose.orientation.w = quat.w();
      msg.pose.orientation.x = quat.x();
      msg.pose.orientation.y = quat.y();
      msg.pose.orientation.z = quat.z();
      
      msg.header.frame_id = "map";
      msg.header.stamp = ros::Time::now();
      
      //pos_msg_pub.publish(msg);*/
    }
    catch (tf::TransformException ex)
    {

    }

    ros::spinOnce();
    rate.sleep();

  }

  
  return 0;
};
