/** 
* This demo checks which elevator door is open (left or right) and try to drive into the elevator
* 
* 
* 
* 
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <open_door_detector/detect_open_door.h>

// global variables
ros::Time last_button_msg_time;	// last time a button has been pressed
int current_goal = 0; // currently active goal

// function declarations
void createPoseFromParams(std::string param_name, geometry_msgs::PoseStamped* pose);
void buttonCb(std_msgs::Bool msg);
bool last_button_state = false;
void createOffsetPose(geometry_msgs::PoseStamped* goal, float posX_off, float posY_off, float rotW_off);

// position parameter
float posX_off;
float posY_off;
float rotW_off;
float angle;
float dist;
float width;





// define state names
enum state {
	START, START_POS, WAIT_BUTTON, WAIT_ELEVATOR, DRIVE_DOOR, DRIVE_IN, ROTATION 
};

int main(int argc, char** argv) {
	
	// init ROS
	ros::init(argc, argv, "state_machine_elevator");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	// set up publishers
	ros::Publisher speech_pub = nh.advertise<std_msgs::String>("/speech", 1);
	
	// set up subscribers
	ros::Subscriber button_sub = nh.subscribe<std_msgs::Bool>("/pushed", 1, buttonCb);
	
	// client for navigation goals
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
	
	// init time
	ros::Time ask_time;
	last_button_msg_time = ros::Time(0);
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	
	// init state machine
	state st = START;
	
	// wait for other nodes to start up
	ros::Duration(10).sleep();
	
	// tf listener
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	geometry_msgs::PoseStamped door_pos;
	
	
      
	
	while(ros::ok()) {
		


		switch(st) {
			case START: {
				ROS_INFO("START");
				createPoseFromParams("start", &(goal.target_pose));
				client.sendGoal(goal);
				
				st=START_POS;
				ROS_INFO("Switching to state %d", st);
				break;
				
			}
			
			case START_POS: {
				//ROS_INFO("START_POS");

				
				if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					st = WAIT_BUTTON;
					ROS_INFO("Reached start position");
					ROS_INFO("Switching to state %d", st);
				}
				
				break;
			}
			case WAIT_BUTTON: {
				
				
				std_msgs::String say_callelevator;
				say_callelevator.data = "Please call Elevator";
				speech_pub.publish(say_callelevator);
				
				//ROS_INFO("Waiting for button");
				if(last_button_state) { // person called elevator
					st = WAIT_ELEVATOR;
					ROS_INFO("Waiting for elevator");
					ROS_INFO("Switching to state %d", st);

				}
				break;
			}
			
			case WAIT_ELEVATOR: {
				//check which door is open and get posX, posY, rotW and choose next state
				
				//initialze service
				ros::ServiceClient client = nh.serviceClient<open_door_detector::detect_open_door>("detect_open_door");
				ros::ServiceClient mapclient = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
				open_door_detector::detect_open_door srv;
				std_srvs::Empty mapSrv;
				
				
				ros::param::get("/goals/doorparams/apertureangle", angle);
				ros::param::get("/goals/doorparams/walldistance", dist);
				ros::param::get("/goals/doorparams/mindoorwidth", width);
				//angle =0;
				//dist = 2;
				//width = 0.5;
				
				srv.request.aperture_angle = angle;
				srv.request.wall_distance = dist;
				srv.request.min_door_width = width;
				
				
				if (client.call(srv)){
				  
				  door_pos = srv.response.door_pos;
				  
				} else {
				  ROS_ERROR("Failed to call service detect_open_door");
				  return 1;
				  
				}	
				posX_off = door_pos.pose.position.x;
				posY_off = door_pos.pose.position.y;
				
				if(posX_off != 0 || posY_off != 0) {
				   ros::Duration(2).sleep();
				   if (client.call(srv)){
				  
				    door_pos = srv.response.door_pos;
				    
				    } else {
				      ROS_ERROR("Failed to call service detect_open_door");
				      return 1;
				    
				    }	
				    posX_off = door_pos.pose.position.x;
				    posY_off = door_pos.pose.position.y;
				    if(posX_off != 0 || posY_off != 0) {
					st = DRIVE_DOOR; 
					ROS_INFO("posX is: %f", posX_off);
					ROS_INFO("posY is: %f", posY_off);
					mapclient.call(mapSrv);
					ros::Duration(0.5).sleep();
				    }
				}
				
				
				ROS_INFO("Driving to Door");
				ROS_INFO("Switching to state %d", st);
				
				
				
				break;
			}
			
			case DRIVE_DOOR: {
				//calculate the left point from inertial point + kinect data
				createOffsetPose(&(goal.target_pose), posX_off, posY_off, rotW_off);
				
				/*
				ros::Time now = ros::Time::now();
				listener.waitForTransform("map", "base_footprint", now, 2.0);
				listener.lookupTransform("map", "base_footprint", now, transform);
				
				tf::Transform toGoal;
				toGoal.setOrigin(tf::Vector3(x, y, 0));
				toGoal.setRotation(tf::Quaternion(tf::Vector3(0,0,1), phi));
				
				tf::Transform final = transform * toGoal;
				
				pose->pose.position.x
				goal.x = final.getOrigin().y();
				goal.x = final.getOrigin().phi();
				*/

				//createPoseFromParams("goal_1", &(goal.target_pose));
				client.sendGoal(goal);
				st=DRIVE_IN;
				
				break;
			}
			
			case DRIVE_IN: {
				if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					
					ROS_INFO("Im at the door");
					
					ros::param::get("/goals/endPose/x", posX_off);
					ros::param::get("/goals/endPose/y", posY_off);
					ros::param::get("/goals/endPose/w", rotW_off);
									
					createOffsetPose(&(goal.target_pose), posX_off, posY_off, rotW_off);
					client.sendGoal(goal);
					st = ROTATION;
					ROS_INFO("Driving into the elevator");
					ROS_INFO("Switching to state %d", st);
				}
				
				break;
				
			}
			
			case ROTATION: {
				if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				  //ROS_INFO("Im in the elevator!");
				  
				  
				}
				break;
			}	
		}
	ros::spinOnce();
		
	}
	
	
	
	

	
	return 0;
}

void buttonCb(std_msgs::Bool msg) {
	last_button_msg_time = ros::Time::now();
	last_button_state = msg.data;
}

/** 
 * Function that extracts a PoseStamped representation from the
 * parameter group param_name of the goals namespace and stores it in 
 * the provided.
 * 
 * INPUT:	std::string param_name: name of the PoseStamped parameter
 * 			geometry_msgs::PoseStamped* pose: reference to the pose that
 * 										    	to be changed
 * OUTPUT:	none
 */
void createPoseFromParams(std::string param_name, geometry_msgs::PoseStamped* pose) {
	pose->header.stamp = ros::Time::now();
	ros::param::get("/goals/"+param_name+"/x", pose->pose.position.x);
	ros::param::get("/goals/"+param_name+"/y", pose->pose.position.y);
	ros::param::get("/goals/"+param_name+"/z", pose->pose.position.z);
	ros::param::get("/goals/"+param_name+"/q1", pose->pose.orientation.x);
	ros::param::get("/goals/"+param_name+"/q2", pose->pose.orientation.y);
	ros::param::get("/goals/"+param_name+"/q3", pose->pose.orientation.z);
	ros::param::get("/goals/"+param_name+"/q4", pose->pose.orientation.w);
}

void createOffsetPose(geometry_msgs::PoseStamped* goal, float posX_off, float posY_off, float rotW_off) {
	goal->header.stamp = ros::Time::now();
	goal->pose.position.x = goal->pose.position.x + posX_off;
	goal->pose.position.y= goal->pose.position.y + posY_off;
	
	tf::Quaternion quat(tf::Vector3(0,0,1), rotW_off);
	goal->pose.orientation.x = quat.x();
	goal->pose.orientation.y = quat.y();
	goal->pose.orientation.z = quat.z();
	goal->pose.orientation.w = quat.w();
	
}




