/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Austin Hendrix
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Stephan Hofstaetter
*********************************************************************/

#include <dt_local_planner/dt_planner_ros.h>
#include <dt_local_planner/polyfit.hpp>
#include <cmath>
#include <limits>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dt_local_planner::DTPlannerROS, nav_core::BaseLocalPlanner)

namespace dt_local_planner {

  void DTPlannerROS::reconfigureCB(DTPlannerConfig &config, uint32_t level) {
      max_vel_ = config.max_vel;
      max_ang_ = config.max_ang;
      max_vel_deltaT0_ = config.max_vel_deltaT0;
      stepCnt_max_vel_ = config.stepCnt_max_vel;
      nPolyGrad_ = config.nPolyGrad;
      K_p_ = config.K_p;
      K_d_ = config.K_d;
      move_ = config.move;
  }

  DTPlannerROS::DTPlannerROS() : initialized_(false),goal_reached_(false), firstComputeVelocityCommands_(true) {

  }

  void DTPlannerROS::initialize(
    std::string name,
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      tf_ = tf;
      costmap_ros_ = costmap_ros;

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      std::string odom_topic;
      private_nh.param<std::string>("odom_topic", odom_topic, "odom");
      odom_helper_.setOdomTopic( odom_topic );
      
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<DTPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DTPlannerConfig>::CallbackType cb = boost::bind(&DTPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool DTPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //getting the plan
    plan_ = orig_global_plan;
    planSize_ = plan_.size();
    ROS_INFO_NAMED("dt_local_planner", "Got new plan with size: %d",planSize_);

    //setting our goal
    goal_x_ = plan_[planSize_-1].pose.position.x;
    goal_y_ = plan_[planSize_-1].pose.position.y;

    //declaration of the polyfit parameter
    std::vector<double> p_x (planSize_, 0.);
    std::vector<double> p_y (planSize_, 0.);
    std::vector<double> p_t (planSize_, 0.);

    //generating p_x,p_y out of the global plan
    for(int index = 0;index < planSize_;index++){
      p_x[index] = plan_[index].pose.position.x;
      p_y[index] = plan_[index].pose.position.y;
      //ROS_INFO_NAMED("dt_local_planner", "p_x %f",p_x[index]);
    }
    ROS_INFO_NAMED("dt_local_planner", "p_x,p_y generated!");
  
    //calculating time between each points with x = v * t
    double max_vel_deltaT;
    double deltaR;
    double deltaT;
    
    //calculating time between each points with x = v * t
    //with a weighting of the first steps until the full speed should apply

    for(int index = 1;index < planSize_;index++){
      deltaR = sqrt( pow(p_x[index] - p_x[index-1], 2.0) + pow(p_y[index] - p_y[index-1], 2.0) );
      if(index <= stepCnt_max_vel_){
        max_vel_deltaT = max_vel_deltaT0_*index/stepCnt_max_vel_;
      }
      deltaT = deltaR / max_vel_deltaT;
      p_t[index] = p_t[index-1] + deltaT;
      //ROS_INFO_NAMED("dt_local_planner", "p_t %f",p_t[index]);
    }
    ROS_INFO_NAMED("dt_local_planner", "p_t generated!");

    //define polygrad
    //nPolyGrad_ = 15;//planSize_-1;

    //calculation of the polynom coefficiencies from a0 to an
    //http://vilipetek.com/2013/10/07/polynomial-fitting-in-c-using-boost/
    aCoeff_x_ = polyfit(p_t, p_x, nPolyGrad_);
    aCoeff_y_ = polyfit(p_t, p_y, nPolyGrad_);
    ROS_INFO_NAMED("dt_local_planner", "Calculated polyfit for x and y with grade %d",
    nPolyGrad_);

    //we are at he beginning; goal_reached_ set to false;
    goal_reached_ = false;
    // return false here if we would like the global planner to re-plan
    ROS_INFO_NAMED("dt_local_planner", "----------------------");
    return true;
  }

  bool DTPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    return goal_reached_;
  }

  DTPlannerROS::~DTPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }

  bool DTPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // if we don't have a plan, what are we doing here???
    if( plan_.size() < 2) {
      return false;
    }

    if(firstComputeVelocityCommands_){
      start_time_ = ros::Time::now().toSec();
      lastCall_time_ = start_time_;
    }

    //use our current pose
    tf::Stamped<tf::Pose> current_pose;
    costmap_ros_->getRobotPose(current_pose);
    double pos_cur_x = current_pose.getOrigin().x();
    double pos_cur_y = current_pose.getOrigin().y();
    double gamma_cur = tf::getYaw(current_pose.getRotation());
    ROS_INFO_NAMED("dt_local_planner", "Current position (costmap) is (%f, %f) with Angular (%f)",
        pos_cur_x, pos_cur_y, gamma_cur);

    //check if we are at the goal
    double dist_x = fabs(goal_x_ - current_pose.getOrigin().x());
    double dist_y = fabs(goal_y_ - current_pose.getOrigin().x());
    ROS_INFO_NAMED("dt_local_planner", "Current distance to goal (%f, %f)",dist_x, dist_y);
  
    //(dist_x > 0.00001 || dist_y > 0.00001) && 
    if(move_){//we are not at the goal, compute on when we are allowed to move
      ROS_INFO_NAMED("dt_local_planner", "preparing to move:"); 
  
      //get the current velocity -> vel_cur
      tf::Stamped<tf::Pose> robot_vel;
      odom_helper_.getRobotVel(robot_vel);
      double vel_cur_lin = robot_vel.getOrigin().x();
      double vel_cur_ang = robot_vel.getOrigin().y();
      
      /* //different way to get the velocity
      nav_msgs::Odometry odom;
      odom_helper_.getOdom(odom);
      double vel_cur_lin = odom.twist.twist.linear.x;
      double vel_cur_ang = odom.twist.twist.angular.z;
      */
      ROS_INFO_NAMED("dt_local_planner", "Current robot velocity (lin/ang): (%f/%f)",	vel_cur_lin,vel_cur_ang);

      //calculate time in seconds
      double current_time = ros::Time::now().toSec() - start_time_;
      ROS_INFO_NAMED("dt_local_planner", "time in seconds: %f",current_time); 

      std::vector<double> time_vec (nPolyGrad_, 0.);

      for(int index = 0; index < nPolyGrad_;index++){
        time_vec[index] = pow(current_time,index);
        //ROS_INFO_NAMED("dt_local_planner", "calculated poly_t: %f",t_vec[index]); 
      }
      //ROS_INFO_NAMED("dt_local_planner", "calculated poly_t index n-1: %f",t_vec[nPolyGrad_-1]); 

      double temp_x = 0;
      double temp_y = 0;
      double pos_dest_x = 0;
      double pos_dest_y = 0;
      double vel_dest_x = 0;
      double vel_dest_y = 0;      
      double acc_dest_x = 0;
      double acc_dest_y = 0;

      for(int i=0;i<nPolyGrad_;i++)
      {
        temp_x = aCoeff_x_[i]*time_vec[i];
        temp_y = aCoeff_y_[i]*time_vec[i];
        pos_dest_x += temp_x;
        pos_dest_y += temp_y;
      }
      ROS_INFO_NAMED("dt_local_planner", "p_x0=%f and p_y0=%f",pos_dest_x,pos_dest_y);
      for(int i=1;i<nPolyGrad_-1;i++)
      {
        temp_x = aCoeff_x_[i]*i*time_vec[i-1];
        temp_y = aCoeff_y_[i]*i*time_vec[i-1];
        vel_dest_x += temp_x;
        vel_dest_y += temp_y;
      }
      ROS_INFO_NAMED("dt_local_planner", "p_x1=%f and p_y1=%f",vel_dest_x,vel_dest_y);
      for(int i=2;i<nPolyGrad_-2;i++)
      {
        temp_x = aCoeff_x_[i]*i*(i+1)*time_vec[i-2];
        temp_y = aCoeff_y_[i]*i*(i+1)*time_vec[i-2];
        acc_dest_x += temp_x;
        acc_dest_y += temp_y;
      }
      ROS_INFO_NAMED("dt_local_planner", "p_x2=%f and p_y2=%f",acc_dest_x,acc_dest_y);

      //calculate only at the first run
      if(firstComputeVelocityCommands_){
        double vel_dest_x0 = vel_dest_x;
        double vel_dest_y0 = vel_dest_y;
        vel_dest0_ = sqrt(fabs(vel_dest_x0) + fabs(vel_dest_y0));
        ROS_INFO_NAMED("dt_local_planner", "sqrt(vel_dest_x0(%f)+vel_dest_y0(%f))=vel_dest0_(%f)",vel_dest_x0,vel_dest_y0,vel_dest0_);
      }

      //dynamic trajectory
      double lambda_x = acc_dest_x + K_d_ * (vel_dest_x - vel_cur_lin * cos(gamma_cur)) + K_p_ * (pos_dest_x - pos_cur_x);
      double lambda_y = acc_dest_y + K_d_ * (vel_dest_y - vel_cur_lin * sin(gamma_cur)) + K_p_ * (pos_dest_y - pos_cur_y);
      ROS_INFO_NAMED("dt_local_planner", "lambda x, y: (%f, %f)",lambda_x, lambda_y);

      double acc_l = cos( gamma_cur ) * lambda_x + sin( gamma_cur ) * lambda_y;

      
      double omega = 0;
      if(fabs(vel_cur_lin) > 0.001){
        omega = (-sin(gamma_cur) * lambda_x + cos(gamma_cur) * lambda_y) / vel_cur_lin;
      }//else omega = 0

      //euler integragtion
      //delta_time  = time between calls of the computeVelocityCommands function
      double delta_time = ros::Time::now().toSec() - lastCall_time_;
      ROS_INFO_NAMED("dt_local_planner", "deltaT in seconds: %f",delta_time); 

      double vel_dest1 = vel_dest0_ + acc_l * delta_time;
      ROS_INFO_NAMED("dt_local_planner", "vel_dest0_ + acc_l: (%f, %f)", vel_dest0_, acc_l*delta_time);
      double gamma_dest1 = gamma_dest0_ + omega * delta_time;

      //saving params for the next call
      vel_dest0_ = vel_dest1;
      gamma_dest0_ = gamma_dest1;
      lastCall_time_ = current_time;

      //setting the velocity commands
      ROS_INFO_NAMED("dt_local_planner", "Trying to set velocity and angular speed: (%f, %f)",
        vel_dest1, gamma_dest1);
      if(max_vel_ < fabs(vel_dest1)){
        ROS_WARN_NAMED("dt_local_planner", "speed exceed: velocity set to: %f m/s",max_vel_);
        cmd_vel.linear.x = max_vel_;
      }else{
        cmd_vel.linear.x = vel_dest1;
      }
      if(max_ang_ < fabs(gamma_dest1)){
        cmd_vel.angular.z = max_ang_;
        ROS_WARN_NAMED("dt_local_planner", "speed exceed: angular velocity set to: %f m/s",max_ang_);
      }else{
        cmd_vel.angular.z = gamma_dest1; 
      }
    }else//goal reached
    {
      ROS_INFO_NAMED("dt_local_planner", "goal reached"); 
      if(!goal_reached_) goal_reached_ = true;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
    }
    
    //false when the first call ended
    if(firstComputeVelocityCommands_) firstComputeVelocityCommands_ = false;

    // return true if we were abtmwtypes.hle to find a path, false otherwise
    ROS_INFO_NAMED("dt_local_planner", "----------------------");
    return true;
     
    
  }

};
