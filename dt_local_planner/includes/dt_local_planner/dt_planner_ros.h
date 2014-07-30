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
* Author: Austin Hendrix
*********************************************************************/
#ifndef DT_LOCAL_PLANNER_DT_PLANNER_ROS_H_
#define DT_LOCAL_PLANNER_DT_PLANNER_ROS_H_

// SHUT UP BOOST SIGNALS
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <dt_local_planner/DTPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace dt_local_planner {
  /**
   * @class DTPlannerROS
   * @brief ROS Wrapper for the AckermannPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DTPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DTPlannerROS
       */
      DTPlannerROS();

      /**
       * @brief  Constructs the planner
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      virtual void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the planner
       */
      virtual ~DTPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached();

    private:
      bool isInitialized() {
        return initialized_;
      }

      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DTPlannerConfig &config, uint32_t level);

      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      costmap_2d::Costmap2DROS* costmap_ros_;

      // TODO(hendrix): shared pointer?
      dynamic_reconfigure::Server<DTPlannerConfig> *dsrv_;

      bool initialized_;

      base_local_planner::OdometryHelperRos odom_helper_;

      std::vector<geometry_msgs::PoseStamped> plan_;
      double goal_x_ ;
      double goal_y_;

      std::vector<double> aCoeff_x_;
      std::vector<double> aCoeff_y_;
      
      int nPolyGrad_;
      int planSize_;
      double start_time_;
      double lastCall_time_;
      double vel_dest0_;
      double gamma_dest0_;

      //controller params
      double K_p_;
      double K_d_;

      // Limits
      double max_vel_;
      double max_ang_;
      double max_vel_deltaT0_;
      int stepCnt_max_vel_;


      // configuration
      bool move_;
      bool firstComputeVelocityCommands_;

      // transient data

      bool goal_reached_;
  };

  // Helper functions that don't need class context

  /**
   * @brief determine if one point is forward or backwards from another
   */
  bool isForwards(geometry_msgs::PoseStamped &start,
      geometry_msgs::PoseStamped &end);

  /**
   * @brief compute the distance between two points
   */
  double dist(geometry_msgs::PoseStamped &start,
      geometry_msgs::PoseStamped &end);

};
#endif
