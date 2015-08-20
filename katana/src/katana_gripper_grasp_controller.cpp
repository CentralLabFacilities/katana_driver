/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * katana_gripper_grasp_controller.cpp
 *
 *  Created on: 29.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 * based on pr2_gripper_grasp_controller
 *
 */

#include <katana/katana_gripper_grasp_controller.h>

namespace katana
{

KatanaGripperGraspController::KatanaGripperGraspController(boost::shared_ptr<AbstractKatana> katana) :
  katana_(katana)
{
  ros::NodeHandle root_nh("");
  ros::NodeHandle pn("~");

  pn.param<double> ("goal_threshold", goal_threshold_, 0.01);
  pn.param<int> ("force_threshold", force_threshold_, 140);

  std::string query_service_name = root_nh.resolveName("gripper_grasp_status");
  query_srv_ = root_nh.advertiseService(query_service_name, &KatanaGripperGraspController::serviceCallback, this);
  ROS_INFO_STREAM("katana gripper grasp query service started on topic " << query_service_name);

  std::string gripper_grasp_posture_controller = root_nh.resolveName("gripper_grasp_posture_controller");
  action_server_ = new actionlib::SimpleActionServer<control_msgs::GripperCommandAction>(root_nh,
      gripper_grasp_posture_controller, boost::bind(&KatanaGripperGraspController::executeCB, this, _1), false);
  action_server_->start();
  ROS_INFO_STREAM("katana gripper grasp hand posture action server started on topic " << gripper_grasp_posture_controller);

  std::string gripper_grasp_posture_with_sensors_controller = root_nh.resolveName("gripper_grasp_posture_with_sensors_controller");
  action_sensor_server_ = new actionlib::SimpleActionServer<control_msgs::GripperCommandAction>(root_nh,
		  gripper_grasp_posture_with_sensors_controller, boost::bind(&KatanaGripperGraspController::executeCBwithSensor, this, _1), false);
  action_sensor_server_->start();
  ROS_INFO_STREAM("katana gripper grasp with sensors hand posture action server started on topic " << gripper_grasp_posture_with_sensors_controller);

}

KatanaGripperGraspController::~KatanaGripperGraspController()
{
  delete action_server_;
  delete action_sensor_server_;
}

void KatanaGripperGraspController::setKatana(boost::shared_ptr<AbstractKatana> katana)
{
  katana_ = katana;
}

void KatanaGripperGraspController::executeCB(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  ROS_INFO("Moving gripper to position: %f", goal->command.position);
  bool moveJointSuccess = katana_->moveJoint(GRIPPER_INDEX, goal->command.position);
  // wait for gripper to open/close
  ros::Duration(GRIPPER_OPENING_CLOSING_DURATION).sleep();
  
  control_msgs::GripperCommandResult result;
  result.position = katana_->getMotorAngles()[GRIPPER_INDEX];
  result.reached_goal = false;
  result.stalled = false;
  
  if (moveJointSuccess && fabs(result.position - goal->command.position) < goal_threshold_)
  {  
    ROS_INFO("Gripper goal reached");
    result.reached_goal = true;
    action_server_->setSucceeded(result); 
  }
  else if (moveJointSuccess && fabs(result.position - goal->command.position) > goal_threshold_)
  {
    ROS_INFO("Gripper stalled.");
    result.stalled = true;
    action_server_->setSucceeded(result);
  }
  else
  {
    ROS_WARN("Goal position (%f) outside gripper range. Or some other stuff happened.", goal->command.position);
    action_server_->setAborted(result);
  }


}


void KatanaGripperGraspController::executeCBwithSensor(const control_msgs::GripperCommandGoalConstPtr &goal)
{

  ros::NodeHandle root_nh("");
  ROS_INFO("Moving gripper to position: %f", goal->command.position);
  bool moveJointSuccess = katana_->moveJoint(GRIPPER_INDEX, goal->command.position);
  // wait for gripper to open/close
  ros::Duration duration(GRIPPER_OPENING_CLOSING_DURATION);
  ros::Time begin = ros::Time::now();

  // Sensors not required when opening
  if (goal->command.position > katana_->getMotorAngles()[GRIPPER_INDEX]) {
    while ( !isGoalReached(goal) && ros::Time::now() < begin + duration) {
      usleep(10 * 1000);
    }
  } else {
    while ( !isForceLimitReached() && !isGoalReached(goal) && ros::Time::now() < begin + duration) {
      usleep(10 * 1000);
    }

    katana_->freezeMotor(GRIPPER_INDEX);
  }
  control_msgs::GripperCommandResult result;
  result.position = katana_->getMotorAngles()[GRIPPER_INDEX];
  result.reached_goal = false;
  result.stalled = false;

  if (moveJointSuccess && fabs(result.position - goal->command.position) < goal_threshold_)
  {
    ROS_INFO("Gripper goal reached");
    result.reached_goal = true;
    action_sensor_server_->setSucceeded(result);
  }
  else if (moveJointSuccess && fabs(result.position - goal->command.position) > goal_threshold_)
  {
    ROS_INFO("Gripper stalled.");
    result.stalled = true;
    action_sensor_server_->setSucceeded(result);
  }
  else
  {
    ROS_WARN("Goal position (%f) outside gripper range. Or some other stuff happened.", goal->command.position);
    action_sensor_server_->setAborted(result);
  }


}

bool KatanaGripperGraspController::isForceLimitReached()
{
    std::vector<short> gripper = katana_->getGripperSensorReadings();
    short readingRightNear = gripper[6];
    short readingRightFar = gripper[7];
    short readingLeftNear = gripper[14];
    short readingLeftFar = gripper[15];
    ROS_DEBUG("Check force limit(%i): right,near %i, right,far %i, left,near %i, left,far %i", force_threshold_, readingRightNear, readingRightFar, readingLeftNear, readingLeftFar);
    return ((readingRightNear > (short) force_threshold_
    		|| readingRightFar > (short) force_threshold_)
    		&& (readingLeftNear > (short) force_threshold_
            || readingRightFar > (short) force_threshold_));
}

bool KatanaGripperGraspController::isGoalReached(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  double goal_distance = fabs(katana_->getMotorAngles()[GRIPPER_INDEX] - goal->command.position);
  ROS_DEBUG("Check goal distance (threshold: %f): %f", goal_threshold_, goal_distance);
  return goal_distance < goal_threshold_;
}

bool KatanaGripperGraspController::serviceCallback(control_msgs::QueryTrajectoryState::Request &request, 
                                                   control_msgs::QueryTrajectoryState::Response &response)
{
  response.position.resize(1);
  response.position[0] = katana_->getMotorAngles()[GRIPPER_INDEX];
  return true;
}

}
