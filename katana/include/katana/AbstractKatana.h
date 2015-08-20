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
 * AbstractKatana.h
 *
 *  Created on: 20.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef ABSTRACTKATANA_H_
#define ABSTRACTKATANA_H_

#include <ros/ros.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <katana/SpecifiedTrajectory.h>
#include <katana/katana_constants.h>

#include <moveit_msgs/JointLimits.h>

namespace katana
{

class AbstractKatana
{
public:
  AbstractKatana();
  virtual ~AbstractKatana();

  virtual void refreshEncoders() = 0;
  virtual void refreshSensors() = 0;
  virtual bool executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj,
                                 boost::function<bool()> isPreemptRequested) = 0;
  virtual void freezeRobot();
  virtual void freezeMotor(int motorIndex);

  /**
   * Move the joint to the desired angle. Do not wait for result,
   * but return immediately.
   *
   * @param jointIndex the joint to move
   * @param turningAngle the target angle
   * @return true iff command was successfully sent to Katana
   */
  virtual bool moveJoint(int jointIndex, double turningAngle) = 0;

  virtual int getJointIndex(std::string joint_name);
  virtual int getGripperSensorIndex(std::string senosr_name);

  virtual std::vector<std::string> getJointNames();
  virtual std::vector<int> getJointTypes();

  virtual std::vector<std::string> getGripperJointNames();
  virtual std::vector<int> getGripperJointTypes();

  virtual std::vector<std::string> getGripperSensorNames();

  virtual std::vector<double> getMotorAngles();
  virtual std::vector<double> getMotorVelocities();

  /**
   * Get the readings from the sensor gripper. Readings range from 0 to 255.
   *
   * For distance sensors readings correlate to the measured distance, but also
   * slightly depend on the surrounding lighting conditions.
   * The reading's measuring unit roughly equals millimeters for small distances.
   *
   * For force sensors readings correlate to the measured force applied to the
   * sensors. A value of 0 means no measured force, 255 means maximal measurable force.
   *
   * Index map:
   *
   * 0:  Right finger, distance sensor, inside, near to wrist
   * 1:  Right finger, distance sensor, inside, far from wrist
   * 2:  ??
   * 3:  ??
   * 4:  Right finger, distance sensor, outside
   * 5:  Right finger, distance sensor, tip
   * 6:  Right finger, force sensor,    inside, near to wrist
   * 7:  Right finger, force sensor,    inside, far from wrist
   * 8:  Left finger,  distance sensor, inside, near to wrist
   * 9:  Left finger,  distance sensor, inside, far from wrist
   * 10: ??
   * 11: Wrist      ,  distance sensor, between fingers
   * 12: Left finger,  distance sensor, outside
   * 13: Left finger,  distance sensor, tip
   * 14: Left finger,  force sensor,    inside, near to wrist
   * 15: Left finger,  force sensor,    inside, far from wrist
   *
   */
  virtual std::vector<short> getGripperSensorReadings();

  virtual std::vector<moveit_msgs::JointLimits> getMotorLimits();
  virtual double getMotorLimitMax(std::string joint_name);
  virtual double getMotorLimitMin(std::string joint_name);

  virtual void refreshMotorStatus();
  virtual bool someMotorCrashed() = 0;
  virtual bool allJointsReady() = 0;
  virtual bool allMotorsReady() = 0;


protected:
  // only the 5 "real" joints:
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;

  // the two "finger" joints of the gripper:
  std::vector<std::string> gripper_joint_names_;
  std::vector<int> gripper_joint_types_;

  std::vector<std::string> sensor_names_;

  // all motors (the 5 "real" joints plus the gripper)

  std::vector<double> motor_angles_;
  std::vector<double> motor_velocities_;

  // the motor limits of the 6 motors

  std::vector<moveit_msgs::JointLimits> motor_limits_;

  // readings from gripper's distance sensors

  std::vector<short> gripper_sensor_readings_;
};

}

#endif /* ABSTRACTKATANA_H_ */
