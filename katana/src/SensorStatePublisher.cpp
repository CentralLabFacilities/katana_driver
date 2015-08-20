#include "katana/SensorStatePublisher.h"

namespace katana
{

SensorStatePublisher::SensorStatePublisher(boost::shared_ptr<AbstractKatana> katana) :
  katana(katana)
{
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::JointState> ("sensor_states", 1000);
}

SensorStatePublisher::~SensorStatePublisher()
{
}

void SensorStatePublisher::update()
{
  /* ************** Publish joint angles ************** */
  sensor_msgs::JointStatePtr msg = boost::make_shared<sensor_msgs::JointState>();
  std::vector<std::string> sensor_names = katana->getGripperSensorNames();
  std::vector<short> readings = katana->getGripperSensorReadings();

  for (size_t i = 0; i < NUM_GRIPPER_SENSORS && i < readings.size(); i++)
  {
    msg->name.push_back(sensor_names[i].c_str());
    msg->position.push_back((float)readings[i]);
    msg->velocity.push_back(0);
  }

  msg->header.stamp = ros::Time::now();
  pub.publish(msg); // NOTE: msg must not be changed after publishing; use reset() if necessary (http://www.ros.org/wiki/roscpp/Internals)
}

}
