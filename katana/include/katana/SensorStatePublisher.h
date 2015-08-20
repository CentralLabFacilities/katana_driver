#ifndef SENSORSTATEPUBLISHER_H_
#define SENSORSTATEPUBLISHER_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <vector>

#include <katana/AbstractKatana.h>

namespace katana
{

class SensorStatePublisher
{
public:
	SensorStatePublisher(boost::shared_ptr<AbstractKatana>);
  virtual ~SensorStatePublisher();
  void update();

private:
  boost::shared_ptr<AbstractKatana> katana;
  ros::Publisher pub;

};

}

#endif /* SENSORSTATEPUBLISHER_H_ */
