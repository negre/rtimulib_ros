#include "RTIMU_Node.h"
#include <ros/ros.h>

int main(int argc, char**argv)
{
  ros::init(argc, argv, "rtimu_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  rtimulib_ros::RTIMU_Node rtimu_node;

  rtimu_node.init(nh, private_nh);
  rtimu_node.run();
}
