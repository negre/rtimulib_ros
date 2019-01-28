#ifndef __RTIMU_NODE_H__
#define __RTIMU_NODE_H__

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rosflight_msgs/OutputRaw.h>
#include <std_srvs/Empty.h>
#include <RTIMULib.h>

namespace rtimulib_ros
{
  
class RTIMU_Node
{
protected:
  ros::Subscriber outputSub;
  ros::Publisher imuPub;
  ros::ServiceServer calibService;
    
  RTIMUSettings *settings;
  RTIMU *imu;

  std::string frame_id;
  sensor_msgs::Imu imu_msg;

  double thrust_coeff_ax, thrust_coeff_ay, thrust_coeff_az;
  double thrust_coeff_bx, thrust_coeff_by, thrust_coeff_bz;
  

  bool isCalibrating;
  double sw2, sw, swtx, swty, swtz, stx, sty, stz, stx0, sty0, stz0;
  int s, n;
  
  bool beginCalibration(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);
    
  void outputCallback(const rosflight_msgs::OutputRaw::ConstPtr& msg);
  
public:
  RTIMU_Node();

  void init(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  void run();
  
};

}; // namespace rtimulib_ros

#endif // __RTIMU_NODE_H__
