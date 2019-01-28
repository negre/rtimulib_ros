#include "RTIMU_Node.h"

#include <iostream>

using namespace std;

namespace rtimulib_ros
{
#if 0
}
#endif

static const double G_TO_MPSS = 9.80665;

RTIMU_Node::RTIMU_Node()
{
}

void RTIMU_Node::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  std::string calibration_file_path;
  if(!private_nh.getParam("calibration_file_path", calibration_file_path))
  {
    ROS_ERROR("The calibration_file_path parameter must be set to use a "
              "calibration file.");
    ROS_BREAK();
  }

  std::string calibration_file_name = "RTIMULib";
  if(!private_nh.getParam("calibration_file_name", calibration_file_name))
  {
    ROS_WARN_STREAM("No calibration_file_name provided - default: "
                    << calibration_file_name);
  }
  
  if(!private_nh.getParam("frame_id", frame_id))
  {
    ROS_WARN_STREAM("No frame_id provided - default: " << frame_id);
  }

  private_nh.param("thrust_coeff_ax", thrust_coeff_ax, 0.); // 1.57627
  private_nh.param("thrust_coeff_ay", thrust_coeff_ay, 0.); //-0.850739);
  private_nh.param("thrust_coeff_az", thrust_coeff_az, 0.); //2.69143);
  private_nh.param("thrust_coeff_bx", thrust_coeff_bx, 0.);
  private_nh.param("thrust_coeff_by", thrust_coeff_by, 0.);
  private_nh.param("thrust_coeff_bz", thrust_coeff_bz, 0.);
  
  
  // Load the RTIMULib.ini config file
  settings = new RTIMUSettings(calibration_file_path.c_str(),
                               calibration_file_name.c_str());
  
  imu = RTIMU::createIMU(settings);
  
  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
  {
    ROS_ERROR("No Imu found");
    ROS_BREAK();
  }

  // Initialise the imu object
  imu->IMUInit();
  
  // Set the Fusion coefficient
  imu->setSlerpPower(0.02);
  // Enable the sensors
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);
  
  isCalibrating = false;
  
  imuPub = nh.advertise<sensor_msgs::Imu>("imu", 1);
  outputSub = nh.subscribe("output_raw", 10, &RTIMU_Node::outputCallback, this);

  calibService = nh.advertiseService("calibrate_thrust_mag", &RTIMU_Node::beginCalibration, this);
  
}

void RTIMU_Node::run()
{
  ros::Rate loop_rate(1000./imu->IMUGetPollInterval());
  while (ros::ok())
  {
    if (imu->IMURead())
    {
      RTIMU_DATA imu_data = imu->getIMUData();

//       cout << imu_data.compass.x() << " "
//            << imu_data.compass.y() << " "
//            << imu_data.compass.z() <<endl;

      
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = frame_id;
      
      imu_msg.orientation.x = imu_data.fusionQPose.x(); 
      imu_msg.orientation.y = imu_data.fusionQPose.y(); 
      imu_msg.orientation.z = imu_data.fusionQPose.z(); 
      imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 
      
      imu_msg.angular_velocity.x = imu_data.gyro.x();
      imu_msg.angular_velocity.y = imu_data.gyro.y();
      imu_msg.angular_velocity.z = imu_data.gyro.z();
      
      imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
      imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
      imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;
      
      imuPub.publish(imu_msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool RTIMU_Node::beginCalibration(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
{
  sw2 = 0.;
  sw = 0.;
  swtx = 0.;
  swty = 0.;
  swtz = 0.;
  stx = 0.;
  sty = 0.;
  stz = 0.;
  stx0 = 0.;
  sty0 = 0.;
  stz0 = 0.;
  s = 0;
  n = 0;
  
  isCalibrating = true;

  ROS_INFO("Begin thrust mag calibration");

  cout << "Begin thrust mag calibration" << endl;
  
  return true;
}

void RTIMU_Node::outputCallback(const rosflight_msgs::OutputRaw::ConstPtr& msg)
{
  float w = 0.f;
  for(int i=0; i<4; i++)
  {
    w+=msg->values[i];
  }
  
  RTIMU_DATA imu_data = imu->getIMUData();
  RTVector3 compass = imu_data.compass;
  
  if(isCalibrating)
  {
    if(w==0)
    {
      n++;
      stx0 += compass.x();
      sty0 += compass.y();
      stz0 += compass.z();      
    }else{
      s++;
      stx += compass.x();
      sty += compass.y();
      stz += compass.z();
      
      sw2 += w*w;
      sw += w;
      
      swtx += w*compass.x();
      swty += w*compass.y();
      swtz += w*compass.z();
    }

    cout << "get " << n << " / " << s << " values" << endl;
    
    if( n > 100 && s > 500)
    {
      double tx0 = stx0 / n;
      double ty0 = sty0 / n;
      double tz0 = stz0 / n;
      
      double det = s*sw2 - sw*sw;
      
      thrust_coeff_ax = (s*swtx-sw*stx)/det;
      thrust_coeff_ay = (s*swty-sw*sty)/det;
      thrust_coeff_az = (s*swtz-sw*stz)/det;
      
      thrust_coeff_bx = (sw*(-swtx+sw*tx0)+sw2*(stx-s*tx0))/det;
      thrust_coeff_by = (sw*(-swty+sw*ty0)+sw2*(sty-s*ty0))/det;
      thrust_coeff_bz = (sw*(-swtz+sw*tz0)+sw2*(stz-s*tz0))/det;
      
      isCalibrating = false;

      cout << "Calibration finished : " << endl
           << " tx : " << tx0 << " " << ty0 << " " << tz0 << endl
           << " thrust_coeff_ax = " << thrust_coeff_ax << endl
           << " thrust_coeff_ay = " << thrust_coeff_ay << endl
           << " thrust_coeff_az = " << thrust_coeff_az << endl
           << " thrust_coeff_bx = " << thrust_coeff_bx << endl
           << " thrust_coeff_by = " << thrust_coeff_by << endl
           << " thrust_coeff_bz = " << thrust_coeff_bz << endl;
    }
    
  }
  else{
    imu->setCompassCalVariableOffset(RTVector3(thrust_coeff_ax*w + thrust_coeff_bx,
                                               thrust_coeff_ay*w + thrust_coeff_by,
                                               thrust_coeff_az*w + thrust_coeff_bz));
  }

  /*
    cout << sum << " "
    << compass.x() - offset.x() << " " 
    << compass.y() - offset.y() << " " 
    << compass.z() - offset.z() << endl;
  */
}

}; // namespace rtimulib_ros
