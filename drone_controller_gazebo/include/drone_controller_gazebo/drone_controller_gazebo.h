
#ifndef DRONE_CONTROLLER_GAZEBO_H
#define DRONE_CONTROLLER_GAZEBO_H

#include <vector>
#include <string>

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>

#include <gazebo_ros_control/robot_hw_sim.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>


#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <drone_controller/drone_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include "drone_msgs_srvs/WayPoint.h"


#include <hardware_interface/joint_command_interface.h>

namespace drone_controller_gazebo
{

using namespace drone_controller;

class DroneControllerGazebo : public gazebo_ros_control::RobotHWSim, public drone_controller::DroneStateInterface
{
public:
  DroneControllerGazebo();
  virtual ~DroneControllerGazebo();

  bool initSim(const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

  void imuCallback(const sensor_msgs::ImuConstPtr &imu);
  void pathCallback(const drone_msgs_srvs::WayPointConstPtr &path);
  void dronePositionEstimate();


  void getDroneEulerRPY(sensor_msgs::Imu &imu, double *roll_, double *pitch_, double *yaw_) const;
  void getDroneEulerRPYVelocity(sensor_msgs::Imu &imu, double *roll_, double *pitch_, double *yaw_);
  void getDronePosition();
  void getDroneVelocity(sensor_msgs::Imu &imu);


  void setRotorsVelocities(double* T_, double* tau_phi_, double* tau_theta_, double* tau_psi_);
  void pidDroneControler(geometry_msgs::Pose::_position_type& desiredPosision_, double* T_,double* tau_phi_, double* tau_theta_, double* tau_psi_, double *roll_, double *pitch_, double *yaw_);



private:

  ros::NodeHandle *model_nh;
  boost::mutex lock;

  double pitch, yaw, roll;
  gazebo::math::Vector3 force;
  gazebo::math::Vector3 torque;


  geometry_msgs::Pose drone_pos_;
  geometry_msgs::Twist drone_vel_;
  sensor_msgs::Imu drone_imu_;

  std::vector<double> drone_rotors_vel_;

  gazebo::physics::LinkPtr body_main;
  sensor_msgs::Imu drone_sensor_imu_;
  sensor_msgs::Imu drone_sensor_imu;


  // Drone Hardware interface
  DroneStateInterface    drone_state_interface_;


  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;

  gazebo::physics::LinkPtr *body_motors;

  gazebo::physics::PhysicsEnginePtr physics_;

  std::string base_link_frame_, world_frame_;

  gazebo::math::Pose gz_pose_;
  gazebo::math::Vector3 gz_velocity_, gz_acceleration_, gz_angular_velocity_;

  ros::CallbackQueue callback_queue_;
  ros::Subscriber subscriber_imu_;

  ros::CallbackQueue callback_queue_path_;

  std::vector<double> rotor_position;
  std::vector<double> rotor_velocity;
  std::vector<double> rotor_effort;

  std::vector<double> rotor_command;

  std::vector<gazebo::physics::JointPtr> rotors;
  std::vector<gazebo::physics::LinkPtr> links;
  gazebo::physics::JointPtr* joints;

  drone_msgs_srvs::WayPoint drone_path_;
  ros::Subscriber subscriber_path_;


  // Hardware interface: joints
  hardware_interface::JointStateInterface    rotors_state_interface;
  hardware_interface::ImuSensorInterface drone_imu_interface;

  unsigned int n_rotors;
  unsigned int n_links;

  gazebo::math::Vector3 motor_force[4];

  double prop_speed[4];
  double omega_square[4];

  double Kz, Kphi, Ktheta, Kpsi, Dphi, Dtheta, Dpsi, Dz;
  double T, tau_phi, tau_theta, tau_psi;

  geometry_msgs::Pose::_position_type desiredPosision;

};

} // namespace arm_bot_hardware_gazebo

#endif // ARM_BOT_HARDWARE_GAZEBO_H
