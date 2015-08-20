
#include <angles/angles.h>
#include <math.h>

#include <urdf_parser/urdf_parser.h>

#include <drone_controller_gazebo/drone_controller_gazebo.h>
#include <drone_controller/drone_interface.h>

namespace drone_controller_gazebo
{
  using namespace hardware_interface;
  using namespace drone_controller;


  DroneControllerGazebo::~DroneControllerGazebo()
  {

  }

  DroneControllerGazebo::DroneControllerGazebo()
  {
    this->registerInterface(static_cast<DroneStateInterface *>(this));
  }



  bool DroneControllerGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {

    ros::NodeHandle param_nh(model_nh, "controller");
    // store parent model pointer
    model_ = parent_model;
    physics_ = model_->GetWorld()->GetPhysicsEngine();

    model_nh.param<std::string>("world_frame", world_frame_, "world");
    model_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    // Link Interface
    // Cleanup
    links.clear();

    // Get Links
    links = model_->GetLinks();

    // Number of links
    n_links = links.size();
    ROS_INFO_STREAM(n_links);

    // Get links name
    std::vector<std::string> lnk_names;
    for (size_t i = 0; i < n_links; ++i)
    {
    	lnk_names.push_back(links[i]->GetName());
    	ROS_INFO_STREAM(links[i]->GetName());
    }

    drone_rotors_vel_.resize(4);

    drone_state_interface_.registerHandle(
        		DroneStateHandle(lnk_names[0], &drone_pos_, &drone_vel_, &drone_imu_, &drone_rotors_vel_));

    ROS_INFO_STREAM("Registered link '" << lnk_names[0] << "' in the DroneStateInterface.");


    // Simulation joints
    // Cleanup
//    joints.clear();
    rotor_position.clear();
    rotor_velocity.clear();
    rotor_effort.clear();
    rotor_command.clear();

	body_main = parent_model->GetLink("base_link");
    // Get Joints
    rotors = model_->GetJoints();

	joints = new gazebo::physics::JointPtr[4];
	body_motors = new gazebo::physics::LinkPtr[4];

    // Number of joint
    n_rotors = rotors.size();

    ROS_INFO_STREAM(n_rotors);

    std::vector<std::string> rotor_names;
    for (size_t i = 0; i < n_rotors; ++i)
    {
    	rotor_names.push_back(rotors[i]->GetName());
    }

    // Raw data
    rotor_position.resize(n_rotors);
    rotor_velocity.resize(n_rotors);
    rotor_effort.resize(n_rotors);
    rotor_command.resize(n_rotors);


    rotors[0]->SetMaxForce(2, 100000.0);
    rotors[1]->SetMaxForce(2, 100000.0);
    rotors[2]->SetMaxForce(2, 100000.0);
    rotors[3]->SetMaxForce(2, 100000.0);


	body_motors[0] = parent_model->GetLink("motor1");
	body_motors[1] = parent_model->GetLink("motor2");
	body_motors[2] = parent_model->GetLink("motor3");
	body_motors[3] = parent_model->GetLink("motor4");


	int childrens_num = parent_model->GetChildCount();

	childrens_num = parent_model->GetJointCount();
	std::cout << "TreeJoint: " << childrens_num << std::endl;


	joints[0] = parent_model->GetJoint("arm_motor1"); // motor up
	joints[1] = parent_model->GetJoint("arm_motor2"); // motor right
	joints[2] = parent_model->GetJoint("arm_motor3"); // motor down
	joints[3] = parent_model->GetJoint("arm_motor4"); // motor left

	double maxMotorForce = 10000000.0;
	joints[0]->SetMaxForce(2, maxMotorForce);
	joints[1]->SetMaxForce(2, maxMotorForce);
	joints[2]->SetMaxForce(2, maxMotorForce);
	joints[3]->SetMaxForce(2, maxMotorForce);

    // subscribe iMU Data
    std::string imu_topic;
    imu_topic = "imu_data";
    ROS_INFO_STREAM(imu_topic);
    if (!imu_topic.empty()) {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(imu_topic, 1, boost::bind(&DroneControllerGazebo::imuCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
        subscriber_imu_ = model_nh.subscribe(ops);
        ROS_INFO_STREAM("OK");

        gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << subscriber_imu_.getTopic() << "' as imu input for control" << std::endl;
    } else {
          ROS_INFO_STREAM("KO");
        gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as imu input for control" << std::endl;
    }

    // subscribe Path Data
    std::string path_topic;
    path_topic = "path";
    ROS_INFO_STREAM(path_topic);

        ros::SubscribeOptions ops_path = ros::SubscribeOptions::create<drone_msgs_srvs::WayPoint>(path_topic, 1, boost::bind(&DroneControllerGazebo::pathCallback, this, _1), ros::VoidConstPtr(), &callback_queue_path_);
        subscriber_path_ = model_nh.subscribe(ops_path);

    return true;


  }

  void DroneControllerGazebo::readSim(ros::Time time, ros::Duration period)
  {
	  callback_queue_.callAvailable();
	  callback_queue_path_.callAvailable();

	  DroneControllerGazebo::getDroneEulerRPY(drone_imu_, &roll, &pitch, &yaw);

	  desiredPosision.x=drone_path_.position.position.x;
	  desiredPosision.y=drone_path_.position.position.y;
      desiredPosision.z=drone_path_.position.position.z;
	  ROS_INFO_STREAM("  x : " << desiredPosision.x <<"  y : " << desiredPosision.y << "  z : " << desiredPosision.z);


  }

  void DroneControllerGazebo::writeSim(ros::Time time, ros::Duration period)
  {

	  DroneControllerGazebo::getDronePosition();
	  DroneControllerGazebo::getDroneVelocity(drone_imu_);


	  Kz = 100;
	  Kphi = 1.0;
	  Ktheta = 1.0;
	  Kpsi = 1.0;
	  Dphi = 0.1;
	  Dtheta = 0.05;
	  Dpsi = 0.05;
	  Dz = 4;


	  DroneControllerGazebo::pidDroneControler(desiredPosision, &T, &tau_phi, &tau_theta, &tau_psi, &roll, &pitch, &yaw);

	  // Set Rotors Velocities
	  DroneControllerGazebo::setRotorsVelocities(&T, &tau_phi, &tau_theta, &tau_psi);

	  torque.x=tau_phi;
 	  torque.y=tau_theta;
 	  torque.z=tau_psi;

	  force.x=0;
	  force.y=0;
	  force.z=T;

	  body_main->AddRelativeForce(force);
	  body_main->AddRelativeTorque(torque - body_main->GetInertial()->GetCoG().Cross(force));
  }

 void DroneControllerGazebo::imuCallback(const sensor_msgs::ImuConstPtr &imu) {
	 drone_imu_ = *imu;
 }

 void DroneControllerGazebo::pathCallback(const drone_msgs_srvs::WayPointConstPtr &path) {
	 drone_path_ = *path;
 }


 void DroneControllerGazebo::getDroneEulerRPY(sensor_msgs::Imu &imu, double *roll_, double *pitch_, double *yaw_) const
 {
   const geometry_msgs::Quaternion::_w_type& w = imu.orientation.w;
   const geometry_msgs::Quaternion::_x_type& x = imu.orientation.x;
   const geometry_msgs::Quaternion::_y_type& y = imu.orientation.y;
   const geometry_msgs::Quaternion::_z_type& z = imu.orientation.z;

   *roll_  =  atan2(2.*y*z + 2.*w*x, z*z - y*y - x*x + w*w);
   *pitch_ = -asin(2.*x*z - 2.*w*y);
   *yaw_   =  atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
 }

 void DroneControllerGazebo::getDronePosition()
 {
	  const geometry_msgs::Pose::_position_type::_x_type&  x = body_main->GetWorldPose().pos.x;
	  const geometry_msgs::Pose::_position_type::_y_type&  y = body_main->GetWorldPose().pos.y;
	  const geometry_msgs::Pose::_position_type::_z_type&  z = body_main->GetWorldPose().pos.z;

	  drone_pos_.position.x= x;
	  drone_pos_.position.y= y;
	  drone_pos_.position.z= z;
 }

 void DroneControllerGazebo::getDroneVelocity(sensor_msgs::Imu &imu)
 {
	  const geometry_msgs::Twist::_linear_type::_x_type&  lin_x_dot = body_main->GetRelativeLinearVel().x;
	  const geometry_msgs::Twist::_linear_type::_y_type&  lin_y_dot = body_main->GetRelativeLinearVel().y;
	  const geometry_msgs::Twist::_linear_type::_z_type&  lin_z_dot = body_main->GetRelativeLinearVel().z;

	  const geometry_msgs::Twist::_angular_type::_x_type&  ang_x_dot = imu.angular_velocity.x;
	  const geometry_msgs::Twist::_angular_type::_y_type&  ang_y_dot = imu.angular_velocity.y;
	  const geometry_msgs::Twist::_angular_type::_z_type&  ang_z_dot = imu.angular_velocity.z;

	  drone_vel_.linear.x = lin_x_dot;
	  drone_vel_.linear.y = lin_y_dot;
	  drone_vel_.linear.z = lin_z_dot;

	  drone_vel_.angular.x = ang_x_dot;
	  drone_vel_.angular.y = ang_y_dot;
	  drone_vel_.angular.z = ang_z_dot;
 }

 void DroneControllerGazebo::setRotorsVelocities(double* T_,double* tau_phi_, double* tau_theta_, double* tau_psi_)
 {
	  drone_rotors_vel_[0] = *T_ - *tau_theta_ - *tau_psi_;
	  drone_rotors_vel_[1] = *T_ - *tau_phi_ + *tau_psi_;
	  drone_rotors_vel_[2] = *T_ + *tau_theta_ - *tau_psi_;
	  drone_rotors_vel_[3] = *T_ + *tau_phi_ + *tau_psi_;

	  prop_speed[0] =  sqrt(fabs(*T_ - *tau_theta_ - *tau_psi_))*1000;
	  prop_speed[1] = -sqrt(fabs(*T_ - *tau_phi_ + *tau_psi_))*1000;
	  prop_speed[2] =  sqrt(fabs(*T_ + *tau_theta_ - *tau_psi_))*1000;
	  prop_speed[3] = -sqrt(fabs(*T_ + *tau_phi_ + *tau_psi_))*1000;

	  joints[0]->SetVelocity(2, prop_speed[0]);
	  joints[1]->SetVelocity(2, prop_speed[1]);
	  joints[2]->SetVelocity(2, prop_speed[2]);
	  joints[3]->SetVelocity(2, prop_speed[3]);

 }

 void DroneControllerGazebo::pidDroneControler(geometry_msgs::Pose::_position_type& desiredPosision_, double* T_,double* tau_phi_, double* tau_theta_, double* tau_psi_, double *roll_, double *pitch_, double *yaw_)
 {
	 *T_ =  (9.81 + Kz * ( desiredPosision_.z - drone_pos_.position.z ) + Dz * ( 0 - drone_vel_.linear.z));

	 *tau_phi_ = Kphi * (-( 0.02 * ( desiredPosision_.y - drone_pos_.position.y ) - 0.05 * drone_vel_.linear.y ) - *roll_ ) - Dphi * drone_vel_.angular.x;

	 *tau_theta_ = Ktheta * ( ( 0.02 * ( desiredPosision_.x - drone_pos_.position.x ) - 0.05 * drone_vel_.linear.x ) - *pitch_ ) - Dtheta * drone_vel_.angular.y;

	 *tau_psi_ = Kpsi * ( 0 - *yaw_ ) - Dpsi * drone_vel_.angular.z;
 }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(drone_controller_gazebo::DroneControllerGazebo, gazebo_ros_control::RobotHWSim)
