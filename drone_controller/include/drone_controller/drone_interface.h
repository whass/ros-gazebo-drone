/// \author Walid Hassani

#ifndef HARDWARE_INTERFACE_DRONE_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_DRONE_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/robot_hw.h>

#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>


//typedef boost::shared_ptr<geometry_msgs::Twist> TwistHandlePtr;
//typedef boost::shared_ptr<geometry_msgs::Vector3> AccelerationHandlePtr;
//typedef boost::shared_ptr<sensor_msgs::Imu> ImuHandlePtr;

namespace drone_controller {

using namespace hardware_interface;

/** A handle used to read the state of a the drone. */
class DroneStateHandle : public HardwareInterface
{

public:

	DroneStateHandle() : name_(), Position_(), Velocity_(), Imu_(), RotorVelocity_() {};
	DroneStateHandle(std::string& name, const geometry_msgs::Pose* Position, const geometry_msgs::Twist* Velocity, const sensor_msgs::Imu* Imu, const std::vector<double>* RotorVelocity) : name_(name), Position_(Position), Velocity_(Velocity), Imu_(Imu), RotorVelocity_(RotorVelocity) {};

	std::string getName() const {return name_;}

	geometry_msgs::Pose getPosition()  { return *Position_; };
	geometry_msgs::Twist getVelocity()  { return *Velocity_; };
	sensor_msgs::Imu getImu()  { return *Imu_; };
	std::vector<double> getRotorVelocity() { return *RotorVelocity_; };

private:

	std::string name_;
	const geometry_msgs::Pose *Position_;
	const geometry_msgs::Twist *Velocity_;
	const sensor_msgs::Imu *Imu_;
	const std::vector<double>* RotorVelocity_;

};

/** \brief Hardware interface to support reading the state of a drone
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * drone, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class DroneStateInterface : public HardwareResourceManager<DroneStateHandle> {};


}
#endif

