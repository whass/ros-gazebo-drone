#include "ros/ros.h"
#include "drone_msgs_srvs/WayPoint.h"
#include <sstream>
int main(int argc, char **argv)
{
ros::init(argc, argv, "way_point_pub");
ros::NodeHandle n;
ros::Publisher pub = n.advertise<drone_msgs_srvs::WayPoint>("path", 1000);
ros::Rate loop_rate(10);
while (ros::ok())
{
	drone_msgs_srvs::WayPoint msg;
	msg.position.position.x = 4;
	msg.position.position.y = -4;
	msg.position.position.z = 1;
	pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
}
return 0;
}
