#include <iostream>
#include <stdio.h>
#include <ncurses.h>
#include <drone_msgs_srvs/WayPoint.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class DroneTelOp
{
private:
  //! The node handle we'll be using
  ros::NodeHandle n_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  DroneTelOp(ros::NodeHandle &n)
  {
    n_ = n;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = n_.advertise<drone_msgs_srvs::WayPoint>("/path", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use 'w','a','s', and 'd' to navigate and 'q' to exit.\n";

    //we will be sending commands of type "twist"
    drone_msgs_srvs::WayPoint path_cmd;

    char cmd;
    initscr(); //get terminal environment variables
    cbreak();  //line buffering disabled; pass on everything
    timeout(1000);  //getch blocks for 1 second
    path_cmd.position.position.x = path_cmd.position.position.y = path_cmd.position.position.z = 0.0;

    while(n_.ok()){

      cmd = getch();

      //move x +
      if(cmd =='d'){
    	  path_cmd.position.position.x += 0.01;
      }
      //move x -
      if(cmd =='q'){
    	  path_cmd.position.position.x -= 0.01;
      }
      //move y +
      if(cmd =='z'){
    	  path_cmd.position.position.y += 0.01;
      }
      //move y -
      if(cmd =='x'){
    	  path_cmd.position.position.y -= 0.01;
      }

      //move z +
      if(cmd =='s'){
    	  path_cmd.position.position.z += 0.01;
      }

      //move f -
      if(cmd =='f'){
    	  path_cmd.position.position.z -= 0.01;
      }

      //turn left (yaw) and drive forward at the same time
      if(cmd =='e'){
    	  path_cmd.position.position.x += 0.01;
    	  path_cmd.position.position.y -= 0.01;
      }

      if(cmd =='c'){
    	  path_cmd.position.position.x += 0.01;
    	  path_cmd.position.position.y += 0.01;
      }

      if(cmd =='w'){
    	  path_cmd.position.position.x -= 0.01;
    	  path_cmd.position.position.y += 0.01;
      }

      if(cmd =='a'){
    	  path_cmd.position.position.x -= 0.01;
    	  path_cmd.position.position.y -= 0.01;
      }

      else if(cmd =='g'){
    	  path_cmd.position.position.x = 0;
    	  path_cmd.position.position.y = 0;
    	  path_cmd.position.position.z = 0;

    	  break;
      }

      //publish the assembled command
      cmd_vel_pub_.publish(path_cmd);

      std::cout << path_cmd.position.position << "\n\r";
    }
    nocbreak(); //return terminal to "cooked" mode
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle n;

  DroneTelOp driver(n);
  driver.driveKeyboard();
}
