/*
 * Author : Chun-Jong Lin
 * Date : 28 05 2018
 * Brief : using keyboard control waffle
*/

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <termios.h>
geometry_msgs::Twist cmd;

ros::Publisher cmd_pub;

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

void keyboard_control()
{
  int c = getch();
  //ROS_INFO("C: %d",c);
  if (c != EOF) {
    switch (c)
    {
    case 119: //w
      cmd.linear.x += 0.01;
      break;
    case 115: //s
      cmd.linear.x -= 0.01;
      break;
    case 97: //a
      cmd.angular.z += 0.01;
      break;
    case 100: //d
      cmd.angular.z -= 0.01;
      break;
    case 114: //r
      cmd.linear.x = 0;
      cmd.angular.z = 0;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_ctrl");
  ros::NodeHandle nh;
  cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    keyboard_control();
    ROS_INFO("linear : %f angular : %f",cmd.linear.x,cmd.angular.z);
    cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
