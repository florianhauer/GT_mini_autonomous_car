#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  ros::Publisher pub = n.advertise<std_msgs::Float64>("/vx_filtered", 1);

  double frequency,learning_rate;
  nh_rel.param("frequency",frequency,100.0);
  nh_rel.param("learning_rate",learning_rate,0.2);

  ros::Rate rate(frequency);

  tf::TransformListener listener;
  geometry_msgs::Twist twist;
  double vx=0;
  std_msgs::Float64 vx_msg;

  while (n.ok()){
      try{
	listener.lookupTwist("base_link","map","base_link",tf::Point(),"base_link",ros::Time::now(),ros::Duration(1/frequency),twist);
	vx+=learning_rate*(twist.linear.x-vx);
	vx_msg.data=vx;
	pub.publish(vx_msg);
      }catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
      }
      rate.sleep();
      ros::spinOnce();
    }

  ros::spin();

  return 0;
}
