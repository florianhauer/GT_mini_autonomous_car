#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <px_comm/OpticalFlow.h>

ros::Publisher pub;
double vx=0;
double learning_rate=0.05;

void callback(const px_comm::OpticalFlow::ConstPtr& msg)
{
	vx+=learning_rate*(msg->velocity_x-vx);
	std_msgs::Float64 vx_msg;
	vx_msg.data=vx;
	pub.publish(vx_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  nh_rel.param("learning_rate",learning_rate,0.05);

  pub = n.advertise<std_msgs::Float64>("/vx_filtered", 1);

  ros::Subscriber steering_sub = n.subscribe("/px4flow/opt_flow", 1, callback);

  ros::spin();

  return 0;
}
