#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>

double delta=0;
double delta_max=0.5;

float sat(float val,float val_max){
	if(val>val_max)
		return val_max;
	if(val<-val_max)
		return -val_max;
	return val;
}

void steeringCallback(const std_msgs::Float64::ConstPtr& msg)
{
	delta=delta_max*sat(-msg->data,1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Subscriber steering_sub = n.subscribe("/steering/command", 1, steeringCallback);

  //getting params
  double frequency;
  nh_rel.param("frequency",frequency,100.0);

  ros::Rate rate(frequency);
  sensor_msgs::JointState msg;

  msg.name.push_back("left_front_shock");
  msg.name.push_back("left_rear_shock");
  msg.name.push_back("right_front_shock");
  msg.name.push_back("right_rear_shock");

  msg.name.push_back("left_front_axle");
  msg.name.push_back("left_rear_axle");
  msg.name.push_back("right_front_axle");
  msg.name.push_back("right_rear_axle");

  msg.name.push_back("left_steering_joint");
  msg.name.push_back("right_steering_joint");

  msg.position.insert(msg.position.begin(),10,0.0);

  while (n.ok()){
      msg.header.stamp=ros::Time::now();
      msg.position[8]=delta;
      msg.position[9]=delta;
      pub.publish(msg);
      rate.sleep();
      ros::spinOnce();
    }

  ros::spin();

  return 0;
}
