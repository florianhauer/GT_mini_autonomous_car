#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

ros::Publisher throttle_pub;
ros::Publisher steering_pub;

geometry_msgs::PoseStamped goal;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal=*msg;
}

float sat(float val,float val_max){
	if(val>val_max)
		return val_max;
	if(val<-val_max)
		return -val_max;
	return val;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  throttle_pub = n.advertise<std_msgs::Float32>("throttle_command", 10);
  steering_pub = n.advertise<std_msgs::Float32>("steering_command", 10);

  ros::Subscriber goal_sub = n.subscribe("goal_pose", 10, goalCallback);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (n.ok()){
      geometry_msgs::PoseStamped goalInOdom;
      try{
		  listener.transformPose("odom",goal,goalInOdom);
		  double theta=atan2(goalInOdom.pose.position.y,goalInOdom.pose.position.x);
		  double dist=sqrt(goalInOdom.pose.position.y*goalInOdom.pose.position.y+goalInOdom.pose.position.x*goalInOdom.pose.position.x);
		  if(dist<0.2){
			  std_msgs::Float32 zero;
			  zero.data=0;
			  throttle_pub.publish(zero);
			  steering_pub.publish(zero);
		  }else{
			  std_msgs::Float32 throttle;
			  throttle.data=sat(dist-0.1,0.5);
			  throttle_pub.publish(throttle);
			  std_msgs::Float32 steering;
			  steering.data=sat(theta,1);
			  steering_pub.publish(steering);
		  }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      rate.sleep();
    }

  ros::spin();

  return 0;
}
