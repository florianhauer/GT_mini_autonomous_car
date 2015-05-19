#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

ros::Publisher throttle_pub;
ros::Publisher steering_pub;

geometry_msgs::PointStamped goal;

void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::cout << "receiving goal update " << msg->point.x << "," << msg->point.y << std::endl;
  goal.header.frame_id=msg->header.frame_id;
  goal.header.stamp=ros::Time(0);
  goal.point.x=msg->point.x;
  goal.point.y=msg->point.y;
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
  ros::init(argc, argv, "waypoint_tracker");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  throttle_pub = n.advertise<std_msgs::Float64>("/throttle/command", 10);
  steering_pub = n.advertise<std_msgs::Float64>("/steering/command", 10);

  ros::Subscriber goal_sub = n.subscribe("/goal_pose", 10, goalCallback);

  tf::TransformListener listener;

  //getting params
  double filter_alpha,frequency,throttle_max,throttle_min,throttle_gain,steering_gain;
  nh_rel.param("filter_alpha",filter_alpha,0.8);
  nh_rel.param("frequency",frequency,100.0);
  nh_rel.param("throttle_max",throttle_max,0.75);
  nh_rel.param("throttle_min",throttle_min,0.6);
  nh_rel.param("throttle_gain",throttle_gain,0.25);
  nh_rel.param("steering_gain",steering_gain,1.0);
  std::cout << "throttle_max " << throttle_max << std::endl;

  double filtered_throttle=0;
  double filtered_steering=0;
  geometry_msgs::PointStamped goalInOdom;
  goal.header.stamp=ros::Time(0);
  goal.header.frame_id="map";

  ros::Rate rate(frequency);

  while (n.ok()){
      try{
		  listener.transformPoint("base_link",goal,goalInOdom);
		  double theta=atan2(goalInOdom.point.y,goalInOdom.point.x);
		  double dist=sqrt(goalInOdom.point.y*goalInOdom.point.y+goalInOdom.point.x*goalInOdom.point.x);
		  if(dist<0.2){
			  std_msgs::Float64 zero;
			  zero.data=0;
			  throttle_pub.publish(zero);
			  steering_pub.publish(zero);
		  }else{
			  filtered_throttle=filter_alpha*filtered_throttle+(1-filter_alpha)*sat(throttle_gain*dist+throttle_min,throttle_max);
			  std_msgs::Float64 throttle;
			  throttle.data=filtered_throttle;
			  throttle_pub.publish(throttle);
			  filtered_steering=filter_alpha*filtered_steering+(1-filter_alpha)*sat(-steering_gain*theta,1);
			  std_msgs::Float64 steering;
			  steering.data=filtered_steering;
			  steering_pub.publish(steering);
		  }
      }catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
      }
      rate.sleep();
      ros::spinOnce();
    }

  ros::spin();

  return 0;
}
