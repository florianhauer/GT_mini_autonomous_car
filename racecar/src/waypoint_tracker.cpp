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
  goal.header.stamp=msg->header.stamp;
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
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  throttle_pub = n.advertise<std_msgs::Float64>("/throttle/command", 10);
  steering_pub = n.advertise<std_msgs::Float64>("/steering/command", 10);

  ros::Subscriber goal_sub = n.subscribe("/goal_pose", 10, goalCallback);

  tf::TransformListener listener;

  goal.header.stamp=ros::Time::now();
  goal.header.frame_id="map";
  ros::Rate rate(100.0);
  float filtered_throttle=0;
  float filtered_steering=0;
  double filter_alpha=0.8;
  while (n.ok()){
      geometry_msgs::PointStamped goalInOdom;
      try{
//    	  goal.header.stamp=ros::Time::now()-ros::Duration(0.5);
    	  goal.header.stamp=ros::Time(0);
//    	  std::string err;
//    	  if(listener.canTransform("base_link","map",ros::Time::now()-ros::Duration(0.05)))
//    		  std::cout<<"transform possible" << std::endl;
//    	  else
//    		  std::cout << "fail :" << err << std::endl;
//    	  std::cout << "goal in map " << goal.point.x << " , " << goal.point.y <<std::endl;
//		  listener.transformPoint("base_link",ros::Time(0),goal,"map",goalInOdom);
		  listener.transformPoint("base_link",goal,goalInOdom);
//    	  std::cout << "goal in odom " << goalInOdom.point.x << " , " << goalInOdom.point.y <<std::endl;
		  double theta=atan2(goalInOdom.point.y,goalInOdom.point.x);
		  double dist=sqrt(goalInOdom.point.y*goalInOdom.point.y+goalInOdom.point.x*goalInOdom.point.x);
//		  std::cout << "theta " << theta/3.14159*180 << " , dist " << dist << std::endl;
		  if(dist<0.2){
			  std_msgs::Float64 zero;
			  zero.data=0;
			  throttle_pub.publish(zero);
			  steering_pub.publish(zero);
		  }else{
			  filtered_throttle=filter_alpha*filtered_throttle+(1-filter_alpha)*sat(0.25*dist+0.5,0.75);
			  std_msgs::Float64 throttle;
			  throttle.data=filtered_throttle;
			  throttle_pub.publish(throttle);
			  filtered_steering=filter_alpha*filtered_steering+(1-filter_alpha)*sat(-theta,1);
			  std_msgs::Float64 steering;
			  steering.data=filtered_steering;
			  steering_pub.publish(steering);
		  }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      rate.sleep();
      ros::spinOnce();
    }

  ros::spin();

  return 0;
}
