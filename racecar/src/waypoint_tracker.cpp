#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

ros::Publisher throttle_pub;
ros::Publisher steering_pub;

geometry_msgs::PointStamped goal;

void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  goal.header=msg->header;
  goal.point=msg->point;
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
  ros::Rate rate(10.0);
  while (n.ok()){
      geometry_msgs::PointStamped goalInOdom;
      try{
    	  goal.header.stamp=ros::Time::now()-ros::Duration(0.05);
    	  std::string err;
    	  if(listener.canTransform("base_link","map",ros::Time::now()-ros::Duration(0.05)))
    		  std::cout<<"transform possible" << std::endl;
    	  else
    		  std::cout << "fail :" << err << std::endl;
		  listener.transformPoint("base_link",goal,goalInOdom);
		  double theta=atan2(goalInOdom.point.y,goalInOdom.point.x);
		  double dist=sqrt(goalInOdom.point.y*goalInOdom.point.y+goalInOdom.point.x*goalInOdom.point.x);
		  if(dist<0.2){
			  std_msgs::Float64 zero;
			  zero.data=0;
			  throttle_pub.publish(zero);
			  steering_pub.publish(zero);
		  }else{
			  std_msgs::Float64 throttle;
			  throttle.data=sat(dist-0.1,0.5);
			  throttle_pub.publish(throttle);
			  std_msgs::Float64 steering;
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
