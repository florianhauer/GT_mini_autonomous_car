#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#define NORMAL 0
#define UTURN 1

ros::Publisher throttle_pub;
ros::Publisher steering_pub;
ros::Publisher marker_pub;

geometry_msgs::PointStamped goal;

bool validated=true;
int mode=NORMAL;

double closest_obstacle=10;

geometry_msgs::PointStamped uturn_center;
double uturn_radius=1;
double current_turn=1;
double current_speed=1;
bool canSwitch=false;


void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  //std::cout << "receiving waypoint update " << msg->point.x << "," << msg->point.y << std::endl;
  if(goal.point.x!=msg->point.x && goal.point.y!=msg->point.y){
	  goal.header.frame_id=msg->header.frame_id;
	  goal.header.stamp=ros::Time(0);
	  goal.point.x=msg->point.x;
	  goal.point.y=msg->point.y;
	  validated=false;
  }
}

void closestObstacleCallback(const std_msgs::Float64::ConstPtr& msg)
{
  closest_obstacle=msg->data;
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
  marker_pub = n.advertise<visualization_msgs::Marker>("uturn_marker", 1);

  ros::Subscriber goal_sub = n.subscribe("/waypoint", 10, goalCallback);
  ros::Subscriber closestObstacle_sub = n.subscribe("/closest_obstacle", 10, closestObstacleCallback);

  tf::TransformListener listener;

  //getting params
  double filter_alpha,frequency,throttle_max,throttle_min,throttle_dist_gain,throttle_speed_gain, steering_gain,speed_max,waypoint_check_distance,uturn_theta_threshold,uturn_throttle, uturn_theta_threshold_hysteresis, uturn_radius_hysteresis;
  nh_rel.param("filter_alpha",filter_alpha,0.8);
  nh_rel.param("frequency",frequency,100.0);
  nh_rel.param("throttle_max",speed_max,2.0);
  nh_rel.param("throttle_max",throttle_max,0.75);
  nh_rel.param("throttle_min",throttle_min,0.6);
  nh_rel.param("throttle_dist_gain",throttle_dist_gain,0.5);
  nh_rel.param("throttle_speed_gain",throttle_speed_gain,5.0);
  nh_rel.param("steering_gain",steering_gain,1.0);
  nh_rel.param("waypoint_check_distance",waypoint_check_distance,0.5);
  nh_rel.param("uturn_theta_threshold",uturn_theta_threshold,1.0);
  nh_rel.param("uturn_throttle",uturn_throttle,0.8);
  nh_rel.param("uturn_theta_threshold_hysteresis",uturn_theta_threshold_hysteresis,0.15);
  nh_rel.param("uturn_radius_hysteresis",uturn_radius_hysteresis,0.05);

  double filtered_throttle=0;
  double filtered_steering=0;
  double filtered_speed=0;
  double filtered_dist=0;
  double prev_dist=1;
  ros::Time prev_time=ros::Time::now();
  geometry_msgs::PointStamped goalInOdom;
  goal.header.stamp=ros::Time(0);
  goal.header.frame_id="map";

  ros::Rate rate(frequency);

  while (n.ok()){
      try{
		  listener.transformPoint("base_link",goal,goalInOdom);
		  if(goalInOdom.header.stamp==prev_time){
		      rate.sleep();
		      ros::spinOnce();
		      continue;
		  }
		  double theta=atan2(goalInOdom.point.y,goalInOdom.point.x);
		  double dist=sqrt(goalInOdom.point.y*goalInOdom.point.y+goalInOdom.point.x*goalInOdom.point.x);
		  filtered_dist=filter_alpha*filtered_dist+(1-filter_alpha)*dist;
		  double speed=(filtered_dist*prev_dist)/(goalInOdom.header.stamp-prev_time).toSec();
		  filtered_speed=filter_alpha*filtered_speed+(1-filter_alpha)*speed;
		  prev_dist=filtered_dist;
		  prev_time=goalInOdom.header.stamp;

		  if(fabs(theta)>uturn_theta_threshold+uturn_theta_threshold_hysteresis){
			if(mode==NORMAL){
				//initialize uturn variables
				//center = current position
				geometry_msgs::PointStamped zero;
				zero.point.x=0;
				zero.point.y=0;
				zero.point.z=0;
				zero.header.stamp=ros::Time(0);
				zero.header.frame_id="base_link";
		  		listener.transformPoint("map",zero,uturn_center);
				uturn_center.header.stamp=ros::Time(0);
				//max distance from uturn = 0.5* closest obstacle
				uturn_radius=0.5*closest_obstacle;
				current_turn=theta>0?-1:1;
				current_speed=1;
				canSwitch=false;
				std::cout << "Initiating uturn" << std::endl;
				std::cout << "centered at " << uturn_center.point.x << " , " << uturn_center.point.y << std::endl;
				std::cout << "uturn radius : " << uturn_radius << std::endl;
				visualization_msgs::Marker marker;
				marker.header.frame_id = "map";
				marker.header.stamp = ros::Time::now();
				marker.ns = "uturn";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = uturn_center.point.x;
				marker.pose.position.y = uturn_center.point.y;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 2*uturn_radius;
				marker.scale.y = 2*uturn_radius;
				marker.scale.z = 0.1;
				marker.color.r = 0.0f;
				marker.color.g = 0.0f;
				marker.color.b = 1.0f;
				marker.color.a = 0.3;
				marker.lifetime = ros::Duration();
				marker_pub.publish(marker);
			}
			mode=UTURN;
		  }else{
			if(fabs(theta)<uturn_theta_threshold-uturn_theta_threshold_hysteresis){
				mode=NORMAL;
			}
		  }
		
		  if(dist<waypoint_check_distance || validated){
			  validated=true;
			  std_msgs::Float64 zero;
			  zero.data=0;
			  throttle_pub.publish(zero);
			  steering_pub.publish(zero);
		  }else{
			  std_msgs::Float64 throttle,steering;
			  switch(mode){
				case UTURN:
					listener.transformPoint("base_link",uturn_center,goalInOdom);
					if(goalInOdom.point.x*goalInOdom.point.x+goalInOdom.point.y*goalInOdom.point.y>(uturn_radius+uturn_radius_hysteresis)*(uturn_radius+uturn_radius_hysteresis)){
						if(canSwitch){
							current_turn*=-1;
							current_speed*=-1;
							canSwitch=false;
						}			
					}else{
						if(goalInOdom.point.x*goalInOdom.point.x+goalInOdom.point.y*goalInOdom.point.y<(uturn_radius-uturn_radius_hysteresis)*(uturn_radius-uturn_radius_hysteresis)){
							canSwitch=true;
						}
					}
					throttle.data=uturn_throttle*current_speed;
					steering.data=current_turn;
					break;
				case NORMAL:
					double Vd=sat(-throttle_dist_gain*dist,speed_max);
					double u=-throttle_speed_gain*(Vd-filtered_speed);
					filtered_throttle=filter_alpha*filtered_throttle+(1-filter_alpha)*sat(u+throttle_min,throttle_max);
					throttle.data=filtered_throttle;
					filtered_steering=filter_alpha*filtered_steering+(1-filter_alpha)*sat(-steering_gain*theta,1);
					steering.data=filtered_steering;				
					break;
			  }
			  throttle_pub.publish(throttle);
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
