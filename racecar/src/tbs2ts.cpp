#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <racecar/ThrottleBrakeSteering.h>

ros::Publisher throttle_pub;
ros::Publisher steering_pub;

double vx=0;
double throttle=0;
double brake=0;
double steering=0;

double sign(double a){
	if(a>0)
		return 1.0;
	if(a<0)
		return -1.0;
	return 0.0;
}

double sat(double val,double val_max){
	if(val>val_max)
		return val_max;
	if(val<-val_max)
		return -val_max;
	return val;
}

void vxCallback(const std_msgs::Float64::ConstPtr& msg)
{
	vx=msg->data;
}

void commandCallback(const racecar::ThrottleBrakeSteering::ConstPtr& msg)
{
	throttle=msg->throttle;
	brake=msg->brake;
	steering=msg->steering;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  throttle_pub = n.advertise<std_msgs::Float64>("/throttle/command", 1);
  steering_pub = n.advertise<std_msgs::Float64>("/steering/command", 1);

  ros::Subscriber steering_sub = n.subscribe("/car_commands", 1, commandCallback);
  ros::Subscriber vx_sub = n.subscribe("/vx_filtered", 1, vxCallback);

  //getting params
  double frequency,max_brake,max_throttle,vx_thres;
  nh_rel.param("frequency",frequency,100.0);
  nh_rel.param("max_brake",max_brake,1.0);
  nh_rel.param("max_throttle",max_throttle,0.4);
  nh_rel.param("vx_thres",vx_thres,0.2);

  ros::Rate rate(frequency);

  std_msgs::Float64 t_msg;
  std_msgs::Float64 s_msg;

  while (n.ok()){
      double t,s,b;
      t=throttle;
      b=brake;
      s=steering;
      /*if(t*vx<0 && fabs(vx)>vx_thres){
	t=0;
	b=1;
      }*/
      if(b>0){
	//brake
	/*if(fabs(vx)>vx_thres){
		t_msg.data=-max_brake*sign(vx);
	}else{
		t_msg.data=0;
	}*/
	t_msg.data=-1;
      }else{
	/*if(t*vx<0){
		if(fabs(vx)>vx_thres/2){
			t_msg.data=0;
		}else{
			t_msg.data=sat(t,max_throttle);
		}
	}else{
		t_msg.data=sat(t,max_throttle);
	}
	*/
	t_msg.data=sat(t,max_throttle);
      }
      //t_msg.data=0.5*sign(t_msg.data)+0.5*t_msg.data;
      s_msg.data=s;
      throttle_pub.publish(t_msg);
      steering_pub.publish(s_msg);
      rate.sleep();
      ros::spinOnce();
  }

  ros::spin();

  return 0;
}
