#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <visualization_msgs/MarkerArray.h>

#include <msp/MSP.h>

ros::Publisher waypoint_pub;
ros::Publisher traj_pub;

geometry_msgs::PointStamped goal;
geometry_msgs::PointStamped pose;
State<2> startState(0), goalState(0);
geometry_msgs::PointStamped waypoint;
bool planning=false;
bool planned=false;

std::deque<State<2>> current_path;//current pose not included

double inflation_radius=0.8;
nav_msgs::OccupancyGrid::Ptr local_map;

double minX (const nav_msgs::MapMetaData& info) {
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return min(p.points[0].x, min(p.points[1].x, min(p.points[2].x, p.points[3].x)));
}

double maxX (const nav_msgs::MapMetaData& info){
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return max(p.points[0].x, max(p.points[1].x, max(p.points[2].x, p.points[3].x)));
}

double minY (const nav_msgs::MapMetaData& info){
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return min(p.points[0].y, min(p.points[1].y, min(p.points[2].y, p.points[3].y)));
}

double maxY (const nav_msgs::MapMetaData& info){
	const geometry_msgs::Polygon p=occupancy_grid_utils::gridPolygon(info);
	return max(p.points[0].y, max(p.points[1].y, max(p.points[2].y, p.points[3].y)));
}

void publishTraj(){
	visualization_msgs::Marker traj_visu;
	traj_visu.header.frame_id="/map";
	traj_visu.header.stamp=ros::Time::now();
	traj_visu.ns="traj";
	traj_visu.type=visualization_msgs::Marker::LINE_STRIP;
	traj_visu.action=visualization_msgs::Marker::ADD;
	traj_visu.id=1;
	traj_visu.scale.x=0.05;
	traj_visu.scale.y=0.05;
	traj_visu.scale.z=0.05;
	traj_visu.color.r = 1.0;
	traj_visu.color.a = 1.0;

	traj_visu.points.push_back(pose.point);

	for(int i=0;i<current_path.size();++i){
		geometry_msgs::Point p;
		p.x=current_path[i][0];
		p.y=current_path[i][1];
		p.z=0.1;
		traj_visu.points.push_back(p);
	}
	traj_pub.publish(traj_visu);
}

void sendWaypoint(){
	waypoint.header.frame_id="/map";
	waypoint.header.stamp=ros::Time::now();
	if(planned){
		waypoint_pub.publish(waypoint);
		publishTraj();
	}
}

void setWaypoint(State<2> s){
	waypoint.point.x=s[0];
	waypoint.point.y=s[1];
	sendWaypoint();
}

void stop(){
	waypoint=pose;
	sendWaypoint();
}

bool isObstacle(State<2> state){
	geometry_msgs::Point point;
	point.x=state[0];
	point.y=state[1];
	try{
		occupancy_grid_utils::index_t index=occupancy_grid_utils::pointIndex(local_map->info,point);
		int val=local_map->data[index];
		//TODO (maybe use probabilities
		//std::cout << val << std::endl;
		if(val>50){
			return true;
		}else{
			return false;
		}
	}catch(occupancy_grid_utils::CellOutOfBoundsException e){
		return true;
	}
	
}

bool segmentFeasibility(State<2> a,State<2> b){
	double res=0.001;
	State<2> inc=b-a;
	double l=inc.norm();
	inc=inc*(res/l);
	for(int i=1;i<l/res;++i){
		if(isObstacle(a+inc*i))
			return false;
	}
	return true;
}

void smoothTraj(){
	if(!planned || current_path.size()<2)
		return;
	int i=1;
	State<2> a=startState;
	State<2> b=current_path[i];
	while(i<current_path.size()){ // situtation always is of type a->m->b->n
		//if a->b feasible, remove m and set b=n to try to remove b
		if(segmentFeasibility(a,b)){
			current_path.erase(current_path.begin()+i-1);
			b=current_path[i];
		}else{ // if a->b unfeasible, we need to keep a->m, so a becomes m, b becomes n to try to remove the previous b 
			a=current_path[i-1];
			i++;
			b=current_path[i];
		}
	}
}

bool checkFeasibility(){
	//ROS_INFO("checking feasibility");
	if(!planned)
		return true;

	//remove first waypoint if nescessary
	if(sqrt((pose.point.x-waypoint.point.x)*(pose.point.x-waypoint.point.x)+(pose.point.y-waypoint.point.y)*(pose.point.y-waypoint.point.y))<0.5){	
		if(current_path.size()>0){
			current_path.pop_front();
			setWaypoint(current_path.front());
		}else{
			return true;
		}
	}
	//check feasibility
	//test feasibility from pose to first waypoint
	if(!segmentFeasibility(startState,current_path.front()))
		return false;
	//test feasibility of rest of path
	for(int i=0;i<current_path.size()-1;++i){
		if(!segmentFeasibility(current_path[i],current_path[i+1]))
		return false;
	}
	return true;
}

void plan(){
	ROS_INFO("planning");
	if((ros::Time::now()-local_map->header.stamp)>ros::Duration(5.0))
		return;
	planning=true;
	stop();
	//TODO
	Tree<2>* t=new Tree<2>();
	//Set Search Space Bounds
	State<2> minState;
	minState[0]=minX(local_map->info);
	minState[1]=minY(local_map->info);
	State<2> maxState;
	maxState[0]=maxX(local_map->info);
	maxState[1]=maxY(local_map->info);
	t->setStateBounds(minState,maxState);
	std::cout << "min bound : " << minState <<std::endl;
	std::cout << "max bound : " << maxState <<std::endl;
	//Set Tree Max Depth
	int depth=6;
	t->setMaxDepth(depth);
	MSP<2> algo(t);
	//Set algo parameters
	algo.setNewNeighboorCheck(true);
	algo.setMapLearning(true,10,isObstacle);
	algo.setSpeedUp(true);
	algo.setAlpha(2*sqrt(2));
	algo.setEpsilon(0.5);
	algo.setMinRGcalc(true);
	bool initAlgo=algo.init(startState,goalState);
	std::cout << "start : " << startState <<std::endl;
	std::cout << "goal : " << goalState <<std::endl;
	std::cout << "init : " << initAlgo <<std::endl;
	//Run algo
	if(initAlgo && algo.run()){
		ROS_INFO_STREAM("Algo init "<<initAlgo<<", planning in progress ...");	
		std::deque<State<2>> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State<2>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
		current_path=sol;
		current_path.push_back(goalState);
		//*
		std::cout << "smoothed solution" <<std::endl;
		smoothTraj();
		std::cout << "Path length: " << current_path.size() << std::endl;
		for(std::deque<State<2>>::iterator it=current_path.begin(),end=current_path.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}//*/
		setWaypoint(current_path.front());
		planned=true;
	}else{
		ROS_INFO("Planning failed");
		planned=false;
	}
	planning=false;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO_STREAM("pose update: "<< msg->pose.position.x << "," << msg->pose.position.y);
	if(planning)
		return;
	//update local pose
	 pose.header.frame_id=msg->header.frame_id;
	 pose.header.stamp=ros::Time(0);
	 pose.point.x=msg->pose.position.x;
	 pose.point.y=msg->pose.position.y;

	startState[0]=pose.point.x;
	startState[1]=pose.point.y;

	if(!checkFeasibility()){
		plan();
	}
	sendWaypoint();
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//ROS_INFO("map update");
	if(planning)
		return;
	//inflate
	//update local map
	local_map=occupancy_grid_utils::inflateObstacles(*msg,inflation_radius,true);
	if(!checkFeasibility()){
		plan();
	}
	sendWaypoint();
}

void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	ROS_INFO_STREAM("goal update: "<< msg->point.x << "," << msg->point.y);
	if(planning)
		return;
	//update local goal
	goal.header.frame_id=msg->header.frame_id;
	goal.header.stamp=ros::Time(0);
	goal.point.x=msg->point.x;
	goal.point.y=msg->point.y;

	goalState[0]=goal.point.x;
	goalState[1]=goal.point.y;

	plan();
	sendWaypoint();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::NodeHandle nh_rel=ros::NodeHandle("~");

  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/waypoint", 10);
traj_pub = n.advertise<visualization_msgs::Marker>("/rviz_traj", 10);

  ros::Subscriber goal_sub = n.subscribe("/goal_pose", 1, goalCallback);
  ros::Subscriber pose_sub = n.subscribe("/slam_out_pose", 1, poseCallback);
  ros::Subscriber map_sub = n.subscribe("/map", 1, mapCallback);




  ros::spin();

  return 0;
}
