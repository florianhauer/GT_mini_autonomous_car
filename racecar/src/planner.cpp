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

bool isObstacle(State<2> state){
	geometry_msgs::Point point;
	point.x=state[0];
	point.y=state[1];
	try{
		occupancy_grid_utils::index_t index=occupancy_grid_utils::pointIndex(local_map->info,point);
		int val=local_map->data[index];
		//TODO (maybe use probabilities)
		if(val>50){
			return true;
		}else{
			return false;
		}
	}catch(occupancy_grid_utils::CellOutOfBoundsException e){
		return false;
	}
	
}

void sendWaypoint(){
	if(planned)
		waypoint_pub.publish(waypoint);
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

void plan(){
	if((ros::Time::now()-local_map->header.stamp)>ros::Duration(0.5))
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
	//Set Tree Max Depth
	int depth=5;
	t->setMaxDepth(depth);
	MSP<2> algo(t);
	//Set algo parameters
	algo.setNewNeighboorCheck(true);
	algo.setMapLearning(true,10,isObstacle);
	algo.setSpeedUp(true);
	algo.setAlpha(2*sqrt(2));
	algo.setEpsilon(0.5);
	bool initAlgo=algo.init(startState,goalState);
	//Run algo
	if(initAlgo && algo.run()){
		std::deque<State<2>> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State<2>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
		std::cout << "smoothed solution" <<std::endl;
		sol=algo.getSmoothedPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		for(std::deque<State<2>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		current_path=sol;
		//current_path.push_front(startState);
		current_path.push_back(goalState);
		setWaypoint(current_path.front());
	}
	planned=true;
	planning=false;
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
}

bool checkFeasibility(){
	if(!planned)
		return true;

	//remove first waypoint if nescessary
	if(sqrt((pose.point.x-waypoint.point.x)*(pose.point.x-waypoint.point.x)+(pose.point.y-waypoint.point.y)*(pose.point.y-waypoint.point.y))<0.5){
		current_path.pop_front();
		setWaypoint(current_path.front());
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
}

void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(planning)
		return;
	//update local pose
	 pose.header.frame_id=msg->header.frame_id;
	 pose.header.stamp=ros::Time(0);
	 pose.point.x=msg->point.x;
	 pose.point.y=msg->point.y;

	startState[0]=pose.point.x;
	startState[1]=pose.point.y;

	if(checkFeasibility()){
		plan();
	}
	sendWaypoint();
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	if(planning)
		return;
	//inflate
	//update local map
	local_map=occupancy_grid_utils::inflateObstacles(*msg,inflation_radius,true);
	if(checkFeasibility()){
		plan();
	}
	sendWaypoint();
}

void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(planning)
		return;
	//update local goal
	std::cout << "receiving goal update " << msg->point.x << "," << msg->point.y << std::endl;
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
traj_pub = n.advertise<visualization_msgs::Marker>("rviz_traj", 10);

  ros::Subscriber goal_sub = n.subscribe("/goal_pose", 10, goalCallback);
  ros::Subscriber pose_sub = n.subscribe("/slam_out_pose", 10, poseCallback);
  ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);




  ros::spin();

  return 0;
}
