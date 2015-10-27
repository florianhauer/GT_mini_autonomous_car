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
ros::Publisher traj_pub_raw;
ros::Publisher traj_pub_smooth;
ros::Publisher map_pub;

geometry_msgs::PointStamped goal;
geometry_msgs::PointStamped pose;
State<2> startState(0), goalState(0);
geometry_msgs::PointStamped waypoint;
bool planning=false;
bool planned=false;

std::deque<State<2>> current_path;//current pose not included
std::deque<State<2>> current_path_raw;//current pose not included

//parameters set in the launch file
double inflation_radius=0.3;
double waypoint_check_distance=0.3;
int depth=8;
int nb_obstacle_check=100;
double epsilon=0.1;

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
	traj_pub_smooth.publish(traj_visu);

	traj_visu.points.clear();
	traj_visu.header.stamp=ros::Time::now();
	traj_visu.ns="traj_raw";
	traj_visu.id=2;

	traj_visu.color.r = 0.0;
	traj_visu.color.g = 1.0;

	traj_visu.points.push_back(pose.point);

	for(int i=0;i<current_path_raw.size();++i){
		geometry_msgs::Point p;
		p.x=current_path_raw[i][0];
		p.y=current_path_raw[i][1];
		p.z=0.1;
		traj_visu.points.push_back(p);
	}
	traj_pub_raw.publish(traj_visu);
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

int Ocount=0;
time_t timerStart;

bool addObstacles(Key<2> k, int depth, int size, Tree<2>* t){
	if(depth==t->getMaxDepth()){
		Ocount++;
		if(Ocount%((int)pow(2,8))==0){
			std::cout << "\r" << "Obstacle creation " << std::setw(10) << Ocount*100.0/pow(2,2*depth) << "\% done, ";
			time_t timerNow=time(NULL);	
			int seconds = (int)difftime(timerNow,timerStart);
			seconds=(int)(seconds*(1-( Ocount/pow(2,2*depth)))/( Ocount/pow(2,2*depth)));
			int hours = seconds/3600;
			int minutes= (seconds/60)%60;
			seconds=seconds%60;
			std::cout << "time left: " 	<< std::setw(4) << hours << ":" 
							<< std::setw(2) << minutes << ":" 
							<< std::setw(2) << seconds;
		}
		//finest resolution: update obstacle presence
		//if obstacles
		if(isObstacle(t->getState(k))){
			//add obstacle to the tree
			t->addObstacle(k);
			//indicate that the tree was updated
			return true;
		}else{
			//indicate that not changes were performed on the tree
			return false;
		}
	}else{
		bool update=false;
		//update children
		int size2=size>>1;
		 for(const Key<2>& dir: *(t->getDirections())){
			 update=addObstacles(k+dir*size2,depth+1,size2,t) || update;
		 }
		//if any children created, get node
		 if(update){
			 Node<2>* cur=t->getNode(k);
			 //prune and update val (single stage, no recurrence (children are up to date))
			 cur->update(false);
		 }
		 //indicate if updates were performed on the tree
		 return update;
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
	if(sqrt((pose.point.x-waypoint.point.x)*(pose.point.x-waypoint.point.x)+(pose.point.y-waypoint.point.y)*(pose.point.y-waypoint.point.y))<waypoint_check_distance){	
		if(current_path.size()>1){
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
	t->setMaxDepth(depth);
	//Depth First Obstacle Creation
	std::cout << "Obstacle creation " << std::setw(10) << 0.0 << "\% done.";
	Ocount=0;
	timerStart=time(NULL);
	addObstacles(t->getRootKey(),0,t->getRootKey()[0],t);
	std::cout << std::endl;
	time_t timerNow=time(NULL);	
	int seconds = (int)difftime(timerNow,timerStart);
	int hours = seconds/3600;
	int minutes= (seconds/60)%60;
	seconds=seconds%60;
	std::cout << "Obstacles created in " 	<< std::setw(4) << hours << ":" 
						<< std::setw(2) << minutes << ":" 
						<< std::setw(2) << seconds 
						<< std::endl;
	MSP<2> algo(t);
	//Set algo parameters
	algo.setNewNeighboorCheck(true);
	//algo.setMapLearning(true,nb_obstacle_check,isObstacle);
	algo.setSpeedUp(true);
	algo.setAlpha(2*sqrt(2));
	algo.setEpsilon(epsilon);
	algo.setLambda1(0.1);
	//algo.setMinRGcalc(true);
	bool initAlgo=algo.init(startState,goalState);
	std::cout << "start : " << startState <<std::endl;
	std::cout << "goal : " << goalState <<std::endl;
	std::cout << "init : " << initAlgo <<std::endl;
	//Run algo
	if(initAlgo && algo.run()){
		ROS_INFO_STREAM("Algo init "<<initAlgo<<", planning in progress ...");	
		current_path_raw=algo.getPath();
		std::cout << "Path length: " << current_path_raw.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State<2>>::iterator it=current_path_raw.begin(),end=current_path_raw.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
		current_path_raw.push_back(goalState);
		current_path=std::deque<State<2>>();
		current_path.assign(current_path_raw.begin(),current_path_raw.end());
		//*
		std::cout << "smoothed solution" <<std::endl;
		for(int i=0;i<current_path_raw.size();++i)
			smoothTraj();
		//current_path=algo.getSmoothedPath();
		//current_path.push_back(goalState);
		std::cout << "Path length: " << current_path.size() << std::endl;
		for(std::deque<State<2>>::iterator it=current_path.begin(),end=current_path.end();it!=end;++it){
			std::cout << (*it) << " -- ";}
		std::cout << std::endl << "raw:" <<std::endl;
		for(std::deque<State<2>>::iterator it=current_path_raw.begin(),end=current_path_raw.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
		//*/
		planned=true;
		setWaypoint(current_path.front());
	}else{
		ROS_INFO("Planning failed");
		planned=false;
	}
	algo.clear();
	free(t);
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
	map_pub.publish(local_map);
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

  //reading parameters
  nh_rel.param("tree_depth",depth,8);
  nh_rel.param("nb_obstacle_check",nb_obstacle_check,100);
  nh_rel.param("epsilon",epsilon,0.1);
  nh_rel.param("inflation_radius",inflation_radius,0.3);
  nh_rel.param("waypoint_check_distance",waypoint_check_distance,0.3);


  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/waypoint", 1);
  traj_pub_raw = n.advertise<visualization_msgs::Marker>("/traj_raw", 1);
  traj_pub_smooth = n.advertise<visualization_msgs::Marker>("/traj_smooth", 1);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_inflated", 1);

  ros::Subscriber goal_sub = n.subscribe("/goal_pose", 10, goalCallback);
  ros::Subscriber pose_sub = n.subscribe("/slam_out_pose", 10, poseCallback);
  ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);




  ros::spin();

  return 0;
}
