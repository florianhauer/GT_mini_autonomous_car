#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher goal_pub;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	if(feedback->event_type==visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP ){
	  /*ROS_INFO_STREAM( feedback->marker_name << " is now at "
		  << feedback->pose.position.x << ", " << feedback->pose.position.y
		  << ", " << feedback->pose.position.z );*/
	  geometry_msgs::PointStamped pose_msg;
	  pose_msg.point=feedback->pose.position;
	  pose_msg.header=feedback->header;
	  goal_pub.publish(pose_msg);
	  server->setPose(feedback->marker_name,feedback->pose);
	  server->applyChanges();
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_marker_node");
  ros::NodeHandle n;
  goal_pub=n.advertise<geometry_msgs::PointStamped>("/goal_pose", 10);

  // create an interactive marker server on the topic namespace simple_marker
 // server=interactive_markers::InteractiveMarkerServer("goal_marker_server");
  server.reset( new interactive_markers::InteractiveMarkerServer("goal_marker_server") );

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.name = "goal";
  int_marker.description = "Goal";
  //put the marker higher up to prevent overlapping objects which create interaction issues 
  int_marker.pose.position.z=0.5;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_xy";
  //rotate_control.pose.position.z=0.5;
  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 1;
  rotate_control.orientation.z = 0;
  rotate_control.interaction_mode =
  visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
