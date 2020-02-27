#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

//Flags that provide the status of the robot
bool pick_up= false;
bool drop_off=false;

void chatterCallback (const nav_msgs::Odometry::ConstPtr& msg){
  //Retrieve position of the robot provided by the Odom
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  //ROS_INFO(" x: %f, y: %f",x,y);

  // Robot lies in the Pick Up Area (Including Error)
  if(x >= -4.40 && x <= -4.35 && y >= -3.60 && y <= -3.50){
    pick_up= true;
  }
  // Robot lies in the Drop Off Area (Including Error)
  else if (x >= 2.50 && x <= 3.0 && y >= -2.03 && y <= -1.95){
    drop_off=true;
  }
  // Robot is in its way to a specific location 
  else{
    pick_up= false;
    drop_off=false;
  }
} 

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  // Subscriber 
  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  // Publisher
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type to be a cube
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action for the pick up location
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker for the pick up zone
  marker.pose.position.x = -0.5;
  marker.pose.position.y = 3.5;
  marker.pose.position.z = 0.05;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker. Define the dimensions of the Cube.
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  //Publish the marker (Make it appear on the Pick Up location on Rviz) 
  marker_pub.publish(marker);

  while (ros::ok()){
    //Checks the Callback function
    ros::spinOnce(); 
    // Robot in Pick Up zone
    if(pick_up == true){
      //Publish the marker (Make it disappear on Rviz)
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }
    // Robot in Drop off zone
    else if (drop_off == true){
      //Publish the marker (Make it appear on the Drop off location on Rviz)
      marker.action = visualization_msgs::Marker::ADD;
      // Set the pose of the marker for the Drop Off location
      marker.pose.position.x = 1.0;
      marker.pose.position.y = -4.0;
      marker.pose.position.z = 0.05;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker_pub.publish(marker);     
    } 
  }
  return 0;
}