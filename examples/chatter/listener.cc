#include <example_msgs/Example.h>
#include <ros/ros.h>

void ChatterCallback(const example_msgs::Example& msg) {
  ROS_INFO_STREAM("Received: " << msg.message);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Subscriber chatter_sub = n.subscribe("chatter", 1000, ChatterCallback);

  ros::spin();

  return 0;
}
