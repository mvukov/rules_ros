#include <sstream>

#include <example_msgs/Example.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher chatter_pub =
      n.advertise<example_msgs::Example>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    example_msgs::Example msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.message = ss.str();

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
