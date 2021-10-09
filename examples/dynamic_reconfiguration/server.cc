#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tutorial_cfg/TutorialConfig.h>

void Callback(tutorial_cfg::TutorialConfig& config, uint32_t /*level*/) {
  ROS_INFO("Reconfigure request: %d %f %s %s %d", config.int_param,
           config.double_param, config.str_param.c_str(),
           config.bool_param ? "True" : "False", config.size);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "server");

  using Server = dynamic_reconfigure::Server<tutorial_cfg::TutorialConfig>;
  Server server;
  server.setCallback(boost::bind(&Callback, _1, _2));

  ros::spin();
  return EXIT_SUCCESS;
}
