#include <dynamic_reconfigure/client.h>
#include <ros/ros.h>
#include <tutorial_cfg/TutorialConfig.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "client");

  using Client = dynamic_reconfigure::Client<tutorial_cfg::TutorialConfig>;
  Client client("server");

  tutorial_cfg::TutorialConfig config;
  ros::Rate loop_rate(2);

  int count = 0;
  while (ros::ok()) {
    config.int_param = count;
    client.setConfiguration(config);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return EXIT_SUCCESS;
}
