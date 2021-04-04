#include <actionlib/client/simple_action_client.h>
#include <dishwasher_actions/DoDishesAction.h>

using Client =
    actionlib::SimpleActionClient<dishwasher_actions::DoDishesAction>;

int main(int argc, char** argv) {
  ros::init(argc, argv, "dishwasher_client");

  Client client("do_dishes", true);
  client.waitForServer();

  dishwasher_actions::DoDishesGoal goal;

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Yay! The dishes are now clean.");
  }
  ROS_INFO_STREAM("Current state: " << client.getState().toString().c_str());
  return EXIT_SUCCESS;
}
