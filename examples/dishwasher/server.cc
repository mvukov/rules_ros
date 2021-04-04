#include <actionlib/server/simple_action_server.h>
#include <dishwasher_actions/DoDishesAction.h>

using Server =
    actionlib::SimpleActionServer<dishwasher_actions::DoDishesAction>;

void execute(const dishwasher_actions::DoDishesGoalConstPtr& goal,
             Server* server) {
  server->setSucceeded();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dishwasher_server");
  ros::NodeHandle n;
  Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return EXIT_SUCCESS;
}
