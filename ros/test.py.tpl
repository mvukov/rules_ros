import os
import sys

from third_party.ros.rostest import rostest_main

test_outputs_dir = os.environ['TEST_UNDECLARED_OUTPUTS_DIR']
os.environ['ROS_LOG_DIR'] = test_outputs_dir
os.environ['ROS_TEST_RESULTS_DIR'] = test_outputs_dir
os.environ['ROS_HOSTNAME'] = 'localhost'

LAUNCH_FILE = {launch_file}

sys.argv = sys.argv + [LAUNCH_FILE]

rostest_main.rostestmain()
