import os
import sys

from third_party.legacy_rostest import rostest_main

test_outputs_dir = os.environ['TEST_UNDECLARED_OUTPUTS_DIR']
os.environ['ROS_LOG_DIR'] = test_outputs_dir
os.environ['ROS_TEST_RESULTS_DIR'] = test_outputs_dir

LAUNCH_FILE = {launch_file}
LAUNCH_ARGS = [{launch_args}]

sys.argv = sys.argv + [LAUNCH_FILE] + LAUNCH_ARGS

rostest_main.rostestmain()
