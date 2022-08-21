import os
import sys

from third_party.legacy_rostest import rostest_main

test_outputs_dir = os.environ['TEST_UNDECLARED_OUTPUTS_DIR']
os.environ['ROS_LOG_DIR'] = test_outputs_dir
os.environ['ROS_TEST_RESULTS_DIR'] = test_outputs_dir

LAUNCH_FILE = {launch_file}

sys.argv = sys.argv[:1] + [LAUNCH_FILE] + sys.argv[1:]

rostest_main.rostestmain()
