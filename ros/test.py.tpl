import os
import sys

from third_party.legacy_rostest import rostest_main

test_outputs_dir = os.environ['TEST_UNDECLARED_OUTPUTS_DIR']
os.environ['ROS_LOG_DIR'] = test_outputs_dir
os.environ['ROS_TEST_RESULTS_DIR'] = test_outputs_dir
os.environ['ROS_MASTER_URI'] = "http://0.0.0.0:11311"
os.environ['ROS_HOSTNAME'] = "0.0.0.0"
os.environ['ROS_ROOT'] = os.getcwd()
os.environ['ROS_PACKAGE_PATH'] = os.getcwd() + "/ROS"

LAUNCH_FILE = {launch_file}

# stupid hack for swapping with gen rules
if "bazel-out/k8-fastbuild/bin/" in LAUNCH_FILE:
    LAUNCH_FILE = "/".join(LAUNCH_FILE.split("/")[3:])
sys.argv = sys.argv + [LAUNCH_FILE]
path_override = [{node_path_override}]

if path_override:
    for override in path_override:
        os.symlink(override,override.split("/")[-1])

rostest_main.rostestmain()
