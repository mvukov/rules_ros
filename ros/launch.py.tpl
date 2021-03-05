import os
import sys

from third_party.legacy_roslaunch import main as roslaunch_main

DEFAULT_PYTHON_LOGGING_CONF = '{default_python_logging_conf}'
LAUNCH_FILES = [{launch_files}]

if not os.environ.get('ROS_PYTHON_LOG_CONFIG_FILE', None):
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = DEFAULT_PYTHON_LOGGING_CONF

all_args = sys.argv + LAUNCH_FILES

roslaunch_main.main(all_args)
