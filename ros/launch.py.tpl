import sys

from third_party.ros.roslaunch import main as roslaunch_main

LAUNCH_FILES = [{launch_files}]

all_args = sys.argv + LAUNCH_FILES

roslaunch_main.main(all_args)
