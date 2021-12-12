import sys

from third_party.legacy_roslaunch import main as roslaunch_main

LAUNCH_FILES = [{launch_files}]
LAUNCH_ARGS = [{launch_args}]

all_args = sys.argv + LAUNCH_FILES + LAUNCH_ARGS

roslaunch_main.main(all_args)
