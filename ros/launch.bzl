"""Implements functionality for launching ROS master and ROS nodes.
"""

load("@rules_python//python:defs.bzl", "py_binary")

ROSCORE_XML = "//third_party/legacy_roslaunch:roscore.xml"

def ros_launch(name, nodes, launch_files):
    all_launch_targets = [ROSCORE_XML] + launch_files
    all_launch_file_paths = ["$(location {})".format(x) for x in all_launch_targets]

    py_binary(
        name = name,
        srcs = ["//third_party/legacy_roslaunch:main.py"],
        data = [
            ROSCORE_XML,
            "@ros_comm//:rosmaster",
            "@ros_comm//:rosout",
        ] + nodes + launch_files,
        main = "//third_party/legacy_roslaunch:main.py",
        visibility = ["//visibility:public"],
        deps = ["//third_party/legacy_roslaunch"],
        args = all_launch_file_paths,
    )
