""" Implements a macro for setting up target-dependent rostopic app.
"""

load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//ros:interfaces.bzl", "py_ros_interface_collector")

def ros_topic(name, deps):
    """ Defines rostopic app for a set of deps.

    Args:
        name: The app (target) name.
        deps: A set of deps for which all ros_interface_library targets are
        collected and on which this target can operate on.
    """

    py_msgs = "{}_py_msgs".format(name)
    py_ros_interface_collector(
        name = py_msgs,
        deps = deps,
    )
    py_lib = "{}_lib".format(name)
    py_library(
        name = py_lib,
        srcs = [py_msgs],
        imports = [py_msgs],
        deps = ["@ros_genpy//:genpy"],
    )
    py_binary(
        name = name,
        srcs = ["@ros_comm//:rostopic_app.py"],
        main = "@ros_comm//:rostopic_app.py",
        deps = [
            py_lib,
            "@ros_comm//:rostopic_lib",
        ],
    )
