""" Implements a macro for setting up target-dependent rostopic app.
"""

load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//ros:interfaces.bzl", "py_ros_interface_collector")

def ros_topic(name, deps):
    """ Defines rostopic app for a set of deps.

    Args:
        name: The app (target) name.
        deps: A list of deps for which all ros_interface_library targets are
        collected and on which this target can operate on. This would typically
        be a list of ROS node targets.
    """
    interfaces = "{}_interfaces".format(name)
    py_ros_interface_collector(
        name = interfaces,
        deps = deps,
    )
    py_binary(
        name = name,
        srcs = ["@ros_comm//:rostopic_app.py"],
        main = "@ros_comm//:rostopic_app.py",
        deps = [
            interfaces,
            "@ros_comm//:rostopic_lib",
            "@ros_genpy//:genpy",
        ],
    )
