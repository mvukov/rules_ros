load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//ros:message_generation.bzl", "py_ros_interface_collector")

def ros_topic(name, deps):
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
