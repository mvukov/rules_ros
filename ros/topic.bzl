load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//ros:message_generation.bzl", "py_ros_msg_collector")

def ros_topic(name, deps):
    py_msgs = "{}_py_msgs".format(name)
    py_ros_msg_collector(
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
        srcs = ["//third_party:rostopic_app.py"],
        main = "//third_party:rostopic_app.py",
        deps = [
            py_lib,
            "@ros_comm//:rostopic",
        ],
    )
