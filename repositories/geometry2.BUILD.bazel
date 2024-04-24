""" Builds geometry2.
"""

load(
    "@com_github_mvukov_rules_ros//ros:cc_defs.bzl",
    "cc_ros_binary",
    "cc_ros_library",
)
load(
    "@com_github_mvukov_rules_ros//ros:interfaces.bzl",
    "cc_ros_interface_library",
    "ros_interface_library",
)

ros_interface_library(
    name = "tf2_msgs",
    srcs = glob([
        "tf2_msgs/action/*.action",
        "tf2_msgs/msg/*.msg",
        "tf2_msgs/srv/*.srv",
    ]),
    visibility = ["//visibility:public"],
    deps = [
        "@ros_common_msgs//:actionlib_msgs",
        "@ros_common_msgs//:geometry_msgs",
    ],
)

cc_ros_interface_library(
    name = "cc_tf2_msgs",
    deps = [":tf2_msgs"],
)

cc_ros_library(
    name = "tf2",
    srcs = glob(["tf2/src/*.cpp"]),
    hdrs = glob(["tf2/include/**/*.h"]),
    includes = ["tf2/include"],
    visibility = ["//visibility:public"],
    copts = [ "-std=c++17"],
    deps = [
        ":cc_tf2_msgs",
        "@boost//:signals2",
        "@boost//:thread",
        "@boost//:unordered",
        "@console_bridge",
        "@ros_common_msgs//:cc_geometry_msgs",
        "@roscpp_core//:rostime",
    ],
)

cc_ros_library(
    name = "tf2_ros",
    srcs = [
        "tf2_ros/src/buffer.cpp",
        "tf2_ros/src/buffer_client.cpp",
        "tf2_ros/src/buffer_server.cpp",
        "tf2_ros/src/static_transform_broadcaster.cpp",
        "tf2_ros/src/transform_broadcaster.cpp",
        "tf2_ros/src/transform_listener.cpp",
    ],
    hdrs = glob(["tf2_ros/include/tf2_ros/*.h"]),
    copts = ["-w",  "-std=c++17"],
    includes = ["tf2_ros/include"],
    visibility = ["//visibility:public"],
    deps = [
        ":cc_tf2_msgs",
        ":tf2",
        "@ros_actionlib//:actionlib",
        "@ros_comm//:message_filters",
        "@ros_comm//:roscpp_lib",
        "@ros_common_msgs//:cc_actionlib_msgs",
        "@ros_common_msgs//:cc_geometry_msgs",
    ],
)

cc_ros_binary(
    name = "tf2_ros_buffer_server",
    srcs = ["tf2_ros/src/buffer_server_main.cpp"],
    visibility = ["//visibility:public"],
    deps = [":tf2_ros"],
)

cc_ros_binary(
    name = "tf2_ros_static_transform_publisher",
    srcs = ["tf2_ros/src/static_transform_broadcaster_program.cpp"],
    visibility = ["//visibility:public"],
    deps = [":tf2_ros"],
)

cc_ros_library(
    name = "tf2_eigen",
    srcs = [
        "tf2_eigen/include/tf2_eigen/tf2_eigen.h"
    ],
    includes = ["tf2_eigen/include"],
    visibility = ["//visibility:public"],
    deps = [
        ":tf2",
    ],
)

cc_ros_library(
    name = "tf2_geometry_msgs",
    srcs = [ "tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.h" ],
    includes = ["tf2_geometry_msgs/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@orocos_kdl//:orocos_kdl"
    ]
)
