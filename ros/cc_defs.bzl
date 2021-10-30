""" Defines commonly used macros.
"""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

# This is needed to e.g. set correct logger names.
_ROS_PACKAGE_NAME_DEF = "ROS_PACKAGE_NAME=\\\"{}\\\""

def cc_ros_library(name, ros_package_name = None, **kwargs):
    ros_package_name = ros_package_name or name
    local_defines = kwargs.pop("local_defines", [])
    local_defines.append(_ROS_PACKAGE_NAME_DEF.format(ros_package_name))
    cc_library(
        name = name,
        local_defines = local_defines,
        **kwargs
    )

def cc_ros_binary(name, ros_package_name = None, **kwargs):
    ros_package_name = ros_package_name or name
    local_defines = kwargs.pop("local_defines", [])
    local_defines.append(_ROS_PACKAGE_NAME_DEF.format(ros_package_name))
    cc_binary(
        name = name,
        local_defines = local_defines,
        **kwargs
    )
