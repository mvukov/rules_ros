""" Defines commonly used macros.
"""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

# This is needed to e.g. set correct logger names.
_ROS_PACKAGE_NAME_DEF = "ROS_PACKAGE_NAME=\\\"{}\\\""

def cc_ros_library(name, ros_package_name = None, **kwargs):
    """ Defines a ROS cc_library.

    Adds common ROS definitions on top of a cc_library.

    Args:
        name: A unique target name.
        ros_package_name: If given, defines a ROS package name for the target.
            Otherwise, the `name` is used as the package name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    ros_package_name = ros_package_name or name
    local_defines = kwargs.pop("local_defines", [])
    local_defines.append(_ROS_PACKAGE_NAME_DEF.format(ros_package_name))
    cc_library(
        name = name,
        local_defines = local_defines,
        target_compatible_with = [
            "@platforms//os:linux",
            "@platforms//os:osx",
        ],
        **kwargs
    )

def cc_ros_binary(name, ros_package_name = None, **kwargs):
    """ Defines a ROS cc_binary.

    Adds common ROS definitions on top of a cc_binary.

    Args:
        name: A unique target name.
        ros_package_name: If given, defines a ROS package name for the target.
            Otherwise, the `name` is used as the package name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    ros_package_name = ros_package_name or name
    local_defines = kwargs.pop("local_defines", [])
    local_defines.append(_ROS_PACKAGE_NAME_DEF.format(ros_package_name))
    cc_binary(
        name = name,
        local_defines = local_defines,
        target_compatible_with = [
            "@platforms//os:linux",
            "@platforms//os:osx",
        ],
        **kwargs
    )
