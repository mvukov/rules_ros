"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros_repositories():
    """Imports external/third-party repositories.
    """

    maybe(
        http_archive,
        name = "console_bridge",
        build_file = "@rules_ros//ros/repositories:console_bridge.BUILD.bazel",
        sha256 = "2ff175a9bb2b1849f12a6bf972ce7e4313d543a2bbc83b60fdae7db6e0ba353f",
        strip_prefix = "console_bridge-1.0.1",
        urls = ["https://github.com/ros/console_bridge/archive/1.0.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "roscpp_core",
        build_file = "@rules_ros//ros/repositories:roscpp_core.BUILD.bazel",
        sha256 = "9ada9ab00f1c28ef39069f8bb8642a33f127d6c5c49bc3d9057cadbd8d7c172e",
        strip_prefix = "roscpp_core-0.7.3",
        urls = ["https://github.com/ros/roscpp_core/archive/0.7.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "rosconsole",
        build_file = "@rules_ros//ros/repositories:rosconsole.BUILD.bazel",
        sha256 = "0b2cbc4f9a92466c0fbae7863482b286ef87692de4941527cb429e6c74639246",
        strip_prefix = "rosconsole-1.14.3",
        urls = ["https://github.com/ros/rosconsole/archive/1.14.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genmsg",
        build_file = "@rules_ros//ros/repositories:genmsg.BUILD.bazel",
        sha256 = "0e414846823a2aaa7781f81268251c7c9a45ff96cef8e6a78bbbbcf7e4c28d56",
        strip_prefix = "genmsg-0.5.16",
        urls = ["https://github.com/ros/genmsg/archive/0.5.16.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_gencpp",
        build_file = "@rules_ros//ros/repositories:gencpp.BUILD.bazel",
        sha256 = "05acfeeb1bbc374356bf7674fee2a7aab3bf6a48ebad4a06fd0f0d4455a60720",
        strip_prefix = "gencpp-0.6.5",
        urls = ["https://github.com/ros/gencpp/archive/0.6.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genpy",
        build_file = "@rules_ros//ros/repositories:genpy.BUILD.bazel",
        sha256 = "523d20068171ce7e5b4c453eba7976aafa819e6b5af806ffdf6bc1d7a1dfc2a8",
        strip_prefix = "genpy-0.6.14",
        urls = ["https://github.com/ros/genpy/archive/0.6.14.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_std_msgs",
        build_file = "@rules_ros//ros/repositories:std_msgs.BUILD.bazel",
        sha256 = "ee6592d37b00a94cab8216aac2cfb5120f6da09ffa94bfe197fe8dc76dd21326",
        strip_prefix = "std_msgs-0.5.13",
        urls = ["https://github.com/ros/std_msgs/archive/0.5.13.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_comm_msgs",
        build_file = "@rules_ros//ros/repositories:ros_comm_msgs.BUILD.bazel",
        sha256 = "5b8b91e8671d03ea84ba32a3ea7360bc4594655e7ba3ec6677a984f393aaafbd",
        strip_prefix = "ros_comm_msgs-1.11.3",
        urls = ["https://github.com/ros/ros_comm_msgs/archive/1.11.3.tar.gz"],
    )

    # Branched off ros_comm 1.15.9. Changes are in branch feature/rules_ros.
    maybe(
        http_archive,
        name = "ros_comm",
        build_file = "@rules_ros//ros/repositories:ros_comm.BUILD.bazel",
        sha256 = "65906eab18bdf5832af7b6de8c21e3d87d3fac9732a1219637f5b6f5c948508d",
        strip_prefix = "ros_comm-30922861f247937001f82094c0934a6b802b6325",
        urls = ["https://github.com/mvukov/ros_comm/archive/30922861f247937001f82094c0934a6b802b6325.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_ros",
        build_file = "@rules_ros//ros/repositories:ros.BUILD.bazel",
        sha256 = "7a1e729de9be807862b6ed721475fec74583f6dc0c06b233b06b1b9fda31291e",
        strip_prefix = "ros-1.15.7",
        urls = ["https://github.com/ros/ros/archive/1.15.7.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_common_msgs",
        build_file = "@rules_ros//ros/repositories:common_msgs.BUILD.bazel",
        sha256 = "74af8cc88bdc9c23cbc270d322e50562857e2c877359423f389d51c0735ee230",
        strip_prefix = "common_msgs-1.13.1",
        urls = ["https://github.com/ros/common_msgs/archive/1.13.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_actionlib",
        build_file = "@rules_ros//ros/repositories:actionlib.BUILD.bazel",
        sha256 = "b741755881e30b9aea6bcdd9831e3f0932a8bbba02fa59e5c0e5970280024055",
        strip_prefix = "actionlib-1.13.2",
        urls = ["https://github.com/ros/actionlib/archive/1.13.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_dynamic_reconfigure",
        build_file = "@rules_ros//ros/repositories:dynamic_reconfigure.BUILD.bazel",
        sha256 = "7cdb46269ae222a0ed5632d7c9b9d6f1e351c35dc9cc14b48930ad89273a5da5",
        strip_prefix = "dynamic_reconfigure-1.7.1",
        urls = ["https://github.com/ros/dynamic_reconfigure/archive/refs/tags/1.7.1.tar.gz"],
    )

    ############################################################################

    maybe(
        http_archive,
        name = "ros_geometry2",
        build_file = "@rules_ros//ros/repositories:geometry2.BUILD.bazel",
        sha256 = "0b5d461c71d6dc1dbdb99a2ba39e1515194cd451c2e53d53caadb3ecea13367a",
        strip_prefix = "geometry2-0.7.5",
        urls = ["https://github.com/ros/geometry2/archive/0.7.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "orocos_kdl",
        build_file = "@rules_ros//ros/repositories:orocos_kdl.BUILD.bazel",
        sha256 = "5acb90acd82b10971717aca6c17874390762ecdaa3a8e4db04984ea1d4a2af9b",
        strip_prefix = "orocos_kinematics_dynamics-1.5.1",
        urls = ["https://github.com/orocos/orocos_kinematics_dynamics/archive/refs/tags/v1.5.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "urdfdom_headers",
        build_file = "@rules_ros//ros/repositories:urdfdom_headers.BUILD.bazel",
        sha256 = "76a68657c38e54bb45bddc4bd7d823a3b04edcd08064a56d8e7d46b9912035ac",
        strip_prefix = "urdfdom_headers-1.0.5",
        urls = ["https://github.com/ros/urdfdom_headers/archive/refs/tags/1.0.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "tinyxml",
        build_file = "@rules_ros//ros/repositories:tinyxml.BUILD.bazel",
        sha256 = "15bdfdcec58a7da30adc87ac2b078e4417dbe5392f3afb719f9ba6d062645593",
        urls = [
            "https://sourceforge.net/projects/tinyxml/files/tinyxml/2.6.2/tinyxml_2_6_2.tar.gz",
            "http://archive.ubuntu.com/ubuntu/pool/universe/t/tinyxml/tinyxml_2.6.2.orig.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "urdfdom",
        build_file = "@rules_ros//ros/repositories:urdfdom.BUILD.bazel",
        sha256 = "8f3d56b0cbc4b84436d8baf4c8346cd2ee7ffb257bba5ddd9892c41bf516edc4",
        strip_prefix = "urdfdom-1.0.4",
        urls = ["https://github.com/ros/urdfdom/archive/refs/tags/1.0.4.tar.gz"],
    )
