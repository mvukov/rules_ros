"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros_repositories():
    """Imports external/third-party repositories.
    """

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "778197e26c5fbeb07ac2a2c5ae405b30f6cb7ad1f5510ea6fdac03bded96cc6f",
        urls = ["https://github.com/bazelbuild/rules_python/releases/download/0.2.0/rules_python-0.2.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_gflags_gflags",
        sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
        strip_prefix = "gflags-2.2.2",
        urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_glog",
        sha256 = "62efeb57ff70db9ea2129a16d0f908941e355d09d6d83c9f7b18557c0a7ab59e",
        strip_prefix = "glog-d516278b1cd33cd148e8989aec488b6049a4ca0b",
        urls = ["https://github.com/google/glog/archive/d516278b1cd33cd148e8989aec488b6049a4ca0b.zip"],
    )

    maybe(
        http_archive,
        name = "rules_foreign_cc",
        sha256 = "4f2207a83c500a16c9095a224acc4ec8c1406c6febd0a042b3ac15ef6ff0c640",
        strip_prefix = "rules_foreign_cc-83e6cf48cf00214bb4348691beecef8f3bc42f7d",
        urls = ["https://github.com/bazelbuild/rules_foreign_cc/archive/83e6cf48cf00214bb4348691beecef8f3bc42f7d.zip"],
    )

    maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        strip_prefix = "rules_boost-98495a618246683c9058dd87c2c78a2c06087999",
        sha256 = "fc2b63b293d0d5eebe7a6b3328bec8a13f4b0bc2b532f4e117b51c7f4e5d421b",
        urls = ["https://github.com/nelhage/rules_boost/archive/98495a618246683c9058dd87c2c78a2c06087999.zip"],
    )

    maybe(
        http_archive,
        name = "console_bridge",
        build_file = "@com_github_mvukov_rules_ros//repositories:console_bridge.BUILD.bazel",
        sha256 = "2ff175a9bb2b1849f12a6bf972ce7e4313d543a2bbc83b60fdae7db6e0ba353f",
        strip_prefix = "console_bridge-1.0.1",
        urls = ["https://github.com/ros/console_bridge/archive/1.0.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "roscpp_core",
        build_file = "@com_github_mvukov_rules_ros//repositories:roscpp_core.BUILD.bazel",
        sha256 = "a2aa77814ed97b48995c872a405c51f6b0f1ab9d40e38ece483852bbd273ad7b",
        strip_prefix = "roscpp_core-0.7.2",
        urls = ["https://github.com/ros/roscpp_core/archive/0.7.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "rosconsole",
        build_file = "@com_github_mvukov_rules_ros//repositories:rosconsole.BUILD.bazel",
        sha256 = "0b2cbc4f9a92466c0fbae7863482b286ef87692de4941527cb429e6c74639246",
        strip_prefix = "rosconsole-1.14.3",
        urls = ["https://github.com/ros/rosconsole/archive/1.14.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genmsg",
        build_file = "@com_github_mvukov_rules_ros//repositories:genmsg.BUILD.bazel",
        sha256 = "0e414846823a2aaa7781f81268251c7c9a45ff96cef8e6a78bbbbcf7e4c28d56",
        strip_prefix = "genmsg-0.5.16",
        urls = ["https://github.com/ros/genmsg/archive/0.5.16.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_gencpp",
        build_file = "@com_github_mvukov_rules_ros//repositories:gencpp.BUILD.bazel",
        sha256 = "05acfeeb1bbc374356bf7674fee2a7aab3bf6a48ebad4a06fd0f0d4455a60720",
        strip_prefix = "gencpp-0.6.5",
        urls = ["https://github.com/ros/gencpp/archive/0.6.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genpy",
        build_file = "@com_github_mvukov_rules_ros//repositories:genpy.BUILD.bazel",
        sha256 = "523d20068171ce7e5b4c453eba7976aafa819e6b5af806ffdf6bc1d7a1dfc2a8",
        strip_prefix = "genpy-0.6.14",
        urls = ["https://github.com/ros/genpy/archive/0.6.14.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_std_msgs",
        build_file = "@com_github_mvukov_rules_ros//repositories:std_msgs.BUILD.bazel",
        sha256 = "ee6592d37b00a94cab8216aac2cfb5120f6da09ffa94bfe197fe8dc76dd21326",
        strip_prefix = "std_msgs-0.5.13",
        urls = ["https://github.com/ros/std_msgs/archive/0.5.13.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_comm_msgs",
        build_file = "@com_github_mvukov_rules_ros//repositories:ros_comm_msgs.BUILD.bazel",
        sha256 = "5b8b91e8671d03ea84ba32a3ea7360bc4594655e7ba3ec6677a984f393aaafbd",
        strip_prefix = "ros_comm_msgs-1.11.3",
        urls = ["https://github.com/ros/ros_comm_msgs/archive/1.11.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "lz4",
        build_file = "@com_github_mvukov_rules_ros//repositories:lz4.BUILD.bazel",
        sha256 = "030644df4611007ff7dc962d981f390361e6c97a34e5cbc393ddfbe019ffe2c1",
        strip_prefix = "lz4-1.9.3",
        urls = ["https://github.com/lz4/lz4/archive/v1.9.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "bzip2",
        build_file = "@com_github_mvukov_rules_ros//repositories:bzip2.BUILD.bazel",
        sha256 = "ab5a03176ee106d3f0fa90e381da478ddae405918153cca248e682cd0c4a2269",
        strip_prefix = "bzip2-1.0.8",
        urls = ["https://sourceware.org/pub/bzip2/bzip2-1.0.8.tar.gz"],
    )

    # Branched off ros_comm 1.15.9. Changes are in branch feature/rules_ros.
    maybe(
        http_archive,
        name = "ros_comm",
        build_file = "@com_github_mvukov_rules_ros//repositories:ros_comm.BUILD.bazel",
        sha256 = "a9dc58e6001f839c4c54f0a3d63c1272a7b800ae142baa048201ff59c9d17833",
        strip_prefix = "ros_comm-38549f0fe9a85a62b8ca5b46f82a9bae40eca2e0",
        urls = ["https://github.com/mvukov/ros_comm/archive/38549f0fe9a85a62b8ca5b46f82a9bae40eca2e0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_ros",
        build_file = "@com_github_mvukov_rules_ros//repositories:ros.BUILD.bazel",
        sha256 = "7a1e729de9be807862b6ed721475fec74583f6dc0c06b233b06b1b9fda31291e",
        strip_prefix = "ros-1.15.7",
        urls = ["https://github.com/ros/ros/archive/1.15.7.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_common_msgs",
        build_file = "@com_github_mvukov_rules_ros//repositories:common_msgs.BUILD.bazel",
        sha256 = "74af8cc88bdc9c23cbc270d322e50562857e2c877359423f389d51c0735ee230",
        strip_prefix = "common_msgs-1.13.1",
        urls = ["https://github.com/ros/common_msgs/archive/1.13.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_actionlib",
        build_file = "@com_github_mvukov_rules_ros//repositories:actionlib.BUILD.bazel",
        sha256 = "b741755881e30b9aea6bcdd9831e3f0932a8bbba02fa59e5c0e5970280024055",
        strip_prefix = "actionlib-1.13.2",
        urls = ["https://github.com/ros/actionlib/archive/1.13.2.tar.gz"],
    )
