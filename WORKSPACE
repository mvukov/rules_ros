workspace(name = "com_github_mvukov_rules_ros")

load("//repositories:repositories.bzl", "ros_repositories")

ros_repositories()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "rules_ros_python",
    python_version = "3.10",
)

load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros_python//:defs.bzl", python_interpreter_target = "interpreter")

pip_parse(
    name = "rules_ros_pip_deps",
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "@com_github_mvukov_rules_ros//:requirements_lock.txt",
)

load(
    "@rules_ros_pip_deps//:requirements.bzl",
    install_rules_ros_pip_deps = "install_deps",
)

install_rules_ros_pip_deps()

load("//repositories:deps.bzl", "ros_deps")

ros_deps()
