workspace(name = "com_github_mvukov_rules_ros")

load("//repositories:repositories.bzl", "ros_repositories")

ros_repositories()

load("//repositories:deps.bzl", "ros_deps")

PYTHON_INTERPRETER = "python3.8"

ros_deps(
    python_interpreter = PYTHON_INTERPRETER,
    python_requirements = "//:requirements.txt",
)

# Install via apt:
# For rosbag: libbz2-dev liblz4-dev
