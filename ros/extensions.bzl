load("//ros/repositories:repositories.bzl", "ros_repositories")

def _non_module_dependencies_impl(_ctx):
    ros_repositories()

non_module_dependencies = module_extension(
    implementation = _non_module_dependencies_impl,
)
