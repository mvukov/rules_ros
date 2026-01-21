""" Implements functionality for launching ROS deployments using roslaunch.
"""

load("@aspect_bazel_lib//lib:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

# RosLaunchInfo Provider:
#
# This provider propagates information about ROS nodes, launch files, and parameter files through
# the build graph. It allows downstream targets to access the necessary files for launching ROS systems.
#
# Fields:
# - nodes: A `depset` of binaries or scripts to be launched as ROS nodes.
# - launch_files: A `depset` of ROS launch files that define how the nodes are launched.
# - param_files: A `depset` of YAML files used to configure the ROS nodes.
RosLaunchInfo = provider(
    doc = "Relevant information for a ROS launch target to be depended on another target.",
    fields = {
        "nodes": "A depset of node binaries or scripts.",
        "launch_files": "A depset of launch files.",
        "param_files": "A depset of parameter files.",
    },
)

def _ros_launch_impl(ctx):
    """
    Implementation of the ros_launch_rule.

    This function generates a ROS launch script by expanding a template file and collects all transitive
    dependencies such as nodes, launch files, and parameter files. The generated script, along with
    the necessary resources, is included in the runfiles, ensuring they are available at runtime.

    Key steps:
    - Expands a Python script template to handle the ROS launch process.
    - Collects transitive dependencies from the current target and its dependencies.
    - Defines runfiles to ensure the generated launch script has access to all necessary files.

    Returns:
    - RosLaunchInfo: Provides the nodes, launch files, and parameter files for downstream rules.
    - DefaultInfo: Marks the generated script as an executable and ensures the necessary runfiles are provided.
    """

    launch_file_paths = ["'{}'".format(launch_file.path) for launch_file in ctx.files.launch_files]
    substitutions = {
        "{launch_files}": ", ".join(launch_file_paths),
    }

    # Create the launch script file
    launch_script = ctx.actions.declare_file("{}.py".format(ctx.attr.name))
    template = ctx.file._template
    ctx.actions.expand_template(
        template = template,
        output = launch_script,
        substitutions = substitutions,
        is_executable = True,
    )

    # Collect transitive information from dependencies
    transitive_nodes = []
    transitive_launch_files = []
    transitive_param_files = []
    for dep in ctx.attr.deps:
        dep_info = dep[RosLaunchInfo]
        transitive_nodes.append(dep_info.nodes)
        transitive_launch_files.append(dep_info.launch_files)
        transitive_param_files.append(dep_info.param_files)

    # Combine all transitive and direct files into a single list for runfiles
    runfiles_list = list(ctx.files.launch_files) + list(ctx.files.param_files) + list(ctx.files.nodes) + [launch_script]
    for t_nodes in transitive_nodes:
        runfiles_list.extend(t_nodes.to_list())
    for t_launch_files in transitive_launch_files:
        runfiles_list.extend(t_launch_files.to_list())
    for t_param_files in transitive_param_files:
        runfiles_list.extend(t_param_files.to_list())

    # Create runfiles
    runfiles = ctx.runfiles(files = runfiles_list)

    # Return custom RosLaunchInfo provider
    return [
        RosLaunchInfo(
            nodes = depset(direct = ctx.files.nodes, transitive = transitive_nodes),
            launch_files = depset(direct = ctx.files.launch_files, transitive = transitive_launch_files),
            param_files = depset(direct = ctx.files.param_files, transitive = transitive_param_files),
        ),
        DefaultInfo(executable = launch_script, data_runfiles = runfiles),
    ]

ros_launch_rule = rule(
    doc = "Defines a rule for launching ROS nodes and configurations.",
    implementation = _ros_launch_impl,
    attrs = {
        "nodes": attr.label_list(mandatory = True),
        "launch_files": attr.label_list(mandatory = True, allow_files = [".launch"]),
        "param_files": attr.label_list(allow_files = [".yaml"], default = []),
        "deps": attr.label_list(allow_empty = True, providers = [RosLaunchInfo]),
        "_template": attr.label(
            default = Label("@com_github_mvukov_rules_ros//ros:launch.py.tpl"),
            allow_single_file = True,
        ),
    },
    outputs = {"launch_script": "%{name}.py"},
    provides = [RosLaunchInfo, DefaultInfo],
)

def ros_launch(name, nodes, launch_files, param_files = [], deps = [], **kwargs):
    """
    Macro for simplifying the creation of a ROS launch target.

    This macro generates a launch script for running a ROS system and defines a `py_binary` target to
    execute the script. It wraps around the `ros_launch_rule` and handles the setup of the necessary
    runtime dependencies and resources.

    Args:
    - name: The name of the target.
    - nodes: Labels for node binaries or scripts that need to be launched.
    - launch_files: Labels for ROS launch files.
    - param_files: Optional labels for YAML parameter files.
    - deps: Optional list of dependencies that may provide RosLaunchInfo for transitive collection.
    - **kwargs: Additional arguments passed to the `py_binary` rule.
    """

    ros_launch_name = name + "_launch"

    data = kwargs.pop("data", [])

    # Define the ros_launch_rule target
    ros_launch_rule(
        name = name,
        nodes = nodes,
        launch_files = launch_files,
        param_files = param_files,
        deps = deps,
    )

    # Create the py_binary, passing the transitive data to the 'data' attribute
    py_binary(
        name = ros_launch_name,
        srcs = ["{}.py".format(name)],
        main = "{}.py".format(name),
        deps = ["@com_github_mvukov_rules_ros//third_party/ros:roslaunch"],
        data = [":{}".format(name)] + data,
        **kwargs
    )
