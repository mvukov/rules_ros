"""Implements functionality for action generation.
"""

load("//ros:message_generation.bzl", "RosMsgInfo", "cc_ros_msg_library", "py_ros_msg_library")
load("//ros:utils.bzl", "get_stem")

RosActionInfo = provider("Provides info for action generation.", fields = [
    "info",
])

def _ros_action_library_impl(ctx):
    import_path = ctx.files.srcs[0].dirname
    for src in ctx.files.srcs[1:]:
        if src.dirname != import_path:
            fail("All action files must be in the same folder!")

    package_name = ctx.attr.package_name
    if not package_name:
        package_name = ctx.label.name

    return [
        DefaultInfo(files = depset(ctx.files.srcs)),
        RosActionInfo(
            info = struct(
                package_name = package_name,
                srcs = ctx.files.srcs,
            ),
        ),
        RosMsgInfo(
            deps = depset(
                direct = [dep[RosMsgInfo].info for dep in ctx.attr.deps],
                transitive = [dep[RosMsgInfo].deps for dep in ctx.attr.deps],
            ),
        ),
    ]

ros_action_library = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".action"],
            mandatory = True,
        ),
        "deps": attr.label_list(providers = [RosMsgInfo]),
        "package_name": attr.string(),
    },
    implementation = _ros_action_library_impl,
)

def _ros_action_msgs_library_impl(ctx):
    action_info = ctx.attr.action_library[RosActionInfo].info
    package_name = action_info.package_name

    rel_output_dir = "{}/{}".format(ctx.label.name, package_name)
    all_msgs = []
    for src in action_info.srcs:
        src_stem = get_stem(src)
        rel_generated_files = [
            "{}/{}Goal.msg".format(rel_output_dir, src_stem),
            "{}/{}ActionGoal.msg".format(rel_output_dir, src_stem),
            "{}/{}Action.msg".format(rel_output_dir, src_stem),
            "{}/{}Result.msg".format(rel_output_dir, src_stem),
            "{}/{}ActionResult.msg".format(rel_output_dir, src_stem),
            "{}/{}Feedback.msg".format(rel_output_dir, src_stem),
            "{}/{}ActionFeedback.msg".format(rel_output_dir, src_stem),
        ]
        generated_files = [
            ctx.actions.declare_file(f)
            for f in rel_generated_files
        ]
        all_msgs += generated_files

        args = [
            src.path,
            "-o",
            generated_files[0].dirname,
        ]

        ctx.actions.run(
            inputs = [src],
            outputs = generated_files,
            executable = ctx.executable._genaction,
            arguments = args,
        )

    return [
        DefaultInfo(files = depset(all_msgs)),
        RosMsgInfo(
            info = struct(
                package_name = package_name,
                import_path = all_msgs[0].dirname,
                srcs = all_msgs,
            ),
            deps = ctx.attr.action_library[RosMsgInfo].deps,
        ),
    ]

ros_action_msgs_library = rule(
    implementation = _ros_action_msgs_library_impl,
    output_to_genfiles = True,
    attrs = {
        "action_library": attr.label(
            mandatory = True,
            providers = [RosActionInfo, RosMsgInfo],
        ),
        "_genaction": attr.label(
            default = Label("@ros_common_msgs//:genaction"),
            executable = True,
            cfg = "host",
        ),
    },
)

def cc_ros_action_library(name, action_library, visibility = None):
    name_msgs = "{}_msgs".format(name)
    ros_action_msgs_library(
        name = name_msgs,
        action_library = action_library,
    )
    cc_ros_msg_library(
        name = name,
        deps = [name_msgs, "@ros_common_msgs//:actionlib_msgs"],
        visibility = visibility,
    )

def py_ros_action_library(name, action_library, visibility = None):
    name_msgs = "{}_msgs".format(name)
    ros_action_msgs_library(
        name = name_msgs,
        action_library = action_library,
    )
    py_ros_msg_library(
        name = name,
        deps = [name_msgs, "@ros_common_msgs//:actionlib_msgs"],
        visibility = visibility,
    )
