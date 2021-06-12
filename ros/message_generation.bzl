"""Implements functionality for message generation.

Inspired by code in https://github.com/nicolov/ros-bazel repo.
"""

load("//ros:utils.bzl", "get_stem")
load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")

RosInterfaceInfo = provider(
    "Provides info for interface code generation.", fields = [
    "info",
    "deps",
])

_ACTION_OUTPUT_MAPPING = [
    "{}Goal.msg",
    "{}ActionGoal.msg",
    "{}Action.msg",
    "{}Result.msg",
    "{}ActionResult.msg",
    "{}Feedback.msg",
    "{}ActionFeedback.msg",
]

def _ros_interface_library_impl(ctx):
    ros_package_name = ctx.label.name
    output_srcs = []  # Messages and services.
    for src in ctx.files.srcs:
        if src.extension == "action":
            stem = get_stem(src)
            action_msgs = [
                ctx.actions.declare_file(
                    "{}/{}".format(ros_package_name, t.format(stem)))
                for t in _ACTION_OUTPUT_MAPPING
            ]

            genaction_args = ctx.actions.args()
            genaction_args.add(src)
            genaction_args.add("-o", action_msgs[0].dirname)
            ctx.actions.run(
                inputs = [src],
                outputs = action_msgs,
                executable = ctx.executable._genaction,
                arguments = [genaction_args],
            )
            output_srcs.extend(action_msgs)

        else:
            src_symlink = ctx.actions.declare_file(
                "{}/{}".format(ros_package_name, src.basename))
            ctx.actions.symlink(output = src_symlink, target_file = src)
            output_srcs.append(src_symlink)

    return [
        DefaultInfo(files = depset(output_srcs)),
        RosInterfaceInfo(
            info = struct(
                ros_package_name = ros_package_name,
                srcs = output_srcs,
            ),
            deps = depset(
                direct = [dep[RosInterfaceInfo].info for dep in ctx.attr.deps],
                transitive = [dep[RosInterfaceInfo].deps for dep in ctx.attr.deps],
            ),
        ),
    ]

ros_interface_library = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".action", ".msg", ".srv"],
            mandatory = True,
        ),
        "deps": attr.label_list(providers = [RosInterfaceInfo]),
        "_genaction": attr.label(
            default = Label("@ros_common_msgs//:genaction"),
            executable = True,
            cfg = "exec",
        ),
    },
    implementation = _ros_interface_library_impl,
)

def _get_deps(attr_deps):
    return depset(
        direct = [src[RosInterfaceInfo].info for src in attr_deps],
        transitive = [src[RosInterfaceInfo].deps for src in attr_deps],
    ).to_list()

def _get_include_flags(deps):
    include_flags = []
    for dep in deps:
        include_flags += [
            "-I",
            "{}:{}".format(dep.ros_package_name, dep.srcs[0].dirname),
        ]
    return include_flags

def _get_all_srcs(deps):
    srcs = []
    for dep in deps:
        srcs += dep.srcs
    return srcs

def _cc_ros_interface_compile_impl(ctx):
    deps = _get_deps(ctx.attr.deps)
    include_flags = _get_include_flags(deps)
    all_srcs = _get_all_srcs(deps)

    all_headers = []
    for dep in deps:
        ros_package_name = dep.ros_package_name
        rel_output_dir = "{}/{}".format(ctx.label.name, ros_package_name)
        for src in dep.srcs:
            src_stem = get_stem(src)
            msg_header = ctx.actions.declare_file(
                "{}/{}.h".format(rel_output_dir, src_stem),
            )
            msg_headers = [msg_header]

            if src.extension == "srv":
                msg_headers.append(ctx.actions.declare_file(
                    "{}/{}Request.h".format(rel_output_dir, src_stem),
                ))
                msg_headers.append(ctx.actions.declare_file(
                    "{}/{}Response.h".format(rel_output_dir, src_stem),
                ))

            all_headers.extend(msg_headers)

            args = [
                "-o",
                msg_header.dirname,
                "-p",
                ros_package_name,
            ] + include_flags + [
                src.path,
            ]

            ctx.actions.run(
                inputs = all_srcs,
                outputs = msg_headers,
                executable = ctx.executable._gencpp,
                arguments = args,
            )

    return [
        DefaultInfo(files = depset(all_headers)),
    ]

cc_ros_interface_compile = rule(
    implementation = _cc_ros_interface_compile_impl,
    output_to_genfiles = True,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            providers = [RosInterfaceInfo],
        ),
        "_gencpp": attr.label(
            default = Label("@ros_gencpp//:gencpp"),
            executable = True,
            cfg = "exec",
        ),
    },
)

def cc_ros_interface_library(name, deps, visibility = None):
    name_gencpp = "{}_gencpp".format(name)
    cc_ros_interface_compile(
        name = name_gencpp,
        deps = deps,
    )
    cc_library(
        name = name,
        hdrs = [name_gencpp],
        includes = [name_gencpp],
        deps = [
            "@roscpp_core//:roscpp_core",
            "@ros_std_msgs//:cc_std_msgs_headers",
        ],
        visibility = visibility,
    )

def _py_generate(ctx, include_flags, all_srcs, ros_package_name, rel_output_dir, msgs):
    if not msgs:
        return []

    extension = msgs[0].extension
    if extension == "msg":
        generator = ctx.executable._genmsg_py
    else:
        generator = ctx.executable._gensrv_py

    py_msg_files = []
    for msg in msgs:
        msg_stem = get_stem(msg)
        py_file = ctx.actions.declare_file(
            "{}/{}/_{}.py".format(rel_output_dir, extension, msg_stem),
        )
        py_msg_files.append(py_file)

    args = [
        "-o",
        py_msg_files[0].dirname,
        "-p",
        ros_package_name,
    ] + include_flags + [
        msg.path
        for msg in msgs
    ]

    ctx.actions.run(
        inputs = all_srcs,
        outputs = py_msg_files,
        executable = generator,
        arguments = args,
    )

    init_py = ctx.actions.declare_file(
        "{}/{}/__init__.py".format(rel_output_dir, extension),
    )

    args = [
        "--initpy",
        "-o",
        py_msg_files[0].dirname,
        "-p",
        ros_package_name,
    ]

    ctx.actions.run(
        inputs = py_msg_files,
        outputs = [init_py],
        executable = generator,
        arguments = args,
    )

    return py_msg_files + [init_py]

def _py_ros_interface_compile_internal(ctx, deps):
    include_flags = _get_include_flags(deps)
    all_srcs = _get_all_srcs(deps)

    ros_package_names_to_srcs = {}
    for dep in deps:
        if dep.ros_package_name not in ros_package_names_to_srcs:
            ros_package_names_to_srcs[dep.ros_package_name] = dep.srcs
        else:
            ros_package_names_to_srcs[dep.ros_package_name] = (
                ros_package_names_to_srcs[dep.ros_package_name] + dep.srcs
            )

    all_py_files = []
    for ros_package_name, srcs in ros_package_names_to_srcs.items():
        rel_output_dir = "{}/{}".format(ctx.label.name, ros_package_name)

        msgs = [src for src in srcs if src.extension == "msg"]
        py_msg_files = _py_generate(
            ctx,
            include_flags,
            all_srcs,
            ros_package_name,
            rel_output_dir,
            msgs,
        )
        all_py_files.extend(py_msg_files)

        srvs = [src for src in srcs if src.extension == "srv"]
        py_srv_files = _py_generate(
            ctx,
            include_flags,
            all_srcs,
            ros_package_name,
            rel_output_dir,
            srvs,
        )
        all_py_files.extend(py_srv_files)

    return [
        DefaultInfo(files = depset(all_py_files)),
    ]

def _py_ros_interface_compile_impl(ctx):
    deps = _get_deps(ctx.attr.deps)
    return _py_ros_interface_compile_internal(ctx, deps)

_PY_GENERATOR_DEPS = {
    "_genmsg_py": attr.label(
        default = Label("@ros_genpy//:genmsg_py"),
        executable = True,
        cfg = "exec",
    ),
    "_gensrv_py": attr.label(
        default = Label("@ros_genpy//:gensrv_py"),
        executable = True,
        cfg = "exec",
    ),
}

py_ros_interface_compile = rule(
    implementation = _py_ros_interface_compile_impl,
    output_to_genfiles = True,
    attrs = dicts.add(_PY_GENERATOR_DEPS, {
        "deps": attr.label_list(
            mandatory = True,
            providers = [RosInterfaceInfo],
        ),
    }),
)

def py_ros_interface_library(name, deps, visibility = None):
    name_genpy = "{}_genpy".format(name)
    py_ros_interface_compile(
        name = name_genpy,
        deps = deps,
    )
    py_library(
        name = name,
        srcs = [name_genpy],
        imports = [name_genpy],
        deps = ["@ros_genpy//:genpy"],
        visibility = visibility,
    )

RosInterfaceCollectorAspectInfo = provider(
    "Collects ros_interface_library targets.",
    fields = [
        "deps",
    ],
)

def _getattr_as_list(rule_attr, attr_name):
    if not hasattr(rule_attr, attr_name):
        return []
    return getattr(rule_attr, attr_name)

def _collect_dependencies(rule_attr, attr_name):
    return [
        dep
        for dep in _getattr_as_list(rule_attr, attr_name)
        if type(dep) == "Target" and RosInterfaceCollectorAspectInfo in dep
    ]

_ROS_MSG_COLLECTOR_ASPECT_ATTRS = ["data", "deps", "hdrs", "srcs"]

def _ros_interface_collector_aspect_impl(target, ctx):
    direct_deps = []
    if ctx.rule.kind == "ros_interface_library":
        direct_deps = [target]

    transitive_deps = []
    for attr_name in _ROS_MSG_COLLECTOR_ASPECT_ATTRS:
        for dep in _collect_dependencies(ctx.rule.attr, attr_name):
            transitive_deps.append(dep[RosInterfaceCollectorAspectInfo].deps)

    return [
        RosInterfaceCollectorAspectInfo(
            deps = depset(
                direct = direct_deps,
                transitive = transitive_deps,
            ),
        ),
    ]

ros_interface_collector_aspect = aspect(
    implementation = _ros_interface_collector_aspect_impl,
    attr_aspects = _ROS_MSG_COLLECTOR_ASPECT_ATTRS,
)

def _py_ros_interface_collector_impl(ctx):
    msg_targets = depset(
        transitive = [
            dep[RosInterfaceCollectorAspectInfo].deps
            for dep in ctx.attr.deps
        ],
    ).to_list()
    deps = _get_deps(msg_targets)
    return _py_ros_interface_compile_internal(ctx, deps)

py_ros_interface_collector = rule(
    implementation = _py_ros_interface_collector_impl,
    attrs = dicts.add(_PY_GENERATOR_DEPS, {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [ros_interface_collector_aspect],
            providers = [RosInterfaceCollectorAspectInfo],
        ),
    }),
)
