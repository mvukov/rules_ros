"""Implements functionality for message generation.

Inspired by code in https://github.com/nicolov/ros-bazel repo.
"""

load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")

RosMsgInfo = provider("Provides info for message generation.", fields = [
    "info",
    "deps",
])

def _ros_msg_library_impl(ctx):
    import_path = ctx.files.srcs[0].dirname
    for src in ctx.files.srcs[1:]:
        if src.dirname != import_path:
            fail("All message/service files must be in the same folder!")

    package_name = ctx.attr.package_name
    if not package_name:
        package_name = ctx.label.name

    return [
        DefaultInfo(files = depset(ctx.files.srcs)),
        RosMsgInfo(
            info = struct(
                package_name = package_name,
                import_path = import_path,
                srcs = ctx.files.srcs,
            ),
            deps = depset(
                direct = [dep[RosMsgInfo].info for dep in ctx.attr.deps],
                transitive = [dep[RosMsgInfo].deps for dep in ctx.attr.deps],
            ),
        ),
    ]

ros_msg_library = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".msg", ".srv"],
            mandatory = True,
        ),
        "deps": attr.label_list(providers = [RosMsgInfo]),
        "package_name": attr.string(),
    },
    implementation = _ros_msg_library_impl,
)

ros_srv_library = ros_msg_library

def _get_deps(attr_deps):
    return depset(
        direct = [src[RosMsgInfo].info for src in attr_deps],
        transitive = [src[RosMsgInfo].deps for src in attr_deps],
    ).to_list()

def _get_include_flags(deps):
    include_flags = []
    for dep in deps:
        include_flags += [
            "-I",
            "{}:{}".format(dep.package_name, dep.import_path),
        ]
    return include_flags

def _get_all_srcs(deps):
    srcs = []
    for dep in deps:
        srcs += dep.srcs
    return srcs

EXT_LEN = 4

def _cc_ros_msg_compile_impl(ctx):
    deps = _get_deps(ctx.attr.deps)
    include_flags = _get_include_flags(deps)
    all_srcs = _get_all_srcs(deps)

    all_headers = []
    for dep in deps:
        package_name = dep.package_name
        rel_output_dir = "{}/{}".format(ctx.label.name, package_name)
        for src in dep.srcs:
            src_stem = src.basename[:-EXT_LEN]
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
                package_name,
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

cc_ros_msg_compile = rule(
    implementation = _cc_ros_msg_compile_impl,
    output_to_genfiles = True,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            providers = [RosMsgInfo],
        ),
        "_gencpp": attr.label(
            default = Label("@ros_gencpp//:gencpp"),
            executable = True,
            cfg = "host",
        ),
    },
)

def cc_ros_msg_library(name, deps, visibility = None):
    name_gencpp = "{}_gencpp".format(name)
    cc_ros_msg_compile(
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

cc_ros_srv_library = cc_ros_msg_library

def _py_generate(ctx, include_flags, all_srcs, package_name, rel_output_dir, msgs):
    if not msgs:
        return []

    extension = msgs[0].extension
    if extension == "msg":
        generator = ctx.executable._genmsg_py
    else:
        generator = ctx.executable._gensrv_py

    py_msg_files = []
    for msg in msgs:
        msg_stem = msg.basename[:-EXT_LEN]
        py_file = ctx.actions.declare_file(
            "{}/{}/_{}.py".format(rel_output_dir, extension, msg_stem),
        )
        py_msg_files.append(py_file)

    args = [
        "-o",
        py_msg_files[0].dirname,
        "-p",
        package_name,
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
        package_name,
    ]

    ctx.actions.run(
        inputs = py_msg_files,
        outputs = [init_py],
        executable = generator,
        arguments = args,
    )

    return py_msg_files + [init_py]

def _py_ros_msg_compile_internal(ctx, deps):
    include_flags = _get_include_flags(deps)
    all_srcs = _get_all_srcs(deps)

    package_names_to_srcs = {}
    for dep in deps:
        if dep.package_name not in package_names_to_srcs:
            package_names_to_srcs[dep.package_name] = dep.srcs
        else:
            package_names_to_srcs[dep.package_name] = (
                package_names_to_srcs[dep.package_name] + dep.srcs
            )

    all_py_files = []
    for package_name, srcs in package_names_to_srcs.items():
        rel_output_dir = "{}/{}".format(ctx.label.name, package_name)

        msgs = [src for src in srcs if src.extension == "msg"]
        py_msg_files = _py_generate(
            ctx,
            include_flags,
            all_srcs,
            package_name,
            rel_output_dir,
            msgs,
        )
        all_py_files.extend(py_msg_files)

        srvs = [src for src in srcs if src.extension == "srv"]
        py_srv_files = _py_generate(
            ctx,
            include_flags,
            all_srcs,
            package_name,
            rel_output_dir,
            srvs,
        )
        all_py_files.extend(py_srv_files)

    return [
        DefaultInfo(files = depset(all_py_files)),
    ]

def _py_ros_msg_compile_impl(ctx):
    deps = _get_deps(ctx.attr.deps)
    return _py_ros_msg_compile_internal(ctx, deps)

_py_generator_deps = {
    "_genmsg_py": attr.label(
        default = Label("@ros_genpy//:genmsg_py"),
        executable = True,
        cfg = "host",
    ),
    "_gensrv_py": attr.label(
        default = Label("@ros_genpy//:gensrv_py"),
        executable = True,
        cfg = "host",
    ),
}

py_ros_msg_compile = rule(
    implementation = _py_ros_msg_compile_impl,
    output_to_genfiles = True,
    attrs = dicts.add(_py_generator_deps, {
        "deps": attr.label_list(
            mandatory = True,
            providers = [RosMsgInfo],
        ),
    }),
)

def py_ros_msg_library(name, deps, visibility = None):
    name_genpy = "{}_genpy".format(name)
    py_ros_msg_compile(
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

py_ros_srv_library = py_ros_msg_library

RosMsgCollectorAspectInfo = provider(
    "Collects ros_msg_library targets.",
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
        if type(dep) == "Target" and RosMsgCollectorAspectInfo in dep
    ]

_ROS_MSG_COLLECTOR_ASPECT_ATTRS = ["data", "deps", "hdrs", "srcs"]

def _ros_msg_collector_aspect_impl(target, ctx):
    direct_deps = []
    if ctx.rule.kind == "ros_msg_library":
        direct_deps = [target]

    transitive_deps = []
    for attr_name in _ROS_MSG_COLLECTOR_ASPECT_ATTRS:
        for dep in _collect_dependencies(ctx.rule.attr, attr_name):
            transitive_deps.append(dep[RosMsgCollectorAspectInfo].deps)

    return [
        RosMsgCollectorAspectInfo(
            deps = depset(
                direct = direct_deps,
                transitive = transitive_deps,
            ),
        ),
    ]

ros_msg_collector_aspect = aspect(
    implementation = _ros_msg_collector_aspect_impl,
    attr_aspects = _ROS_MSG_COLLECTOR_ASPECT_ATTRS,
)

def _py_ros_msg_collector_impl(ctx):
    msg_targets = depset(
        transitive = [
            dep[RosMsgCollectorAspectInfo].deps
            for dep in ctx.attr.deps
        ],
    ).to_list()
    deps = _get_deps(msg_targets)
    return _py_ros_msg_compile_internal(ctx, deps)

py_ros_msg_collector = rule(
    implementation = _py_ros_msg_collector_impl,
    attrs = dicts.add(_py_generator_deps, {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [ros_msg_collector_aspect],
            providers = [RosMsgCollectorAspectInfo],
        ),
    }),
)
