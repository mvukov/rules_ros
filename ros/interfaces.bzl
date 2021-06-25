# Copyright 2021 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Implements functionality for code generation of ROS interfaces.

Inspired by code in https://github.com/nicolov/ros-bazel repo.
"""

load("//ros:utils.bzl", "get_stem")
load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")

RosInterfaceInfo = provider(
    "Provides info for interface code generation.",
    fields = [
        "info",
        "deps",
    ],
)

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
                    "{}/{}".format(ros_package_name, t.format(stem)),
                )
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
                "{}/{}".format(ros_package_name, src.basename),
            )
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

def _cc_generator_aspect_impl(target, ctx):
    ros_package_name = target.label.name
    srcs = target[RosInterfaceInfo].info.srcs

    include_flags = ["-I", "{}:{}".format(ros_package_name, srcs[0].dirname)]
    for dep in ctx.rule.attr.deps:
        include_flags += ["-I", "{}:{}".format(
            dep.label.name,
            dep[RosInterfaceInfo].info.srcs[0].dirname,
        )]

    all_srcs = depset(
        direct = srcs,
        transitive = [
            depset(dep[RosInterfaceInfo].info.srcs)
            for dep in ctx.rule.attr.deps
        ],
    )

    all_headers = []
    for src in srcs:
        src_stem = get_stem(src)
        msg_header = ctx.actions.declare_file(
            "{}/{}.h".format(ros_package_name, src_stem),
        )
        msg_headers = [msg_header]

        if src.extension == "srv":
            msg_headers.append(ctx.actions.declare_file(
                "{}/{}Request.h".format(ros_package_name, src_stem),
            ))
            msg_headers.append(ctx.actions.declare_file(
                "{}/{}Response.h".format(ros_package_name, src_stem),
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

    cc_include_dir = "/".join(srcs[0].dirname.split("/")[:-1])
    compilation_context = cc_common.create_compilation_context(
        headers = depset(all_headers),
        system_includes = depset([cc_include_dir]),
    )
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [CcInfo(compilation_context = compilation_context)] + [
            dep[CcInfo]
            for dep in ctx.rule.attr.deps
        ],
    )
    return [cc_info]

cc_generator_aspect = aspect(
    implementation = _cc_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_gencpp": attr.label(
            default = Label("@ros_gencpp//:gencpp"),
            executable = True,
            cfg = "exec",
        ),
    },
    provides = [CcInfo],
)

def _cc_generator_impl(ctx):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [cc_info]

cc_generator = rule(
    implementation = _cc_generator_impl,
    output_to_genfiles = True,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [cc_generator_aspect],
            providers = [RosInterfaceInfo],
        ),
    },
)

def cc_ros_interface_library(name, deps, visibility = None):
    name_gencpp = "{}_gencpp".format(name)
    cc_generator(
        name = name_gencpp,
        deps = deps,
    )
    cc_library(
        name = name,
        deps = [
            name_gencpp,
            "@roscpp_core//:roscpp_core",
            "@ros_std_msgs//:cc_std_msgs_headers",
        ],
        visibility = visibility,
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
