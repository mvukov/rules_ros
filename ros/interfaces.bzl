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

""" Implements functionality for code generation of ROS interfaces.
"""

# Inspired by code in https://github.com/nicolov/ros-bazel repo.

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")
load("//ros:utils.bzl", "get_stem")

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
    if ctx.attr.strip_end and "interface" in ros_package_name:
        ros_package_name = ros_package_name.split("_")
        ros_package_name = "_".join(ros_package_name[0:-1])
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
                transitive = [
                    dep[RosInterfaceInfo].deps
                    for dep in ctx.attr.deps
                ],
            ),
        ),
    ]

ros_interface_library = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".action", ".msg", ".srv"],
            mandatory = True,
            doc = " A list of interface files: actions, messages and services. ",
        ),
        "deps": attr.label_list(
            providers = [RosInterfaceInfo],
            doc = " A list of other `ros_interface_library` targets. ",
        ),
        "strip_end": attr.bool(
            doc = "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile"
        ),
        "_genaction": attr.label(
            default = Label("@ros_common_msgs//:genaction"),
            executable = True,
            cfg = "exec",
        ),
    },
    implementation = _ros_interface_library_impl,
    doc = """ Defines a rule for grouping ROS interfaces: actions, messages and services.

The target name defines the corresponding ROS package name.
For C++ generated code the target name defines the C++ namespace.
For Python generated code the target name defines the Python package name.
""",
)

def _get_include_flags(target, ctx):
    ros_package_name = target.label.name
    if ctx.attr.strip_end and "interface" in ros_package_name:
        ros_package_name = ros_package_name.split("_")
        ros_package_name = "_".join(ros_package_name[0:-1])
    srcs = target[RosInterfaceInfo].info.srcs
    deps = target[RosInterfaceInfo].deps

    include_flags = ["-I", "{}:{}".format(ros_package_name, srcs[0].dirname)]
    for dep in deps.to_list():
        include_flags += ["-I", "{}:{}".format(
            dep.ros_package_name,
            dep.srcs[0].dirname,
        )]
    return include_flags

def _get_all_srcs(target, ctx):
    srcs = target[RosInterfaceInfo].info.srcs
    deps = target[RosInterfaceInfo].deps

    return depset(
        direct = srcs,
        transitive = [depset(dep.srcs) for dep in deps.to_list()],
    )

def _cc_ros_generator_aspect_impl(target, ctx):
    include_flags = _get_include_flags(target, ctx)
    all_srcs = _get_all_srcs(target, ctx)

    ros_package_name = target.label.name

    if ctx.attr.strip_end and "interface" in ros_package_name:
        ros_package_name = ros_package_name.split("_")
        ros_package_name = "_".join(ros_package_name[0:-1])

    srcs = target[RosInterfaceInfo].info.srcs
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

        args = ctx.actions.args()
        args.add("-o", msg_header.dirname)
        args.add("-p", ros_package_name)
        args.add_all(include_flags)
        args.add(src)
        ctx.actions.run(
            inputs = all_srcs,
            outputs = msg_headers,
            executable = ctx.executable._gencpp,
            arguments = [args],
        )

    cc_include_dir = "/".join(srcs[0].dirname.split("/")[:-1])
    compilation_context = cc_common.create_compilation_context(
        headers = depset(all_headers),
        system_includes = depset([cc_include_dir]),
    )
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [
            CcInfo(compilation_context = compilation_context),
        ] + [
            dep[CcInfo]
            for dep in ctx.rule.attr.deps
        ],
    )
    return [cc_info]

cc_ros_generator_aspect = aspect(
    implementation = _cc_ros_generator_aspect_impl,
    attr_aspects = ["deps" , "strip_end"],
    attrs = {
        "_gencpp": attr.label(
            default = Label("@ros_gencpp//:gencpp"),
            executable = True,
            cfg = "exec",
        ),
        "strip_end": attr.bool(
            doc = "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile"
        ),
    },
    provides = [CcInfo],
)

def _cc_ros_generator_impl(ctx):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [cc_info]

cc_ros_generator = rule(
    implementation = _cc_ros_generator_impl,
    output_to_genfiles = True,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [cc_ros_generator_aspect],
            providers = [RosInterfaceInfo],
        ),
        "strip_end": attr.bool(
            doc = "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile"
        ),
    },
)

def cc_ros_interface_library(name, deps, strip_end = False, **kwargs):
    """ Defines a C++ ROS interface library.

    Args:
        name: A unique target name.
        deps: A list of deps (list of `ros_interface_library` targets).
        strip_end: "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    name_gencpp = "{}_gencpp".format(name)
    cc_ros_generator(
        name = name_gencpp,
        deps = deps,
        strip_end = strip_end,
    )
    cc_library(
        name = name,
        deps = [
            name_gencpp,
            "@roscpp_core//:roscpp_core",
            "@ros_std_msgs//:cc_std_msgs_headers",
        ],
        **kwargs
    )

def _py_generate(
        ctx,
        include_flags,
        all_srcs,
        ros_package_name,
        rel_output_dir,
        msgs):
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

    args = ctx.actions.args()
    args.add("-o", py_msg_files[0].dirname)
    args.add("-p", ros_package_name)
    args.add_all(include_flags)
    args.add_all(msgs)
    ctx.actions.run(
        inputs = all_srcs,
        outputs = py_msg_files,
        executable = generator,
        arguments = [args],
    )

    init_py = ctx.actions.declare_file(
        "{}/{}/__init__.py".format(rel_output_dir, extension),
    )

    args = ctx.actions.args()
    args.add("--initpy")
    args.add("-o", py_msg_files[0].dirname)
    args.add("-p", ros_package_name)
    ctx.actions.run(
        inputs = py_msg_files,
        outputs = [init_py],
        executable = generator,
        arguments = [args],
    )

    return py_msg_files + [init_py]

PyRosGeneratorAspectInfo = provider(
    "Accumulates Python ROS interfaces.",
    fields = [
        "transitive_sources",
        "imports",
    ],
)

def _get_list_attr(rule_attr, attr_name):
    if not hasattr(rule_attr, attr_name):
        return []
    candidate = getattr(rule_attr, attr_name)
    if type(candidate) != "list":
        fail("Expected a list for attribute `{}`!".format(attr_name))
    return candidate

def _collect_py_ros_generator_deps(rule_attr, attr_name):
    return [
        dep
        for dep in _get_list_attr(rule_attr, attr_name)
        if type(dep) == "Target" and PyRosGeneratorAspectInfo in dep
    ]

def _merge_py_ros_generator_aspect_infos(py_infos):
    return PyRosGeneratorAspectInfo(
        transitive_sources = depset(
            transitive = [info.transitive_sources for info in py_infos],
        ),
        imports = depset(transitive = [info.imports for info in py_infos]),
    )

_PY_ROS_GENERATOR_ATTR_ASPECTS = ["data", "deps"]

def _py_ros_generator_aspect_impl(target, ctx):
    py_infos = []
    if ctx.rule.kind == "ros_interface_library":
        include_flags = _get_include_flags(target, ctx)
        all_srcs = _get_all_srcs(target, ctx)

        ros_package_name = target.label.name
        srcs = target[RosInterfaceInfo].info.srcs
        rel_output_dir = ros_package_name
        all_py_files = []

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

        the_file = all_py_files[0]
        relative_path_parts = paths.relativize(
            the_file.dirname,
            the_file.root.path,
        ).split("/")
        if relative_path_parts[0] == "external":
            py_import_path = paths.join(*relative_path_parts[1:-2])
        else:
            py_import_path = paths.join(
                ctx.workspace_name,
                *relative_path_parts[0:-2]
            )

        py_infos = [PyRosGeneratorAspectInfo(
            transitive_sources = depset(all_py_files),
            imports = depset([py_import_path]),
        )]

    for attr_name in _PY_ROS_GENERATOR_ATTR_ASPECTS:
        for dep in _collect_py_ros_generator_deps(ctx.rule.attr, attr_name):
            py_infos.append(dep[PyRosGeneratorAspectInfo])

    merged_py_info = _merge_py_ros_generator_aspect_infos(py_infos)
    return [merged_py_info]

py_ros_generator_aspect = aspect(
    implementation = _py_ros_generator_aspect_impl,
    attr_aspects = _PY_ROS_GENERATOR_ATTR_ASPECTS,
    attrs = {
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
        "strip_end": attr.bool(
            doc = "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile"
        ),
    },
    provides = [PyRosGeneratorAspectInfo],
)

def _py_ros_generator_impl(ctx):
    py_info = _merge_py_ros_generator_aspect_infos([
        dep[PyRosGeneratorAspectInfo]
        for dep in ctx.attr.deps
    ])
    return [
        DefaultInfo(runfiles = ctx.runfiles(
            transitive_files = py_info.transitive_sources,
        )),
        PyInfo(
            transitive_sources = py_info.transitive_sources,
            imports = py_info.imports,
        ),
    ]

py_ros_generator = rule(
    implementation = _py_ros_generator_impl,
    output_to_genfiles = True,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [py_ros_generator_aspect],
            providers = [RosInterfaceInfo],
        ),
        "strip_end": attr.bool(
            doc = "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile"
        ),
    },
)

def py_ros_interface_library(name, deps, **kwargs):
    """ Defines a Python ROS interface library.

    Args:
        name: A unique target name.
        deps: A list of deps (list of `ros_interface_library` targets).
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    name_genpy = "{}_genpy".format(name)
    py_ros_generator(
        name = name_genpy,
        deps = deps,
    )
    py_library(
        name = name,
        deps = [name_genpy, "@ros_genpy//:genpy"],
        **kwargs
    )

py_ros_interface_collector = rule(
    implementation = _py_ros_generator_impl,
    output_to_genfiles = True,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [py_ros_generator_aspect],
        ),
        "strip_end": attr.bool(
            doc = "An override to override the ros pkg name. Use this to avoid changing import paths. Warning: Fragile"
        ),
    },
)
