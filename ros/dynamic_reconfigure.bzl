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

""" Implements code generation functionality for dynamic reconfiguration.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")
load("//ros:utils.bzl", "get_stem")

RosDynamicReconfigureInfo = provider(
    "Provides code generation info for dynamic reconfigure.",
    fields = [
        "info",
    ],
)

def _ros_dynamic_reconfigure_library_impl(ctx):
    src = ctx.files.src
    ros_package_name = ctx.label.name

    if ctx.attr.pkg_override:
        ros_package_name = ctx.attr.pkg_override
    
    return [
        DefaultInfo(files = depset(src)),
        RosDynamicReconfigureInfo(
            info = struct(
                ros_package_name = ros_package_name,
                src = src,
            ),
        ),
    ]

ros_dynamic_reconfigure_library = rule(
    attrs = {
        "src": attr.label(
            allow_files = [".cfg"],
            mandatory = True,
            doc = "A configuration file (.cfg).",
        ),
        "pkg_override": attr.string(
            doc = "An override for pkg name."
        )
    },
    implementation = _ros_dynamic_reconfigure_library_impl,
    doc = " Defines a rule for storing a dynamic_reconfigure configuration. ",
)

def _get_parent_dir(path):
    return "/".join(path.split("/")[:-1])

def _cc_ros_dynamic_reconfigure_generator_impl(ctx):
    info = ctx.attr.dep[RosDynamicReconfigureInfo].info
    cfg_file = info.src[0]
    ros_package_name = info.ros_package_name
    templates = ctx.attr._templates[DefaultInfo].files.to_list()
    dynconfpath = _get_parent_dir(templates[0].dirname)

    stem = get_stem(cfg_file)
    output = ctx.actions.declare_file(
        "{}/{}Config.h".format(ros_package_name, stem),
    )

    args = ctx.actions.args()
    args.add("--input", cfg_file)
    args.add("--ros_package_name", ros_package_name)
    args.add("--dynconfpath", dynconfpath)
    args.add("--cpp_gen_dir", output.dirname)

    ctx.actions.run(
        inputs = [cfg_file] + templates,
        outputs = [output],
        executable = ctx.executable._generator,
        arguments = [args],
    )

    cc_include_dir = _get_parent_dir(output.dirname)
    compilation_context = cc_common.create_compilation_context(
        headers = depset([output]),
        system_includes = depset([cc_include_dir]),
    )
    return [CcInfo(compilation_context = compilation_context)]

cc_ros_dynamic_reconfigure_generator = rule(
    implementation = _cc_ros_dynamic_reconfigure_generator_impl,
    output_to_genfiles = True,
    attrs = {
        "dep": attr.label(
            mandatory = True,
            providers = [RosDynamicReconfigureInfo],
        ),
        "_generator": attr.label(
            default = Label("@com_github_mvukov_rules_ros//ros:parameter_generator_app"),
            executable = True,
            cfg = "exec",
        ),
        "_templates": attr.label(
            default = Label("@ros_dynamic_reconfigure//:cc_templates"),
        ),
    },
)

def cc_ros_dynamic_reconfigure_library(name, dep, **kwargs):
    """ Defines a C++ dynamic reconfiguration library.

    Args:
        name: A unique target name.
        dep: A configuration file -- a `ros_dynamic_reconfigure_library` target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    generator_name = name + "_generator"
    cc_ros_dynamic_reconfigure_generator(
        name = generator_name,
        dep = dep,
    )
    cc_library(
        name = name,
        deps = [
            generator_name,
            "@boost//:any",
            "@boost//:smart_ptr",
            "@boost//:thread",
            "@ros_dynamic_reconfigure//:dynamic_reconfigure_lib",
        ],
        **kwargs
    )
