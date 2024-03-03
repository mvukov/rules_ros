# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Utility module of roslaunch that extracts dependency information from
roslaunch files, including calculating missing package dependencies.
"""
# pylint: disable=unnecessary-pass,singleton-comparison,redefined-builtin,raise-missing-from,len-as-condition,dangerous-default-value
# pylint: disable=line-too-long
# pylint: disable=consider-using-f-string
import os
import sys
from xml.dom import Node as DomNode
from xml.dom.minidom import parse

import rospkg

from third_party.ros.roslaunch.loader import convert_value
from third_party.ros.roslaunch.loader import load_mappings
from third_party.ros.roslaunch.substitution_args import resolve_args

NAME = 'roslaunch-deps'


class RoslaunchDepsException(Exception):
    """
    Base exception of roslaunch.depends errors.
    """
    pass


class RoslaunchDeps(object):
    """
    Represents dependencies of a roslaunch file.
    """

    def __init__(self, nodes=None, includes=None, pkgs=None):
        if nodes == None:
            nodes = []
        if includes == None:
            includes = []
        if pkgs == None:
            pkgs = []
        self.nodes = nodes
        self.includes = includes
        self.pkgs = pkgs

    def __eq__(self, other):
        if not isinstance(other, RoslaunchDeps):
            return False
        return set(self.nodes) == set(other.nodes) and \
               set(self.includes) == set(other.includes) and \
               set(self.pkgs) == set(other.pkgs)

    def __repr__(self):
        return 'nodes: %s\nincludes: %s\npkgs: %s' % (str(
            self.nodes), str(self.includes), str(self.pkgs))

    def __str__(self):
        return 'nodes: %s\nincludes: %s\npkgs: %s' % (str(
            self.nodes), str(self.includes), str(self.pkgs))


def _get_arg_value(tag, context):
    name = tag.attributes['name'].value
    if 'value' in tag.attributes.keys():
        return resolve_args(tag.attributes['value'].value, context)
    elif name in context['arg']:
        return context['arg'][name]
    elif 'default' in tag.attributes.keys():
        return resolve_args(tag.attributes['default'].value, context)
    else:
        raise RoslaunchDepsException('No value for arg [%s]' % (name))


def _check_ifunless(tag, context):
    if 'if' in tag.attributes.keys():
        val = resolve_args(tag.attributes['if'].value, context)
        if not convert_value(val, 'bool'):
            return False
    elif 'unless' in tag.attributes.keys():
        val = resolve_args(tag.attributes['unless'].value, context)
        if convert_value(val, 'bool'):
            return False
    return True


def _parse_subcontext(tags, context):
    subcontext = {'arg': {}}

    if tags == None:
        return subcontext

    for tag in [t for t in tags if t.nodeType == DomNode.ELEMENT_NODE]:
        if tag.tagName == 'arg' and _check_ifunless(tag, context):
            subcontext['arg'][tag.attributes['name'].value] = _get_arg_value(
                tag, context)
    return subcontext


def _parse_launch(tags, launch_file, file_deps, verbose, context):
    context['filename'] = os.path.abspath(launch_file)
    dir_path = os.path.dirname(os.path.abspath(launch_file))
    launch_file_pkg = rospkg.get_package_name(dir_path)

    # process group, include, node, and test tags from launch file
    for tag in [t for t in tags if t.nodeType == DomNode.ELEMENT_NODE]:
        if not _check_ifunless(tag, context):
            continue

        if tag.tagName == 'group':

            #descend group tags as they can contain node tags
            _parse_launch(tag.childNodes, launch_file, file_deps, verbose,
                          context)

        elif tag.tagName == 'arg':
            context['arg'][tag.attributes['name'].value] = _get_arg_value(
                tag, context)

        elif tag.tagName == 'include':
            try:
                sub_launch_file = resolve_args(tag.attributes['file'].value,
                                               context)
            except KeyError as e:
                raise RoslaunchDepsException(
                    'Cannot load roslaunch <%s> tag: missing required attribute %s.\nXML is %s'
                    % (tag.tagName, str(e), tag.toxml()))

            # Check if an empty file is included, and skip if so.
            # This will allow a default-empty <include> inside a conditional to pass
            if sub_launch_file == '':
                if verbose:
                    print('Empty <include> in %s. Skipping <include> of %s' %
                          (launch_file, tag.attributes['file'].value))
                continue

            if verbose:
                print('processing included launch %s' % sub_launch_file)

            # determine package dependency for included file
            sub_pkg = rospkg.get_package_name(
                os.path.dirname(os.path.abspath(sub_launch_file)))
            if sub_pkg is None:
                print('ERROR: cannot determine package for [%s]' %
                      sub_launch_file,
                      file=sys.stderr)

            if sub_launch_file not in file_deps[launch_file].includes:
                file_deps[launch_file].includes.append(sub_launch_file)
            if launch_file_pkg != sub_pkg:
                file_deps[launch_file].pkgs.append(sub_pkg)

            # recurse
            file_deps[sub_launch_file] = RoslaunchDeps()
            try:
                dom = parse(sub_launch_file).getElementsByTagName('launch')
                if not len(dom):
                    print('ERROR: %s is not a valid roslaunch file' %
                          sub_launch_file,
                          file=sys.stderr)
                else:
                    launch_tag = dom[0]
                    sub_context = _parse_subcontext(tag.childNodes, context)
                    try:
                        if tag.attributes['pass_all_args']:
                            sub_context['arg'] = context['arg']
                            sub_context['arg'].update(
                                _parse_subcontext(tag.childNodes,
                                                  context)['arg'])
                    except KeyError:
                        pass
                    _parse_launch(launch_tag.childNodes, sub_launch_file,
                                  file_deps, verbose, sub_context)
            except IOError:
                raise RoslaunchDepsException(
                    "Cannot load roslaunch include '%s' in '%s'" %
                    (sub_launch_file, launch_file))

        elif tag.tagName in ['node', 'test']:
            try:
                pkg, type = [
                    resolve_args(tag.attributes[a].value, context)
                    for a in ['pkg', 'type']
                ]
            except KeyError as e:
                raise RoslaunchDepsException(
                    'Cannot load roslaunch <%s> tag: missing required attribute %s.\nXML is %s'
                    % (tag.tagName, str(e), tag.toxml()))
            if (pkg, type) not in file_deps[launch_file].nodes:
                file_deps[launch_file].nodes.append((pkg, type))
            # we actually want to include the package itself if that's referenced
            #if launch_file_pkg != pkg:
            if pkg not in file_deps[launch_file].pkgs:
                file_deps[launch_file].pkgs.append(pkg)


def parse_launch(launch_file, file_deps, verbose):
    if verbose:
        print('processing launch %s' % launch_file)

    try:
        dom = parse(launch_file).getElementsByTagName('launch')
    except:
        raise RoslaunchDepsException('invalid XML in %s' % (launch_file))
    if not len(dom):
        raise RoslaunchDepsException('invalid XML in %s' % (launch_file))

    file_deps[launch_file] = RoslaunchDeps()
    launch_tag = dom[0]
    context = {'arg': load_mappings(sys.argv)}
    _parse_launch(launch_tag.childNodes, launch_file, file_deps, verbose,
                  context)
