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
#
# Revision $Id$
"""
General routines and representations for loading roslaunch model.
"""
# pylint: disable=broad-except,unnecessary-pass,raise-missing-from,unidiomatic-typecheck,
# pylint: disable=line-too-long
# pylint: disable=consider-using-f-string,unspecified-encoding
import errno
import os
import shlex
import stat
import subprocess
import sys
from copy import deepcopy
from xmlrpc.client import Binary

import yaml
from rosgraph.names import canonicalize_name
from rosgraph.names import is_legal_name
from rosgraph.names import load_mappings
from rosgraph.names import make_global_ns
from rosgraph.names import ns_join
from rosgraph.names import PRIV_NAME

from third_party.ros.roslaunch.core import Param
from third_party.ros.roslaunch.core import PHASE_SETUP
from third_party.ros.roslaunch.core import RLException
from third_party.ros.roslaunch.core import RosbinExecutable


class LoadException(RLException):
    """Error loading data as specified (e.g. cannot find included files, etc...)"""
    pass


#TODO: lists, maps(?)
def convert_value(value, type_):
    """
    Convert a value from a string representation into the specified
    type

    @param value: string representation of value
    @type  value: str
    @param type_: int, double, string, bool, or auto
    @type  type_: str
    @raise ValueError: if parameters are invalid
    """
    type_ = type_.lower()
    # currently don't support XML-RPC date, dateTime, maps, or list
    # types
    if type_ == 'auto':
        #attempt numeric conversion
        try:
            if '.' in value:
                return float(value)
            # Python 3 ignores/drops underscores in number literals (due to PEP515).
            # Here, we want to handle literals with underscores as plain strings.
            if '_' not in value:
                return int(value)
        except ValueError:
            pass
        #bool
        lval = value.lower()
        if lval == 'true' or lval == 'false':
            return convert_value(value, 'bool')
        #string
        return value
    elif type_ == 'str' or type_ == 'string':
        return value
    elif type_ == 'int':
        return int(value)
    elif type_ == 'double':
        return float(value)
    elif type_ == 'bool' or type_ == 'boolean':
        value = value.lower().strip()
        if value == 'true' or value == '1':
            return True
        elif value == 'false' or value == '0':
            return False
        raise ValueError("%s is not a '%s' type" % (value, type_))
    elif type_ == 'yaml':
        try:
            return yaml.safe_load(value)
        except yaml.parser.ParserError as e:
            raise ValueError(e)
    else:
        raise ValueError("Unknown type '%s'" % type_)


def process_include_args(context):
    """
    Processes arg declarations in context and makes sure that they are
    properly declared for passing into an included file. Also will
    correctly setup the context for passing to the included file.
    """

    # make sure all arguments have values. arg_names and resolve_dict
    # are cleared when the context was initially created.
    arg_dict = context.include_resolve_dict.get('arg', {})
    for arg in context.arg_names:
        if arg not in arg_dict:
            raise LoadException('include args must have declared values')

    # save args that were passed so we can check for unused args in post-processing
    context.args_passed = list(arg_dict.keys())
    # clear arg declarations so included file can re-declare
    context.arg_names = []

    # swap in new resolve dict for passing
    context.resolve_dict = context.include_resolve_dict
    context.include_resolve_dict = None


def post_process_include_args(context):
    bad = [a for a in context.args_passed if a not in context.arg_names]
    if bad:
        raise LoadException('unused args [%s] for include of [%s]' %
                            (', '.join(bad), context.filename))


def load_sysargs_into_context(context, argv):
    """
    Load in ROS remapping arguments as arg assignments for context.

    @param context: context to load into. context's resolve_dict for 'arg' will be reinitialized with values.
    @type  context: L{LoaderContext{
    @param argv: command-line arguments
    @type  argv: [str]
    """
    # we use same command-line spec as ROS nodes
    mappings = load_mappings(argv)
    context.resolve_dict['arg'] = mappings


class LoaderContext(object):
    """
    Container for storing current loader context (e.g. namespace,
    local parameter state, remapping state).
    """

    def __init__(self, ns, filename, parent=None, params=None, env_args=None, \
                     resolve_dict=None, include_resolve_dict=None, arg_names=None):
        """
        @param ns: namespace
        @type  ns: str
        @param filename: name of file this is being loaded from
        @type  filename: str
        @param resolve_dict: (optional) resolution dictionary for substitution args
        @type  resolve_dict: dict
        @param include_resolve_dict: special resolution dictionary for
        <include> tags. Must be None if this is not an <include>
        context.
        @type include_resolve_dict: dict
        @param arg_names: name of args that have been declared in this context
        @type  arg_names: [str]
        """
        self.parent = parent
        self.ns = make_global_ns(ns or '/')
        self._remap_args = []
        self.params = params or []
        self.env_args = env_args or []
        self.filename = filename
        # for substitution args
        self.resolve_dict = resolve_dict or {}
        # arg names. Args that have been declared in this context
        self.arg_names = arg_names or []
        # special scoped resolve dict for processing in <include> tag
        self.include_resolve_dict = include_resolve_dict or None
        # when this context was created via include, was pass_all_args set?
        self.pass_all_args = False

    def add_param(self, p):
        """
        Add a ~param to the context. ~params are evaluated by any node
        declarations that occur later in the same context.

        @param p: parameter
        @type  p: L{Param}
        """

        # override any params already set
        matches = [m for m in self.params if m.key == p.key]
        for m in matches:
            self.params.remove(m)
        self.params.append(p)

    def add_remap(self, remap):
        """
        Add a new remap setting to the context. if a remap already
        exists with the same from key, it will be removed

        @param remap: remap setting
        @type  remap: (str, str)
        """
        remap = [canonicalize_name(x) for x in remap]
        if not remap[0] or not remap[1]:
            raise RLException('remap from/to attributes cannot be empty')
        if not is_legal_name(remap[0]):
            raise RLException('remap from [%s] is not a valid ROS name' %
                              remap[0])
        if not is_legal_name(remap[1]):
            raise RLException('remap to [%s] is not a valid ROS name' %
                              remap[1])

        matches = [r for r in self._remap_args if r[0] == remap[0]]
        for m in matches:
            self._remap_args.remove(m)
        self._remap_args.append(remap)

    def add_arg(self, name, default=None, value=None, doc=None):
        """
        Add 'arg' to existing context. Args are only valid for their immediate context.
        """
        if name in self.arg_names:
            # Ignore the duplication if pass_all_args was set
            if not self.pass_all_args:
                raise LoadException("arg '%s' has already been declared" % name)
        else:
            self.arg_names.append(name)

        resolve_dict = self.resolve_dict if self.include_resolve_dict is None else self.include_resolve_dict

        if 'arg' not in resolve_dict:
            resolve_dict['arg'] = {}
        arg_dict = resolve_dict['arg']

        # args can only be declared once. they must also have one and
        # only value at the time that they are declared.
        if value is not None:
            # value is set, error if declared in our arg dict as args
            # with set values are constant/grounded.
            # But don't error if pass_all_args was used to include this
            # context; rather just override the passed-in value.
            if name in arg_dict and not self.pass_all_args:
                raise LoadException(
                    "cannot override arg '%s', which has already been set" %
                    name)
            arg_dict[name] = value
        elif default is not None:
            # assign value if not in context
            if name not in arg_dict:
                arg_dict[name] = default
        else:
            # no value or default: appending to arg_names is all we
            # need to do as it declares the existence of the arg.
            pass

        # add arg documentation string dict if it doesn't exist yet and if it can be used
        if 'arg_doc' not in resolve_dict:
            resolve_dict['arg_doc'] = {}
        arg_doc_dict = resolve_dict['arg_doc']

        if not value:
            # store the documentation for this argument
            arg_doc_dict[name] = (doc, default)

    def remap_args(self):
        """
        @return: copy of the current remap arguments
        @rtype: [(str, str)]
        """
        if self.parent:
            args = []
            # filter out any parent remap args that have the same from key
            for pr in self.parent.remap_args():
                if not [r for r in self._remap_args if r[0] == pr[0]]:
                    args.append(pr)
            args.extend(self._remap_args)
            return args
        return self._remap_args[:]

    def include_child(self, ns, filename):
        """
        Create child namespace based on include inheritance rules
        @param ns: sub-namespace of child context, or None if the
           child context shares the same namespace
        @type  ns: str
        @param filename: name of include file
        @type  filename: str
        @return: A child xml context that inherits from this context
        @rtype: L{LoaderContext}jj
        """
        ctx = self.child(ns)
        # arg declarations are reset across include boundaries
        ctx.arg_names = []
        ctx.filename = filename
        # keep the resolve_dict for now, we will do all new assignments into include_resolve_dict
        ctx.include_resolve_dict = {}
        #del ctx.resolve_dict['arg']
        return ctx

    def child(self, ns):
        """
        @param ns: sub-namespace of child context, or None if the
           child context shares the same namespace
        @type  ns: str
        @return: A child xml context that inherits from this context
        @rtype: L{LoaderContext}
        """
        if ns:
            if ns[0] == '/':  # global (discouraged)
                child_ns = ns
            elif ns == PRIV_NAME:  # ~name
                # private names can only be scoped privately or globally
                child_ns = PRIV_NAME
            else:
                child_ns = ns_join(self.ns, ns)
        else:
            child_ns = self.ns
        return LoaderContext(child_ns,
                             self.filename,
                             parent=self,
                             params=self.params,
                             env_args=self.env_args[:],
                             resolve_dict=deepcopy(self.resolve_dict),
                             arg_names=self.arg_names[:],
                             include_resolve_dict=self.include_resolve_dict)


#TODO: in-progress refactorization. I'm slowly separating out
#non-XML-specific logic from xmlloader and moving into Loader. Soon
#this will mean that it will be easier to write coverage tests for
#lower-level logic.


class Loader(object):
    """
    Lower-level library for loading ROS launch model. It provides an
    abstraction between the representation (e.g. XML) and the
    validation of the property values.
    """

    def add_param(self, ros_config, param_name, param_value, verbose=True):
        """
        Add L{Param} instances to launch config. Dictionary values are
        unrolled into individual parameters.

        @param ros_config: launch configuration
        @type  ros_config: L{ROSLaunchConfig}
        @param param_name: name of parameter namespace to load values
            into. If param_name is '/', param_value must be a dictionary
        @type  param_name: str
        @param param_value: value to assign to param_name. If
            param_value is a dictionary, it's values will be unrolled
            into individual parameters.
        @type  param_value: str
        @raise ValueError: if parameters cannot be processed into valid Params
        """

        # shouldn't ever happen
        if not param_name:
            raise ValueError('no parameter name specified')

        if param_name == '/' and type(param_value) != dict:
            raise ValueError(
                "Cannot load non-dictionary types into global namespace '/'")

        if type(param_value) == dict:
            # unroll params
            for k, v in param_value.items():
                self.add_param(ros_config,
                               ns_join(param_name, k),
                               v,
                               verbose=verbose)
        else:
            ros_config.add_param(Param(param_name, param_value),
                                 verbose=verbose)

    def load_rosparam(self,
                      context,
                      ros_config,
                      cmd,
                      param,
                      file_,
                      text,
                      verbose=True,
                      subst_function=None):
        """
        Load rosparam setting

        @param context: Loader context
        @type  context: L{LoaderContext}
        @param ros_config: launch configuration
        @type  ros_config: L{ROSLaunchConfig}
        @param cmd: 'load', 'dump', or 'delete'
        @type  cmd: str
        @param file_: filename for rosparam to use or None
        @type  file_: str
        @param text: text for rosparam to load. Ignored if file_ is set.
        @type  text: str
        @raise ValueError: if parameters cannot be processed into valid rosparam setting
        """
        if cmd not in ('load', 'dump', 'delete'):
            raise ValueError("command must be 'load', 'dump', or 'delete'")
        if file_ is not None:
            if cmd == 'load' and not os.path.isfile(file_):
                raise ValueError('file does not exist [%s]' % file_)
            if cmd == 'delete':
                raise ValueError(
                    "'file' attribute is invalid with 'delete' command.")

        full_param = ns_join(context.ns, param) if param else context.ns

        if cmd == 'dump':
            ros_config.add_executable(
                RosbinExecutable('rosparam', (cmd, file_, full_param),
                                 PHASE_SETUP))
        elif cmd == 'delete':
            ros_config.add_executable(
                RosbinExecutable('rosparam', (cmd, full_param), PHASE_SETUP))
        elif cmd == 'load':
            # load YAML text
            if file_:
                with open(file_, 'r') as f:
                    text = f.read()

            if subst_function is not None:
                text = subst_function(text)
            try:
                data = yaml.safe_load(text)
                # #3162: if there is no YAML, load() will return an
                # empty string.  We want an empty dictionary instead
                # for our representation of empty.
                if data is None:
                    data = {}
            except yaml.MarkedYAMLError as e:
                if not file_:
                    raise ValueError(
                        'Error within YAML block:\n\t%s\n\nYAML is:\n%s' %
                        (str(e), text))
                else:
                    raise ValueError('file %s contains invalid YAML:\n%s' %
                                     (file_, str(e)))
            except Exception as e:
                if not file_:
                    raise ValueError('invalid YAML: %s\n\nYAML is:\n%s' %
                                     (str(e), text))
                else:
                    raise ValueError('file %s contains invalid YAML:\n%s' %
                                     (file_, str(e)))

            # 'param' attribute is required for non-dictionary types
            if not param and type(data) != dict:
                raise ValueError(
                    "'param' attribute must be set for non-dictionary values")

            self.add_param(ros_config, full_param, data, verbose=verbose)

        else:
            raise ValueError('unknown command %s' % cmd)

    def load_env(self, context, _ros_config, name, value):
        """
        Load environment variable setting

        @param context: Loader context
        @type  context: L{LoaderContext}
        @param ros_config: launch configuration
        @type  ros_config: L{ROSLaunchConfig}
        @param name: environment variable name
        @type  name: str
        @param value: environment variable value
        @type  value: str
        """
        if not name:
            raise ValueError("'name' attribute must be non-empty")
        context.env_args.append((name, value))

    def param_value(self, verbose, name, ptype, value, textfile, binfile,
                    command):
        """
        Parse text representation of param spec into Python value
        @param name: param name, for error message use only
        @type  name: str
        @param verbose: print verbose output
        @type  verbose: bool
        @param textfile: name of text file to load from, or None
        @type  textfile: str
        @param binfile: name of binary file to load from, or None
        @type  binfile: str
        @param command: command to execute for parameter value, or None
        @type  command: str
        @raise ValueError: if parameters are invalid
        """
        if value is not None:
            return convert_value(value.strip(), ptype)
        elif textfile is not None:
            with open(textfile, 'r') as f:
                return convert_value(f.read(), ptype)
        elif binfile is not None:
            with open(binfile, 'rb') as f:
                return Binary(f.read())
        elif command is not None:
            try:
                if type(command) == unicode:
                    command = command.encode(
                        'utf-8'
                    )  #attempt to force to string for shlex/subprocess
            except NameError:
                pass
            if verbose:
                print('... executing command param [%s]' % command)
            try:
                if os.name != 'nt':
                    command = shlex.split(command)
                else:
                    cl = shlex.split(
                        command, posix=False)  # use non-posix method on Windows

                    # On Linux, single quotes are commonly used to enclose a path to escape spaces.
                    # However, on Windows, the single quotes are treated as part of the arguments.
                    # Special handling is required to remove the extra single quotes.
                    if "'" in command:
                        cl = [
                            token[1:-1] if token.startswith("'") and
                            token.endswith("'") else token for token in cl
                        ]
                    command = cl

                    # Python scripts in ROS tend to omit .py extension since they could become executable with shebang line
                    # special handle the use of Python scripts in Windows environment:
                    # 1. search for a wrapper executable (of the same name) under the same directory with stat.S_IXUSR flag
                    # 2. if no wrapper is present, prepend command with 'python' executable
                    if os.path.isabs(cl[0]):
                        # trying to launch an executable from a specific location(package), e.g. xacro
                        rx_flag = stat.S_IRUSR | stat.S_IXUSR
                        if not os.path.exists(cl[0]) or os.stat(
                                cl[0]).st_mode & rx_flag != rx_flag:
                            d = os.path.dirname(cl[0])
                            files_of_same_name = [
                                os.path.join(d, f)
                                for f in os.listdir(d)
                                if os.path.splitext(f)[0].lower() == os.path.
                                splitext(os.path.basename(cl[0]))[0].lower()
                            ] if os.path.exists(d) else []
                            executable_command = None
                            for f in files_of_same_name:
                                if os.stat(f).st_mode & rx_flag == rx_flag:
                                    # found an executable wrapper of the desired Python script
                                    executable_command = f

                            if not executable_command:
                                for f in files_of_same_name:
                                    mode = os.stat(f).st_mode
                                    if (mode & stat.S_IRUSR == stat.S_IRUSR
                                       ) and (mode & stat.S_IXUSR
                                              != stat.S_IXUSR):
                                        # when there is read permission but not execute permission, this is typically a Python script (in ROS)
                                        if os.path.splitext(f)[1].lower() in [
                                                '.py', ''
                                        ]:
                                            executable_command = ' '.join(
                                                [sys.executable, f])
                            if executable_command:
                                command[0] = executable_command
                p = subprocess.Popen(command, stdout=subprocess.PIPE)
                c_value = p.communicate()[0]
                if not isinstance(c_value, str):
                    c_value = c_value.decode('utf-8')
                if p.returncode != 0:
                    raise ValueError(
                        'Cannot load command parameter [%s]: command [%s] returned with code [%s]'
                        % (name, command, p.returncode))
            except OSError as e:
                if e.errno == errno.ENOENT:
                    raise ValueError(
                        'Cannot load command parameter [%s]: no such command [%s]'
                        % (name, command))
                raise
            if c_value is None:
                raise ValueError(
                    'parameter: unable to get output of command [%s]' % command)
            return convert_value(c_value, ptype)
        else:  #_param_tag prevalidates, so this should not be reachable
            raise ValueError('unable to determine parameter value')
