# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
Uncategorized utility routines for roslaunch.

This API should not be considered stable.
"""
# pylint: disable=bare-except,unnecessary-pass,invalid-name,global-statement
# pylint: disable=line-too-long
# pylint: disable=consider-using-f-string
import os
import platform
import sys
import time

import rosclean
import rosgraph
import rospkg

from third_party.ros.roslaunch import core
from third_party.ros.roslaunch import xmlloader
from third_party.ros.roslaunch.config import load_config_default


def check_log_disk_usage():
    """
    Check size of log directory. If high, print warning to user
    """
    pass
    try:
        d = rospkg.get_log_dir()
        core.printlog(
            'Checking log directory for disk usage. This may take a while.\nPress Ctrl-C to interrupt'
        )
        disk_usage = rosclean.get_disk_usage(d)
        # warn if over a gig
        if disk_usage > 1073741824:
            core.printerrlog(
                "WARNING: disk usage in log directory [%s] is over 1GB.\nIt's recommended that you use the 'rosclean' command."
                % d)
        else:
            core.printlog('Done checking log file disk usage. Usage is <1GB.')
    except:
        pass


def resolve_launch_arguments(args):
    """
    Resolve command-line args to roslaunch filenames.

    :returns: resolved filenames, ``[str]``
    """

    # strip remapping args for processing
    args = rosgraph.myargv(args)
    return list(set(args))


def wait_for_master():
    """
    Block until ROS Master is online

    :raise: :exc:`RuntimeError` If unexpected error occurs
    """
    m = core.Master()  # get a handle to the default master
    is_running = m.is_running()
    if not is_running:
        core.printlog(
            'roscore/master is not yet running, will wait for it to start')
    while not is_running:
        time.sleep(0.1)
        is_running = m.is_running()
    if is_running:
        core.printlog('master has started, initiating launch')
    else:
        raise RuntimeError('unknown error waiting for master to start')


_terminal_name = None


def _set_terminal(s):
    if platform.system() in ['FreeBSD', 'Linux', 'Darwin', 'Unix']:
        try:
            print('\033]2;%s\007' % (s))
        except:
            pass


def update_terminal_name(ros_master_uri):
    """
    append master URI to the terminal name
    """
    if _terminal_name:
        _set_terminal(_terminal_name + ' ' + ros_master_uri)


def change_terminal_name(args, is_core):
    """
    use echo (where available) to change the name of the terminal window
    """
    global _terminal_name
    _terminal_name = 'roscore' if is_core else ','.join(args)
    _set_terminal(_terminal_name)


def get_or_generate_uuid(options_runid, options_wait_for_master):
    """
    :param options_runid: run_id value from command-line or ``None``, ``str``
    :param options_wait_for_master: the wait_for_master command
      option. If this is True, it means that we must retrieve the
      value from the parameter server and need to avoid any race
      conditions with the roscore being initialized. ``bool``
    """

    # Three possible sources of the run_id:
    #
    #  - if we're a child process, we get it from options_runid
    #  - if there's already a roscore running, read from the param server
    #  - generate one if we're running the roscore
    if options_runid:
        return options_runid

    # #773: Generate a run_id to use if we launch a master
    # process.  If a master is already running, we'll get the
    # run_id from it instead
    param_server = rosgraph.Master('/roslaunch')
    val = None
    while val is None:
        try:
            val = param_server.getParam('/run_id')
        except:
            if not options_wait_for_master:
                val = core.generate_run_id()
    return val


def print_file_list(roslaunch_files):
    """
    :param roslaunch_files: list of launch files to load, ``str``

    :returns: list of files involved in processing roslaunch_files, including the files themselves.
    """
    try:
        loader = xmlloader.XmlLoader(resolve_anon=True)
        config = load_config_default(roslaunch_files,
                                     None,
                                     loader=loader,
                                     verbose=False,
                                     assign_machines=False)
        files = [os.path.abspath(x) for x in set(config.roslaunch_files)]
        print('\n'.join(files))
    except core.RLException as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)
