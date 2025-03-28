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
roslaunch.parent providees the L{ROSLaunchParent} implementation,
which represents the main 'parent' roslaunch process.

ROSLaunch has a client/server architecture for running remote
processes. When a user runs roslaunch, this creates a "parent"
roslaunch process, which is responsible for managing local
processes. This parent process will also start "child" processes on
remote machines. The parent can then invoke methods on this child
process to launch remote processes, and the child can invoke methods
on the parent to provide feedback.
"""
# pylint: disable=bare-except
# pylint: disable=line-too-long
# pylint: disable=consider-using-f-string
import logging

from rosmaster.master_api import NUM_WORKERS

from third_party.ros.roslaunch import config
from third_party.ros.roslaunch import launch
from third_party.ros.roslaunch import pmon
from third_party.ros.roslaunch import server
from third_party.ros.roslaunch.core import printerrlog
from third_party.ros.roslaunch.core import printlog_bold
from third_party.ros.roslaunch.core import RLException
from third_party.ros.roslaunch.nodeprocess import DEFAULT_TIMEOUT_SIGINT
from third_party.ros.roslaunch.nodeprocess import DEFAULT_TIMEOUT_SIGTERM


class ROSLaunchParent(object):
    """
    ROSLaunchParent represents the main 'parent' roslaunch process. It
    is responsible for loading the launch files, assigning machines,
    and then starting up any remote processes. The __main__ method
    delegates most of runtime to ROSLaunchParent.

    This must be called from the Python Main thread due to signal registration.
    """

    def __init__(self,
                 run_id,
                 roslaunch_files,
                 is_core=False,
                 port=None,
                 local_only=False,
                 process_listeners=None,
                 verbose=False,
                 force_screen=False,
                 force_log=False,
                 is_rostest=False,
                 roslaunch_strs=None,
                 num_workers=NUM_WORKERS,
                 timeout=None,
                 master_logger_level=False,
                 show_summary=True,
                 force_required=False,
                 sigint_timeout=DEFAULT_TIMEOUT_SIGINT,
                 sigterm_timeout=DEFAULT_TIMEOUT_SIGTERM):
        """
        @param run_id: UUID of roslaunch session
        @type  run_id: str
        @param roslaunch_files: list of launch configuration
            files to load
        @type  roslaunch_files: [str]
        @param is_core bool: if True, this launch is a roscore
            instance. This affects the error behavior if a master is
            already running (i.e. it fails).
        @type  is_core: bool
        @param process_listeners: (optional) list of process listeners
            to register with process monitor once launch is running
        @type  process_listeners: [L{pmon.ProcessListener}]
        @param port: (optional) override master port number from what is specified in the master URI.
        @type  port: int
        @param verbose: (optional) print verbose output
        @type  verbose: boolean
        @param show_summary: (optional) whether to show a summary or not
        @type  show_summary: boolean
        @param force_screen: (optional) force output of all nodes to screen
        @type  force_screen: boolean
        @param force_log: (optional) force output of all nodes to log
        @type  force_log: boolean
        @param is_rostest bool: if True, this launch is a rostest
            instance. This affects validation checks.
        @type  is_rostest: bool
        @param num_workers: If this is the core, the number of worker-threads to use.
        @type num_workers: int
        @param timeout: If this is the core, the socket-timeout to use.
        @type timeout: Float or None
        @throws RLException
        @param master_logger_level: Specify roscore's rosmaster.master logger level, use default if it is False.
        @type master_logger_level: str or False
        @param force_required: (optional) whether to make all nodes required
        @type force_required: boolean
        @param sigint_timeout: The SIGINT timeout used when killing nodes (in seconds).
        @type sigint_timeout: float
        @param sigterm_timeout: The SIGTERM timeout used when killing nodes if SIGINT does not stop the node (in seconds).
        @type sigterm_timeout: float
        @raise RLException: If sigint_timeout or sigterm_timeout are nonpositive.
        """
        if sigint_timeout <= 0:
            raise RLException(
                'sigint_timeout must be a positive number, received %f' %
                sigint_timeout)
        if sigterm_timeout <= 0:
            raise RLException(
                'sigterm_timeout must be a positive number, received %f' %
                sigterm_timeout)

        self.logger = logging.getLogger('roslaunch.parent')
        self.run_id = run_id
        self.process_listeners = process_listeners

        self.roslaunch_files = roslaunch_files
        self.roslaunch_strs = roslaunch_strs
        self.is_core = is_core
        self.is_rostest = is_rostest
        self.port = port
        self.local_only = local_only
        self.verbose = verbose
        self.show_summary = show_summary
        self.num_workers = num_workers
        self.timeout = timeout
        self.master_logger_level = master_logger_level
        self.sigint_timeout = sigint_timeout
        self.sigterm_timeout = sigterm_timeout

        # I don't think we should have to pass in so many options from
        # the outside into the roslaunch parent. One possibility is to
        # allow alternate config loaders to be passed in.
        self.force_screen = force_screen
        self.force_log = force_log
        self.force_required = force_required

        # flag to prevent multiple shutdown attempts
        self._shutting_down = False

        self.config = self.runner = self.server = self.pm = self.remote_runner = None
        self.exit_code = 0

    def _load_config(self):
        self.config = config.load_config_default(
            self.roslaunch_files,
            self.port,
            roslaunch_strs=self.roslaunch_strs,
            verbose=self.verbose)

        # #2370 (I really want to move this logic outside of parent)
        if self.force_screen:
            for n in self.config.nodes:
                n.output = 'screen'
        if self.force_log:
            for n in self.config.nodes:
                n.output = 'log'
        if self.force_required:
            for n in self.config.nodes:
                n.required = True
                if n.respawn and n.required:
                    raise ValueError(
                        'respawn and required cannot both be set to true')

    def _start_pm(self):
        """
        Start the process monitor
        """
        self.pm = pmon.start_process_monitor()

    def _init_runner(self):
        """
        Initialize the roslaunch runner
        """
        if self.config is None:
            raise RLException('config is not initialized')
        if self.pm is None:
            raise RLException('pm is not initialized')
        if self.server is None:
            raise RLException('server is not initialized')
        self.runner = launch.ROSLaunchRunner(
            self.run_id,
            self.config,
            server_uri=self.server.uri,
            pmon=self.pm,
            is_core=self.is_core,
            remote_runner=self.remote_runner,
            is_rostest=self.is_rostest,
            num_workers=self.num_workers,
            timeout=self.timeout,
            master_logger_level=self.master_logger_level,
            sigint_timeout=self.sigint_timeout,
            sigterm_timeout=self.sigterm_timeout)

        if self.show_summary:
            print(self.config.summary(local=self.remote_runner is None))
        if self.config:
            for err in self.config.config_errors:
                printerrlog('WARNING: %s' % err)

    def _start_server(self):
        """
        Initialize the roslaunch parent XML-RPC server
        """
        if self.config is None:
            raise RLException('config is not initialized')
        if self.pm is None:
            raise RLException('pm is not initialized')

        self.logger.info('starting parent XML-RPC server')
        self.server = server.ROSLaunchParentNode(self.config, self.pm)
        self.server.start()
        if not self.server.uri:
            raise RLException('server URI did not initialize')
        self.logger.info('... parent XML-RPC server started')

    def _init_remote(self):
        """
        Initialize the remote process runner, if required. Subroutine
        of _start_remote, separated out for easier testing
        """
        if self.config is None:
            raise RLException('config is not initialized')
        if self.pm is None:
            raise RLException('pm is not initialized')
        if self.server is None:
            raise RLException('server is not initialized')

        if not self.local_only and self.config.has_remote_nodes():
            raise RuntimeError('Running without support for remote ROS runner!')
        elif self.local_only:
            printlog_bold(
                'LOCAL\nlocal only launch specified, will not launch remote nodes\nLOCAL\n'
            )

    def _start_remote(self):
        """
        Initializes and runs the remote process runner, if required
        """
        if self.remote_runner is None:
            self._init_remote()

        if self.remote_runner is not None:
            # start_servers() runs the roslaunch children
            self.remote_runner.start_children()

    def _start_infrastructure(self):
        """
        load config, start XMLRPC servers and process monitor
        """
        if self.config is None:
            self._load_config()

        # Start the process monitor
        if self.pm is None:
            self._start_pm()

        # Startup the roslaunch runner and XMLRPC server.
        # Requires pm
        if self.server is None:
            self._start_server()

        # Startup the remote infrastructure.
        # Requires config, pm, and server
        self._start_remote()

    def _stop_infrastructure(self):
        """
        Tear down server and process monitor. Not multithread safe.
        """
        #TODO more explicit teardown of remote infrastructure

        # test and set flag so we don't shutdown multiple times
        if self._shutting_down:
            return
        self._shutting_down = True

        if self.server:
            try:
                self.server.shutdown('roslaunch parent complete')
            except:
                # don't let exceptions halt the rest of the shutdown
                pass
        if self.pm:
            self.pm.shutdown()
            self.pm.join()
            self.exit_code = self.pm.exit_code

    def start(self, auto_terminate=True):
        """
        Run the parent roslaunch.

        @param auto_terminate: stop process monitor once there are no
        more processes to monitor (default True). This defaults to
        True, which is the command-line behavior of roslaunch. Scripts
        may wish to set this to False if they wish to keep the
        roslauch infrastructure up regardless of processes being
        monitored.
        """
        self.logger.info('starting roslaunch parent run')

        # load config, start XMLRPC servers and process monitor
        try:
            self._start_infrastructure()
        except:
            # infrastructure did not initialize, do teardown on whatever did come up
            self._stop_infrastructure()
            raise

        # Initialize the actual runner.
        # Requires config, pm, server and remote_runner
        self._init_runner()

        # Start the launch
        self.runner.launch()

        # inform process monitor that we are done with process registration
        if auto_terminate:
            self.pm.registrations_complete()

        self.logger.info(
            '... roslaunch parent running, waiting for process exit')
        if self.process_listeners:
            for l in self.process_listeners:
                self.runner.pm.add_process_listener(l)
                # Add listeners to server as well, otherwise they won't be
                # called when a node on a remote machine dies.
                self.server.add_process_listener(l)

    def spin_once(self):
        """
        Run the parent roslaunch event loop once
        """
        if self.runner:
            self.runner.spin_once()

    def spin(self):
        """
        Run the parent roslaunch until exit
        """
        if not self.runner:
            raise RLException('parent not started yet')
        try:
            # Blocks until all processes dead/shutdown
            self.runner.spin()
        finally:
            self._stop_infrastructure()

    def shutdown(self):
        """
        Stop the parent roslaunch.
        """
        self._stop_infrastructure()
