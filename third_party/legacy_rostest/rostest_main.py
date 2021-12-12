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

# pylint: disable=deprecated-module,consider-using-f-string,unspecified-encoding,logging-not-lazy
# pylint: disable=line-too-long

import os
import socket
import sys
import unittest
import logging
from optparse import OptionParser

import rosgraph.roslogging
import rospkg
import third_party.legacy_roslaunch as roslaunch
from third_party.legacy_roslaunch.pmon import pmon_shutdown
from third_party.legacy_rostest import runner
from third_party.legacy_rostest.rostestutil import (createXMLRunner,
                                                    printRostestSummary,
                                                    rostest_name_from_path)

_NAME = 'rostest'


def configure_logging():
    logfile_basename = 'rostest-%s-%s.log' % (socket.gethostname(), os.getpid())
    logfile_name = rosgraph.roslogging.configure_logging(
        'rostest', filename=logfile_basename)
    if logfile_name:
        print("... logging to %s" % logfile_name)
    return logfile_name


def write_bad_filename_failure(test_file, results_file, outname):
    # similar to rostest-check-results
    results_file_dir = os.path.dirname(results_file)
    if not os.path.isdir(results_file_dir):
        os.makedirs(results_file_dir)
    with open(results_file, 'w') as f:
        d = {'test': outname, 'test_file': test_file}
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="Results">
    <failure message="rostest file [%(test_file)s] does not exist" type=""/>
  </testcase>
</testsuite>""" % d)


def rostestmain():
    parser = OptionParser(usage="usage: %prog [options] <filename>", prog=_NAME)
    parser.add_option("-t",
                      "--text",
                      action="store_true",
                      dest="text_mode",
                      default=False,
                      help="Run with stdout output instead of XML output")
    parser.add_option("--results-filename",
                      metavar="RESULTS_FILENAME",
                      dest="results_filename",
                      default=None,
                      help="results_filename")
    parser.add_option(
        "-r",
        "--reuse-master",
        action="store_true",
        help=
        "Connect to an existing ROS master instead of spawning a new ROS master on a custom port"
    )
    parser.add_option(
        "-c",
        "--clear",
        action="store_true",
        help=
        "Clear all parameters when connecting to an existing ROS master (only works with --reuse-master)"
    )
    (options, args) = parser.parse_args()

    if options.clear and not options.reuse_master:
        print("The --clear option is only valid with --reuse-master",
              file=sys.stderr)
        sys.exit(1)

    try:
        args = roslaunch.rlutil.resolve_launch_arguments(args)
    except roslaunch.core.RLException as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    # make sure all loggers are configured properly
    logfile_name = configure_logging()
    logger = logging.getLogger('rostest')
    roslaunch.core.add_printlog_handler(logger.info)
    roslaunch.core.add_printerrlog_handler(logger.error)

    logger.info('rostest starting with options %s, args %s' % (options, args))
    if len(args) == 0:
        parser.error("You must supply a test file argument to rostest.")
    if len(args) != 1:
        parser.error("rostest only accepts a single test file")

    # compute some common names we'll be using to generate test names and files
    if options.results_filename:
        outname = options.results_filename
        if '.' in outname:
            outname = outname[:outname.rfind('.')]
    else:
        test_file = args[0]
        outname = rostest_name_from_path("", test_file)

    results_dir = rospkg.get_test_results_dir()
    results_file = os.path.join(results_dir, "results.xml")

    # #1140
    if not os.path.isfile(test_file):
        write_bad_filename_failure(test_file, results_file, outname)
        parser.error(
            "test file is invalid. Generated failure case result file in %s" %
            results_file)

    try:
        testCase = runner.createUnitTest(test_file, options.reuse_master,
                                         options.clear, results_dir)
        suite = unittest.TestLoader().loadTestsFromTestCase(testCase)

        if options.text_mode:
            runner.setTextMode(True)
            result = unittest.TextTestRunner(verbosity=2).run(suite)
        else:
            is_rostest = True
            xml_runner = createXMLRunner("",
                                         outname,
                                         results_file=results_file,
                                         is_rostest=is_rostest)
            result = xml_runner.run(suite)
    finally:
        # really make sure that all of our processes have been killed
        test_parents = runner.getRostestParents()
        for r in test_parents:
            logger.info("finally rostest parent tearDown [%s]", r)
            r.tearDown()
        del test_parents[:]
        logger.info("calling pmon_shutdown")
        pmon_shutdown()
        logger.info("... done calling pmon_shutdown")

    # print config errors after test has run so that we don't get caught up in .xml results
    config = runner.getConfig()
    if config:
        if config.config_errors:
            print("\n[ROSTEST WARNINGS]" + '-' * 62 + '\n', file=sys.stderr)
        for err in config.config_errors:
            print(" * %s" % err, file=sys.stderr)
        print('')

    # summary is worthless if textMode is on as we cannot scrape .xml results
    subtest_results = runner.getResults()
    if not options.text_mode:
        printRostestSummary(result, subtest_results)
    else:
        print(
            "WARNING: overall test result is not accurate when --text is enabled"
        )

    if logfile_name:
        print("rostest log file is in %s" % logfile_name)

    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
