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
Interface for using rostest from other Python code as well as running
Python unittests with additional reporting mechanisms.
"""

# pylint: disable=invalid-name,consider-using-f-string,unspecified-encoding, global-statement
# pylint: disable=line-too-long

import os
import sys
import unittest
import warnings
import xmlrpc.client as xmlrpcclient

import coverage
import rosgraph
import rospy
import rosunit

XML_OUTPUT_FLAG = '--gtest_output=xml:'  #use gtest-compatible flag

_GLOBAL_CALLER_ID = '/script'


#TODO: replace with rosgraph.masterapi
def get_master():
    """
    Get an XMLRPC handle to the Master. It is recommended to use the
    `rosgraph.masterapi` library instead, as it provides many
    conveniences.

    @return: XML-RPC proxy to ROS master
    @rtype: xmlrpclib.ServerProxy
    """
    uri = rosgraph.get_master_uri()
    return xmlrpcclient.ServerProxy(uri)


def is_subscriber(topic, subscriber_id):
    """
    Check whether or not master think subscriber_id subscribes to topic

    :returns: ``True`` if still register as a subscriber, ``bool``
    :raises: IOError If communication with master fails
    """
    m = get_master()
    code, msg, state = m.getSystemState(_GLOBAL_CALLER_ID)
    if code != 1:
        raise IOError("Unable to retrieve master state: %s" % msg)
    _, subscribers, _ = state
    for t, l in subscribers:
        if t == topic:
            return subscriber_id in l
    return False


def is_publisher(topic, publisher_id):
    """
    Predicate to check whether or not master think publisher_id
    publishes topic
    :returns: ``True`` if still register as a publisher, ``bool``
    :raises: IOError If communication with master fails
    """
    m = get_master()
    code, msg, state = m.getSystemState(_GLOBAL_CALLER_ID)
    if code != 1:
        raise IOError("Unable to retrieve master state: %s" % msg)
    pubs, _, _ = state
    for t, l in pubs:
        if t == topic:
            return publisher_id in l
    return False


def rosrun(package, test_name, test, sysargs=None):
    """
    Run a rostest/unittest-based integration test.

    @param package: name of package that test is in
    @type  package: str
    @param test_name: name of test that is being run
    @type  test_name: str
    @param test: a test case instance or a name resolving to a test case or suite
    @type  test: unittest.TestCase, or string
    @param sysargs: command-line args. If not specified, this defaults to sys.argv. rostest
      will look for the --text and --gtest_output parameters
    @type  sysargs: list
    """
    if sysargs is None:
        sysargs = sys.argv

    #parse sysargs
    result_file = None
    for arg in sysargs:
        if arg.startswith(XML_OUTPUT_FLAG):
            result_file = arg[len(XML_OUTPUT_FLAG):]
    text_mode = '--text' in sysargs
    # coverage_mode = '--cov' in sysargs
    # if coverage_mode:
    #     _start_coverage([package])

    suite = None
    if isinstance(test, str):
        suite = unittest.TestLoader().loadTestsFromName(test)
    else:
        # some callers pass a TestCase type (instead of an instance)
        suite = unittest.TestLoader().loadTestsFromTestCase(test)

    if text_mode:
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        result = rosunit.create_xml_runner(package, test_name,
                                           result_file).run(suite)
    # if coverage_mode:
    #     _stop_coverage([package])
    rosunit.print_unittest_summary(result)

    # shutdown any node resources in case test forgets to
    rospy.signal_shutdown('test complete')
    if not result.wasSuccessful():
        sys.exit(1)


# TODO: rename to rosrun -- migrating name to avoid confusion and enable easy xmlrunner use
run = rosrun
