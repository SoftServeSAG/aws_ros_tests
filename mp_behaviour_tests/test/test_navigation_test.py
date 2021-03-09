#!/usr/bin/env python

PKG = 'mp_behaviour_tests'
NAME = 'navigation_test'

import sys 
import time
import unittest
import os

try:
    from unittest.mock import MagicMock
except ImportError:
    from mock import MagicMock


import rospy
import rosunit
import roslib.scriptutil as scriptutil
from mp_behaviour_tests.navigation_test import NavigationTest

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

class TestNavigationTest(unittest.TestCase):
    """
    This test class implements unittests for NavigationTest class
    """
    def setUp(self):
        self.navigation_test = NavigationTest()
        self.navigation_test.test_name = "navigation_test"

    def test_is_aws_env(self):
        self.navigation_test.environment = "LOCAL"
        self.assertFalse(self.navigation_test.is_aws_env())

        self.navigation_test.environment = "AWS"
        self.assertTrue(self.navigation_test.is_aws_env())


    def test_check_timeout(self):
        # create mocks for used methods
        self.navigation_test.set_tag = MagicMock()
        self.navigation_test.cancel_job = MagicMock()

        
        self.navigation_test.is_cancelled = False
        self.navigation_test.timeout = 10 # 10 seconds

        # time is less than timeout
        self.navigation_test.check_timeout(Clock())
        self.assertFalse(self.navigation_test.set_tag.called)

        # set clock to 11 secs, so timeout should occur
        # but is cancelled is true, so set_tag function should not be called
        self.navigation_test.is_cancelled = True
        msg = Clock()
        msg.clock.secs = 11

        self.navigation_test.check_timeout(msg)
        self.assertFalse(self.navigation_test.set_tag.called)

        # now timeoud should occur, and function should be called
        self.navigation_test.is_cancelled = False
        self.navigation_test.check_timeout(msg)
        self.assertTrue(self.navigation_test.set_tag.called)

    def test_check_complete(self):
        # create mocks for used methods
        self.navigation_test.set_tag = MagicMock()
        self.navigation_test.cancel_job = MagicMock()
        self.navigation_test.is_complete = MagicMock()
        self.navigation_test.set_latched = MagicMock()
        self.navigation_test.set_unlatched = MagicMock()
        self.navigation_test.increment_navigations = MagicMock()

        self.navigation_test.is_completed = False
        self.navigation_test.goal_tolerance = 1.0
        self.navigation_test.latch = False
        self.navigation_test.successful_navigations = 0

        msg = Float64()
        msg.data = 2
        self.navigation_test.check_complete(msg)
        self.assertFalse(self.navigation_test.set_latched.called)
        self.assertTrue(self.navigation_test.set_unlatched.called)

        msg.data = 0.5
        self.navigation_test.is_complete.return_value = False
        self.navigation_test.check_complete(msg)
        self.assertTrue(self.navigation_test.set_latched.called)
        self.assertFalse(self.navigation_test.cancel_job.called)

        msg.data = 0.5
        self.navigation_test.is_complete.return_value = True
        self.navigation_test.check_complete(msg)
        self.assertTrue(self.navigation_test.set_latched.called)
        self.assertTrue(self.navigation_test.cancel_job.called)
        self.navigation_test.set_tag.called_with(name=self.navigation_test.test_name + "_Status", value="Passed")


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestNavigationTest, sys.argv)