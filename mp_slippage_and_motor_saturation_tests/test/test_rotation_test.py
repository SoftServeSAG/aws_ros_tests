#!/usr/bin/env python

PKG = 'mp_slippage_and_motor_saturation_tests'
NAME = 'rotation_test'

import sys 
import time
import unittest
import os

try:
    from unittest.mock import MagicMock
except ImportError:
    from mock import MagicMock


import rosunit
from mp_slippage_and_motor_saturation_tests.rotation_test import RotationTest

from rosgraph_msgs.msg import Clock

from test_utils.utils import Utils


class TestRotationTest(unittest.TestCase):
    """
    This test class implements unittests for RotationTest class
    """
    def setUp(self):
        self.rotation_test = RotationTest()
        self.rotation_test.utils = Utils(Utils.AWS_ENV)
        self.rotation_test.test_name = "rotation_test"

    def test_check_timeout(self):
        # create mocks for used methods
        self.rotation_test.utils.set_tag = MagicMock()
        self.rotation_test.utils.cancel_job = MagicMock()

        self.rotation_test.is_cancelled = False
        self.rotation_test.timeout = 10  # 10 seconds

        # time is less than timeout
        self.rotation_test.check_timeout(Clock())
        self.assertFalse(self.rotation_test.utils.set_tag.called)

        # set clock to 11 secs, so timeout should occur
        # but is cancelled is true, so set_tag function should not be called
        self.rotation_test.is_cancelled = True
        msg = Clock()
        msg.clock.secs = 11

        self.rotation_test.check_timeout(msg)
        self.assertFalse(self.rotation_test.utils.set_tag.called)

        # now timeoud should occur, and function should be called
        self.rotation_test.is_cancelled = False
        self.rotation_test.check_timeout(msg)
        self.assertTrue(self.rotation_test.utils.set_tag.called)

    def test_check_deviation(self):

        self.rotation_test._imu_rotated_angle = 0.2
        self.rotation_test._gazebo_rotated_angle = 0.0
        self.rotation_test.rotation_tolerance = 0.1
        self.assertFalse(self.rotation_test.check_deviation())


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestRotationTest, sys.argv)