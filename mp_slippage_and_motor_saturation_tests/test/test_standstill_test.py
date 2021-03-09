#!/usr/bin/env python

PKG = 'mp_slippage_and_motor_saturation_tests'
NAME = 'standstill_test'

import sys 
import time
import unittest
import os

try:
    from unittest.mock import MagicMock
except ImportError:
    from mock import MagicMock


import rosunit
from mp_slippage_and_motor_saturation_tests.standstill_test import StandstillTest
from test_utils.utils import Utils

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose


class TestStandstillTest(unittest.TestCase):
    """
    This test class implements unittests for NavigationTest class
    """
    def setUp(self):
        self.standstill_test = StandstillTest()
        self.standstill_test.utils = Utils(Utils.AWS_ENV)
        self.standstill_test.test_name = "standstill_test"

    def test_check_time(self):
        # create mocks for used methods
        self.standstill_test.utils.set_tag = MagicMock()
        self.standstill_test.utils.cancel_job = MagicMock()


        self.standstill_test.is_cancelled = False
        self.standstill_test.time_init = 0  # 0 seconds
        self.standstill_test.time_end = 10 # 10 seconds

        # time is less than time_end
        msg = Clock()
        msg.clock.secs = 1
        self.standstill_test.check_time(msg)
        self.assertFalse(self.standstill_test.utils.set_tag.called)

        # set clock to 11 secs, so time_end should occur
        # but is cancelled is true, so set_tag function should not be called
        self.standstill_test.is_cancelled = True
        msg.clock.secs = 12

        self.standstill_test.check_time(msg)
        self.assertFalse(self.standstill_test.utils.set_tag.called)

        # now time_end should occur, and function should be called
        self.standstill_test.is_cancelled = False
        self.standstill_test.check_time(msg)
        self.standstill_test.utils.set_tag.assert_called_with(name=self.standstill_test.test_name + "_Status", value="Passed")

    def set_state_init(self):
        msg_state_init = Pose()
        msg_state_init.position.x = 0.0
        msg_state_init.position.y = 0.0
        msg_state_init.position.z = 0.0
        msg_state_init.orientation.x = 0.0
        msg_state_init.orientation.y = 0.0
        msg_state_init.orientation.z = 0.0
        msg_state_init.orientation.w = 0.0
        return msg_state_init

    def test_is_standstill(self):

        self.standstill_test.position_tolerance = 0.001
        self.standstill_test.orientation_tolerance = 0.001

        msg_state_current = self.set_state_init()
        msg_state_init = self.set_state_init()

        msg_state_current.position.x = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))

        msg_state_current = self.set_state_init()
        msg_state_current.position.y = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))

        msg_state_current = self.set_state_init()
        msg_state_current.position.z = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))

        msg_state_current = self.set_state_init()
        msg_state_current.orientation.x = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))

        msg_state_current = self.set_state_init()
        msg_state_current.orientation.y = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))

        msg_state_current = self.set_state_init()
        msg_state_current.orientation.z = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))

        msg_state_current = self.set_state_init()
        msg_state_current.orientation.w = 0.1
        self.assertFalse(self.standstill_test.is_standstill(msg_state_current, msg_state_init))


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestStandstillTest, sys.argv)