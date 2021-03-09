#!/usr/bin/env python

from environment_utils.aws_env_utils import AwsUtils
from environment_utils.local_env_utils import LocalEnvUtils


class Utils():

    AWS_ENV = "AWS"

    def __init__(self, env=AWS_ENV):
        self.set_env(env)

    def cancel_job(self):
        self._environment.cancel_job()
    
    def set_tag(self, name, value):
        self._environment.set_tag(name, value)

    def set_env(self, env):
        if env == self.AWS_ENV:
            self._environment = AwsUtils()
        else:
            self._environment = LocalEnvUtils()
