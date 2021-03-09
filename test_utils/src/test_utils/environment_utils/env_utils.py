import abc
class EnvUtils(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def cancel_job(self):
        pass

    @abc.abstractmethod
    def set_tag(self, name, value):
        pass