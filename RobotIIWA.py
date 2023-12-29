from abc import ABCMeta, abstractmethod, abstractproperty



class AbstractRobot():
    __metaclass__=ABCMeta

    @abstractmethod
    def move():
        pass
    
    @abstractproperty
    def speed():
        pass