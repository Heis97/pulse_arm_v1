from abc import ABCMeta, abstractmethod, abstractproperty



class AbstractRobot():
    __metaclass__=ABCMeta

    @abstractmethod
    def connect():
        pass

    @abstractmethod
    def disconnect():
        pass

    @abstractmethod
    def get_position():
        pass

    @abstractmethod
    def get_pose():
        pass

    @abstractmethod
    def set_tool():
        pass

    @abstractmethod
    def set_base():
        pass

    @abstractmethod
    def move_ptp():
        pass

    @abstractmethod
    def move_cart():
        pass

    @abstractmethod
    def move_ps_ptp():
        pass

    @abstractmethod
    def move_ps_cart():
        pass

    @abstractmethod
    def get_current_line():
        pass

    @abstractmethod
    def stop():
        pass

    @abstractmethod
    def off_motors():
        pass