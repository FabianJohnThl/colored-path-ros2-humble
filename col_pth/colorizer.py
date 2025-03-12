from abc import ABC, abstractmethod

class Colorizer(ABC):
    @abstractmethod
    def query_current_color(self):
        pass

    @abstractmethod
    def get_color(self):
        pass

    @abstractmethod
    def get_value(self):
        pass
    
    @abstractmethod
    def is_same_as_current_color(self, prev_color):
        pass

