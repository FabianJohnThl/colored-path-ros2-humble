import random
from .colorizer import Colorizer

class DummyColorizer(Colorizer):
    def __init__(self):
        self.current_color = None
        self.current_value = None
        self.query_cnt = 0
        self.color_map = {
            'poor': (1.0, 0.0, 0.0),  # Red
            'good': (1.0, 1.0, 0.0),       # Yellow
            'excellent': (0.0, 1.0, 0.0),       # Green
            'disconnected': (0.0, 0.0, 1.0),        # Blue
        }
        self.values = list(self.color_map.keys())

    def query_current_color(self):
        # switch color not more than each third call
        if ((self.query_cnt == 0) or ( (self.query_cnt%3) == 0)):
            self.current_value = random.choice(self.values)
            self.current_color = self.get_color()
        self.query_cnt += 1
        return self.current_color

    def get_color(self):
        return self.color_map.get(self.current_value.lower(), (0.5, 0.5, 0.5))

    def get_value(self):
        return self.current_value

    def is_same_as_current_color(self, prev_color):
        if not self.current_color:
            return False
        
        return ( (prev_color[0] == self.current_color[0]) and 
                 (prev_color[1] == self.current_color[1]) and 
                 (prev_color[2] == self.current_color[2]) )

