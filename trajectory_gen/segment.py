

from math import cos, sin, radians

class Segment:

    def __init__(self, x0 = None, y0 = None, xf = None, yf = None, hdg = None):
        self.x0 = x0
        self.xf = xf
        self.y0 = y0
        self.yf = yf
        self.hdg = hdg

    def set_start(self, x0, y0):
        self.x0 = x0
        self.y0 = y0

    def set_end(self, xf, yf):
        self.xf = xf
        self.yf = yf

    def set_hdg(self, hdg):
        self.hdg = hdg

    def get_offset_points(self, offset):
        x_offset = offset * sin(radians(self.hdg))
        y_offset = -(offset * cos(radians(self.hdg)))

        return(self.x0 + x_offset, self.y0 + y_offset, self.xf + x_offset, self.yf + y_offset)
