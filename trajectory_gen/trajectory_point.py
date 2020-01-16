

class TrajectoryPoint:

    def __init__(self, t = None, x = None, y = None, v = None, a = None, hdg = None):
        self.t = t
        self.x = x
        self.y = y
        self.v = v
        self.a = a
        self.hdg = hdg

    def export_csv_string(self):
        return str(self.t) + ", " + str(self.x) + ", " + str(self.y) + ", " + str(self.v) + ", " + str(self.a) + ", " + str(self.hdg) + "\n"