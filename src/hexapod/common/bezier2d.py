import numpy as np


class BezierCurve:
    def __init__(self, n=100):
        self.p = []
        self.cp = np.empty((0, 2), dtype=np.float64)
        self.d = 0
        self.t = 0
        self.step = np.float64(1.0/n)
        self.run = False

    def add_point(self, x, y):
        if self.d == 0:
            self.p.append([[x, y]])
        else:
            self.p[0].append([x, y])
            self.p.append([])
        self.d += 1

    def bezier(self):
        for d in range(1, self.d):
            self.p[d] = []
            for i in range(self.d - d):
                self.p[d].append([
                    self.p[d - 1][i][0] + self.t * (self.p[d - 1][i + 1][0] - self.p[d - 1][i][0]),
                    self.p[d - 1][i][1] + self.t * (self.p[d - 1][i + 1][1] - self.p[d - 1][i][1])
                ])
        xy = np.array(self.p[-1])
        self.cp = np.append(self.cp, xy, axis=0)

    def generate(self, t=None):
        if t:
            if type(t) is not list:
                t = [t]
            for x in t:
                self.t = x
                self.bezier()
        else:
            while True:
                self.bezier()
                if self.t == 1:
                    break
                self.t = min(self.t + self.step, 1)
        self.run = True

    def curve(self, t=None):
        if not self.run:
            self.generate(t)
        return self.cp

    def points(self):
        return np.array(self.p[0])