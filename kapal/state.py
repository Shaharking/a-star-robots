
import kapal

class State():
    pass

class State2d(State):
    def __init__(self, y=0, x=0):
        self.y = y
        self.x = x
    def __str__(self):
        return "(" + str(self.y) + ", " + str(self.x) + ")"

class State2dAStar(State2d):
    def __init__(self, y=0, x=0, g=kapal.inf, h=0, pr=None):
        State2d.__init__(self, y, x)
        self.g = g
        self.h = h
        self.pr = pr
    
    def reset(self):
        self.g = kapal.inf

#     def __cmp__(self, other):
#         # TODO: allow any key function?
#         # heapq library is a min heap
#         self_f = self.g + self.h
#         other_f = other.g + other.h
#         if self_f < other_f or (self_f == other_f and self.g > other.g):
#             # priority(self) > priority(other), so self < other
#             return -1
#         elif self_f == other_f and self.g == other.g:
#             return 0
#        return self_f < other_f
    
    def __lt__(self, other):
        """
        nodes are sorted by f value 

        :param other: compare Node
        :return:
        """
        self.f = self.g + self.h
        other.f = other.g + other.h
        #print('Compared with another object')
        return self.f < other.f

    def __str__(self):
        s = State2d.__str__(self) + "-->"
        if self.pr is None:
            s += "None"
        else:
            s += State2d.__str__(self.bp)
        s += ": g = " + str(self.g) + "; h = " + str(self.h)
        return s
