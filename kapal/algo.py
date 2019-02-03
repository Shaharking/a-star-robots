import heapq
import numpy as np
import random
import sys
import os
BASE_PATH = os.path.abspath(os.path.dirname(__file__))
print(BASE_PATH)
sys.path.insert(0,BASE_PATH)
from .state import *
from .world import *


def VROF(function, message, *args, **kwargs):     
    try:            
        result = None        
        result = function( *args, **kwargs)
    except:
        print('Something wrong there.. - '+message)        
        pass
    return result

class Algo:
    """
    A base class for algorithms.
    All algorithms should inherit Algo and should overwrite Algo.plan.
    """
    def __init__(self, world, start, goal):
        self.world = world
        self.start = start
        self.goal = goal
        
    def plan(self):
        pass

class AStar(Algo):
    """
    A* algorithm.

    A* makes a couple of assumptions:
        - non-negative edge weights
        - heuristics function is consistent (and thus admissible)
            - http://en.wikipedia.org/wiki/Consistent_heuristic
    """

    def __init__(self, world, start=None, goal = None, backwards=False):
        Algo.__init__(self, world, start, goal)
        self.goal = [g for g in goal]
        self.backwards = backwards
        self.open = []

    def plan(self):
        """
        Plans and returns the optimal path, from start to goal.
        """
        return list(self.__plan_gen())

    def __plan_gen(self):
        print (self.goal)
        """
        Plans the optimal path via a generator.

        A generator that yields states as it is popped off
        the open list, which is the optimal path in A* assuming
        all assumptions regarding heuristics are held.

        The user should not call AStar.__plan_gen. Call
        AStar.plan instead. This is a generator for the sake of
        easy debugging; it is usually unsafe to use the yielded
        states as the path.
        """
        for world in self.world:
            world.reset()      # forget previous search's g-vals

        goal = self.goal
        # succ = self.world.succ  # successor function

        if self.backwards:
            self.goal.g = 0
            self.open = [self.goal]
            goal = self.start
            # succ = self.world.pred  # flip map edges
        else:
            for start in self.start:
                start.g = 0
            self.open = [[s] for s in self.start]

        # A*
        heapy = [[None], [None], [None]]
        every_robot_found_goal = False
        trace_steps = {0: [], 1: [], 2: [] }
        iteration = 1
        while not every_robot_found_goal:
            steps = {}

            #if any(len(open) == 0 for open in self.open):
            #   break

            robot_skips = [False, False, False]
            for robot_index in range(len(heapy)):
                s = heapy[robot_index]
                #other_robot_current_location = []
                #for i in range(len(heapy)):
                #    if i != robot_index and len(trace_steps[i]) > 0:
                #        last_step = (trace_steps[i])[-1]
                #        other_robot_current_location.append((last_step, i))

                last_place = None
                if len(trace_steps[robot_index]) >= 1:
                    last_place = trace_steps[robot_index][-1]

                if last_place is not goal[robot_index] and len(self.open[robot_index]) > 0:
                    s = heapq.heappop(self.open[robot_index])
                    try:
                        for n, cost in self.world[robot_index].succ(s):
                            if n.g > s.g + cost:
                                # s improves n
                                n.g = s.g + cost
                                n.h = self.h(n, goal[robot_index], robot=robot_index)
                                n.pr = s
                                heapq.heappush(self.open[robot_index], n)
                        steps[robot_index] = s
                        trace_steps[robot_index].append(s)
                    except Exception as e:
                        print(str(e))
            yield steps
            iteration = iteration + 1
            every_robot_found_goal = all(trace_steps[robot_index][-1] is goal[robot_index]
                                         or len(self.open[robot_index]) == 0
                                         for robot_index in range(len(heapy))
                                         )
        #print('End of algo planning')


    def path(self):
        """
        Returns the path from goal to the first state with pr = None.

        This method assumes that 
        """

        p = []
        #s = self.goal
        #if self.backwards:
        #    s = self.start
        #while s is not None:
        #    p.append(s)
        #    if s is not None:
        #        s = s.pr
        # return p

        for goal_index in range(len(self.world)):
            row = []
            s = self.goal[goal_index]
            if self.backwards:
                s = self.start

            while s is not None:
                row.append(s)
                if s is not None:
                    s = s.pr
            p.append(row)
#
        return p
        
    def h(self, s1, s2, h_func=None, robot=0):
        """
        Returns the heuristic value between s1 and s2.

        Uses h_func, a user-defined heuristic function, if
        h_func is passed in.
        """
        if h_func is None:
            return self.world[robot].h(s1, s2)
        else:
            return h_func(s1, s2)

class Dijkstra(AStar):
    """
    Classic Dijkstra search.

    Assumptions:
        - non-negative edge weights
    """
    def __init__(self, world, start=None, goal=None, backwards=True):
        Algo.__init__(self, world, start, goal)
        self.goal = [g for g in goal]
        self.backwards = backwards
        self.open = []

        """
        TBD
        
        """

    def h(self, s1, s2, h_func=None, robot=0):
        """
        Returns the heuristic value equal to zero.

        """
        return 0

class PRM(AStar):
    """
    Classic Probabalistic roadmap search.

    Assumptions:
        - non-negative edge weights
    """

    def distance(self, co1, co2):
        dist = ((co1[0] - co2[0]) ** 2) + ((co1[1] - co2[1]) ** 2)
        return dist

    def __init__(self, world, start=None, goal=None, backwards=True):
        Algo.__init__(self, world, start, goal)
        self.backwards = backwards
        self.open = []
        
        self.nsamp=60
        self.sdist=100
        self.points = []

        self.new_maps = []

        #coordinates = []
        #first_world = world[0].costs
        #width = len(first_world)
        #height = width

        #for i in range(width):
        #    for j in range(height):
        #        self.points.append((i,j))

        #self.points = random.sample(self.points, self.nsamp)
        #self.points.sort()
        #for p in self.points:
        #    temp = []
        #    for p2 in self.points:
        #        if p == p2:
        #            continue
        #        temp.append(self.distance(p, p2))

        #    index_sort = np.argsort(temp)
        #    for z in index_sort[:10]:
        #        if temp[z] <= self.sdist:

        #    print (index_sort)
        #print(self.points)
        

    def h(self, s1, s2, h_func=None):
        """
        Returns the heuristic value equal to zero.

        """
        return 0

    def rnodes(self):
        """ 
        Sample algorithm for allowed nodes
        
        """
        print('rnodes')
        
    def checkPath(self):
        """
        procedure for checking collision free path between 2 give nodes
        
        """
        print('checkPath')    
        
        
    def plan(self):
        """
        Plans and returns the optimal path, from start to goal.
        """
        return list(self.__plan_gen())
        
    def __plan_gen(self):
        """
        Plans the optimal path via a generator.

        A generator that yields states as it is popped off
        the open list, which is the optimal path in PRM assuming
        all assumptions regarding heuristics are held.

        The user should not call PRM.__plan_gen. Call
        AStar.plan instead. This is a generator for the sake of
        easy debugging; it is usually unsafe to use the yielded
        states as the path.
        """
        self.world.reset()      # forget previous search's g-vals
        goal = self.goal
        succ = self.world.succ  # successor function        
        self.start.g = 0
        self.open = [self.start]

        # PRM
        s = None        
        while s is not goal and len(self.open) > 0:            
            s = heapq.heappop(self.open)                                  
            for n, cost in succ(s):
                if n.g > s.g + cost:
                    # s improves n
                    n.g = s.g + cost
                    n.h = self.h(n, goal)
                    n.pr = s                    
                    heapq.heappush(self.open, n)                    
            yield s
        #print('End of algo planning')