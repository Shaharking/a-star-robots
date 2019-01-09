import heapq
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

    def __init__(self, world, start=None, goal=None, backwards=False):
        Algo.__init__(self, world, start, goal)
        self.backwards = backwards
        self.open = []

    def plan(self):
        """
        Plans and returns the optimal path, from start to goal.
        """
        return list(self.__plan_gen())

    def __plan_gen(self):
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
        self.world.reset()      # forget previous search's g-vals
        goal = self.goal
        succ = self.world.succ  # successor function

        if self.backwards:
            self.goal.g = 0
            self.open = [self.goal]
            goal = self.start
            succ = self.world.pred  # flip map edges
        else:
            for start in self.start:
                start.g = 0
            self.open = [[s] for s in self.start]

        # A*
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


    def path(self):
        """
        Returns the path from goal to the first state with pr = None.

        This method assumes that 
        """
        p = []
        s = self.goal
        if self.backwards:
            s = self.start
        while s is not None:
            p.append(s)
            if s is not None:
                s = s.pr
        return p
        
    def h(self, s1, s2, h_func=None):
        """
        Returns the heuristic value between s1 and s2.

        Uses h_func, a user-defined heuristic function, if
        h_func is passed in.
        """
        if h_func is None:
            return self.world.h(s1, s2)
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
        self.backwards = backwards
        self.open = []

        """
        TBD
        
        """

    def h(self, s1, s2, h_func=None):
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
    def __init__(self, world, start=None, goal=None, backwards=True):
        Algo.__init__(self, world, start, goal)
        self.backwards = backwards
        self.open = []
        
        self.nsamp=200
        self.sdist=1
        
        

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