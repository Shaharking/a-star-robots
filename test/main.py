import os
import sys
import PyQt5
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from functools import reduce

BASE_PATH = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0,BASE_PATH.split('\\test')[0])
import kapal
import kapal.algo
import kapal.state
import kapal.world
import kapal.tools
import copy
WIDTH = 50      # pitch size WIDTH x WIDTH
HEIGHT = 35
CELL_SIZE = 16  # cell (tile) size in pixels CELL_SIZE x CELL_SIZE
CELL_RADIUS = 4 # circle radius for 
SQUARE_SIZE = 3 # square size SQUARE_SIZE x SQUARE_SIZE

num_of_robots = 3

class WorldCanvas(object):
    STATE_OPEN = 0x01
    STATE_CLOSED = 0x02
    STATE_EXPANDED = 0x04
    STATE_START = 0x10
    STATE_GOAL = 0x20
    STATE_PATH_ROBOT_1 = 0x80
    STATE_PATH_ROBOT_2 = 0x81
    STATE_PATH_ROBOT_3 = 0x82

    COLOR_RED = (255, 0, 0, 255)
    COLOR_REDTRAN = (255, 0, 0, 128)
    COLOR_BLUE = (0, 80, 255, 255)
    COLOR_BLACK = (0,0 ,0 ,255)
    COLOR_GREEN = (0, 255, 0, 255)
    COLOR_YELLOW = (255, 255, 0, 255)
    COLOR_TRANSPARENT = (0, 0, 0, 0)

    ROBOT_INDEX_TO_STATE = {
        0: (0, 255, 0, 255),
        1: (255, 0, 0, 255),
        2: (255, 204, 0, 255)
    }
    def __init__(self):
        self.painter = QPainter()

    def draw_image(self, image, x=0, y=0):
        point = QtCore.QPoint(x*self.cell_size, y*self.cell_size)

        try:
            if not self.painter.begin(self):
                raise Exception("painter failed to begin().")
            self.painter.drawImage(point, image)
        finally:
            self.painter.end()

    def draw_square(self, x=0, y=0, color=(0, 0, 0, 0),
            size=None, brush=None, image=None):
        if size is None:
            size = self.cell_size
        if brush is None:
            brush = QBrush(QtCore.Qt.SolidPattern)
        brush.setColor(QColor(*color))

        # to put square in center of cell
        padding = (self.cell_size-size)/2

        try:
            if not self.painter.begin(self):
                raise Exception("painter failed to begin().")
            self.painter.setBrush(brush)
            self.painter.drawRect(x*self.cell_size + padding, y*self.cell_size +
                    padding, size, size)
        finally:
            self.painter.end()

    def draw_line(self, x1=0, y1=0, x2=0, y2=0, pen=None, color = COLOR_GREEN, margin = 0):
        if pen is None:
            pen = QPen(QtCore.Qt.black, 2, QtCore.Qt.DashLine)
            pen.setColor(QColor(*color))

        # to put line in center of cell
        padding = self.cell_size/2 + margin
        try:
            if not self.painter.begin(self):
                raise Exception("painter failed to begin().")
            self.painter.setPen(pen)
            self.painter.drawLine(x1*self.cell_size+padding,
                    y1*self.cell_size+padding, x2*self.cell_size+padding,
                    y2*self.cell_size+padding)
        finally:
            self.painter.end()
        
    def draw_circle(self, x=0, y=0, radius=CELL_RADIUS, color=None):
        if color is None:
            color = WorldCanvas.COLOR_YELLOW
        brush = QBrush(QColor(*color))

        # to put circle in center of cell
        padding = self.cell_size/2
        center = QtCore.QPointF(x*self.cell_size+padding,
                y*self.cell_size+padding)

        try:
            if not self.painter.begin(self):
                raise Exception("painter failed to begin().")
            self.painter.setBrush(brush)
            self.painter.setRenderHint(QPainter.Antialiasing)
            self.painter.drawEllipse(center, radius, radius)
        finally:
            self.painter.end()

class World2dCanvas(QWidget, WorldCanvas):
    def __init__(self, parent=None, world_cost=None, world_cond=None,
            painter=None):
        QWidget.__init__(self, parent)
        WorldCanvas.__init__(self)
        self.paths = [[],[],[]]
        self.c_prev = -1
        self.r_prev = -1
        self.steps_shown = 0

        # cost of cells in the world
        if world_cost is None:
            self.world_cost = [[1]]
        self.world_cost = world_cost

        # world_cond is a 2d grid, where each cell holds
        # the condition of that cell
        if world_cond is None:
            self.world_cond = [[0]]
        self.world_cond = world_cond

        # size of each world cell drawn
        self.cell_size = CELL_SIZE

        if painter is None:
            painter = QPainter()
        self.painter = painter

    def paintEvent(self, event):
        self.draw_world2d()
        self.update()

    def draw_world2d(self, x_start=0, y_start=0, x_goal=0,
            y_goal=0):

        # previous c, r values of the path, for drawing path lines
        c_prev = -1
        r_prev = -1

        for r in range(len(self.world_cost)):
            for c in range(len(self.world_cost[r])):
                if self.world_cost[r][c] == kapal.inf:
                    # obstacle
                    self.draw_square(c, r, color=WorldCanvas.COLOR_BLACK)
                else:
                    # free space
                    self.draw_square(c, r, color=WorldCanvas.COLOR_BLUE)
                
                # show state of cell

                #if self.world_cond[r][c] & self.world_cond[r][c] >= WorldCanvas.STATE_PATH_ROBOT_1 and self.world_cond[r][c] <= WorldCanvas.STATE_PATH_ROBOT_1_AND_2_AND_3:
                #    # current cell is part of path
                #    if c_prev != -1:
                #        self.draw_line(c, r, c_prev, r_prev, state = self.world_cond[r][c])
                #    c_prev = c
                #    r_prev = r

                # draw start point
                if self.world_cond[r][c] & WorldCanvas.STATE_START:
                    #ship_img = QImage("icons/ship.png")
                    #self.draw_image(ship_img, c, r)
                    self.draw_circle(c, r, CELL_RADIUS)
                # draw goal points
                if self.world_cond[r][c] & WorldCanvas.STATE_GOAL:
                    self.draw_circle(c, r, CELL_RADIUS,
                            color=WorldCanvas.COLOR_GREEN)

                if self.world_cond[r][c] & WorldCanvas.STATE_EXPANDED:
                    # current cell was expanded
                    self.draw_square(c, r, color=WorldCanvas.COLOR_RED,
                            size=SQUARE_SIZE)

        # Insert stops whenever robots come to another robot place
        last_steps = [0,0,0]
        for robot_index in range(len(self.paths)):
            last_steps[robot_index] = len(self.paths[robot_index]) - 1

        step_index = 0
        max_last_steps = max(last_steps)
        while step_index <= max_last_steps:
            #for robot_path_idx in range(len(robot_path) - 1):

             if max_last_steps > 10000:
                 print('WTF')

             for robot_index in range(len(self.paths)):
                robot_path = self.paths[robot_index]

                if step_index > len(robot_path) - 2:
                    # Anything expect from the goal
                    continue

                real_path_idx = len(robot_path) - 1 - step_index
                previous_step_index = last_steps[robot_index]
                last_steps[robot_index] = real_path_idx
                current_robo_path = robot_path[real_path_idx]
                for other_robot in range(len(self.paths)):
                    if other_robot != robot_index:
                        other_robot_paths = self.paths[other_robot]
                        last_other_robo_path = other_robot_paths[last_steps[other_robot]]
                        if (last_other_robo_path.x == current_robo_path.x and
                            last_other_robo_path.y == current_robo_path.y):
                            # We added a pause by adding the same previous step to his current step
                            robot_path.insert(previous_step_index, robot_path[previous_step_index])
                            last_steps[robot_index] = previous_step_index
                            break
             step_index = step_index + 1
             max_last_steps = max(last_steps)


        # Draw Each path alone:
        for idx in range(len(self.paths)):
            robot_path = self.paths[idx]
            self.c_prev = -1
            self.r_prev = -1
            for robot_path_idx in range(len(robot_path)):
                real_path_idx = len(robot_path) - 1 - robot_path_idx
                path = robot_path[real_path_idx]

                if robot_path_idx > self.steps_shown:
                    break
                self.draw_robot_movment(path, idx)
                #QtCore.QTimer.singleShot(120, lambda: self.draw_robot_movment(path, idx))
                # c_prev, r_prev = path.x, path.y
            # QtCore.QTimer.singleShot(10000, lambda: self.increase_steps_shown())



    def increase_steps_shown(self):
        self.steps_shown = self.steps_shown + 1

    def draw_robot_movment(self, path, idx):
        #if path == None:
        #    return

        c = path.x
        r = path.y
        margin = 0
        if idx > 0:
            margin = ((-1) ** idx) * 4
        if self.c_prev != -1:
            self.draw_line(c, r, self.c_prev, self.r_prev, color=self.ROBOT_INDEX_TO_STATE[idx], margin=margin)

        #return c,r
        self.c_prev = c
        self.r_prev = r

class World2dSettingsDock(QDockWidget):
    """Settings for World2d. - TBD"""

    def __init__(self):
        QDockWidget.__init__(self)

        # size boxes        
        self.size_y_box = QSpinBox(self)
        self.size_y_box.setValue(20)
        self.size_x_box = QSpinBox(self)
        self.size_x_box.setValue(20)

        # start boxes
        self.start_y_box = QSpinBox(self)
        self.start_y_box.setValue(4)
        self.start_x_box = QSpinBox(self)
        self.start_x_box.setValue(4)

        #start boxes 2
        self.start_y_box_2 = QSpinBox(self)
        self.start_y_box_2.setValue(6)
        self.start_x_box_2 = QSpinBox(self)
        self.start_x_box_2.setValue(6)

        self.start_y_box_3 = QSpinBox(self)
        self.start_y_box_3.setValue(8)
        self.start_x_box_3 = QSpinBox(self)
        self.start_x_box_3.setValue(8)


        #goal boxes
        self.goal_y_box = QSpinBox(self)
        self.goal_y_box.setValue(16)
        self.goal_x_box = QSpinBox(self)
        self.goal_x_box.setValue(16)

        # main box layout
        vbox = QVBoxLayout()
        vbox.setAlignment(Qt.AlignTop|Qt.AlignLeft)

        self.rand_widget = QCheckBox("Randomize World")
        self.rand_widget.setCheckState(Qt.Checked)
        vbox.addWidget(self.rand_widget)
        vbox.addWidget(QCheckBox("Traversable Obstacles"))
        
        vbox.addWidget(QLabel("World Size"))
        hbox_world_size = QHBoxLayout()
        hbox_world_size.addWidget(QLabel("Y"))
        hbox_world_size.addWidget(self.size_y_box)
        hbox_world_size.addWidget(QLabel("X"))
        hbox_world_size.addWidget(self.size_x_box)
        world_size_widget = QWidget()
        world_size_widget.setLayout(hbox_world_size)
        vbox.addWidget(world_size_widget)

        vbox.addWidget(QLabel("Start Robot 1"))
        hbox_start = QHBoxLayout()
        hbox_start.addWidget(QLabel("Y"))
        hbox_start.addWidget(self.start_y_box)
        hbox_start.addWidget(QLabel("X"))
        hbox_start.addWidget(self.start_x_box)
        start_widget = QWidget()
        start_widget.setLayout(hbox_start)
        vbox.addWidget(start_widget)

        vbox.addWidget(QLabel("Start Robot 2"))
        hbox_start_2 = QHBoxLayout()
        hbox_start_2.addWidget(QLabel("Y"))
        hbox_start_2.addWidget(self.start_y_box_2)
        hbox_start_2.addWidget(QLabel("X"))
        hbox_start_2.addWidget(self.start_x_box_2)
        start_widget_2 = QWidget()
        start_widget_2.setLayout(hbox_start_2)
        vbox.addWidget(start_widget_2)

        vbox.addWidget(QLabel("Start Robot 3"))
        hbox_start_3 = QHBoxLayout()
        hbox_start_3.addWidget(QLabel("Y"))
        hbox_start_3.addWidget(self.start_y_box_3)
        hbox_start_3.addWidget(QLabel("X"))
        hbox_start_3.addWidget(self.start_x_box_3)
        start_widget = QWidget()
        start_widget.setLayout(hbox_start_3)
        vbox.addWidget(start_widget)

        vbox.addWidget(QLabel("Goal"))
        hbox_goal = QHBoxLayout()
        hbox_goal.addWidget(QLabel("Y"))
        hbox_goal.addWidget(self.goal_y_box)
        hbox_goal.addWidget(QLabel("X"))
        hbox_goal.addWidget(self.goal_x_box)
        goal_widget = QWidget()
        goal_widget.setLayout(hbox_goal)
        vbox.addWidget(goal_widget)

        widget = QWidget(self)
        widget.setLayout(vbox)
        self.setWidget(widget)

class MainSettingsDock(QDockWidget):
    """Dock for choosing algorithm and world."""

    world_list = ["2D 4 Neighbors", "2D 8 Neighbors"]
    algo_list = ["A*", "Dijkstra"]
    heuristic_list = ["Manhattan", "Euclidean"]

    def __init__(self):
        QDockWidget.__init__(self)

        # world chooser
        self.world_combo = QComboBox()
        self.world_combo.addItems(MainSettingsDock.world_list)
        self.world_combo.setItemIcon(0, QIcon('icons/2d_4neigh.png'))
        self.world_combo.setItemIcon(1, QIcon('icons/2d_8neigh.png'))

        # algorithm chooser
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(MainSettingsDock.algo_list)
        #self.algo_combo.currentIndexChanged(MainWindow.update_algo)

        # heuristic chooser
        self.heuristic_combo = QComboBox()
        self.heuristic_combo.addItems(MainSettingsDock.heuristic_list)
        self.heuristic_combo.setItemIcon(0, QIcon('icons/heur_manhattan.png'))
        self.heuristic_combo.setItemIcon(1, QIcon('icons/heur_euclidean.png'))

        # algo settings
        vbox = QVBoxLayout()
        vbox.setAlignment(Qt.AlignTop|Qt.AlignHCenter)
        vbox.addWidget(QLabel("World"))
        vbox.addWidget(self.world_combo)
        vbox.addWidget(QLabel("Algorithm"))
        vbox.addWidget(self.algo_combo)
        vbox.addWidget(QLabel("Heuristics"))
        vbox.addWidget(self.heuristic_combo)

        widget = QWidget()
        widget.setLayout(vbox)
        self.setWidget(widget)
        
class MainWindow(QMainWindow):
    
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        # main settings dock
        self.main_settings = MainSettingsDock()
        self.world_settings = World2dSettingsDock()
        # set up planner
        self.algo_t = kapal.algo.AStar
        self.world_t = kapal.world.World2d
        self.state_t = kapal.state.State2dAStar
        self.random_world(WIDTH, HEIGHT)

        # general GUI settings
        self.setUnifiedTitleAndToolBarOnMac(True)

        # set up window
        self.setGeometry(100, 100, 600, 400)
        self.setWindowTitle('PathFinder')
        #self.painter = QPainter()
       
        # world canvas
        self.worldcanvas = World2dCanvas(parent=self)
        self.setCentralWidget(self.worldcanvas)
        self.addDockWidget(Qt.RightDockWidgetArea, self.main_settings)
        self.addDockWidget(Qt.RightDockWidgetArea, self.world_settings)

        # built tool bar
        # start[play] button
        start_button = QAction(QIcon('icons/play.png'),
                'Start', self)
        start_button.setShortcut('Ctrl+R')
        start_button.setStatusTip('Start Planning')
        start_button.triggered.connect(self.plan)

        # stop button
        stop_button = QAction(QIcon('icons/stop.png'),
                'Start', self)
        stop_button.setShortcut('Ctrl+T')
        stop_button.setStatusTip('Stop')
        stop_button.triggered.connect(self.reset_world)

        # reset button
        reset_button = QAction(QIcon('icons/reset.png'),
                'Random', self)
        reset_button.setShortcut('Ctrl+N')
        reset_button.setStatusTip('Randomize World')
        reset_button.triggered.connect(self.random_world)

        forward_button = QAction(QIcon('icons/forward.png'),
                'Forward', self)
        forward_button.setShortcut('Ctrl+Y')
        forward_button.setStatusTip('Forward Planning')
        forward_button.triggered.connect(self.forward_plan)

        toolbar = self.addToolBar('Control')
        toolbar.addAction(reset_button)
        toolbar.addAction(start_button)
        toolbar.addAction(stop_button)
        toolbar.addAction(forward_button)

        # status bar
        self.statusBar()

    def update_algo(self):
        algo_ind=self.main_settings.algo_combo.currentIndex()
        if algo_ind == 1:
           self.algo_t = kapal.algo.Dijkstra
        if algo_ind == 0:
            self.algo_t = kapal.algo.AStar        
        #if algo_ind == 1:
        #   self.algo_t = kapal.algo.PRM
        # if algo_ind == 3:
        #   self.algo_t = kapal.algo.AStar

    def random_world(self, width=WIDTH, height=HEIGHT):
        # set up world
        if not width:
            width=WIDTH
        # World2d
        min_val=1
        max_val=kapal.inf
        rand_vi=self.world_settings.rand_widget.isChecked()
        if rand_vi:
            self.c = kapal.tools.rand_cost_map_shapes(height, width, min_val, max_val,min_num_of_obstacles = 30, max_num_of_obstacles = 30)
            # self.c = kapal.tools.rand_cost_map(width, width, min_val, max_val,
            #    flip=True, flip_chance=.1)
        else:
            self.c = kapal.tools.const_map(height, width, min_val, max_val,
                wall=True)
        
        self.world_cond = [ [0]*len(self.c[0]) for i in range(len(self.c)) ]
        world_ind = self.main_settings.world_combo.currentIndex()

        self.world = []
        for i in range(num_of_robots):
            if world_ind == 0:
                self.world.append(kapal.world.World2d(self.c, state_type = kapal.state.State2dAStar,diags=False))
            if world_ind == 1:
                self.world.append(kapal.world.World2d(self.c, state_type = kapal.state.State2dAStar,diags=True))

        if (hasattr(self, 'worldcanvas')):
            self.worldcanvas.paths = [[], [], []]

    def reset_world(self):
        self.world_cond = [[0] * len(self.c[0]) for i in range(len(self.c))]
        for world in self.world:
            world.reset()
        self.worldcanvas.paths = [[], [], []]
        self.worldcanvas.steps_shown = 0

    def forward_plan(self):
        self.worldcanvas.increase_steps_shown()

    def plan(self):
        # update algorithm name from GUI
        self.update_algo()
        
        if (self.algo_t is kapal.algo.Dijkstra or
                self.algo_t is kapal.algo.AStar or  
                self.algo_t is kapal.algo.PRM):
            
            start_y = self.world_settings.start_y_box.value()            
            start_x = self.world_settings.start_x_box.value()

            start_x_2 = self.world_settings.start_x_box_2.value()
            start_y_2 = self.world_settings.start_y_box_2.value()

            start_x_3 = self.world_settings.start_x_box_3.value()
            start_y_3 = self.world_settings.start_y_box_3.value()

            states = [
                self.world[0].state(start_y, start_x),
                self.world[1].state(start_y_2, start_x_2),
                self.world[2].state(start_y_3, start_x_3)
            ]

            goal_y = self.world_settings.goal_y_box.value() 
            goal_x = self.world_settings.goal_x_box.value()

            goal = [
                self.world[0].state(goal_y, goal_x),
                self.world[1].state(goal_y, goal_x),
                self.world[2].state(goal_y, goal_x)
            ]

            self.world_cond[start_y][start_x] |= WorldCanvas.STATE_START
            self.world_cond[start_y_2][start_x_2] |= WorldCanvas.STATE_START
            self.world_cond[start_y_3][start_x_3] |= WorldCanvas.STATE_START

            self.world_cond[goal_y][goal_x] |= WorldCanvas.STATE_GOAL
            algo_obj = self.algo_t(self.world, states, goal, False)
            
            num_popped = 0
            pl_list = algo_obj.plan()
            for j in pl_list:
                for key in j:
                    s = j[key]
                    self.world_cond[s.y][s.x] |= WorldCanvas.STATE_EXPANDED
                num_popped += 1

            print (self.algo_t.__name__)
            print ('It searched %d vertex for the 3 robots' %(num_popped))
            paths = algo_obj.path()
            total_distance = [len(path) for path in paths]
            print('distance for 3 robots')
            print (total_distance)
            self.worldcanvas.paths = paths

            #for idx in range(len(paths)):
                #path = paths[idx]
                #for s in path:
                    #self.set_word_cond(s, idx)



    def paintEvent(self, event):
        self.worldcanvas.world_cost = copy.deepcopy(self.c)
        self.worldcanvas.world_cond = self.world_cond
        self.update()

app = QApplication(sys.argv)
mainwin = MainWindow()
mainwin.show()
app.exec_()
