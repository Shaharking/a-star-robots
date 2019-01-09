import random
from .map_helper import generate_rect_obstacle
import kapal

def rand_cost_map_shapes(y_size=1, x_size=1, min_val=1, max_val=kapal.inf, min_num_of_obstacles=10, max_num_of_obstacles=20):
    map = []
    for i in range(y_size):
        row = []
        for j in range(x_size):
            row.append(min_val)
        map.append(row)

    num_of_obstacles = random.randint(min_num_of_obstacles, max_num_of_obstacles)
    map = generate_rect_obstacle(num_of_obstacles, map)
    return map



def rand_cost_map(y_size=1, x_size=1, min_val=1, max_val=kapal.inf,
        flip=False, flip_chance=.1):
    """
    Returns a 2d cost matrix with random values.

    Args:
        y_size - width
        x_size - height
        min_val - minimum random value
        max_val - maximum random value
        flip - if True, then the value in each cell is either min_val
               or max_val;
               if False, then min_val <= value of cell <= max_val
        flip_chance - chance of getting a max_val (only if flip=True)
    """
    map = []
    for i in range(y_size):
        row = []
        for j in range(x_size):
            if flip:
                if random.random() < flip_chance:
                    row.append(max_val) 
                else:
                    row.append(min_val)
            else:
                row.append(random.randint(min_val, max_val))
        map.append(row)
    return map

def const_map(y_size=1, x_size=1, min_val=1, max_val=kapal.inf, wall=False):
    """
    Returns a 2d cost matrix with always same values.

    Args:
        y_size - width
        x_size - height
        min_val - minimum random value
        max_val - maximum random value
       
    """
    map = []
    for i in range(y_size):
        row = []
        
        if i>y_size/2:
            wall=False
        
        for j in range(x_size):
            if wall:
                if j == int(x_size/2):
                    row.append(max_val) 
                else:
                    row.append(min_val)
            else:
                row.append(random.randint(min_val, max_val))
        map.append(row)
    return map

