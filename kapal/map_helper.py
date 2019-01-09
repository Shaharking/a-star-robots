import random
from .shape import Shape
import numpy as np

import kapal


def create_blockable_shape(matrix, w, h, shape):

    height = len(matrix)
    width = len(matrix[0])
    x = random.randint(1, np.floor(width - w - 1))
    y = random.randint(1, np.floor(height - h - 1))

    if shape == Shape.Rectangle:
        for i in range(y, y+h):
            for j in range(x, x+w):
                matrix[i][j] = kapal.inf

    try:
        if shape == Shape.Triangle:
            for loop1 in range(h):
                i = y+loop1
                for j in range(x, x + loop1):
                    matrix[i][j] = kapal.inf
    except:
        print (x, y, w, h)

    if shape == Shape.Rhombus:
        for loop1 in range(int(h/2)):
            i = y+loop1
            middle = int((x+w)/2)
            for j in range(middle-loop1, middle+loop1):
                matrix[i][j] = kapal.inf

    if shape == Shape.Trapezoid:
        middle = (x+w)/2
        extra = w/h
        base = w/h
        for i in range(y,y+h):
            base = base+extra
            for j in range(int(middle-base), int(middle+base)):
                matrix[i][j] = kapal.inf

    return matrix

def generate_rect_obstacle(num_of_obstacles, matrix):

    height = len(matrix)
    width = len(matrix[0])

    obstacle_width_size = (3, int(0.2 * width))
    obstacle_height_size = (3, int(0.2 * height))

    for loop in range(num_of_obstacles):

        choice = random.choice(list(Shape))

        w = random.randint(obstacle_width_size[0], obstacle_width_size[1])
        h = random.randint(obstacle_height_size[0], obstacle_height_size[1])

        matrix = create_blockable_shape(matrix,w,h,choice)

    return matrix