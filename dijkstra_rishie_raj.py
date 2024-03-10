# GitHub Repository: https://github.com/rishieraj/enpm661-dijkstra.git

# importing libraries and dependencies
import numpy as np
from queue import PriorityQueue
import time
import cv2

# creating open and closed lists to manage the search
open_list = PriorityQueue()
close_list = set()

# defining the Configuration Space
def config_space():

    # declaring Configuration Space as an array
    c_space = np.zeros((500, 1200))

    # Boundary + Clearance
    c_space[1:6, :] = 1
    c_space[0, :] = 2

    c_space[495:500, :] = 1
    c_space[499, :] = 2

    c_space[:, 1:6] = 1
    c_space[:, 0] = 2

    c_space[:, 1195:1200] = 1
    c_space[:, 1199] = 2

    # Rectangles + Clearance
    for i in range(0, 1200):
        for j in range(0, 500):
            # Clearance
            a1 = i - 94
            a2 = j - 500
            a3 = i - 181
            a4 = j - 94

            b1 = i - 269
            b2 = j - 406
            b3 = i - 356
            b4 = j

            # initializing pixel values for clearances
            if (a1 > 0) and (a2 < 0) and (a3 < 0) and (a4 > 0):
                c_space[j, i] = 1

            if (b1 > 0) and (b2 < 0) and (b3 < 0) and (b4 > 0):
                c_space[j, i] = 1

            # Rectangles
            c1 = i - 100
            c2 = j - 500
            c3 = i - 175
            c4 = j - 100

            d1 = i - 275
            d2 = j - 400
            d3 = i - 350
            d4 = j

            # initializing pixel values for obstacles
            if (c1 > 0) and (c2 < 0) and (c3 < 0) and (c4 > 0):
                c_space[j, i] = 2

            if (d1 > 0) and (d2 < 0) and (d3 < 0) and (d4 > 0):
                c_space[j, i] = 2

            # Polygon + Clearance
            # Clearance
            e1 = i - 894
            e2 = j - 456
            e3 = i - 1106
            e4 = j - 369

            f1 = i - 894
            f2 = j - 131
            f3 = i - 1106
            f4 = j - 44

            g1 = i - 1014
            g2 = j - 381
            g3 = i - 1106
            g4 = j - 119

            # initializing pixel values for clearances
            if (e1 > 0) and (e2 < 0) and (e3 < 0) and (e4 > 0):
                c_space[j, i] = 1

            if (f1 > 0) and (f2 < 0) and (f3 < 0) and (f4 > 0):
                c_space[j, i] = 1

            if (g1 > 0) and (g2 < 0) and (g3 < 0) and (g4 > 0):
                c_space[j, i] = 1
            
            # Polygon
            h1 = i - 900
            h2 = j - 450
            h3 = i - 1100
            h4 = j - 375

            i1 = i - 900
            i2 = j - 125
            i3 = i - 1100
            i4 = j - 50

            j1 = i - 1020
            j2 = j - 376
            j3 = i - 1100
            j4 = j - 124

            # initializing pixel values for obstacles
            if (h1 > 0) and (h2 < 0) and (h3 < 0) and (h4 > 0):
                c_space[j, i] = 2

            if (i1 > 0) and (i2 < 0) and (i3 < 0) and (i4 > 0):
                c_space[j, i] = 2

            if (j1 > 0) and (j2 < 0) and (j3 < 0) and (j4 > 0):
                c_space[j, i] = 2

            # Hexagon + Clearance           
            # Hexagon
            k1 = i - 520
            k2 = j - np.tan(np.radians(30)) * i - 25
            k3 = j + np.tan(np.radians(30)) * i - 774
            k4 = i - 780
            k5 = j - np.tan(np.radians(30)) * i + 275
            k6 = j + np.tan(np.radians(30)) * i - 474

            # Clearance
            l1 = i - 514
            l2 = j - np.tan(np.radians(30)) * i - 30
            l3 = j + np.tan(np.radians(30)) * i - 780
            l4 = i - 786
            l5 = j - np.tan(np.radians(30)) * i + 281
            l6 = j + np.tan(np.radians(30)) * i - 469

            # initializing pixel values for obstacles
            if (k1 > 0) and (k2 < 0) and (k3 < 0) and (k4 < 0) and (k5 > 0) and (k6 > 0):
                c_space[j, i] = 2

            # initializing pixel values for clearances
            if (l1 > 0) and (l2 < 0) and (l3 < 0) and (l4 < 0) and (l5 > 0) and (l6 > 0):
                c_space[j, i] = 1

    return c_space      

# defining Actions    
def move_right(current_node, obs_space):
    x = current_node[1][0] + 1
    y = current_node[1][1]

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1  
    new_node = [cost, (x, y)]

    return new_node

def move_left(current_node, obs_space):
    x = current_node[1][0] - 1
    y = current_node[1][1]

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1  
    new_node = [cost, (x, y)]

    return new_node

def move_up(current_node, obs_space):
    x = current_node[1][0]
    y = current_node[1][1] + 1

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1  
    new_node = [cost, (x, y)]

    return new_node

def move_down(current_node, obs_space):
    x = current_node[1][0]
    y = current_node[1][1] - 1

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1  
    new_node = [cost, (x, y)]

    return new_node

def move_up_right(current_node, obs_space):
    x = current_node[1][0] + 1
    y = current_node[1][1] + 1

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1.4  
    new_node = [cost, (x, y)]

    return new_node

def move_up_left(current_node, obs_space):
    x = current_node[1][0] - 1
    y = current_node[1][1] + 1

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1.4  
    new_node = [cost, (x, y)]

    return new_node

def move_down_right(current_node, obs_space):
    x = current_node[1][0] + 1
    y = current_node[1][1] - 1

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1.4  
    new_node = [cost, (x, y)]

    return new_node

def move_down_left(current_node, obs_space):
    x = current_node[1][0] - 1
    y = current_node[1][1] - 1

    # checking validity of action
    if (obs_space[y][x] == 1):
        return None
    
    # updating C2C
    cost = current_node[0] + 1.4  
    new_node = [cost, (x, y)]

    return new_node

def dijkstra(start, goal, obs_space):
    # structure of node: (cost_to_come, node_index, parent_index, (x cordinate, y cordinate))
    start_node = (0, start)
    open_list.put(start_node)
    # creating a dict to track parent child relations for backtracking
    parent_map = {}
    parent_map[start] = None

    while not open_list.empty():
        current_node = open_list.get()

        if current_node[1] in close_list:
            continue
        
        close_list.add(current_node[1])

        if current_node[1] == goal:
            print('Goal Reached!')
            # calling backtracking function after goal node is reached
            path = back_tracking(parent_map, start, goal)
            return path, parent_map
        
        next_nodes = []

        # performing action sets
        action_node1 = move_up(current_node, obs_space)
        if action_node1 != None:
            next_nodes.append(action_node1)

        action_node2 = move_down(current_node, obs_space)
        if action_node2 != None:
            next_nodes.append(action_node2)

        action_node3 = move_right(current_node, obs_space)
        if action_node3 != None:
            next_nodes.append(action_node3)

        action_node4 = move_left(current_node, obs_space)
        if action_node4 != None:
            next_nodes.append(action_node4)

        action_node5 = move_up_right(current_node, obs_space)
        if action_node5 != None:
            next_nodes.append(action_node5)

        action_node6 = move_up_left(current_node, obs_space)
        if action_node6 != None:
            next_nodes.append(action_node6)

        action_node7 = move_down_right(current_node, obs_space)
        if action_node7 != None:
            next_nodes.append(action_node7)

        action_node8 = move_down_left(current_node, obs_space)
        if action_node8 != None:
            next_nodes.append(action_node8)

        for next_node in next_nodes:
            if next_node[1] not in close_list:
                if next_node[1] not in [x[1] for x in open_list.queue]:
                    parent_map[next_node[1]] = current_node[1]
                    open_list.put(tuple(next_node))
                
                else:
                    for node in open_list.queue:
                        if node[1] == next_node[1] and node[0] > next_node[0]:
                            open_list.queue.remove(node)
                            parent_map[next_node[1]] = current_node[1]
                            open_list.put(tuple(next_node))

# performing backtracking based on parent map
def back_tracking(parent_map, start, goal):
    path = []
    current_node = goal

    while current_node != start:
        path.append(current_node)
        current_node = parent_map[current_node]

    path.append(start)
    path.reverse()
    return path

# function for asking for user input of start and goal nodes
def user_input(obs_space):
    while True:
        user_input_start = input("Enter the coordinates of the  start node as (X, Y): ")
        user_input_goal = input("Enter the coordinates of the  goal node as (X, Y): ")

        try:
            start_state = tuple(map(int, user_input_start.strip().split()))
            goal_state = tuple(map(int, user_input_goal.strip().split()))
            
            if (start_state[0] not in range(0, obs_space.shape[0])) and (start_state[1] not in range(0, obs_space.shape[1])):
                raise ValueError("Invalid start node.")
            
            if (obs_space[start_state[1], start_state[0]] == 1 or obs_space[start_state[1], start_state[0]] == 2):
                raise ValueError("Invalid start node.")
            
            if (goal_state[0] not in range(0, obs_space.shape[0])) and (goal_state[1] not in range(0, obs_space.shape[1])):
                raise ValueError("Invalid goal node.")
            
            if (obs_space[goal_state[1], goal_state[0]] == 1 or obs_space[goal_state[1], goal_state[0]] == 2):
                raise ValueError("Invalid goal node.")
            
            return start_state, goal_state
        
        except ValueError as e:
            print(e)
            continue

if __name__ == '__main__':
	
    # creating configuration space
    obs_space = config_space()
    # taking input for start and goal
    start_point, goal_point = user_input(obs_space)
    # creating opencv video writing objects
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('dijkstra_rishie_raj.mp4', fourcc, 200.0, (1200, 500))

    # timer object to measure computation time
    timer_start = time.time()

    # implementing dijkstra
    optimal_path, visit_map = dijkstra(start_point, goal_point, obs_space)

    clearance = 5

    # creating visualization canvas using opencv
    canvas = np.ones((500, 1200, 3), dtype=np.uint8) * 255

    cv2.line(canvas, pt1=(0, 500 - 0), pt2=(1200, 500 - 0), color=(0, 0, 0), thickness=1)
    cv2.line(canvas, pt1=(0, 500 - 0), pt2=(0, 500 - 500), color=(0, 0, 0), thickness=1)
    cv2.line(canvas, pt1=(1200, 500 - 0), pt2=(1200, 500 - 500), color=(0, 0, 0), thickness=1)
    cv2.line(canvas, pt1=(0, 500 - 500), pt2=(1200, 500 - 500), color=(0, 0, 0), thickness=1)

    cv2.line(canvas, pt1=(0, 500 - 3), pt2=(1200, 500 - 3), color=(255, 0, 0), thickness=5)
    cv2.line(canvas, pt1=(3, 500 - 0), pt2=(3, 500 - 500), color=(255, 0, 0), thickness=5)
    cv2.line(canvas, pt1=(1197, 500 - 0), pt2=(1197, 500 - 500), color=(255, 0, 0), thickness=5)
    cv2.line(canvas, pt1=(0, 500 - 497), pt2=(1200, 500 - 497), color=(255, 0, 0), thickness=5)

    cv2.rectangle(canvas, pt1=(100 - clearance, 500 - (100 - clearance)), pt2=(175 + clearance, 500 - (500 + clearance)), color=(255, 0, 0), thickness=-1)
    cv2.rectangle(canvas, pt1=(100, 500 - 100), pt2=(175, 500 - 500), color=(0, 0, 0), thickness=-1)

    cv2.rectangle(canvas, pt1=(275 - clearance, 500 - (0 - clearance)), pt2=(350 + clearance, 500 - (400 + clearance)), color=(255, 0, 0), thickness=-1)
    cv2.rectangle(canvas, pt1=(275, 500 - 0), pt2=(350, 500 - 400), color=(0, 0, 0), thickness=-1)

    hexagon_clearance = np.array([[520 - clearance, 175 - clearance], [520 - clearance, 325 + clearance], [650 , 400 + clearance * np.sqrt(2)], 
                        [780 + clearance, 325 + clearance], [780 + clearance, 175 - clearance], [650, 100 - clearance * np.sqrt(2)]], dtype=np.int32)
    cv2.fillPoly(canvas, [hexagon_clearance], (255, 0, 0))
    hexagon = np.array([[520, 175], [520, 325], [650 , 400], 
                        [780, 325], [780, 175], [650, 100]], dtype=np.int32)
    cv2.fillPoly(canvas, [hexagon], (0, 0, 0))

    polygon_clearance = np.array([[900 - clearance, 375 - clearance], [900 - clearance, 450 + clearance], [1100 + clearance, 450 + clearance], 
                        [1100 + clearance, 50 - clearance], [900 - clearance, 50 - clearance], [900 - clearance, 125 + clearance], 
                        [1020 - clearance, 125 + clearance], [1020 - clearance, 375 - clearance]], dtype=np.int32)
    cv2.fillPoly(canvas, [polygon_clearance], (255, 0, 0))  
    polygon = np.array([[900, 375], [900, 450], [1100, 450], 
                        [1100, 50], [900, 50], [900, 125], 
                        [1020, 125], [1020, 375]], dtype=np.int32)
    cv2.fillPoly(canvas, [polygon], (0, 0, 0))

    timer_stop = time.time()
    c_time = timer_stop - timer_start
    print("Total Runtime: ", c_time)

    # creating a visualization window
    cv2.namedWindow('Optimal Path Animation', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Optimal Path Animation', 1200, 500)

    count = 0

    # displaying node exploration
    for key in visit_map.keys():
        adjusted_point = (key[0], 500 - key[1])

        cv2.circle(canvas, adjusted_point, 1, (0, 255, 0), -1)

        # skipping frames for faster visualization
        if count % 100 == 0:
            cv2.imshow('Optimal Path Animation', canvas)
            cv2.waitKey(int(0.001 * 1000))
            out.write(canvas)
        
        count += 1

    # displaying optimal path
    for point in optimal_path:
        adjusted_point = (point[0], 500 - point[1])

        cv2.circle(canvas, adjusted_point, 1, (0, 0, 255), -1)
        
        cv2.imshow('Optimal Path Animation', canvas)
        cv2.waitKey(int(0.001 * 1000))
        out.write(canvas)

    # holding final frame till any key is pressed
    cv2.waitKey(0)
    # releasing video write object
    out.release()
    # destroying visualization window after keypress
    cv2.destroyAllWindows()
