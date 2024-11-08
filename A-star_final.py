import requests
import numpy as np
from collections import deque
import time

token = "your token"
base_url = "http://127.0.0.1:8801/api/v1"

maze_array = np.zeros((16, 16), dtype=[
    ('visited', bool),
    ('f', int),
    ('g', int),
    ('h', int)
])

h_values = [
    [14, 13, 12, 11, 10,  9,  8,  7,  7,  8,  9, 10, 11, 12, 13, 14],
    [13, 12, 11, 10,  9,  8,  7,  6,  6,  7,  8,  9, 10, 11, 12, 13],
    [12, 11, 10,  9,  8,  7,  6,  5,  5,  6,  7,  8,  9, 10, 11, 12],
    [11, 10,  9,  8,  7,  6,  5,  4,  4,  5,  6,  7,  8,  9, 10, 11],
    [10,  9,  8,  7,  6,  5,  4,  3,  3,  4,  5,  6,  7,  8,  9, 10],
    [ 9,  8,  7,  6,  5,  4,  3,  2,  2,  3,  4,  5,  6,  7,  8,  9],
    [ 8,  7,  6,  5,  4,  3,  2,  1,  1,  2,  3,  4,  5,  6,  7,  8],
    [ 7,  6,  5,  4,  3,  2,  1,  0,  0,  1,  2,  3,  4,  5,  6,  7],
    [ 7,  6,  5,  4,  3,  2,  1,  0,  0,  1,  2,  3,  4,  5,  6,  7],
    [ 8,  7,  6,  5,  4,  3,  2,  1,  1,  2,  3,  4,  5,  6,  7,  8],
    [ 9,  8,  7,  6,  5,  4,  3,  2,  2,  3,  4,  5,  6,  7,  8,  9],
    [10,  9,  8,  7,  6,  5,  4,  3,  3,  4,  5,  6,  7,  8,  9, 10],
    [11, 10,  9,  8,  7,  6,  5,  4,  4,  5,  6,  7,  8,  9, 10, 11],
    [12, 11, 10,  9,  8,  7,  6,  5,  5,  6,  7,  8,  9, 10, 11, 12],
    [13, 12, 11, 10,  9,  8,  7,  6,  6,  7,  8,  9, 10, 11, 12, 13],
    [14, 13, 12, 11, 10,  9,  8,  7,  7,  8,  9, 10, 11, 12, 13, 14]
]

for i in range(16):
    for j in range(16):
        maze_array[i, j]['h'] = h_values[i][j]

path_q = deque()
back_q = deque()
open_list = []

start_pos = (0, 0)
goal_positions = [(7,7), (7,8), (8,7), (8,8)]

maze_array[start_pos]['visited'] = True
maze_array[start_pos]['g'] = 0
maze_array[start_pos]['f'] = maze_array[start_pos]['h']

def normalize_offset(offset, min_val=-1300, max_val=1300, grid_size=16):
    normalized_value = (offset - min_val) / (max_val - min_val) * (grid_size - 1)
    return int(np.clip(round(normalized_value), 0, grid_size - 1))

def check_and_mark_visited(x, y):
    if not maze_array[15 - y, x]['visited']:
        maze_array[15 - y, x]['visited'] = True
        return True
    return False

def get_orientation(yaw):
    if -45 <= yaw < 45:
        return "forward"
    elif 45 <= yaw < 135:
        return "right"
    elif 135 <= yaw <= 180 or -180 <= yaw < -135:
        return "back"
    elif -135 <= yaw < -45:
        return "left"
    else:
        raise ValueError(f"Недопустимое значение yaw: {yaw}")

def get_next_position(x, y, orientation, direction):
    if orientation == "forward":
        if direction == "forward": return x, y+1
        elif direction == "right": return x+1, y
        elif direction == "left": return x-1, y
        elif direction == "backward": return x, y-1
    elif orientation == "right":
        if direction == "forward": return x+1, y
        elif direction == "right": return x, y-1
        elif direction == "left": return x, y+1
        elif direction == "backward": return x-1, y
    elif orientation == "back":
        if direction == "forward": return x, y-1
        elif direction == "right": return x-1, y
        elif direction == "left": return x+1, y
        elif direction == "backward": return x, y+1
    elif orientation == "left":
        if direction == "forward": return x-1, y
        elif direction == "right": return x, y+1
        elif direction == "left": return x, y-1
        elif direction == "backward": return x+1, y
    return x, y

def check_sensors(x, y, orientation, sensors, threshold=70):
    directions = ['forward', 'right', 'left', 'backward']
    possible_moves = []
    for direction, sensor_value in zip(directions, sensors):
        if sensor_value >= threshold:
            next_x, next_y = get_next_position(x, y, orientation, direction)
            if 0 <= next_x < 16 and 0 <= next_y < 16 and not maze_array[15 - next_y, next_x]['visited']:
                possible_moves.append((direction, next_x, next_y))
    return possible_moves

def update_maze_map():
    url = f"{base_url}/robot-cells/sensor-data"
    params = {"token": token}
    try:
        response = requests.get(url, params=params)
        if response.status_code == 200:
            data = response.json()
            y = normalize_offset(data["down_x_offset"])
            x = normalize_offset(data["down_y_offset"])
            orientation = get_orientation(data["rotation_yaw"])
            sensors = [data["front_distance"], data["right_side_distance"], 
                       data["left_side_distance"], data["back_distance"]]
            check_and_mark_visited(x, y)
            possible_moves = check_sensors(x, y, orientation, sensors)
            time.sleep(0.2)
            return x, y, orientation, possible_moves
        else:
            return None
    except Exception as e:
        return None

def perform_action(action):
    url = f"{base_url}/robot-cells/{action}"
    params = {"token": token}
    try:
        response = requests.post(url, params=params)
        time.sleep(0.25)
        return response.status_code == 200
    except Exception as e:
        return False

def standard_procedure(x, y, orientation, possible_moves):
    global open_list, path_q
    for direction, next_x, next_y in possible_moves:
        if maze_array[15 - next_y, next_x]['g'] == 0:
            maze_array[15 - next_y, next_x]['g'] = maze_array[15 - y, x]['g'] + 1
        maze_array[15 - next_y, next_x]['f'] = maze_array[15 - next_y, next_x]['g'] + maze_array[15 - next_y, next_x]['h']
        open_list.append((next_x, next_y))
    
    if open_list:
        next_move = min(open_list, key=lambda pos: maze_array[15 - pos[1], pos[0]]['f'])
        dx, dy = next_move[0] - x, next_move[1] - y
        
        action = get_action(dx, dy, orientation)
        path_q.append(action)
        
        open_list.clear()
        
        if perform_action(path_q[-1]):
            return True
    return False

def get_action(dx, dy, orientation):
    if orientation == "forward":
        if (dx, dy) == (0, 1): return "forward"
        elif (dx, dy) == (0, -1): return "backward"
        elif (dx, dy) == (1, 0): return "right"
        elif (dx, dy) == (-1, 0): return "left"
    elif orientation == "right":
        if (dx, dy) == (0, 1): return "left"
        elif (dx, dy) == (0, -1): return "right"
        elif (dx, dy) == (1, 0): return "forward"
        elif (dx, dy) == (-1, 0): return "backward"
    elif orientation == "back":
        if (dx, dy) == (0, 1): return "backward"
        elif (dx, dy) == (0, -1): return "forward"
        elif (dx, dy) == (1, 0): return "left"
        elif (dx, dy) == (-1, 0): return "right"
    elif orientation == "left":
        if (dx, dy) == (0, 1): return "right"
        elif (dx, dy) == (0, -1): return "left"
        elif (dx, dy) == (1, 0): return "backward"
        elif (dx, dy) == (-1, 0): return "forward"
    return None

def non_standard_procedure():
    global path_q, back_q
    
    if path_q:
        last_action = path_q.pop()
        inverted_action = invert_action(last_action)
        back_q.append(inverted_action)
        
        if perform_action(inverted_action):
            back_q.pop()
            return True
    
    return False

def invert_action(action):
    return {
        "forward": "backward",
        "backward": "forward",
        "left": "right",
        "right": "left"
    }[action]

def explore_maze():
    global path_q, back_q, open_list
    
    while True:
        current_state = update_maze_map()
        if current_state is None:
            return
        
        x, y, orientation, possible_moves = current_state
        
        if (x, y) in goal_positions:
            return path_q
        
        if possible_moves:
            if standard_procedure(x, y, orientation, possible_moves):
                continue
        else:
            while not possible_moves:
                if not non_standard_procedure():
                    return None
                
                current_state = update_maze_map()
                if current_state is None:
                    return None
                x, y, orientation, possible_moves = current_state
            
            standard_procedure(x, y, orientation, possible_moves)

def restart_robot():
    url = f"{base_url}/maze/restart"
    params = {"token": token}
    try:
        response = requests.post(url, params=params)
        if response.status_code == 200:
            time.sleep(0.5)
            return True
        return False
    except Exception as e:
        return False

def main():
    saved_path = explore_maze()
    if saved_path is None:
        return

    for pass_number in range(2, 4):
        if not restart_robot():
            return

        for action in saved_path:
            if not perform_action(action):
                return
        
        current_state = update_maze_map()
        if current_state and (current_state[0], current_state[1]) in goal_positions:
            pass
        else:
            pass

if __name__ == "__main__":
    main()