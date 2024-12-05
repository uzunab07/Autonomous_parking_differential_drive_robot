import heapq
from parking import sample_environment
import argparse

class AStar:
    def __init__(self, parking_lot, points):
        self.parking_lot = parking_lot
        
        # check bounds
        for point in points:
            if not (0 <= point[0] < self.parking_lot.rows and 0 <= point[1] < self.parking_lot.cols):
                raise ValueError(f'Point {point} is out of bounds.')
        
        self.start = points[0]
        self.start_parking_spot = None
        if self.parking_lot.grid[self.start[0]][self.start[1]]['type'] == 'parking_spot':
            self.start_parking_spot = self.start
            self.start = self.translate_to_lane(self.start)
        
        # validate start
        # if self.parking_lot.grid[self.start[0]][self.start[1]]['type'] != 'lane':
        #     self.start_lane = self.translate_to_lane(self.start)
        
        self.goals = points[1:]
        
        # validate goal points
        for goal in self.goals:
            if self.parking_lot.grid[goal[0]][goal[1]]['type'] != 'parking_spot':
                raise ValueError(f'Goal point {goal} is not a parking spot.')
        
        # direction lookup to help calculations
        self.directions = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}
    
    # translates a parking spot to the lane it must be accessed from
    def translate_to_lane(self, point):
        parking_spot = self.parking_lot.grid[point[0]][point[1]]
        entrance_direction = parking_spot['entrance']
        
        if entrance_direction == 'N':
            return (point[0] - 1, point[1])
        elif entrance_direction == 'S':
            return (point[0] + 1, point[1])
        elif entrance_direction == 'E':
            return (point[0], point[1] + 1)
        elif entrance_direction == 'W':
            return (point[0], point[1] - 1)


    # returns manhattan distance from a point to the goal
    def heuristic(self, point, goal):
        row, col = point
        return abs(row - goal[0]) + abs(col - goal[1])

    # checks if a move to (row, col) in the direction specified is valid
    def is_valid_move(self, point, direction):
        # condition 1: check bounds of (row, col)
        if not (0 <= point[0] < self.parking_lot.rows and 0 <= point[1] < self.parking_lot.cols):
            return False
        cell = self.parking_lot.grid[point[0]][point[1]]
        
        # condition 2: we cannot move through parking spots
        if cell['type'] == 'parking_spot':
            return False

        # condition 3: check if we moved into this point via one of its allowed directions
        # for example, if we moved North, then the point we arrive at must have 'N' in its list of directions
        return direction in cell['directions']

    # find a path to the goal using A*
    def find_path(self, goal):
        # since parking spot and lane cells are structured differently, it's simpler to just path find to the lane in front of the goal
        goal_lane = self.translate_to_lane(goal)
        
        open_list = []
        # (heuristic, cost, cell)
        heapq.heappush(open_list, (self.heuristic(self.start, goal_lane), 0, self.start))
        came_from = {}
        g_costs = {self.start: 0}

        while open_list:
            _, cost, current = heapq.heappop(open_list)

            # check if we've reached the goal
            if current == goal_lane:
                path = self.reconstruct_path(came_from, current, goal)
                return path

            for direction_key, direction in self.directions.items():
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                if self.is_valid_move(neighbor, direction_key):
                    new_cost = cost + 1
                    if neighbor not in g_costs or new_cost < g_costs[neighbor]:
                        g_costs[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal_lane)
                        heapq.heappush(open_list, (priority, new_cost, neighbor))
                        came_from[neighbor] = current
                        
        return None

    # construct the path as a list of tuples
    def reconstruct_path(self, came_from, current, goal):
        path = []
        while current in came_from:
            prev = came_from[current]
            path.append(current)
            current = prev
            
        # add the start and goal manually
        path.append(self.start)
        if self.start_parking_spot:
            path.append(self.start_parking_spot)
        path.reverse()
        path.append(goal)
        return path

    # converts a path to a list of directions
    def export_path(self, path):
        directions = []
        for i in range(1, len(path)):
            current = path[i - 1]
            next_pos = path[i]
            row_diff = next_pos[0] - current[0]
            col_diff = next_pos[1] - current[1]
            if row_diff == 1:
                directions.append('down')
            elif row_diff == -1:
                directions.append('up')
            elif col_diff == 1:
                directions.append('right')
            elif col_diff == -1:
                directions.append('left')
        return directions

# parse passed-in points. must be of the format (x, y), where x and y are integers
def parse_points(points):
    parsed_points = []
    for point in points:
        try:
            x, y = map(int, point.strip('()').split(','))
            parsed_points.append((x, y))
        except ValueError:
            raise ValueError(f"Invalid point format: {point}. Use '(x, y)' format.")
    return parsed_points

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('points', nargs='+')
    
    args = parser.parse_args()
    
    try:
        points = parse_points(args.points)
        start = points[0]
        
        parking_lot = sample_environment()
        parking_lot.display()
        pathfinder = AStar(parking_lot, points)
        
        shortest_path = None
        shortest_distance = float('inf')
        best_goal = None

        for goal in pathfinder.goals:
            path = pathfinder.find_path(goal)
            if path:
                distance = len(path)
                if distance < shortest_distance:
                    shortest_distance = distance
                    shortest_path = path
                    best_goal = goal

        if shortest_path:
            if len(points) > 2:
                print(f'The shortest path from {start} is to goal {best_goal}. Path found:')
            else:
                print(f'Shortest path found from {start} to goal {best_goal}:')
            for point in shortest_path:
                print(point)
            directions = pathfinder.export_path(shortest_path)
            print(f'Directions: {directions}')
        else:
            print('No path found to any goal.')
        
    except ValueError as e:
        print(e)

if __name__ == '__main__':
    main()