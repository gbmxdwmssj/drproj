import math
import matplotlib.pyplot as plt
from robot_model import RobotModel
from grids_info import GridsInfo

class AStarPlanner(object):
    ''' Traditional A* planner.

    See: https://github.com/karlkurzer/path_planner

    Arguments
    ---------
        robot_model (RobotModel): The robot model used for planning.
    '''


    def __init__(self, robot_model):
        # Member
        self.path = None
        self.open_list = None
        self.close_list = None
        self.grids_info = None
        self.robot_model = robot_model



    def ready_for_search(self, start_grid, grid_map):
        # Initialize the path
        self.path = []

        # Initialize open list and close list.
        self.open_list = []
        self.open_list.append(start_grid)
        self.close_list = []

        # Initialize all grids' information.
        self.grids_info = GridsInfo(grid_map.max_x, grid_map.max_y)
        self.grids_info.set_g_cost(start_grid[0], start_grid[1], 0.0)



    def collision(self, grid, grid_map):
        plt.figure(figsize=(5,5))
        radius_in_grid = math.ceil(self.robot_model.radius / grid_map.resolution)
        for dx in range(-radius_in_grid, radius_in_grid+1):
            for dy in range(-radius_in_grid, radius_in_grid+1):
                ddis_in_grid = dx**2 + dy**2
                if ddis_in_grid > radius_in_grid**2:
                    continue

                # Here is grid within robot model.
                x = grid[0] + dx
                y = grid[1] + dy
                plt.plot(x, y, 'ro', label='point')

        plt.show()
        return False



    def get_transition_cost(self, grid, u, grid_map):
        succ_grid = (grid[0]+u[0], grid[1]+u[1])
        if self.collision(succ_grid, grid_map):
            return float('inf')
        else:
            return math.sqrt(u[0]**2 + u[1]**2)



    def get_heuristic(self, grid, goal_grid):
        dx = goal_grid[0] - grid[0]
        dy = goal_grid[1] - grid[1]
        return math.sqrt(dx**2 + dy**2)



    def trace_back(self, grid):
        path = [grid]
        pred = self.grids_info.get_predecessor(grid[0], grid[1])
        while pred is not None:
            path.insert(0, pred)
            pred = self.grids_info.get_predecessor(pred[0], pred[1])

        return path



    def get_path(self, start_grid, goal_grid, grid_map):
        ''' Get path by traditional A* search.

        Arguments
        ---------
            start_grid (tuple): The start grid.

            goal_grid (tuple): The goal grid.

            grid_map (GridMap): The gird map.

        Returns
        -------
            _ (DrPath): The planned path.
        '''
        self.ready_for_search(start_grid, grid_map)

        # Search
        while len(self.open_list) != 0:
            # Pop the grid with minimum f cost.
            min_id = self.grids_info.get_min_id(self.open_list)
            grid = self.open_list.pop(min_id)

            # Push it into close list.
            self.close_list.append(grid)

            # If the goal grid is pushed into close list,
            # the path is found.
            if grid[0] == goal_grid[0] and grid[1] == goal_grid[1]:
                self.path = self.trace_back(grid)
                return self.path

            # For each possible control.
            for u in self.robot_model.get_controls(grid, grid_map):
                succ_grid = (grid[0]+u[0], grid[1]+u[1])
                if succ_grid in self.close_list:
                    continue

                g_cost = self.grids_info.get_g_cost(grid[0], grid[1]) + self.get_transition_cost(grid, u, grid_map)
                if succ_grid in self.open_list and g_cost >= self.grids_info.get_g_cost(succ_grid[0], succ_grid[1]):
                    continue

                # Update succ_grid's information.
                self.grids_info.set_predecessor(succ_grid[0], succ_grid[1], grid)
                self.grids_info.set_g_cost(succ_grid[0], succ_grid[1], g_cost)
                self.grids_info.set_h_cost(succ_grid[0], succ_grid[1], self.get_heuristic(succ_grid, goal_grid))
                if succ_grid not in self.open_list:
                    self.open_list.append(succ_grid)

        self.path.append(start_grid)
        return self.path