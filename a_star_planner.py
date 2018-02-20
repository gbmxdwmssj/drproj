from heap_for_search import HeapForSearch
from node_for_search import NodeForSearch
from dr_path import DrPath
from dr_pose import DrPose
import math

class AStarPlanner(object):
    ''' Hybrid state A* planner.

    See: https://github.com/karlkurzer/path_planner
    '''


    def __init__(self):
        # Member
        pass



    def get_path(self, start, goal, grid_map):
        ''' Get path by A* search.

        Arguments
        ---------
            start (VehicleState): The start vehicle state.

            goal (VehicleState): The goal vehicle state.

            gird_map (GridMap): The gird map.

        Returns
        -------
            path (DrPath): The planned path.
        '''
        # Initialize the path
        path = DrPath()

        # Initialize open list and close list.
        open_list = HeapForSearch(key=lambda x: x.get_f_cost())
        close_list = HeapForSearch(key=lambda x: x.get_f_cost())

        # Initialize start node.
        dx = goal.x - start.x
        dy = goal.y - start.y
        start_to_goal = math.sqrt(dx**2 + dy**2)
        int_x = int(start.x / grid_map.resolution)
        int_y = int(start.y / grid_map.resolution)
        start_node = NodeForSearch(0.0, start_to_goal, int_x, int_y)
        open_list.add(start_node)

        # Search
        while len(open_list) != 0:
            node = open_list.pop(0)
            close_list.add(node)
            if node.x == goal.x and node.y == goal.y:
                return path

            for grid in node.successor_grids:
                if close_list.has(grid.x, grid.y):
                    continue

                pass

        start_pose = DrPose(start.x, start.y)
        path.add_before_start(start_pose)
        return path