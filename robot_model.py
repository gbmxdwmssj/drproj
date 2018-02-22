class RobotModel(object):



    def __init__(self, radius):
        # Member
        self.radius = radius



    def get_controls(self, grid, grid_map):
        '''
        Returns
        -------
            controls (list): Each control is a tuple.
        '''
        if grid[0] < 0 or grid[0] > grid_map.max_x - 1 or grid[1] < 0 or grid[1] > grid_map.max_y - 1:
            return []

        controls = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue

                x = grid[0] + dx
                y = grid[1] + dy
                if x < 0 or x > grid_map.max_x - 1 or y < 0 or y > grid_map.max_y - 1:
                    continue

                controls.append((dx, dy))

        return controls