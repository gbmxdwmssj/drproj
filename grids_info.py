import sys
import numpy as np
from grid_info import GridInfo

class GridsInfo(object):
    ''' The matrix of grid information.

    Arguments
    ---------
        max_x: The width of matrix, from left to right.

        max_y: The height of matrix, from down to up.
    '''


    def __init__(self, max_x, max_y):
        self.mat = np.zeros((max_x, max_y), dtype=GridInfo)
        for x in range(max_x):
            for y in range(max_y):
                self.mat[x][y] = GridInfo()



    def get_min_id(self, open_list):
        '''
        Arguments
        ---------
            open_list (list): The element in list is grid (tuple).

        Returns
        -------
            min_id (int): The index of the grid with the minimum f cost.
                          Return -1 if open_list is empty.
        '''
        min_id = -1
        min_f_cost = sys.float_info.max
        for idx, grid in enumerate(open_list):
            f_cost = self.get_f_cost(grid[0], grid[1])
            if f_cost < min_f_cost:
                min_id = idx
                min_f_cost = f_cost

        return min_id



    def get_g_cost(self, x, y):
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        return self.mat[x][y].g_cost



    def set_g_cost(self, x, y, g_cost):
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        self.mat[x][y].g_cost = g_cost



    def get_h_cost(self, x, y):
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        return self.mat[x][y].h_cost



    def set_h_cost(self, x, y, h_cost):
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        self.mat[x][y].h_cost = h_cost



    def get_f_cost(self, x, y):
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        return self.mat[x][y].g_cost + self.mat[x][y].h_cost



    def get_predecessor(self, x, y):
        '''
        Returns
        -------
            _ (tuple): The predecessor grid.
        '''
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        return self.mat[x][y].predecessor



    def set_predecessor(self, x, y, predecessor):
        '''
        Arguments
        ---------
        predecessor (tuple): The predecessor grid.
        '''
        max_x = self.mat.shape[0]
        max_y = self.mat.shape[1]
        x = np.clip(x, 0, max_x-1)
        y = np.clip(y, 0, max_y-1)
        self.mat[x][y].predecessor = predecessor
