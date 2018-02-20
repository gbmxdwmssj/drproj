import numpy as np
from node_info import NodeInfo

class NodesInfo(object):
    ''' The matrix of node information.

    Arguments
    ---------
        max_x: The width of matrix, from left to right.

        max_y: The height of matrix, from down to up.
    '''


    def __init__(self, max_x, max_y):
        self.mat = np.zeros((max_x, max_y), dtype=NodeInfo)
        for x in range(max_x):
            for y in range(max_y):
                self.mat[x][y] = NodeInfo()



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