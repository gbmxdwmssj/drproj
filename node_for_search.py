class NodeForSearch(object):
    ''' Node (element) used for A* planning

    Arguments
    ---------
        g_cost (float): The g cost of node.

        h_cost (float): The h (heuristic) cost of node.

        x (int): From left to right.

        y (int): From down to up.

        state: The continuous state.

        pred (NodeForSearch): The predecessor of the node.
    '''



    def __init__(self, g_cost, h_cost, x, y, state=None, pred=None):
        # Member
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.x = x
        self.y = y
        self.state = state
        self.pred = pred

        self.successor_grids = []
        for ix in range(x-1, x+2):
            for iy in range(y-1, y+2):
                if ix < 0 or iy < 0: # Max should also be considered.
                    continue

                if ix == x and iy == y:
                    continue

                successor_grid = (ix, iy)
                self.successor_grids.append(successor_grid)



    def set_g_cost(self, g_cost):
        self.g_cost = g_cost



    def get_g_cost(self):
        return self.g_cost



    def set_h_cost(self, h_cost):
        self.h_cost = h_cost



    def get_h_cost(self):
        return self.h_cost



    def get_f_cost(self):
        return self.g_cost + self.h_cost



    def set_state(self, state):
        self.state = state



    def get_state(self):
        return self.state



    def print(self):
        print('Node information\n----------------\n({}, {})\nf cost: {:.3f}\n'.format(self.x,
            self.y, self.g_cost + self.h_cost))
