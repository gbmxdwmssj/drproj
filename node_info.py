import sys

class NodeInfo(object):
    ''' The infomation about node in A* search.

    Members
    -------
        g_cost: The g cost of node.
        
        h_cost: The h (heuristic) cost of node.

        vehicle_state: The continuous vehicle state associated with node.

        predecessor (tuple): The predecessor grid of node.
    '''



    def __init__(self):
        # Member
        self.g_cost = sys.float_info.max
        self.h_cost = 0.0
        self.vehicle_state = None
        self.predecessor = None



    def print(self):
        print('Node information\n----------------\nf cost: {:.3f}'.format(self.g_cost + self.h_cost))