class NodeForSearch(object):
    ''' Node (element) used for A* planning

    Arguments
    ---------
        f_cost (float): The f cost of node, which is the sum of g cost and h (heuristic) cost.

        x (int): From left to right.

        y (int): From down to up.

        vehicle_state (VehicleState): The continuous vehicle state.
    '''



    def __init__(self, f_cost, x, y, vehicle_state=None):
        pass