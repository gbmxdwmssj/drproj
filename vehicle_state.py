class VehicleState(object):
    ''' Continuous vehicle state.
    '''



    def __init__(self, x, y, yaw,
            vx=0.0, vy=0.0, w=0.0,
            ax=0.0, ay=0.0, ayaw=0.0,
            steer=0.0, vsteer=0.0,
            roll=0.0, vroll=0.0,
            pitch=0.0, vpitch=0.0):
        # Member
        self.x = x
        self.y = y
        self.yaw = yaw