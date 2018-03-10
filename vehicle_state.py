class VehicleState(object):
    ''' Continuous vehicle state.

    Arguments
    ---------
        x (flaot): From left to right (m).

        y (flaot): From down to up (m).

        yaw (flaot): Down to up is zero. Left negative and right positive, so [-180.0, 180.0) (degree).
    '''



    def __init__(self, x, y, yaw,
            v=0.0, w=0.0,
            a=0.0, ayaw=0.0,
            steer=0.0, vsteer=0.0,
            roll=0.0, vroll=0.0,
            pitch=0.0, vpitch=0.0):
        # Member
        self.x = x
        self.y = y
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch
