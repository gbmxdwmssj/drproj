class DrPath(object):
    ''' Class path used in drproj.

    Members
    -------
    poses (list): The start pose has index zero.
    '''



    def __init__(self):
        # Member
        self.poses = []



    def add_before_start(self, pose):
        ''' Add the specific pose before the start pose of the path.
        '''
        self.poses.insert(0, pose)



    def len(self):
        return len(self.poses)