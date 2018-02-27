from sortedcontainers import SortedListWithKey

def identity(value):
    "Identity function."
    return value

class HeapForSearch(SortedListWithKey):
    ''' SortedListWithKey used for A* planning.

    See: http://www.grantjenks.com/docs/sortedcontainers/sortedlistwithkey.html
         https://www.programcreek.com/python/example/90730/sortedcontainers.SortedListWithKey

    Arguments
    ---------
        iterable (list): An initial series of items to populate the SortedListWithKey.

        key (function or lambda expression): A callable object used for comparison.

        load (int): The load-factor of the list. The default load factor of ‘1000’ works
                    well for lists from tens to tens of millions of elements. Good practice
                    is to use a value that is the square or cube root of the list size.
    '''



    def __init__(self, iterable=None, key=identity):
        ''' Initialization
        '''
        # Member
        pass

        # Initialization
        super(HeapForSearch, self).__init__(iterable=iterable, key=key)



    def has(self, x, y):
        ''' Whether the heap has the specific grid at (x, y)

        Arguments
        ---------
            x (int): From left to right.

            y (int): From down to up.

        Returns
        -------
            _ (bool): True if the heap has the specific grid at (x, y).
        '''
        for element in iter(self):
            if x == element.x and y == element.y:
                return True

        return False
