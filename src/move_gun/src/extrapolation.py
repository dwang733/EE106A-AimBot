from collections import OrderedDict
import numpy as np
import time

# Just Learning about OrderedDict and random test stuff
# a = OrderedDict()
# a[1] = (1,2,3)
# a[4] = (4,5,6)
# a[2] = (7,8,9)
# print(a.items())
# print(a.keys())
# print(list(a.values()))
# print(len(a))
# print(a.values())
# print(a.popitem(False))
# print(np.array(list(a.values())))
# print(np.array(list(a.keys())))
# p = np.polyfit(np.array(list(a.keys())), np.array(list(a.values())), 1)
# print(p)
# print(p[:,0])

# The actual class
class ExtrapolationQueue():
    """Data is Vector3(). Find x, y, z by:
    vec = Vector3()
    vec.x, vec.y, vec.z"""
    def __init__(self, buff_size, deg=2):
        """ Initializes the cache, max buffer size, and degree of interpolation """
        self.cache = OrderedDict()
        self.max_size = buff_size
        self.len = len(self.cache)
        self.deg = deg
        self.counter = 0

    def size(self):
        """ Returns the current size of the cache """
        return self.len

    def push(self, time, pos):
        """
        Pushes a new position data point to the Queue and pops out the oldest position data point
        if the cache size becomes larger than the max buffer size

        Inputs:
        time - time since start in seconds
        pos - Vector3() position coordinates
        -------------------------------------
        Outputs:
        None
        """
        if self.len + 1 > self.max_size:
            self.pop()
            self.len-=1
        self.cache[self.counter] = (time, pos.x, pos.y, pos.z)
        self.counter = (self.counter + 1)%self.max_size
        self.len+=1
            

    def pop(self):
        """
        Pops off the oldest data point. Returns the oldest data point or None if cache is empty.

        Inputs:
        None
        ----------------
        Outputs:
        data - tuple with first element the time and second element a tuple of the oldest data point that has been popped off
        """
        if self.len > 0:
            data = self.cache.popitem(False)
            return data
        else:
            return None

    def interpolate(self, deg):
        """
        Interpolates x, y, and z trajectories individually.

        Inputs:
        deg - degree of interpolation
        -------------------------------
        Outputs:
        p - interpolation matrix which contains coefficients of the interpolated polynomial. Shape = (self.size, 3)
            coefficients for x trajectory = p[:,0]
            coefficients for y trajectory = p[:,1]
            coefficients for z trajectory = p[:,2]
        """
        p = np.polyfit(np.array(list(self.cache.values()))[:,0], np.array(list(self.cache.values()))[:,1:], deg)
        return p

    def extrapolate(self, time):
        """
        Extrapolates the trajectory of the target in 3D space and estimates position of target at desired future time

        Inputs:
        time - desired future time since start time in seconds
        ----------------------------------------------------------
        Outputs:
        data - extrapolated [x,y,z] position in 3D space. This is a regular python list.
        """
        p = self.interpolate(self.deg)  #this take the most time
        data = []
        t = np.array([time**(p.shape[0]-1-i) for i in range(p.shape[0])])
        for d in range(3):
            data.append(np.dot(p[:,d], t))
        return data

# class Vector3():
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z
# e = ExtrapolationQueue(5)
# e.push(1, Vector3(4, 4, 4))
# e.push(3, Vector3(0, 0, 0))
# e.push(4, Vector3(1, 1, 1))
# e.push(10, Vector3(49, 49, 49))
# e.push(-5, Vector3(64, 64, 64))
# print(e.cache)
# print(e.size())
# e.push(-1, Vector3(16, 16, 16))
# print(e.cache)
# print(e.size())
# start = time.time()
# print(e.extrapolate(7))
# print(time.time() - start)