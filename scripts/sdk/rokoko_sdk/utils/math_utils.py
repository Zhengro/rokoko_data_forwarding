#!/usr/bin/env python

class Vector3D(object):
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = x, y, z
   
    def __str__(self):
        return '(%s, %s, %s)' % (self.x, self.y, self.z)

class Quaternion(object):
    def __init__(self, w=0, x=0, y=0, z=0):
        self.w, self.x, self.y, self.z = w, x, y, z

    def __str__(self):
        return '(%s, %s, %s, %s)' % (self.x, self.y, self.z, self.w)
