#!/usr/bin/env python
# -*- coding: utf-8 -*-
#__author__ = 'ar1'

from math import sqrt

class Vector3:
   def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

   def minus(self, vect):
        res = Vector3()
        res.x = self.x - vect.x
        res.y = self.y - vect.y
        res.z = self.z - vect.z

        return res
    
   def add(self, vect):
        res = Vector3()
        res.x = self.x + vect.x
        res.y = self.y + vect.y
        res.z = self.z + vect.z

        return res
	
# calculates the cross product: self x vect
   def multiply(self, vect):
        res = Vector3()
        res.x = self.y * vect.z - self.z * vect.y
        res.y = self.z * vect.x - self.x * vect.z
        res.z = self.x * vect.y - self.y * vect.x

        return res
# calculates the cross product: self x vect
   def dot(self, vect):
        return self.x * vect.x + self.y * vect.y + self.z * vect.z

    # calculates the length of vector self
   def length(self):
        return sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    # calculates the unit vector of vector self
   def unit_vector(self):
        n = 1.0 / self.length()
        return Vector3(x=self.x * n, y=self.y * n, z=self.z * n)
