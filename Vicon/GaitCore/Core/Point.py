#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020, AIMGaitCore
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <nagoldfarb@wpi.edu>
#     \author    Nathaniel Goldfarb
#     \version   0.1
# */
# //==============================================================================


from __future__ import annotations

from typing import Sequence

import numpy as np


Number = float | int


class Point(object):

    def __init__(self, x: Number, y: Number, z: Number) -> None:
        self._x: float = float(x)
        self._y: float = float(y)
        self._z: float = float(z)


    @classmethod
    def from_points(cls, P1: Sequence[Number], P2: Sequence[Number]) -> "Point":
        return cls(P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2])

    @classmethod
    def new_point(cls) -> "Point":
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def from_array(cls, arr: Sequence[Number]) -> "Point":
        return cls(arr[0], arr[1], arr[2])

    def get_unit_vector(self) -> "Point":
        mag = self.get_magnitude()
        return Point(self.x, self.y, self.z) / mag

    def get_magnitude(self) -> float:
        return float(np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2))

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def z(self) -> float:
        return self._z

    @x.setter
    def x(self, value: Number) -> None:
        self._x = float(value)

    @y.setter
    def y(self, value: Number) -> None:
        self._y = float(value)

    @z.setter
    def z(self, value: Number) -> None:
        self._z = float(value)

    def __add__(self, other: "Point") -> "Point":
        """
        over ride to add points
        """
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        return Point(x, y, z)

    def __radd__(self, other: Number) -> "Point":

        x = self.x + other
        y = self.y + other
        z = self.z + other

        return Point(x, y, z)

    def __rsub__(self, other: Number) -> "Point":
        return self.__radd__(-other)

    def __sub__(self, other: "Point") -> "Point":
        """
        over ride to subtact points
        """
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z

        return Point(x, y, z)

    def __mul__(self, other: Number) -> "Point":
        """
        over ride to mul points
        """
        x = other * self.x
        y = other * self.y
        z = other * self.z
        return Point(x, y, z)

    def __rmul__(self, other: Number) -> "Point":
        return self.__mul__(other)


    def __truediv__(self, other: Number) -> "Point":
        x = self.x / other
        y = self.y / other
        z = self.z / other

        return Point(x, y, z)

    def __floordiv__(self, other: Number) -> "Point":
        x = self.x // other
        y = self.y // other
        z = self.z // other

        return Point(x, y, z)


    def dot(self, other: "Point") -> float:
        v1 = np.squeeze(other.toarray())
        v2 = np.squeeze(self.toarray())
        return float(np.dot(v1, v2))

    def __str__(self) -> str:
        return " X: " + str(self.x) + " Y: " + str(self.y) + " Z: " + str(self.z)

    def __abs__(self) -> "Point":
        return Point(abs(self.x), abs(self.y), abs(self.z))

    def toarray(self) -> np.ndarray:
        return np.array((self.x, self.y, self.z), dtype=float).reshape((-1, 1))


def distance(point1: Point, point2: Point) -> float:
    """
    get the distance between two points
    """
    ans = np.sum(np.power((point1 - point2).toarray(), 2))
    return float(np.sqrt(float(ans)))

def point_to_vector(point: Point) -> np.ndarray:
    """Returns a vectorized representation of a Point object. The vector is of the form [[x], [y], [z]]"""
    return np.array([[point.x], [point.y], [point.z]], dtype=float)

def vector_to_point(vector: np.ndarray | Sequence[Sequence[Number]]) -> Point:
    """Returns a Point object from its vector representation."""
    return Point(vector[0][0], vector[1][0], vector[2][0])

def cross(point1: Point, point2: Point) -> Point:
    """Cross product wrapper """
    v1 = np.squeeze(point1.toarray())
    v2 = np.squeeze(point2.toarray())
    v3 = np.cross(v1, v2)
    return Point.from_array(v3)



def dot(point1: Point, point2: Point) -> float:
    v1 = np.squeeze(point1.toarray())
    v2 = np.squeeze(point2.toarray())
    return float(np.dot(v1, v2))

if __name__ == '__main__':
    p1 = Point(5,5,5)
    p2 = Point(2,2.1,3.2)
    print(cross(p1,p2))
    print(p2.dot(p1))
