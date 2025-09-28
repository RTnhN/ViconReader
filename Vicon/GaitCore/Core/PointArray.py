#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020, AIMGaitCore
#     (www.aimlab.wpi.edu)
#
#     All rights reserved.
#
#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:
#
#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#
#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
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
#
#     \author    <http://www.aimlab.wpi.edu>
#     \author    <ajlewis@wpi.edu>
#     \author    Alek Lewis
#     \author    <nagoldfarb@wpi.edu>
#     \author    Nathaniel Goldfarb
#     \version   0.1
# */
# //==============================================================================

from __future__ import annotations

from typing import Iterator, List, Sequence

import numpy as np

from .Point import Point

Number = float | int


class PointArray:

    def __init__(
        self,
        x: Sequence[float] | None = None,
        y: Sequence[float] | None = None,
        z: Sequence[float] | None = None,
    ) -> None:
        self._x: List[float] = list(x) if x is not None else []
        self._y: List[float] = list(y) if y is not None else []
        self._z: List[float] = list(z) if z is not None else []

    @classmethod
    def init_point_array(cls) -> "PointArray":
        """Create an empty :class:`PointArray`."""
        return cls(x=[], y=[], z=[])

    @classmethod
    def from_array(cls, arr: np.ndarray, dim: int = 1) -> "PointArray":
        """Create a :class:`PointArray` from a ``(N, 3)`` or ``(3, N)`` array."""
        if dim == 0 and arr.shape[1] == 3:
            x = arr[:, 0].tolist()
            y = arr[:, 1].tolist()
            z = arr[:, 2].tolist()
        elif dim == 1 and arr.shape[0] == 3:
            x = arr[0, :].tolist()
            y = arr[1, :].tolist()
            z = arr[2, :].tolist()
        else:
            raise ValueError("Array must have three coordinates along the specified dimension")

        return cls(x=x, y=y, z=z)

    @classmethod
    def from_point_array(cls, point_array: Sequence[Point]) -> "PointArray":
        """Create a :class:`PointArray` from a sequence of :class:`Point` objects."""
        x = [p.x for p in point_array]
        y = [p.y for p in point_array]
        z = [p.z for p in point_array]
        return cls(x=x, y=y, z=z)

    def append(self, point: Point) -> None:
        """Append a :class:`Point` to the array."""
        self._x.append(point.x)
        self._y.append(point.y)
        self._z.append(point.z)

    @property
    def x(self) -> List[float]:
        return self._x

    @property
    def y(self) -> List[float]:
        return self._y

    @property
    def z(self) -> List[float]:
        return self._z

    @x.setter
    def x(self, value: Sequence[float]) -> None:
        self._x = list(value)

    @y.setter
    def y(self, value: Sequence[float]) -> None:
        self._y = list(value)

    @z.setter
    def z(self, value: Sequence[float]) -> None:
        self._z = list(value)

    def get(self, ind: int) -> Point:
        return Point(self.x[ind], self.y[ind], self.z[ind])

    def toarray(self) -> np.ndarray:
        """Return the point data as a ``3 x N`` :class:`numpy.ndarray`."""
        return np.array([self.x, self.y, self.z], dtype=float)

    def toPointList(self) -> list[Point]:
        """Return a list of :class:`Point` objects."""
        return [self.get(i) for i in range(len(self._x))]

    def __add__(self, other: "PointArray") -> "PointArray":
        if len(self) != len(other):
            raise ValueError("PointArrays must be the same length to add")
        x = [self.x[i] + other.x[i] for i in range(len(self.x))]
        y = [self.y[i] + other.y[i] for i in range(len(self.y))]
        z = [self.z[i] + other.z[i] for i in range(len(self.z))]
        return PointArray(x, y, z)

    def __sub__(self, other: "PointArray") -> "PointArray":
        if len(self) != len(other):
            raise ValueError("PointArrays must be the same length to subtract")
        x = [self.x[i] - other.x[i] for i in range(len(self.x))]
        y = [self.y[i] - other.y[i] for i in range(len(self.y))]
        z = [self.z[i] - other.z[i] for i in range(len(self.z))]
        return PointArray(x, y, z)

    def __mul__(self, other: "PointArray") -> "PointArray":
        if len(self) != len(other):
            raise ValueError("PointArrays must be the same length to multiply")
        x = [self.x[i] * other.x[i] for i in range(len(self.x))]
        y = [self.y[i] * other.y[i] for i in range(len(self.y))]
        z = [self.z[i] * other.z[i] for i in range(len(self.z))]
        return PointArray(x, y, z)

    def __rmul__(self, other: Number) -> "PointArray":
        x = [value * other for value in self.x]
        y = [value * other for value in self.y]
        z = [value * other for value in self.z]
        return PointArray(x, y, z)

    def __str__(self) -> str:
        return " X: " + str(self.x) + " Y: " + str(self.y) + " Z: " + str(self.z)

    def __getitem__(self, item: slice | int) -> PointArray | Point:
        if isinstance(item, slice):
            return PointArray(self.x[item], self.y[item], self.z[item])
        return Point(self.x[item], self.y[item], self.z[item])

    def __len__(self) -> int:
        return len(self._x)

    def __iter__(self) -> Iterator[Point]:
        for index in range(len(self)):
            yield self.get(index)

    def __abs__(self) -> "PointArray":
        x = [abs(value) for value in self.x]
        y = [abs(value) for value in self.y]
        z = [abs(value) for value in self.z]
        return PointArray(x, y, z)


if __name__ == "__main__":
    arr = np.array([[1, 2, 3, 4, 5], [4, 5, 6, 7, 8], [7, 8, 9, 10, 11]])
    p = PointArray.from_array(arr, 1)
    print(p)
