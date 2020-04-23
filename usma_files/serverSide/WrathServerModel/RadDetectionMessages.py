#!/usr/bin/env python

import struct
import ctypes

class SparsityMessage(object):

    id = 11
    fmt = "!ll2f"

    def __init__(self):
        self.contourIndex = -1
        self.location = (0, 0)

    
    def pack(self):
        return struct.pack(type(self).fmt, type(self).id,
            self.contourIndex, self.location[0], self.location[1])

    
    def unpack(self, bytes):
        id, self.contourIndex, a, b = struct.unpack_from(type(self).fmt, bytes, 0)
        self.location = (a, b)


class RadLocationMessage(object):

    id = 12
    fmt = "!ll5f"

    def __init__(self):
        self.contourIndex = -1
        self.location = (0, 0)
        self.direction = (0, 0)
        self.error = 0.0

    
    def pack(self):
        return struct.pack(type(self).fmt, type(self).id,
            self.contourIndex, self.location[0], self.location[1], self.direction[0],
            self.direction[1], self.error)

    
    def unpack(self, bytes):
        id, self.contourIndex, a, b, c, d, self.error = struct.unpack_from(type(self).fmt, bytes, 0)
        self.location = (a,b)
        self.direction = (c, d)


class LockLaneGenMessage(object):

    id = 13
    fmt = "!ll"

    def __init__(self):
        self.id = -1


    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.id)


    def unpack(self, bytes):
        id, self.id = struct.unpack_from(type(self).fmt, bytes, 0)


class UnlockLaneGenMessage(object):

    id = 14
    fmt = "!ll"

    def __init__(self):
        self.id = -1


    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.id)


    def unpack(self, bytes):
        id, self.id = struct.unpack_from(type(self).fmt, bytes, 0)


class StartInitPassMessage(object):

    id = 15
    fmt = "!ll"

    def __init__(self):
        self.id = -1

    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.id)

    def unpack(self, bytes):
        id, self.id = struct.unpack(type(self).fmt, bytes)


class FinishInitPassMessage(object):

    id = 16
    fmt = "!ll6f"

    def __init__(self):
        self.loc1 = (0.0, 0.0)
        self.max = (0.0, 0.0)
        self.loc2 = (0.0, 0.0)
        self.id = -1
    
    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.id, self.loc1[0], self.loc1[1],
                            self.max[0], self.max[1], self.loc2[0], self.loc2[1])

    def unpack(self, bytes):
        id, self.id, a, b, c, d, e, f = struct.unpack(type(self).fmt, bytes)
        self.loc1 = (a, b)
        self.max = (c, d)
        self.loc2 = (e, f)


class LaneUpdateMessage(object):

    id = 17
    fmt = "!l6f"

    def __init__(self):
        self.start = (-1, -1)
        self.center = (-1, -1)
        self.end = (-1, -1)
    
    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.start[0], self.start[1],
                           self.center[0], self.center[1],
                           self.end[0], self.end[1])
    
    def unpack(self, bytes):
        id, a, b, c, d, e, f = struct.unpack(type(self).fmt, bytes)
        self.start = (a, b)
        self.center = (c, d)
        self.end = (e, f)


class StartLaneGenerationMessage(object):

    id = 18
    fmt = "!lll"
    lanefmt = "!2f"

    def __init__(self):
        self.contourPoints = list()
    
    def pack(self):
        baseSize = struct.calcsize(type(self).fmt)
        pointSize = struct.calcsize(type(self).lanefmt)

        contourNum = len(self.contourPoints)
        contourLen = len(self.contourPoints[0])

        length = baseSize + contourNum * contourLen * pointSize

        buff = ctypes.create_string_buffer(length)
        struct.pack_into(type(self).fmt, buff, 0, type(self).id, contourNum, contourLen)

        offset = baseSize
        for i in range(contourNum):
            for ii in range(contourLen):
                p = self.contourPoints[i][ii]
                struct.pack_into(type(self).lanefmt, buff, offset, p[0], p[1])
                offset += pointSize
        return "".join(buff)
    
    def unpack(self, bytes):
        id, numLanes, laneLen = struct.unpack_from(type(self).fmt, bytes, 0)

        baseSize = struct.calcsize(type(self).fmt)
        pointSize = struct.calcsize(type(self).lanefmt)

        self.contourPoints = list()

        offset = baseSize
        for i in range(numLanes):
            self.contourPoints.append(list())
            for ii in range(laneLen):
                lat, lon = struct.unpack_from(type(self).lanefmt, bytes, offset)
                self.contourPoints[i].append((lat, lon))
                offset += pointSize


class RadiationMessage(object):

    id = 19
    fmt = "!l3f"

    def __init__(self):
        self.count = 0.0
        self.location = (0.0, 0.0)

    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.count, self.location[0],
                           self.location[1])
    
    def unpack(self, bytes):
        id, self.count, a, b = struct.unpack(type(self).fmt, bytes)
        self.location = (a, b)
