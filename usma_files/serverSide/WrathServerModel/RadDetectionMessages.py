import ctypes
import struct

class StartInitPassMessage(object):

    fmt = "!l"

    def __init__(self):
        self.id = -1

    def pack(self):
        return struct.pack(type(self).fmt, self.id)

    def unpack(self, bytes):
        self.id = struct.unpack(type(self).fmt, bytes)


class FinishInitPassMessage(object):

    fmt = "!l"

    def __init__(self):
        self.id = -1
    
    def pack(self):
        return struct.pack(type(self).fmt, self.id)

    def unpack(self, bytes):
        self.id = struct.unpack(type(self).fmt, bytes)


class UpdateContourLineMessage(object):

    fmt = "!3f"

    def __init__(self):
        self.location = (-1, -1)
        self.error = -1

    def pack(self):
        return struct.pack(type(self).fmt, self.location[0], self.location[1], self.error)
    
    def unpack(self, bytes):
        a, b, self.error = struct.unpack(type(self).fmt, bytes)
        self.location = (a, b)


class LaneUpdateMessage(object):

    fmt = "!6f"

    def __init__(self):
        self.start = (-1, -1)
        self.center = (-1, -1)
        self.end = (-1, -1)
    
    def pack(self):
        return struct.pack(type(self).fmt, self.start[0], self.start[1].
                           self.center[0], self.center[1],
                           self.end[0], self.end[1])
    
    def unpack(self, bytes):
        a, b, c, d, e, f = struct.unpack(type(self).fmt, bytes)
        self.start = (a, b)
        self.center = (c, d)
        self.end = (e, f)


class StartLaneGenerationMessage(object):

    fmt = "!l"
    lanefmt = "!2f"

    def __init__(self):
        self.contourPoints = list()
    
    def pack(self):
        baseSize = struct.calcsize(type(self).fmt)
        pointSize = struct.calcsize(type(self).lanefmt)
        length = baseSize + len(self.contourPoints) * pointSize

        buff = ctypes.create_string_buffer(length)
        struct.pack_into(type(self).fmt, buff, 0, len(self.contourPoints))

        offset = baseSize
        for i in range(len(self.contourPoints)):
            p = self.contourPoints[i]
            struct.pack_into(type(self).lanefmt, buff, offset, p[0], p[1])
            offset += pointSize
        return "".join(buff)
    
    def unpack(self, bytes):
        length = struct.unpack_from(type(self).fmt, bytes, 0)[0]

        baseSize = struct.calcsize(type(self).fmt)
        pointSize = struct.calcsize(type(self).lanefmt)

        self.contourPoints = list()

        offset = baseSize
        for i in range(length):
            lat, lon = struct.unpack_from(type(self).lanefmt, bytes, offset)
            self.contourPoints.append((lat, lon))
            offset += pointSize