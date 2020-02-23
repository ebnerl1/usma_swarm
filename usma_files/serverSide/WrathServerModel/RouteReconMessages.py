import struct
import ctypes

class StartBehaviorMessage(object):

    fmt = "!2l2f"

    def __init__(self):
        self.numDrones = -1
        self.id = -1
        self.location = (-1, -1)


    def pack(self):
        return struct.pack(type(self).fmt, self.numDrones, self.id, self.location[0], self.location[1])

    
    def unpack(self, bytes):
        self.numDrones, self.id, a, b = struct.unpack(type(self).fmt, bytes)
        self.location = (a, b)


class RoadAnalyzedMessage(object):

    fmt = "!4f"

    def __init__(self):
        self.start = (-1, -1)
        self.end = (-1, -1)
    

    def pack(self):
        return struct.pack(type(self).fmt, self.start[0], self.start[1], self.end[0], self.end[1])
    

    def unpack(self, bytes):
        a, b, c, d = struct.unpack(type(self).fmt, bytes)
        self.start = (a, b)
        self.end = (c, d)


class InitGraphMessage(object):

    fmt = "3l"
    vertexfmt = "2f"
    edgefmt = "2l"
    posfmt = "2f"

    def __init__(self):
        self.vertices = list()
        self.edges = list()
        self.dronePositions = list()

    
    def pack(self):
        vlen = len(self.vertices)
        elen = len(self.edges)
        plen = len(self.dronePositions)

        fmtSize = struct.calcsize(type(self).fmt)
        vfmtSize = struct.calcsize(type(self).vertexfmt)
        efmtSize = struct.calcsize(type(self).edgefmt)
        pfmtSize = struct.calcsize(type(self).posfmt)

        length = vlen * vfmtSize + elen * efmtSize + plen * pfmtSize + fmtSize
        buff = ctypes.create_string_buffer(length)

        struct.pack_into(type(self).fmt, buff, 0, vlen, elen, plen)

        offset = fmtSize
        for i in range(vlen):
            lat = self.vertices[i][0]
            lon = self.vertices[i][1]
            struct.pack_into(type(self).vertexfmt, buff, offset, lat, lon)
            offset += vfmtSize
        
        for i in range(elen):
            start = self.edges[i][0]
            end = self.edges[i][1]
            struct.pack_into(type(self).edgefmt, buff, offset, start, end)
            offset += efmtSize

        for i in range(plen):
            lat = self.dronePositions[i][0]
            lon = self.dronePositions[i][1]
            struct.pack_into(type(self).posfmt, buff, offset, lat, lon)
            offset += pfmtSize

        return "".join(buff)


    def unpack(self, bytes):
        vlen, elen, plen = struct.unpack_from(type(self).fmt, bytes, 0)

        fmtSize = struct.calcsize(type(self).fmt)
        vfmtSize = struct.calcsize(type(self).vertexfmt)
        efmtSize = struct.calcsize(type(self).edgefmt)
        pfmtSize = struct.calcsize(type(self).posfmt)

        self.vertices = list()
        self.edges = list()
        self.dronePositions = list()

        offset = fmtSize
        for i in range(vlen):
            lat, lon = struct.unpack_from(type(self).vertexfmt, bytes, offset)
            self.vertices.append((lat, lon))
            offset += vfmtSize
       
        for i in range(elen):
            start, end = struct.unpack_from(type(self).edgefmt, bytes, offset)
            self.edges.append((start, end))
            offset += efmtSize

        for i in range(plen):
            lat, lon = struct.unpack_from(type(self).posfmt, bytes, offset)
            self.dronePositions.append((lat, lon))
            offset += pfmtSize
