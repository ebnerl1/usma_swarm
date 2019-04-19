#!/usr/bin/env python

#-------------------------------------------------------------------------
# swarm_helper
# W. Dale Arbogast, 2016
#
# Extends the bitmapped_bytes.py class to support pursuit messages
#-------------------------------------------------------------------------

import struct
from ap_lib import bitmapped_bytes
#from ap_lib import distributed_algorithms

PURSUIT_MESSAGE = 22     # Data field represents a pursuit message
PURSUIT_UPDATE_REQUEST = 23   # List of Enemies being pursued
PURSUIT_LIST = 24    #Used by swarm commander to give new swarm member the pursuit list


class PursuitMessageParser(bitmapped_bytes.BitmappedBytes):
    ''' Parser for pursue message data
    '''
    fmt = ">Bl?l"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.friendly_id = 0
        self.target_id_new = 0     # 0-255
        self.pursuit_status = False
        self.target_distance = 0.0


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        return struct.pack(type(self).fmt, self.friendly_id, self.target_id_new, \
                           self.pursuit_status, self.target_distance)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.friendly_id, self.target_id_new, self.pursuit_status, self.target_distance = \
            struct.unpack_from(type(self).fmt, bytes, 0)



class PursuitUpdateRequestParser(bitmapped_bytes.BitmappedBytes):
    '''Parser used to pass a boolean value
    '''

    fmt = ">?"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.send_update = False


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        return struct.pack(type(self).fmt, self.send_update)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.send_update = struct.unpack_from(type(self).fmt, bytes, 0)



class PursuitListMessageParser(bitmapped_bytes.BitmappedBytes):
    ''' Parser for pursue message data
    '''
    fmt = ">Bl?l"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.friendly_id = 0
        self.target_id_new = 0     # 0-255
        self.pursuit_status = False
        self.target_distance = 0.0


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        return struct.pack(type(self).fmt, self.friendly_id, self.target_id_new, \
                           self.pursuit_status, self.target_distance)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.friendly_id, self.target_id_new, self.pursuit_status, self.target_distance = \
            struct.unpack_from(type(self).fmt, bytes, 0)

