#!/usr/bin/env python

import rospy
from serial.tools import list_ports
from serial import Serial
from serial.serialutil import SerialException
import time
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import rosbag


frame_dt = np.dtype([
        ('system', [('header', '<1I'),('rtc', '<2I'), ('cpuclock', '<1I'), ('status', '<1I'), ('g_disp', '<1I'), ('n_disp', '<1I'), ('dose', '<1I'), ('battery', '<1I'),('error', '<1I'),('firmware', '<1H'), ('serial_number', '<1H'), ('empty', '<1I')]),
        ('mca', [
            ('status', '<1I'),('time', '<1I'), ('live', '<1I'), ('temp', '<3I'), ('hvm', '<3I'),
            ('ga_counts', [('value', '<1I'), ('sma', '<1f'), ('fma', '<1f')]),
            ('ne_counts', [('value', '<1I'), ('sma', '<1f'), ('fma', '<1f')]),
            ('sp_ne_counts', [('value', '<1I'), ('sma', '<1f'), ('fma', '<1f')]),
            ('psd_ne_counts', [('value', '<1I'), ('sma', '<1f'), ('fma', '<1f')])
            ]),   #'<21I'
        ('spectrum', '<1024H'),
        ('mca_params', [('hv', '<1I'),('ga_th', '<1I'),('ne_th', '<1I'),('sp_ne_th', '<1I'),('sp_ne_delta', '<1I'),('psd', '<2I')]),
        ('dose', [('mca', '<3I'),('counter', '<3I'),('hvm', '<3I'),('rate', '<2I')]),
        ('alarm', [('base', '<1f'),('up', '<1f'),('down', '<1f'),('dose', '<1f'),('nbase', '<1f'),('nup', '<1f'),('ndown', '<1f'),('status', '<1I')]),
        ('checksum', '<1I'),
        ('deviations', '<14f'),
        ('gain', '<1f'),
        ('checksum2', '<1I'),
    ])

frame_size = frame_dt.itemsize


class AcquisitionNode(object):
    """docstring for ObjectDetectionNode."""
    def __init__(self):
        # init the node
        rospy.init_node('HeRMES_DAQ', anonymous=False)
        self.pub = rospy.Publisher('/tactic_interface/radiation', PointStamped, queue_size=10)


        # Create detector object
        self.detector = self.connect()
        # self.detector_id = self.detector.sn
    def connect(self):
        for t in list_ports.comports():
            rospy.loginfo(t)
            if t[2] == 'USB VID:PID=0483:5740 SNR=00000000001A':
                print "Starting communication with HeRMES"
                port = Serial(t[0], 115200, timeout=0.1)
        return port

    def acq_from_det(self):
        """
        #TODO
        """
        self.detector.write('e\r\n'.encode())
        self.detector.flushInput()
        time_start = time.time()
        print "Beginning Acqusition"
        # init

        while not rospy.is_shutdown():
            # spec_arr = np.zeros(1024)
            buff = self.detector
            # rospy.loginfo("Sending data...")
            if buff is not None:

                try:
                    frame = buff.read(frame_size)
                except SerialException as e:
                    continue
                lframe = len(frame)
                if lframe==frame_size:
                # if frame is not None:
                    data =  np.fromstring(frame, dtype=frame_dt)
                    stamp = time.time()
                    rospy.loginfo(data['mca'][0]['ga_counts'][0])
# print 'G: ', data['mca'][0]['ga_counts'][0], "N: ", data['mca'][0]['ne_counts'][0], int(stamp-time_start)
                    # print np.sum(data['spectrum']), np.sum(data['spectrum'][0]), len(data['spectrum'][0])
                    ## I think i will collect the spectra in another node, no need to add it here

                    # if stamp > (time_start + 10):
                    #     if np.sum(data['spectrum'][0]) > 0:
                    #         print "spect on"
                    #         spec_arr = spec_arr + data['spectrum'][0]
                else:
                    continue
            else:
                continue

            # except SerialException as e:
            #     #There is no new data from serial port
            #     print "hi"
            #     continue

            ## Publish each buffer, HeRMES returns data at 1Hz
            msg = PointStamped()
            msg.point.x = time.time()
            msg.point.y = float(data['mca'][0]['ga_counts'][0])
            msg.point.z = 0.0
            # msg = radData()
            # now = time.time()
            # rnow = rospy.Time.now()
            # msg.header.stamp = rnow
            # msg.detid = '0'
            # msg.ts_sys = now
            # msg.counts = data['mca'][0]['ga_counts'][0]
            # msg.channel = data['spectrum'][0]
            # msg.slow_avg = data['mca'][0]['ga_counts'][1]
            # msg.fast_avg = data['mca'][0]['ga_counts'][2]
            # # I think dose rate is reported in uR/h, but as an integer value...
            # # this convers this into a mR/hr float value, not sure at this point...
            # msg.dose_rate = data['dose'][0]['rate'][1]/1000000000
            self.pub.publish(msg)
            # print "publish"
            # except Exception as e:
        #     print "daq exception general"
        return


def main():
    """ main function
    """
    good = False
    while not good:
        try:
            node = AcquisitionNode()
            good = True
        except ValueError:
            print('Could not find a detector, sleepting for 10 seconds.')
            time.sleep(10.)
            good = False

    node.acq_from_det()

    return


if __name__ == '__main__':
    main()
