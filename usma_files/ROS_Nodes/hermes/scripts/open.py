from serial.tools import list_ports
from serial import Serial
import time
import numpy as np
import matplotlib.pyplot as plt

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

def access_port():
    for t in list_ports.comports():
        if t[1] == 'STM32 Virtual ComPort':
            print t[0], t[1]
            print "Starting communication with HeRMES"

    return t[0]

def pull_data(port_name, run_length):
    spec_arr = np.zeros(1024)
    port = Serial(port_name, 115200, timeout=0.1)
    print port

    port.write('e\r\n'.encode())
    port.flushInput()

    print 'Streaming command sent...'
    time_start = time.time()
    print "Beginning Acqusition"
    while run_length >= (time.time() - time_start):

        frame = port.read(frame_size)
        lframe = len(frame)
        if lframe==frame_size:
        # if frame is not None:
            data =  np.fromstring(frame, dtype=frame_dt)
            print data['dose'][0]['rate'][1]/1000000000
            stamp = time.time()
            print 'Gamma: ', data['mca'][0]['ga_counts'][0], "Neutron: ", data['mca'][0]['ne_counts'][0], stamp-time_start
            print np.sum(data['spectrum']), np.sum(data['spectrum'][0]), len(data['spectrum'][0])
            if stamp > (time_start + 10):
                if np.sum(data['spectrum'][0]) > 0:
                    print "spect on"
                    spec_arr = spec_arr + data['spectrum'][0]
                    continue
        else:
            continue
    print np.sum(spec_arr)
    return spec_arr

def print_me(arr):
    plt.plot(range(1024), arr)
    plt.show()


def start_acq(run_length):
    p_name = access_port()
    spectra = pull_data(p_name, run_length)
    if np.sum(spectra) > 200:
        print_me(spectra)
    else:
        print "low spectra stats.."

if __name__ == '__main__':
    start_acq(60*2) #collection time in seconds
