#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
#matplotlib.use('TkAgg')

from std_msgs.msg import String
from time import sleep

from rospy.numpy_msg import numpy_msg
from rntools.tools import arrays_to_rndata, rndata_to_array
from rntools.msg import RNData

plt.ion()

def calibrator2d(ch_num, x1=(-2.67028726e-02), x2= 1.84041723e+01, x3= 1.77473292e+03):
    if type(ch_num) == int:
        E = ((ch_num**2)*x1 + ch_num*x2-x3)
        return E

    else:
        new_list = []
        for ch in ch_num:
            E = ((ch**2)*x1 + ch*x2-x3)
            new_list.append(E)
        return new_list

def calibrator1d(ch_num, m=7.31353916, b=(-733.95545552)):
    if type(ch_num) == int:
        E = m*ch_num + b
        return E

    else:
        new_list = []
        for ch in ch_num:
            E = m*ch + b
            new_list.append(E)
        return new_list

hold_hist = np.zeros(1024)
hold_hist[34] = 1  #semilogy error if all 0s
start_msg_time = np.array(0)
start_CPS = np.array(0)
save_array = [hold_hist, start_msg_time, start_CPS]
np.save('node_hold.npy', save_array)

## Sets up plot window to display Spectra and CPS
fig = plt.figure()
ax_spec = plt.subplot2grid((6, 3), (0, 0), colspan = 3, rowspan = 2)
ax_energy = plt.subplot2grid((6, 3), (2, 0), colspan = 3, rowspan = 2)
ax_rate = plt.subplot2grid((6, 3), (4, 0), colspan = 3, rowspan = 2)

ax_spec.set_title('Uncalibrated Spectra')
ax_spec.set_xlabel('Channel')
ax_spec.set_ylabel('Counts')
ax_spec.set_xlim(0, 700)

ax_energy.set_title('Calibrated Spectra')
ax_energy.set_xlabel('Energy [keV]')
ax_energy.set_ylabel("Counts")
# ax_energy.set_xlim(0, 2000)

ax_rate.set_title('Count Rate')
ax_rate.set_xlabel('Time [s]')
ax_rate.set_ylabel("CPS")

fig.tight_layout()

### Used to plot calibrated energy, not behaving super well.
# x_range = range(0, 1024)
# e = calibrator1d(x_range)
#
# energy1, = ax_energy.semilogy(e, hold_hist, 'purple')
spec1, = ax_spec.semilogy(hold_hist, 'green')
rate1, = ax_rate.plot(0, 0, 'r')
fig.canvas.draw()

def callback(data):
    ch_list = []
    hold_hist = np.load('node_hold.npy')
    #scalar_arr should be None
    data_arr, scalar_arr = rndata_to_array(data)

    for i in data_arr:
        ch_list.append(i[1])  #index 0 for Energy 1 for ch
    hist, bins = np.histogram(ch_list, bins = 1024, range=(.5, 1024.5)) #, was causing issues for energy plot
    summed_hist = hold_hist[0] + hist

    msg_pack_time = int(data_arr[0][7])
    msg_time_arr = np.append(hold_hist[1], msg_pack_time)
    CPS_arr = np.append(hold_hist[2], len(data_arr))

    save_array = [summed_hist, msg_time_arr, CPS_arr]
    np.save('node_hold.npy', save_array)

    make_plot(summed_hist, msg_time_arr, CPS_arr)
    # print len(np.nonzero(summed_hist)[0])
    # print msg_time_arr, CPS_arr



def make_plot(hist, tme, cps):
    x = tme[1:] #removes the 0 when I started the array
    y = cps[1:]

    # updates data set for faster plotting

    # x_spec = np.arange(hist.size)
    # energy = calibrator2d(x_spec)

    spec1.set_ydata(hist)

    # energy1.set_ydata(hist)

    rate1.set_xdata(x)
    rate1.set_ydata(y)
    # scales axes with data
    ax_spec.set_ylim(0, hist.max()**1.5)  #look at log scaling for plot
    # ax_energy.set_ylim(0, hist.max()**1.5)
    # ax_energy.set_xlim(0, 2500)
    rate_low = x.min()
    if (x.max()-x.min()) > 30:
        rate_low = x.max()-30
    ax_rate.set_xlim(rate_low, x.max())
    ax_rate.set_ylim(y.min(), 1.01*y.max())
    fig.canvas.draw()
    fig.canvas.flush_events()

def plot_spectra():
    rospy.init_node('plotter', anonymous=True)

    while not rospy.is_shutdown():

        rospy.Subscriber("/interaction_data", RNData, callback, queue_size=10)
        plt.show(block=True)


if __name__ == '__main__':
    try:
        # sleep(7)
        plot_spectra()
    except rospy.ROSInterruptException:
        pass
