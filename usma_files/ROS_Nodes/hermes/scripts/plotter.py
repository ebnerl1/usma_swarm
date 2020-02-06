#!/usr/bin/env python

import rospy
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from hermes_daq.msg import radData

from std_msgs.msg import String
from time import sleep

'''
Node for plotting output from HeRMES detectors.
-Count rate
-Spectra (currently uncalibrated)
-Waterfall plot (this will be a reach... need to call dan)

Since detector only publishes at 1Hz, I want to rebuild the plot at the publish rate
this will remove the need for a seperate plotting function, when the callback function
is called, it should not be an issue to rebuild the plot with incoming data

need to work the global variables or switch to object stuff....
'''

plt.ion()
fig = plt.figure()

histo = np.zeros(1024)
histo[0] = 1
cps = np.asarray([])
tme_list = np.asarray([])
cps = np.asarray([])
wf_data = np.zeros(1024*100).reshape(100, 1024)


ax_rate = plt.subplot2grid((6, 6), (0, 0), colspan = 3, rowspan = 3)
ax_spec = plt.subplot2grid((6, 6), (3, 0), colspan = 3, rowspan = 3)
ax_wf = plt.subplot2grid((6, 6), (0, 3), colspan = 3, rowspan = 6)


ax_wf = sns.heatmap(wf_data, cmap='viridis', ax=ax_wf)


ax_spec.set_title('Uncalibrated Spectra')
ax_spec.set_xlabel('Channel')
ax_spec.set_ylabel('Counts')
ax_spec.set_xlim(0, 700)

ax_wf.set_title('Waterfall')
ax_wf.set_xlabel('Energy [keV]')
ax_wf.set_ylabel("Time")
# ax_energy.set_xlim(0, 2000)

ax_rate.set_title('Count Rate')
ax_rate.set_xlabel('Time [s]')
ax_rate.set_ylabel("CPS")

fig.tight_layout()


spec, = ax_spec.semilogy(histo, 'green')
rate, = ax_rate.plot(0, 0, 'r')

fig.canvas.draw()

def callback(data):
    global histo, cps, wf_data, tme_list
    histo += data.channel
    cps = np.append(cps, data.counts)
    tme_list = np.append(tme_list, data.ts_sys)
    wf_data = np.append(wf_data, data.channel)
    print data.counts, " Counts"

    make_plot(histo, cps, tme_list, wf_data)


def make_plot(h, r, tmes, wf):

    spec.set_ydata(h)

    rate.set_xdata(tmes)
    rate.set_ydata(r)

    # scales axes with data
    ax_spec.set_ylim(0, h.max()**1.5)  #look at log scaling for plot
    # ax_energy.set_ylim(0, hist.max()**1.5)
    # ax_energy.set_xlim(0, 2500)
    rate_low = tmes.min()
    if (tmes.max()-tmes.min()) > 30:
        rate_low = tmes.max()-30

    ax_rate.set_xlim(rate_low, tmes.max())
    ax_rate.set_ylim(r.min(), 1.01*r.max())

    sns.heatmap(wf_data[-100:], cmap='viridis', ax=ax_wf)

    fig.canvas.draw()
    fig.canvas.flush_events()

def plot_it():
    rospy.init_node('plotter', anonymous=True)

    while not rospy.is_shutdown():

        rospy.Subscriber('/rad_data', radData, callback, queue_size=10)
        # rospy.spin()
        plt.show(block=True)

if __name__ == '__main__':
    try:
        # sleep(7)
        plot_it()
    except rospy.ROSInterruptException:
        pass
