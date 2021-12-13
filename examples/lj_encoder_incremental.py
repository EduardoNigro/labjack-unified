""" lj_encoder_incremental.py 

Displays quadrature encoder angular speed.

This example code shows how to setup a quadrature encoder and how to calculate
and display the angular speed of the shaft. Any rotary encoder with A-B phases 
should work for this short example. A CALT GHS38 rotary encoder was used.

Setup:
    On a U3, connect FIO4 to phase A and FIO5 to phase B
    On a U6, connect FIO0 to phase A and FIO1 to phase B
    On a T7, connect FIO2 to phase A and FIO3 to phase B
    Connect the encoder Vcc and GND to the LabJack VS and GND respectively

The LabJack unified methods in this example are:
    set_quadrature ... Sets LabJack configuration for encoder A-B-Z input
    get_counter ...... Gets edge count from encoder A-B signals
    close ............ Closes the LabJack device 

"""
import time
import numpy as np
from labjack_unified.devices import LabJackU3, LabJackU6, LabJackT7

# To use a different LabJack, change the device name
# from LabJackU3 below to either LabJackU6 or LabJackT7
lj = LabJackU3()

# Setting qudrature incremental encoder
lj.set_quadrature()
# Defining encoder pulses per revolution
# (Because rising and falling edges on phases A and B
#  are counted, multiply the encoder PPR by 4)
ppr = 4 * 1000

# Initializing variables and starting main clock
angleprev = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()
# 30-second execution loop that displays the
# current angular velocity of the encoder shaft
print("Running code for 30 seconds ...")
while tcurr <= 30:
    # Pausing for a bit
    time.sleep(0.1)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    anglecurr = np.pi / ppr * lj.get_counter()
    # Calculating current angular speed (rad/s)
    wcurr = (anglecurr - angleprev) / (tcurr - tprev)
    # Printing angular speed
    if anglecurr != angleprev:
        print("Angular speed = {:0.1f} rad/s".format(wcurr))
    # Updating previous values
    angleprev = anglecurr
    tprev = tcurr
print("Done.")

# Closing the device
lj.close()
del lj
