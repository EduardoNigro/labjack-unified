""" lj_motor.py 

Uses PWM output to set the speed of DC electric motor.

This example shows how to use the PWM output to change the speed of a DC motor.
The PWM output is ramped up to 100% and then back down to 0%.

Setup:
    You will need:
    - Electric motor (I used a CQRobot DC 6V-183RPM/12V-366RPM)
    - An H-Bridge (I used a DROK L298 Dual H Bridge Motor Speed Controller)
    - A 12V power supply
    Connect DAC0 to the bridge DIR1 (or IN1)
    Connect DAC1 to the bridge DIR2 (or IN2)
    On a U3, connect FIO4 to the bridge PWM IN (or ENA)
    On a U6, connect FIO0 to the bridge PWM IN (or ENA)
    On a T7, connect FIO0 to the bridge PWM IN (or ENA)
    Connect the bridge Vcc and GND to the LabJack VS and GND respectively

The LabJack methods in this example are:
    set_PWM .......... Sets LabJack configuration for PWM output
    set_dutycycle .... Sets duty cycle of PWM output (-100 to 100)
    close ............ Closes the LabJack device 

"""
import time
import numpy as np
from labjack_unified.utils import plot_line
from labjack_unified.devices import LabJackU3, LabJackU6, LabJackT7

# To use a LabJack U6 or a T7, change the device name
# from LabJackU3 below to either LabJackU6 or LabJackT7
lj = LabJackU3()

# Assigning parameters
tramp = 5 # PWM ramp up/down time (s)
t = [] # Output time array
pwm = [] # Output pwm signal array

# Configuring PWM putput
lj.set_pwm(dirport1='DAC')

# Initializing timers and starting main clock
tcurr = 0
tstart = time.perf_counter()
# Executing acquisition loop
print('Running code for ' + str(2*tramp) + ' seconds ...')
while tcurr <= 2*tramp:
    # Calculating pwm output
    if tcurr < tramp:
        # Ramping up to 100% for the first `tramp` seconds
        pwmcurr = 100/tramp*tcurr
    else:
        # Ramping down to 0% for the last `tramp` seconds
        pwmcurr = 100-100/tramp*(tcurr-tramp)
    # Updating PWM output
    lj.set_dutycycle(value1=pwmcurr)
    # Updating previous time and getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Appending values to output arrays
    t.append(tcurr)
    pwm.append(pwmcurr)
lj.set_dutycycle(value1=0)    
print('Done.')
# Closing the device
lj.close()
del lj

# Plotting results 
plot_line([t], [pwm], yname=['PWM Output (%)'])
plot_line([t[1::]], [1000*np.diff(t)], yname=['Sampling Period (ms)'])

