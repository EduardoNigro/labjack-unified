# labjack-unified
Classes with unified methods to interact with **LabJacks U3, U6 and T7**.

**labjack-unified** is implemented upon [LabJackPython](https://github.com/labjack/LabJackPython) and [labjack-ljm-python](https://github.com/labjack/labjack-ljm-python) and provides classes to interact with any of the [LabJack](https://labjack.com/) devices listed above, through a set of methods that are device-independent. **labjack-unified** was developed for the MS Windows platform.

## Classes & Methods
Three classes are available (**LabJackU3**, **LabJackU6**, and **LabJackT7**) which have the common methods listed below. These should cover the majority of LabJack applications. There are also device specific methods as well. 

Method | Description
------ | -----------
close | Closes the LabJack device
display_info | Displays summary info about the device
set_digital | Writes digital value to specified port(s)
get_digital | Reads digital value from specified port(s)
set_analog | Writes analog value to specified port(s)
get_analog | Reads analog value from specified port(s)
get_labjacktemp | Gets ambient temperature from internal sensor
set_stream | Sets LabJack configuration for data streaming
get_stream | Gets streaming data
stop_stream | Stops data streaming
set_PWM | Sets LabJack configuration for PWM output
set_dutycycle | Sets duty cycle of PWM output (-100 to 100)
set_quadrature | Sets LabJack configuration for encoder A-B-Z input
get_counter | Gets edge count from encoder A-B signals
reset_counter | Resets edge counter


## Install

    py -m pip install labjack-unified

Installing **labjack-unified** will install its dependencies: _LabJackPython_ and _labjack-ljm_. _Plotly_ will also be installed so graphs can be displayed when running the examples.

## Documentation

Comprehensive documentation for **labjack-unified** is available [here](https://eduardonigro.github.io/labjack-unified/).

## Examples
Nine examples are available in the documentation, with four of them requiring only
a pair of wires to be executed successfully. Below is the code for **lj_io_analog.py**:

```python
""" lj_io_analog.py 

Uses analog input and output channels.

This example shows how to use analog inputs and outputs in a data acquisition
loop. Accurate time execution of output events can be achieved by the use of a
timer logic in the loop. The voltage measurements are executed as fast as
possible. The main latency source is the LabJack I/O times.

Setup:
    Connect DAC0 to AIN0
    Connect DAC1 to AIN1

The LabJack unified methods in this example are:
    set_analog ....... Writes analog value to specified port(s)
    get_analog ....... Reads analog value from specified port(s)
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
tstep = 0.5 # Interval between step changes (s)
t = [] # Output time array
v1 = [] # Output sampled voltage 1 array
v2 = [] # Output sampled voltage 2 array

# Initializing timers and starting main clock
tprev = 0
tcurr = 0
tstart = time.perf_counter()
# Executing acquisition loop
print('Running code for 5 seconds ...')
while tcurr <= 5:
    # Updating analog output every `tstep` seconds
    # with random voltages between 0 and 5 V
    if (np.floor(tcurr/tstep) - np.floor(tprev/tstep)) == 1:
        lj.set_analog('DAC0', 5*np.random.rand())
        lj.set_analog('DAC1', 5*np.random.rand())
    # Updating previous time and getting current time (s)
    tprev = tcurr
    tcurr = time.perf_counter() - tstart
    # Acquiring analog data as fast as possible
    # and appending values to output arrays
    t.append(tcurr)
    v1.append(lj.get_analog('AIN0'))
    v2.append(lj.get_analog('AIN1'))
print('Done.')
# Closing the device
lj.close()
del lj

# Plotting results 
plot_line([t]*2, [v1, v2], yname=['AIN0', 'AIN1'], axes='multi', marker=True)
plot_line([t[1::]], [np.diff(t)], yname=['Sampling Period (s)'])
```
![](/images/lj_io_analog_fig_1.PNG)
![](/images/lj_io_analog_fig_2.PNG)
