Analog I/O
==========

This example :ref:`code <codeanalog>` shows how to use analog inputs and outputs in a data
acquisition loop. Random voltage outputs (between 0 and 5 V) are generated at a constant
time interval of 0.5 s on ports **DAC0** and **DAC1**. Ports **AIN0** and **AIN1**
are used to sample the voltages as fast as possible. The data acquisition latency
(in average under 2 ms) is due mostly to the **LabJack** I/O times. Accurate time
execution of output events can be achieved by the use of a timer logic in the loop.


.. image:: ../../images/lj_io_analog_fig_1.png
.. image:: ../../images/lj_io_analog_fig_2.png

.. _codeanalog:

.. code-block:: python

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
    plot_line([t[1::]], [1000*np.diff(t)], yname=['Sampling Period (ms)'])

