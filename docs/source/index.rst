
labjack-unified
===============

Classes with unified methods to interact with `LabJack <https://labjack.com/>`__
DAQ devices **U3**, **U6** and **T7**.

.. toctree::
   :hidden:

   LabJackU3
   LabJackU6
   LabJackT7


About
-----

Three classes are available (**LabJackU3**, **LabJackU6**, and **LabJackT7**) which have
the common methods listed below. They should cover the majority of LabJack applications.
There are also device specific methods as well (see each class for more details). 

===============  ==================================================
Method           Description
===============  ==================================================
close            Closes the LabJack device
display_info     Displays summary info about the device
set_digital      Writes digital value to specified port(s)
get_digital      Reads digital value from specified port(s)
set_analog       Writes analog value to specified port(s)
get_analog       Reads analog value from specified port(s)
get_labjacktemp  Gets ambient temperature from internal sensor
set_stream       Sets LabJack configuration for data streaming
get_stream       Gets streaming data
stop_stream      Stops data streaming
set_PWM          Sets LabJack configuration for PWM output
set_dutycycle    Sets duty cycle of PWM output (-100 to 100)
set_quadrature   Sets LabJack configuration for encoder A-B-Z input
get_counter      Gets edge count from encoder A-B signals
reset_counter    Resets edge counter
===============  ==================================================


Installation
------------

>>> py -m pip install labjack-unified

Installing **labjack-unified** will install its dependencies: *LabJackPython* and *labjack-ljm*.
*Plotly* will also be installed so graphs can be displayed when running the examples.


Examples
--------
Nine examples are available, with four of them requiring only a pair of wires to be executed
successfully. Below is the code for `lj_io_analog.py`:

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


.. warning::
   Due to their implementation, the classes **LabJackU3** and **LabJackU6** only
   work on a Windows platform.


Issues and questions
--------------------

If you have a feature request, a bug report, or even a question, please open an
`issue on GitHub <https://github.com/EduardoNigro/labjack-unified/issues/new>`__.