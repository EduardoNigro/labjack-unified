Electric Motor with Encoder
===========================

In this example code, the motor speed is ramped up for 5 seconds and down for 5 seconds,
as the PWM output is ramped linearly from 0 to 100 % and then back to 0. The encoder is
used to measure the shaft angular position, from which the motor speed can be calulated.


.. code-block:: python

    """ lj_motor_encoder.py 

    Uses PWM to set the speed of DC electric motor and an encoder do measure it.

    This example shows how to use the PWM output to change the speed of a DC motor.
    The PWM output is ramped up to 100% and then back down to 0%.

    Setup:
        You will need:
        - Electric motor with encoder (I used a CQRobot DC 6V-183RPM/12V-366RPM)
        - An H-Bridge (I used a DROK L298 Dual H Bridge Motor Speed Controller)
        - A 12V power supply
        Connect DAC0 to the bridge DIR1 (or IN1)
        Connect DAC1 to the bridge DIR2 (or IN2)
        On a U6, connect FIO2 to the bridge PWM IN (or ENA)
        On a T7, connect FIO0 to the bridge PWM IN (or ENA)
        Connect the bridge Vcc and GND to the LabJack VS and GND respectively
        On a U6, connect FIO0 to phase A and FIO1 to phase B
        On a T7, connect FIO2 to phase A and FIO3 to phase B
        Connect the encoder Vcc and GND to the LabJack VS and GND respectively

    The LabJack methods in this example are:
        set_PWM .......... Sets LabJack configuration for PWM output
        set_dutycycle .... Sets duty cycle of PWM output (-100 to 100)
        set_pwm_quad ..... Sets simultaneous PWM output and encoder input (U6)
        close ............ Closes the LabJack device 

    Notes:
        1. LabJack U3 does not have enought timers to run a PWM and a quadrature
        encoder at the sme time.
        2. LabJack U6 has a special method (set_pwm_quad) to configure both the PWM
        output and the quadrature input at the same time.
        3. If the measured speed is negative, either switch the direction wires
        connected to DAC0 and DAC1, or the encoder phase wires connected to FIO0
        and FIO1.

    """
    import time
    import numpy as np
    from scipy import signal
    from labjack_unified.utils import plot_line
    from labjack_unified.devices import LabJackU6, LabJackT7

    # To use a LabJack T7, change the device name
    # below from LabJackU6 below to LabJackT7
    lj = LabJackU6()

    # Assigning parameters
    tramp = 5 # PWM ramp up/down time (s)
    t = [] # Output time array
    pwm = [] # Output pwm signal array
    w = [] # Motor angular speed array (rad/s)

    if lj.__class__ == LabJackU6:
        # Configuring PWM and quadrature at the same time
        lj.set_pwm_quad()
    elif lj.__class__ == LabJackT7:
        # Configuring PWM putput
        lj.set_pwm(dirport1='DAC')
        # Setting quadrature incremental encoder
        lj.set_quadrature()
    else:
        print('LabJack U3 does not support this feature.')
    # Defining encoder pulses per revolution
    # (Because rising and falling edges on phases A and B
    #  are counted, multiply the encoder PPR by 4)
    ppr = 4 * 300

    # Initializing timers and starting main clock
    angleprev = 0
    tprev = 0
    tcurr = 0
    tstart = time.perf_counter()
    # Executing acquisition loop
    print('Running code for ' + str(2*tramp) + ' seconds ...')
    while tcurr <= 2*tramp:
        # Updating previous time and getting current time (s)
        tcurr = time.perf_counter() - tstart
        # Calculating pwm output
        if tcurr < tramp:
            # Ramping up to 100% for the first `tramp` seconds
            pwmcurr = 100/tramp*tcurr
        else:
            # Ramping down to 0% for the last `tramp` seconds
            pwmcurr = 100-100/tramp*(tcurr-tramp)
        # Updating PWM output
        lj.set_dutycycle(value1=pwmcurr)
        # Getting angular position of the encoder
        anglecurr = np.pi / ppr * lj.get_counter()
        # Calculating current angular speed (rad/s)
        wcurr = (anglecurr - angleprev) / (tcurr - tprev)
        # Updating previous values
        angleprev = anglecurr
        tprev = tcurr
        # Appending values to output arrays
        t.append(tcurr)
        pwm.append(pwmcurr)
        w.append(wcurr)
    lj.set_dutycycle(value1=0)
    print('Done.')
    # Closing the device
    lj.close()
    del lj

    # Filtering motor speed
    b, a = signal.butter(1, 0.01)
    wf = signal.filtfilt(b, a, w, method='gust')
    # Plotting results 
    plot_line([t] * 2, [pwm, wf],
            yname=['PWM OUtput (%)', 'Filt. Motor Speed (rad/s)'], axes='multi')
    plot_line([t[1::]], [1000*np.diff(t)], yname=['Sampling Period (ms)'])
