T7 Streaming (external clock)
=============================

This example code shows the usage of an external clock for data streaming.
External clocks are particularly useful for rotary machines or internal combustion
engines, where most phenomena are a function of the angular position of the shaft.


.. code-block:: python

    """ t7_streaming_external.py 

    Collects cycle data using an external clock.

    The external clock is generated using the output of a simple encoder, or
    one of the phases of a quadrature encoder, connected to digital port CIO3.
    The train of pulses allows for data to be collected as a function of angular
    position, independent of angular speed. This is typically used for rotary
    machines.

    A closed-loop PI controller aiming for a buffer backlog defined by the user
    is used to account for oscilations in angular velocity. This is accomplished
    by adjusting the duration of the wait between blocks of data in real time.
    Once the streaming starts, data must be pulled from the LabJack buffer at the
    appropriate rate to avoid an overflow. The PI loop is especially useful for
    extended periods of data acquisition.

    The `readrate` value must be selected so that the PI controller can have a
    good reponse. Values between 0.2 and 1.0 seconds seem appropriate.
    For example, for an expected angular speed of 360 rpm, and a block sample
    period of 0.5 seconds, the `readrate` can be calculated as follows:

        readrate (rev) = 360 (rev/min) / 60 (s/min) * 0.5 (s) = 3

    Each block will contain `readrate` complete revolutions, each one with PPR
    (pulses per revolution) data points for each one of the measured channels.

    The `backlogSP` parameter defines the T7 backlog set point. Values around
    25 % seem reaonable. However, `backlogSP` and the PI gains `kp` and `ki`
    should be adjusted accordingly based on the application and how unsteady the
    angular speed is.

    A trigger port can be used so the angular ZERO position (and subsequent 360
    degree increments) line up with the trigger position. That can be accomplished
    by using the Z phase of a quadrature encoder, in this case, connected to the
    digital port DIO1.

    Setup:
        You will need:
        - Electric motor (I used a CQRobot DC 6V-183RPM/12V-366RPM)
        - An encoder with a Z phase (I used a CALT GHS38 rotary encoder)
        - A 12V variable power supply (so you can adjust the motor speed)
        Phase A of the encoder is connected to CIO3 (the external clock source)
        Phase Z is connected to DIO1 (the trigger reference marker)

    The LabJack methods in this example are:
        set_stream ....... Sets LabJack configuration for data streaming
        get_stream ....... Gets streaming data
        stop_stream ...... Stops data streaming
        close ............ Closes the LabJack device 

    """
    import time
    import numpy as np
    from labjack_unified.utils import plot_line
    from labjack_unified.devices import LabJackT7
    #
    # Make sure the motor is already running before you run this program!
    #
    # Assigning streaming parameters
    speed0 = 240 # Expected angular speed (rpm)
    readrate = 2 # Number of revolutions per read block
    nblocks = 100 # Number of blocks to acquire
    nmax = 4 # Maximum number of blocks for plotting
    ppr = 1000 # Encoder's pulses per revolution
    # Assining ports
    portlist = ['DIO1']
    porttrigger = 'DIO1'
    # Creating array with dummy values to enable concatenation
    data = np.zeros((1, len(portlist)))
    # Preallocating output speed
    speed = []

    # PI closed loop control of "backlog" size
    backlogSP = 25 # Desired "backlog" value (%)
    backlog = [] # Backlog data
    kp = 0.001 # Proportional gain
    ki = 0.0001 # Integral gain
    eprev = 0 # Previous error value
    uprev = 1 # Previous execution period adjustment factor

    # Calculating block duration (s)
    Ts = 60 * readrate/speed0
    # Opening LabJack
    lj = LabJackT7()
    # Closing motor relay
    lj.set_analog('DAC0', 5)
    # Configuring streaming
    lj.set_stream(portlist, scanrate=ppr, readrate=readrate,
                    clocksource='EXT', exttrigger=porttrigger)
    # Waiting for first block to become available
    time.sleep(Ts)
    # Executing acquisition loop
    t0 = time.time()
    for i in range(nblocks):
        # Starting overhead time watch
        tstart = time.time()
        # Getting one block of data
        dt, datablock, numscans, commbacklog, devbacklog = lj.get_stream()
        # Appending only last `nmax`
        if i >= nblocks-nmax:
            data = np.vstack((data, datablock))
        # Calculating and storing angular speed
        speed.append(60*readrate/np.sum(dt))
        # Calculating backlog error to set point value
        e = backlogSP - devbacklog
        # Calculating execution period adjustment factor
        u = uprev + kp*(e-eprev) + ki*Ts*e
        # Updating previous values
        eprev = e
        uprev = u
        # Storing backlog
        backlog.append(devbacklog)
        # Showing statistics
        print('Block :', i+1)
        print('Scans :', numscans)
        print('Comm Backlog : {:0.1f}'.format(commbacklog))
        print('U3 Backlog   : {:0.1f}'.format(devbacklog))
        print('Speed (rpm)  : {:0.1f}'.format(speed[-1]))
        # Pausing taking into account overhead and
        # trying to control to a desired backlogSP
        thead = time.time() - tstart
        time.sleep(max(0, u*(Ts-thead)))
    # Stopping streaming
    lj.stop_stream()
    # Opening motor relay
    lj.set_analog('DAC0', 0)
    # Closing LabJack
    lj.close()
    del lj

    # Removing first row of dummy data
    data = data[1:-1, :]
    # Creating angle array
    ang = 360/ppr * np.linspace(0, data.shape[0]-1, data.shape[0])
    # Setting x and y arrays for plotting
    naxes = len(portlist)
    x = [ang] * naxes
    y = [data[:, i] for i in range(naxes)]
    # Plotting results
    plot_line(x, y, xname='Angle (deg.)', yname=portlist)
    plot_line([np.arange(nblocks)], [speed], xname='Block Number',
            yname=['Motor Speed (rpm)'])
    plot_line([np.arange(nblocks)], [backlog], xname='Block Number',
            yname=['LabJack Backlog (%)'])
