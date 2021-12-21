Absolute Encoder
================

In this example code, the reference mark (Z phase pulse) is determined by slowly turning
the encoder shaft. Then, the absolute angular position is displayed during 30 seconds as
the shaft is turned.


.. code-block:: python

    """ lj_encoder_absolute.py 

    Displays quadrature encoder angular speed.

    This example code shows how to setup a quadrature encoder and how to calculate
    and display the angular position of the shaft. A rotary encoder with A-B and Z
    phases is required for this short coding example. A CALT GHS38 rotary encoder
    was used.

    Setup:
        On a U3, connect FIO4 to phase A, FIO5 to phase B, and FIO6 to phase Z
        On a U6, connect FIO0 to phase A, FIO1 to phase B, and FIO2 to phase Z
        On a T7, connect FIO2 to phase A, FIO3 to phase B, and FIO1 to phase Z
        Connect the encoder Vcc and GND to the LabJack VS and GND respectively

    The LabJack unified methods in this example are:
        set_quadrature ... Sets LabJack configuration for encoder A-B-Z input
        get_counter ...... Gets edge count from encoder A-B signals
        close ............ Closes the LabJack device 

    """
    import time
    from labjack_unified.devices import LabJackU3, LabJackU6, LabJackT7

    # To use a different LabJack, change the device name
    # from LabJackU3 below to either LabJackU6 or LabJackT7
    lj = LabJackU3()

    # Selecting phase Z port name based on the LabJack class
    if lj.__class__ == LabJackU3:
        portZ = 'FIO6'
    elif lj.__class__ == LabJackU6:
        portZ = 'FIO2'
    else:
        portZ = 'FIO1'

    # Setting quadrature encoder with Z phase
    lj.set_quadrature(zphase1=True)
    # Defining encoder pulses per revolution
    # (Because rising and falling edges on phases A and B
    #  are counted, multiply the encoder PPR by 4)
    ppr = 4*1000

    # Execution loop to get Z phase and reset count
    flagref = False
    print('Turn encoder until reference is found.')
    while not flagref:
        if lj.get_digital(portZ) == 1:
            flagref = True
            print('Found it!')

    # Initializing variables and starting main clock
    angleprev = 0
    tcurr = 0
    tstart = time.perf_counter()
    # 30-second execution loop that displays the
    # current angular position of the encoder shaft
    time.sleep(1)
    print('Running code for 30 seconds ...')
    while tcurr <= 30:
        # Pausing for a bit
        time.sleep(0.1)
        # Getting current time (s)
        tcurr = time.perf_counter() - tstart
        # Getting angular position of the encoder
        anglecurr = 360/ppr * lj.get_counter()
        # Printing angle when it changes
        if anglecurr != angleprev:
            print('Position = {:0.1f} deg.'.format(anglecurr))
            angleprev = anglecurr
    print('Done.')

    # Closing the device
    lj.close()
    del lj