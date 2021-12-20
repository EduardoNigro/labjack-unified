
"""
devices.py contains unified LabJack classes for the U3, U6 and T7.
A common set of methods is implemented, simplifying writing code for any
of the devices listed above.

The classes are built using the existing LabJack python packages:
    - LabJackPython
    - labjack-ljm-python

Note: Use code folding at the method level to improve readability.

Author: Eduardo Nigro
    rev 0.0.3
    2021-12-13
"""
import re
import time
import numpy as np

# Importing LJM and LJM classes used in
# the LabJackT7 class
from labjack.ljm import ljm, LJMError

# Importing LabJackPython classes and functions used in
# the LabJackU3 and LabJackU6 classes
import u3, u6
import ctypes
from LabJackPython import (
    LabJackException,
    _loadLibrary,
    ePut,
    eGet,
    AddRequest,
    GoOne
)

# Importing constants used in the LabJackU3 and LabJackU6 classes
# See LabJackPython.py for more details on each constant
from LabJackPython import (
    LJ_chALL_CHANNELS,
    LJ_chNUMBER_TIMERS_ENABLED,
    LJ_chSTREAM_BACKLOG_COMM,
    LJ_chSTREAM_BACKLOG_UD,
    LJ_chSTREAM_BUFFER_SIZE,
    LJ_chSTREAM_SCAN_FREQUENCY,
    LJ_chSTREAM_WAIT_MODE,
    LJ_chTIMER_CLOCK_BASE,
    LJ_chTIMER_CLOCK_DIVISOR,
    LJ_chTIMER_COUNTER_PIN_OFFSET,
    LJ_ioADD_STREAM_CHANNEL,
    LJ_ioCLEAR_STREAM_CHANNELS,
    LJ_ioGET_AIN,
    LJ_ioGET_AIN_DIFF,
    LJ_ioGET_ANALOG_ENABLE_PORT,
    LJ_ioGET_CONFIG,
    LJ_ioGET_DIGITAL_BIT,
    LJ_ioGET_DIGITAL_BIT_DIR,
    LJ_ioGET_STREAM_DATA,
    LJ_ioGET_TIMER,
    LJ_ioPIN_CONFIGURATION_RESET,
    LJ_ioPUT_AIN_RANGE,
    LJ_ioPUT_ANALOG_ENABLE_BIT,
    LJ_ioPUT_ANALOG_ENABLE_PORT,
    LJ_ioPUT_CONFIG,
    LJ_ioPUT_DAC,
    LJ_ioPUT_DIGITAL_BIT,
    LJ_ioPUT_TIMER_MODE,
    LJ_ioPUT_TIMER_VALUE,
    LJ_ioSTART_STREAM,
    LJ_ioSTOP_STREAM,
    LJ_rgBIP10V,
    LJ_rgBIP1V,
    LJ_rgBIPP1V,
    LJ_rgBIPP01V,
    LJ_swNONE,
    LJ_tc48MHZ,
    LJ_tc48MHZ_DIV,
    LJ_tmPWM16,
    LJ_tmQUAD,
)


class LabJackU3:
    """
    The class to represent the LabJack U3.


    :param serialnum: The device serial number.
    :type serialnum: int

    Ports that are made available with this class are listed below.
    The port names assume a U3-HV.

        * Analog Output: ``'DAC0'``, ``'DAC1'``
        * Analog Input: ``'AIN0'``, ``'AIN1'``, ``'AIN2'``, ``'AIN3'``
        * Flexible I/O: ``'FIO4'``, ``'FIO5'``, ``'FIO6'``, ``'FIO7'``, 
        * Flexible I/O: ``'EIO0'``, ``'EIO1'``, ... , ``'EIO7'``

    Device-specific methods:

        * `get_config` - Gets port configuration
        * `reset_config` - Resets port configuration to factory defaults
        * `get_bitdir` - Gets digital port bit direction


    Connect to the first found U3:

        >>> from labjack_unified.devices import LabJackU3
        >>> lju3 = LabJackU3()
        >>> lju3.display_info()
        >>> lju3.close()

    You can also connect to a specific device using its serial number:

        >>> lju3 = LabJackU3(320012345)

    """
    # CONSTRUCTOR METHODS
    def __init__(self, serialnum=None):
        """
        Class constructor

        """
        self._staticlib = _loadLibrary()
        # Attempting to open the device
        try:
            if serialnum:
                # Opens LabJack with specific serial number
                self._commhandle = u3.U3(autoOpen=False)
                self._commhandle.open(serial=serialnum)
            else:
                # Opens first available LabJack
                self._commhandle = u3.U3(autoOpen=False)
                self._commhandle.open()
        except LabJackException as error:
            print(error)
        else:
            self._numports = 16
            self._assign_info(self._commhandle.configU3())
            self._assign_ports()
            self.reset_config()
            print('Opened LabJack', self._serialnum)

    def close(self):
        """
        Close the U3 device connection.

        >>> lju3.close()

        """
        self._staticlib.Close(self._commhandle.handle)
        print('Closed LabJack', self._serialnum)

    def display_info(self):
        """
        Displays a summary with the U3 device information.
        
        """
        print('____________________________________________________________')
        print('Device Name........', self._type)
        print('Serial Number......', self._serialnum)
        print('Hardware Version...', self._hardware)
        print('Firmware Version...', self._firmware)
        print('Connection Type....', self._connection)
        print('____________________________________________________________')

    # I/O METHODS
    def set_config(self, name, config):
        """
        Set the LabJack flexible IO port configuration.


        :param name: The port name to configure.
        :type name: str

        :param config:  The configuration option. 
            If `name` is ``'ALL'`` a string of mask bits for all 16 ports
            is used. For single port, `config` can be either a string
            ``'ANALOG'`` or ``'DIGITAL'``. Alternativelly, 1 or 0 can be used.
        :type config: str, int


        Configure flexible ports ``'FIO4'`` and ``'FIO5'`` as analog:

            >>> lju3.set_config('FIO4', 'Analog')
            >>> lju3.set_config('FIO5', 1)

        Configure flexible ports ``'EIO0'`` and ``'EIO1'`` as digital:
            
            >>> lju3.set_config('EIO0', 'Digital')
            >>> lju3.set_config('EIO1', 0)

        Configure flexible ports ``'EI01'`` and ``'EI03'`` as analog and
        ports ``'AIN0'`` to ``'AIN3'`` to analog (if a U3-LV is used):

        >>> lju3.set_config('ALL', '0000101000001111')

        .. note::
            1. LSB (Least Significant Bit) is port ``'AIN0'``.
            2. On a U3-HV, the first 4 ports are always analog and the first 4 LSB settings are ignored.

        """
        if name.lower() == 'all':
            # Checking for correct mask string length
            if len(config) == self._numports:
                ePut(self._commhandle.handle, LJ_ioPUT_ANALOG_ENABLE_PORT,
                     0, int(config, 2), self._numports)
            else:
                raise Exception("'ALL' ports CONFIG must be 16-bit string.")
        else:
            # Checking for valid input port names
            if not self._check_port(name, "AIN"):
                raise Exception('Invalid flexible IO port name.')
            # Checking for valid configuration options
            if type(config) == str:
                if config.lower() == 'analog':
                    config = 1
                elif config.lower() == 'digital':
                    config = 0
                else:
                    raise Exception(
                        "Port configuration must be either 'DIGITAL' or 'ANALOG'.")
            else:
                if (config != 0) and (config != 1):
                    raise Exception('Port configuration must be either 0 or 1')
            ePut(self._commhandle.handle, LJ_ioPUT_ANALOG_ENABLE_BIT,
                 self._get_AINnumber(name), config, 0)

    def get_config(self, name='ALL'):
        """
        Get the LabJack flexible IO port configuration.


        :param name: The port name to get the configuration.
            If `name` is ``'ALL'`` a dictionary with keys `EIOAnalog` and
            `FIOAnalog` is returned.
        :type name: str

        :returns: For single port `name`, an integer is returned, where 
            `1=Analog` and `0=Digital`. If `name` is ``'ALL'`` a dictionary
            containing the bit mask string for the `EIO` and `FIO` ports is
            returned. The least significant bit (LSB) of the `EIO` and `FIO`
            bit strings correspond respectively to ports ``'EIO0'`` and
            ``'FIO0'``.
        :rtype: int, dict


        Get flexible port configuration:

            >>> lju3.get_config()

        .. note::
            In the case of a LabJack U3-HV, the four LSBs of the `FIO` ports
            are always `1s`. They correspond to the always analog ports
            ``'AIN0'`` to ``'AIN3'``.

        """
        mask = eGet(self._commhandle.handle, LJ_ioGET_ANALOG_ENABLE_PORT,
                    0, 0, self._numports)
        mask = '{:016b}'.format(int(mask))
        if name.lower() == 'all':
            config = {
                'EIOAnalog': mask[0:8],
                'FIOAnalog': mask[8::]
            }
        else:
            if not self._check_port(name, "AIN"):
                raise Exception('Invalid flexible IO port name.')
            config = int(mask[self._numports - self._get_AINnumber(name) - 1])
        return config

    def reset_config(self):
        """
        Reset the flexible IO ports to default all digital.

            >>> lju3.reset_config()

        .. note::
            On the LabJack U3-HV the first 4 ports ``'AIN0'`` to ``'AIN3'``
            are always analog.

        """
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)

    def set_digital(self, name, state):
        """
        Write the digital state to an output port.
        It also sets the port direction to output.


        :param name: The port name to set the state.
        :type name: str

        :param state: The digital state `0 = Low`, `1 = High`.
        :type state: int


        Set port ``'FIO4'`` output to high and port ``'FIO5'`` to low:

            >>> lju3.set_digital('FIO4', 1)
            >>> lju3.set_digital('FIO5', 0)

        """
        # Checking for valid inputs
        if not self._check_port(name, "AIN"):
            raise Exception('Invalid flexible IO port name.')
        if (state != 0) and (state != 1):
            raise Exception('Port state must be either 1 or 0')
        # Setting port state by port number
        ePut(self._commhandle.handle, LJ_ioPUT_DIGITAL_BIT,
             self._get_AINnumber(name), state, 0)

    def get_digital(self, name):
        """
        Read the digital state from an input port.
        It also sets the port direction to input.


        :param name: The port name to get the state.
        :type name: str

        :returns: The state of the digital port. `0 = Low`, `1 = High`.
        :rtype: int


        Get port ``'FIO6'`` input state:

            >>> lju3.get_digital('FIO6')

        """
        # Checking for valid inputs
        if not self._check_port(name, "AIN"):
            raise Exception('Invalid flexible IO port name.')
        # Getting port state by port number
        state = int(eGet(self._commhandle.handle, LJ_ioGET_DIGITAL_BIT,
                         self._get_AINnumber(name), 0, 0))
        return state

    def get_bitdir(self, name):
        """
        Read the direction of the digital port.


        :param name: The port name to get the direction.
        :type name: str

        :returns: The direction of the digital port. `Input` or `Output`.
        :rtype: str


        Get the direction of port ``'FIO6'``:

            >>> lju3.get_bitdir('FIO6')

        """
        # Checking for valid input port names
        if not self._check_port(name, "AIN"):
            raise Exception('Invalid analog input port name.')
        # Getting digital port bit direction
        value = eGet(self._commhandle.handle, LJ_ioGET_DIGITAL_BIT_DIR,
                     self._get_AINnumber(name), 0, 0)
        if value == 0:
            bitdir = 'input'
        else:
            bitdir = 'output'
        return bitdir

    def set_analog(self, name, value):
        """
        Set analog output voltage.


        :param name: The port name to set the output voltage.
            Available ports are ``'DAC0'`` and ``'DAC1'``.
        :type name: str

        :param value: The output voltage between ``0`` and ``5`` V.
        :type value: float


        Set port ``'DAC1'`` output voltage to ``2.2`` V:

            >>> lju3.set_analog('DAC1', 2.2)

        """
        # Checking for valid input port names
        if not self._check_port(name, "DAC"):
            raise Exception('Invalid analog output port name.')
        # Getting corresponding port number
        DACnumber = self._get_DACnumber(name)
        # Limiting analog values
        if value > 5: value = 5
        if value < 0: value = 0
        # Setting analog port output voltage
        ePut(self._commhandle.handle, LJ_ioPUT_DAC, DACnumber, value, 0)

    def get_analog(self, namepos, *args):
        """
        Get analog input voltage.


        :param name: The positive port name to get the voltage.
        :type name: str

        :param args: Can be one of the three options:

            * ``'single-ended'`` (default value)
            * The negative port name to get a differential voltage.
            * ``'special'`` to get increased range on the voltage reading.

        :type args: str

        :returns: The input voltage value.

            * On a U3-HV, the range for ports ``'AIN0'`` to ``'AIN3'`` is +/-10 V
            * On a U3-HV, ``'special'`` enables a range of -10 to +20 V
            * `FIO` ports have a range of +/- 2.4 V
            * `FIO` ports using ``'special'`` have a range of 0 to 3.6 V

        :rtype: float


        Get single-ended voltage on port ``'FIO2'``:

            >>> lju3.get_analog('FIO2')

        Get differential voltage betweens ports ``'AIN0'`` and ``'AIN1'``:
        
            >>> lju3.get_analog('AIN0', 'AIN1')

        Get special range voltage on port ``'FIO3'``:

            >>> lju3.get_analog('FIO3', 'special')

        """
        # Checking for differential measurement
        if len(args) > 0:
            nameneg = args[0]
        else:
            nameneg = 'single-ended'
        # Checking for valid input port names
        if not self._check_port([namepos, nameneg], "AIN"):
            raise Exception('Invalid analog input port name.')
        # Getting corresponding port numbers
        AINpos = self._get_AINnumber(namepos)
        if nameneg.lower() == 'single-ended':
            AINneg = 31
        elif nameneg.lower() == 'special':
            AINneg = 32
        else:
            AINneg = self._get_AINnumber(nameneg)
        # Getting analog input voltage
        value = eGet(self._commhandle.handle,
                        LJ_ioGET_AIN_DIFF, AINpos, 0, AINneg)
        return value

    # STREAMING METHODS
    def set_stream(self, names, scanrate=50000, readrate=0.5):
        """
        Set and start data streaming.


        :param name: The U3 port name (or list of names) to be streamed.
        :type name: str, list(str)

        :param scanrate:  The scan rate (Hz) of the data streaming. 
            The default (and maximum) value is ``50000`` Hz. The effective scan
            frequency of each port is the scan rate divided by the number of
            scanned ports.
        :type scanrate: int

        :param readrate:
            The rate in seconds at which blocks of data are retrieved from the
            data buffer. The default value is ``0.5`` seconds.
        :type readrate: float


        Set data streaming on port ``'AIN0'`` at ``25000`` Hz, every ``0.5`` s:
            
            >>> lju3.set_stream('AIN0', scanrate=25000, readrate=0.5)

        Set data streaming on ports ``'AIN0'`` and ``'AIN1'`` at ``50000`` Hz,
        every ``1.0`` s:

            >>> lju3.set_stream(['AIN0', 'AIN1'], scanrate=50000, readrate=1.0)

        .. note::
            Only analog input ports ``'AIN0'`` to ``'AIN3'`` can be
            streamed. Hence, a Labjack U3-HV has to be used. While it's
            possible to stream digital ports, that hasn't been implemented
            in this release.

        """
        # Assigning streamed data block read-in rate (s)
        self._streamreadrate = readrate
        # Assigning buffer size as 2 times block length (s)
        self._streambuffersize = 2*self._streamreadrate
        # Assigning data block scan rate (Hz)
        self._scanrate = scanrate
        # Checking for valid input port names
        if not self._check_port(names, "AIN"):
            raise Exception('Invalid input port name.')
        # Getting port numbers
        if type(names) != list:
            names = [names]
        portnum = []
        for name in names:
            portnum.append(self._get_AINnumber(name))
        # Assigning number of streaming ports
        self._streamnumchannels = len(portnum)
        # Updating scan frequency per port (Hz)
        self._streamscanfreq = int(self._scanrate/self._streamnumchannels)
        # Clearing streaming ports
        ePut(self._commhandle.handle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0)
        # Adding ports to scan list
        for num in portnum:
            ePut(self._commhandle.handle, LJ_ioADD_STREAM_CHANNEL, num, 0, 0)
        # Assigning scanning frequency
        ePut(self._commhandle.handle, LJ_ioPUT_CONFIG,
             LJ_chSTREAM_SCAN_FREQUENCY, self._streamscanfreq, 0)
        # Assigning buffer size
        ePut(self._commhandle.handle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE,
             self._scanrate*self._streambuffersize, 0)
        # Configuring reads to retrieve whatever data is available without waiting
        ePut(self._commhandle.handle, LJ_ioPUT_CONFIG,
             LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0)
        # Starting streaming
        ePut(self._commhandle.handle, LJ_ioSTART_STREAM, 0, 0, 0)

    def stop_stream(self):
        """
        Stop data streaming.

            >>> lju3.stop_stream()

        """
        ePut(self._commhandle.handle, LJ_ioSTOP_STREAM, 0, 0, 0)

    def get_stream(self):
        """
        Get streaming data block.


        :returns: 5-tuple

            * dt
            
                The sampling period (s) between each data point.

            * data

                The numpy `m-by-n` array containing the streamed data where
                `m` is the number of samples per port in the block and `n`
                is the number of ports defined in `set_stream`

            * numscans
            
                The actual number of scans per port in the data block.

            * commbacklog
            
                The communication backlog in % (increasing values indicate that
                the computer cannot keep up with the data download from the U3
                driver)

            * devbacklog
            
                The U3 device backlog in % (increasing values indicate that the
                device cannot keep up with the data streaming - usually not
                the case)

        :rtype: (float, ndarray, int, float, float)


        Retrieve scan period, data, and scan info:

            >>> dt, datablock, numscans, commbacklog, U3backlog = lju3.get_stream()

        """
        # Defining initial number of samples per channnel
        # (double the size for safety)
        numscans = 2 * self._streamreadrate * self._streamscanfreq
        # Preallocating output array
        datasamples = np.zeros(int(numscans)*self._streamnumchannels)
        numscansactual, datalong = self._eGetArray(
            self._commhandle.handle, LJ_ioGET_STREAM_DATA,
            LJ_chALL_CHANNELS, numscans, datasamples)
        # Separating ports
        numscansactual = int(numscansactual)
        nrow = numscansactual
        ncol = self._streamnumchannels
        data = np.array(datalong)
        data = data[0:nrow*ncol].reshape(nrow, ncol)
        # Calculating sample period (s)
        dt = 1/self._streamscanfreq
        # Calculating communication backlog (%)
        commbacklog = eGet(self._commhandle.handle,
                           LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, 0, 0)
        commbacklog = 100 * commbacklog/numscansactual
        # Calculating LabJack device backlog (%)
        devbacklog = eGet(self._commhandle.handle,
                          LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_UD, 0, 0)
        devbacklog = 100 * devbacklog/numscansactual
        # Returning streamed data block parameters
        return dt, data, numscansactual, commbacklog, devbacklog

    # PWM METHODS
    def set_pwm(self, pwmnum=1, dirport1=None, dirport2=None, frequency=366):
        """
        Configure PWM output.


        :param pwmnum: The number of PWM output signals.
            ``1`` or ``2`` PWMs can be used. For one PWM, the output port is
            ``'FIO4'``. For two PWMs, the output ports are ``'FIO4'`` and 
            ``'FIO5'``.
        :type pwmnum: int

        :param dirport1: The type of ports that control the PWM `direction`
            for electric motor control. There are three options:

            * ``None``  - Default value (no direction ports are used)
            * ``'DAC'`` - Uses analog ports ``'DAC0'`` and ``'DAC1'``
            * ``'DIO'`` - Uses digital ports ``'FIO6'`` and ``'FIO7'``

        :type dirport1: None, str

        :param dirport2: Same as `dirport1`.
            It's used when two PWM outputs are enabled. The ``'DAC'`` option
            can only be used for one set of direction ports, unless the two
            motors are running synchronously. For the ``'DIO'`` option,
            digital ports ``'EIO0'`` and ``'EIO1'`` are used.
        :type dirport2: None, str

        :param frequency: The PWM signal frequency in Hz.
            In the case of two PWMs, both will have the same frequency. Valid
            values are ``183``, ``366`` or ``732``.
        :type frequency: int


        Set 1 PWM for motor control on ``'FIO4'`` with direction ports on
        ``'DAC0'`` and ``'DAC1'``. The PWM frequency is the default ``366`` Hz:

            >>> lju3.set_pwm(dirport1='DAC')

        Set 2 PWMs on ports ``'FIO4'`` and ``'FIO5'`` with a frequency of
        ``183`` Hz:
        
            >>> lju3.set_pwm(pwmnum=2, frequency=183)

        Set 2 PWMs for motor control on ports ``'FIO4'`` and ``'FIO5'``, using
        the digital ports ``'FIO6'`` and ``'FIO7'`` for motor 1 direction, and
        ``'EIO0'`` and ``'EIO1'`` for motor 2 direction. The PWM frequency is
        ``732`` Hz:
        
            >>> lju3.set_pwm(pwmnum=2, dirport1='DIO', dirport2='DIO', frequency=732)


        .. note::
            When using digital ports, a 10 kOhm resistor has to be connected from
            the LabJack `VS` port to each one of the `DIO` ports to ensure true
            `high` and `low` states.

        """
        # Defining PWM frequency divisors and checking input
        pwmfreq = {
            '183': 4,
            '366': 2,
            '732': 1
        }
        if str(frequency) not in list(pwmfreq.keys()):
            raise Exception(
                "Valid PWM frequencies are 183, 366, and 732 Hz.")
        # Checking number of PWM outputs and direction ports
        dirport = [dirport1, dirport2]
        pwmname = ['FIO4']
        pwmdir = [None]
        self._pwmtype = [1]
        if dirport[0] == 'DAC':
            pwmdir[0] = ['DAC0', 'DAC1']
            self._pwmtype[0] = 0
        elif dirport[0] == 'DIO':
            pwmdir[0] = ['FIO6', 'FIO7']
        if pwmnum == 2:
            self._pwmtype.append(1)
            pwmname.append('FIO5')
            if dirport[1] is None:
                pwmdir.append(None)
            if dirport[1] == 'DAC':
                pwmdir.append(['DAC0', 'DAC1'])
                self._pwmtype[1] = 0
            elif dirport[1] == 'DIO':
                pwmdir.append(['EIO0', 'EIO1'])
        # Assigning PWM attributes
        self._pwmnum = pwmnum
        self._pwmname = pwmname
        self._pwmdir = pwmdir
        self._pwmfreq = frequency
        # Assinging frequency divisor
        divisor = pwmfreq[str(frequency)]
        # Assigning PWM pin offset
        # (always 4 since the timers always start on FIO4)
        pinoffset = 4
        # Resetting pin configuration
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)
        # Setting the pin offset for the timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_COUNTER_PIN_OFFSET, pinoffset, 0, 0)
        # Configuring the timer clock to 48 MHz
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ_DIV, 0, 0)
        # Setting clock Divisor
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_DIVISOR, divisor, 0, 0)
        # Enabling 1 or 2 timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chNUMBER_TIMERS_ENABLED, pwmnum, 0, 0)
        for num in range(pwmnum):
            # Setting timer to 16 bits
            # Frequency = (48MHz/Divisor ) / 2^16 Hz
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                    num, LJ_tmPWM16, 0, 0)
            # Setting timer duty cycle to 0%
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                    num, 0, 0, 0)
        # Sending command sequence
        GoOne(self._commhandle.handle)

    def set_dutycycle(self, value1=None, value2=None, brake1=False, brake2=False):
        """
        Set PWM duty cycle value.


        :param value1: The PWM 1 duty cycle percent value between ``-100``
            and ``100``.
        :type value1: float

        :param value2: The PWM 2 duty cycle percent value between ``-100``
            and ``100``.
        :type value2: float

        :param brake1: The motor 1 brake option used when dutycycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake1: bool

        :param brake2: The motor 2 brake option used when dutycycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake2: bool


        Set duty cycle to ``50`` % on PWM 1:

            >>> lju3.set_dutycycle(value1=50)

        Set duty cycle to ``25`` % (reverse rotation) on PWM 2:

            >>> lju3.set_dutycycle(value2=-25)

        Set duty cycle to ``20`` % and ``40`` % on PWMs 1 and 2:
  
            >>> lju3.set_dutycycle(value1=20, value2=40)

        Stop motor 2 and apply brake:

            >>> lju3.set_dutycycle(value2=0, brake2=True)


        .. note::
            1. Avoid suddenly switching the direction of rotation to avoid damaging the motor.
            2. You can use the brake option True to hold the motor in position.

        """
        values = [value1, value2]
        brakes = [brake1, brake2]
        for pwmnum, (value, brake, pwmdir, pwmtype) in enumerate(zip(
            values, brakes, self._pwmdir, self._pwmtype)):

            if value is not None:
                # Applying bounds to inputs
                if value > 100:
                    value = 100
                if value < -100:
                    value = -100
                # Applying PWM direction
                if pwmdir:
                    if value > 0:
                        # Forward rotation
                        if pwmtype == 0:
                            self.set_analog(pwmdir[0], 4.5)
                            self.set_analog(pwmdir[1], 0)
                        else:
                            self.set_digital(pwmdir[0], 0)
                            self.get_digital(pwmdir[1])
                    elif value < 0:
                        # Reverse rotation
                        if pwmtype == 0:
                            self.set_analog(pwmdir[0], 0)
                            self.set_analog(pwmdir[1], 4.5)
                        else:
                            self.get_digital(pwmdir[0])
                            self.set_digital(pwmdir[1], 0)
                    elif value == 0:
                        # Brake stop
                        if brake:
                            if pwmtype == 0:
                                self.set_analog(pwmdir[0], 0)
                                self.set_analog(pwmdir[1], 0)
                            else:
                                self.set_digital(pwmdir[0], 0)
                                self.set_digital(pwmdir[1], 0)
                        else:
                            if pwmtype == 0:
                                self.set_analog(pwmdir[0], 4.5)
                                self.set_analog(pwmdir[1], 4.5)
                            else:
                                self.get_digital(pwmdir[0])
                                self.get_digital(pwmdir[1])
                # Calculating duty cycle
                dutycycle = np.ceil(65535*(1-np.abs(value)/100))

                # Setting timer duty cycle to desired value
                AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                            pwmnum, dutycycle, 0, 0)

        # Sending command sequence
        GoOne(self._commhandle.handle)

    # QUADRATURE ENCODER METHODS
    def set_quadrature(self, zphase1=False):
        """
        Configure quadrature encoder input on ports ``'FIO4'`` and ``'FIO5'``.


        :param zphase1: The logic value indicating if a `Z` phase reference
            pulse is used on port ``'FIO6'``. 
        :type zphase1: bool


        Set ports ``'FIO4'`` and ``'FIO5'`` for encoder phase `A` and `B`
        signals:
        
            >>> lju3.set_quadrature()

        Set ports ``'FIO4'`` and ``'FIO5'`` for encoder phase `A` and `B`
        signals and port ``'FIO6'`` for the reference `Z` phase:
            
            >>> lju3.set_quadrature(zphase1=True)

        """
        # Checking input arguments
        quadnameAB = ['FIO4', 'FIO5']
        portnumZ = 0
        if zphase1:
            portnumZ = 32768 + self._get_AINnumber('FIO6')
        # Assigning A-B port names
        self._zphase1 = zphase1
        self._quadnameAB = quadnameAB
        # Assigning quadrature port pin offset
        pinoffset = self._get_AINnumber(quadnameAB[0])
        # Resetting pin configuration
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)
        # Setting the pin offset for the timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_COUNTER_PIN_OFFSET, pinoffset, 0, 0)
        # Configuring the timer clock to 48 MHz
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ, 0, 0)
        # Enabling 2 timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0)
        # Setting quadrature mode
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                   0, LJ_tmQUAD, 0, 0)
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                   1, LJ_tmQUAD, 0, 0)
        # Setting timer values to add Z port (or reset in case there's no Z)
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                    0, portnumZ, 0, 0)
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                    1, portnumZ, 0, 0)
        # Sending command sequence
        GoOne(self._commhandle.handle)

    def get_counter(self):
        """
        Get current quadrature counter value.


        :returns: The counter value.
        :rtype: int

        
            >>> lju3.get_counter()


        .. note::
            Because the qudrature counter counts rising and falling edges
            of phases `A` and `B`, a 1024 pulse/rev encoder will generate 4096
            counts for a full shaft turn.

        """
        value = int(eGet(self._commhandle.handle, LJ_ioGET_TIMER, 0, 0, 0))
        return value

    def reset_counter(self):
        """
        Reset quadrature counter value.

            >>> lju3.reset_counter()

        .. note::
            The count is only reset when a `Z` phase isn't being used.

        """
        if not self._zphase1:
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                       0, 0, 0, 0)
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                       1, 0, 0, 0)
            # Sending command sequence
            GoOne(self._commhandle.handle)

    # OTHER METHODS
    def get_labjacktemp(self, unit='C'):
        """
        Get ambient temperature from LabJack's internal sensor.


        :param unit: The temperature measurement unit.
            Valid values are ``'C'`` or ``'F'``. Default unit is ``'C'``.
        :type unit: str

        :returns: The internal sensor temperature reading.
        :rtype: float


        Get temperature reading in `Celsius`:

            >>> lju3.get_labjacktemp()

        Get temperature reading in `Fahrenheit`:
        
            >>> lju3.get_labjacktemp(unit='F')

        """
        tempabs = eGet(self._commhandle.handle, LJ_ioGET_AIN, 30, 0, 0)
        if unit == 'C':
            temp = tempabs - 273.15
        elif unit == 'F':
            temp = 9/5*(tempabs-273.15) + 32
        else:
            raise Exception(
                "Temperature units must be either 'degC' or 'degF'.")
        return temp

    # HELPER METHODS (PRIVATE)
    def _eGetArray(self, Handle, IOType, Port, pValue, x1):
        """
        Perform one call to the LabJack Device returning a data array.
        
        This method was created to complement the eGet() function in
        LabJackPyhon. It's used primarily in data streaming and like all
        the other C library functions, it is limited to a Windows platform.

        """
        pv = ctypes.c_double(pValue)
        xv = (ctypes.c_double * len(x1))()
        ec = self._staticlib.eGet_DblArray(
            Handle, IOType, Port, ctypes.byref(pv), ctypes.byref(xv))
        if ec != 0:
            raise LabJackException(ec)
        return pv.value, xv

    def _assign_info(self, info):
        # Assigning info collected from LabJack handle to private attributes
        self._type = info['DeviceName']
        self._serialnum = info['SerialNumber']
        self._connection = 'USB'
        self._hardware = info['HardwareVersion']
        self._firmware = info['FirmwareVersion']

    def _assign_ports(self):
        """
        Creates lists with valid LabJack U3 port names
        
        """
        # Assigning analog output port names
        self._portDAC = [
            'DAC0', 'DAC1']
        # Assigning analog / FIO inout port names
        self._portAIN = [
            'AIN0', 'AIN1', 'AIN2', 'AIN3', 'FIO4', 'FIO5', 'FIO6', 'FIO7',
            'EIO0', 'EIO1', 'EIO2', 'EIO3', 'EIO4', 'EIO5', 'EIO6', 'EIO7']
        self._portAINalt = [
            'FIO0', 'FIO1', 'FIO2', 'FIO3', 'AIN4', 'AIN5', 'AIN6', 'AIN7',
            'AIN8', 'AIN9', 'AIN10', 'AIN11', 'AIN12', 'AIN13', 'AIN14', 'AIN15']
        self._portAINsp = ['SINGLE-ENDED', 'SPECIAL']

    def _check_port(self, namelist, porttype, *args):
        """
        Checks if the port NAMES belong to the possible values in
        the correposnding PORTTYPE list of valid names. If all NAMES
        are valid then FLAGPORT = 1, otherwise FLAGPORT = 0

        A user defined list can be passed with porttype = 'USER'
        through an additional argument VARARGIN

        """
        # Making sure inputs are lists
        if type(namelist) != list:
            namelist = [namelist]
        # Assining valid names list based on port type
        if porttype == "DAC":
            validnames = self._portDAC
        elif porttype == "AIN":
            validnames = self._portAIN + self._portAINalt + self._portAINsp
        elif porttype == "QUAD":
            validnames = [
                'FIO4', 'FIO5', 'FIO6', 'FIO7',
                'AIN4', 'AIN5', 'AIN6', 'AIN7']
        elif porttype == "USER":
            validnames = args[0]
        # Returns true if all names in namelist are validnames
        return all([name.upper() in validnames for name in namelist])

    def _get_DACnumber(self, name):
        if name.upper() == 'DAC0':
            DACnumber = 0
        elif name.upper() == 'DAC1':
            DACnumber = 1
        return DACnumber

    def _get_AINnumber(self, name):
        try:
            AINnumber = self._portAIN.index(name)
        except ValueError:
            AINnumber = self._portAINalt.index(name)
        return AINnumber


class LabJackU6:
    """
    The class to represent the LabJack U6


    :param serialnum: The device serial number.
    :type serialnum: int

    Ports that are made available with this class are:

        * Analog Output (0 to 5V) : ``'DAC0'`` , ``'DAC1'``
        * Analog Input (+/-10V)   : ``'AIN0'`` , ``'AIN1'``, ... , ``'AIN13'``
        * Digital I/O             : ``'FIO0'`` , ``'FIO1'``, ... , ``'FIO7'``
        * Digital I/O             : ``'EIO0'`` , ``'EIO1'``, ... , ``'EIO7'``

    Device-specific methods:

        * `get_bitdir` - Gets digital port bit direction
        * `set_range` - Sets analog input voltage range
        * `set_pwm_quad` - Sets simultaneous PWM output and encoder input


    Connect to the first found U6:

        >>> from labjack_unified.devices import LabJackU6
        >>> lju6 = LabJackU6()
        >>> lju6.display_info()
        >>> lju6.close()

    You can also connect to a specific device using its serial number:

        >>> lju6 = LabJackU6(370012345)

    """
    # CONSTRUCTOR METHODS
    def __init__(self, serialnum=None):
        """
        Class constructor

        """
        self._staticlib = _loadLibrary()
        # Attempting to open the device
        try:
            if serialnum:
                # Opens LabJack with specific serial number
                self._commhandle = u6.U6(autoOpen=False)
                self._commhandle.open(serial=serialnum)
            else:
                # Opens first available LabJack
                self._commhandle = u6.U6(autoOpen=False)
                self._commhandle.open()
        except LabJackException as error:
            print(error)
        else:
            self._numports = 34
            self._assign_info(self._commhandle.configU6())
            self._assign_ports()
            self._reset_config()
            self.set_range('all', 10)
            print('Opened LabJack', self._serialnum)

    def close(self):
        """
        Close the U6 device connection.

        >>> lju6.close()

        """
        self._staticlib.Close(self._commhandle.handle)
        print('Closed LabJack', self._serialnum)

    def display_info(self):
        """
        Displays a summary with the U6 device information.
        
        """
        print('____________________________________________________________')
        print('Device Name........', self._type)
        print('Serial Number......', self._serialnum)
        print('Hardware Version...', self._hardware)
        print('Firmware Version...', self._firmware)
        print('Connection Type....', self._connection)
        print('____________________________________________________________')

    # I/O METHODS
    def set_digital(self, name, state):
        """
        Write the digital state to an output port.
        It also sets the port direction to output.


        :param name: The port name to set the state.
        :type name: str

        :param state: The digital state `0 = Low`, `1 = High`.
        :type state: int


        Set port ``'FIO0'`` output to high and port ``'FIO1'`` to low:

            >>> lju6.set_digital('FIO0', 1)
            >>> lju6.set_digital('FIO1', 0)

        """
        # Checking for valid inputs
        if not self._check_port(name, "DIO"):
            raise Exception('Invalid digital IO port name.')
        if (state != 0) and (state != 1):
            raise Exception('Port state must be either 1 or 0')
        # Setting port state by port number
        ePut(self._commhandle.handle, LJ_ioPUT_DIGITAL_BIT,
             self._get_DIOnumber(name), state, 0)

    def get_digital(self, name):
        """
        Read the digital state from an input port.
        It also sets the port direction to input.


        :param name: The port name to get the state.
        :type name: str

        :returns: The state of the digital port. `0 = Low`, `1 = High`.
        :rtype: int


        Get port ``'FIO2'`` input state:

            >>> lju6.get_digital('FIO2')

        """
        # Checking for valid inputs
        if not self._check_port(name, "DIO"):
            raise Exception('Invalid digital IO port name.')
        # Getting port state by port number
        state = int(eGet(self._commhandle.handle, LJ_ioGET_DIGITAL_BIT,
                         self._get_DIOnumber(name), 0, 0))
        return state

    def get_bitdir(self, name):
        """
        Read the direction of the digital port.


        :param name: The port name to get the direction.
        :type name: str

        :returns: The direction of the digital port. `Input` or `Output`.
        :rtype: str


        Get the direction of port ``'FIO2'``:

            >>> lju6.get_bitdir('FIO2')

        """
        # Checking for valid input port names
        if not self._check_port(name, "DIO"):
            raise Exception('Invalid digital I/O port name.')
        # Getting digital port bit direction
        value = eGet(self._commhandle.handle, LJ_ioGET_DIGITAL_BIT_DIR,
                     self._get_DIOnumber(name), 0, 0)
        if value == 0:
            bitdir = 'input'
        else:
            bitdir = 'output'
        return bitdir

    def set_analog(self, name, value):
        """
        Set analog output voltage.


        :param name: The port name to set the output voltage.
            Available ports are ``'DAC0'`` and ``'DAC1'``.
        :type name: str

        :param value: The output voltage between ``0`` and ``5`` V.
        :type value: float


        Set port ``'DAC1'`` output voltage to ``2.2`` V:

            >>> lju6.set_analog('DAC1', 2.2)

        """
        # Checking for valid input port names
        if not self._check_port(name, "DAC"):
            raise Exception('Invalid analog output port name.')
        # Getting corresponding port number
        DACnumber = self._get_DACnumber(name)
        # Limiting analog values
        if value > 5: value = 5
        if value < 0: value = 0
        # Setting analog port output voltage
        ePut(self._commhandle.handle, LJ_ioPUT_DAC, DACnumber, value, 0)

    def get_analog(self, name, mode='single-ended'):
        """
        Get analog input voltage.


        :param name: The positive port name to get the voltage.
        :type name: str

        :param mode: Can be one of the two options:

            * ``'single-ended'`` (default value)
            * ``'differential'`` sets the ports to get a differential voltage.

        :type mode: str

        :returns: The input voltage value.
        :rtype: float


        Get ``'single-ended'`` voltage on port ``'AIN3'``:

            >>> lju6.get_analog('AIN3')

        Get ``'differential'`` voltage betweens ports ``'AIN2'`` and
        ``'AIN3'``:
        
            >>> lju6.get_analog('AIN2', 'differential')

        .. note::
            Differential reading uses two consecutive even-odd ports.
            Valid ports for differential reading are AIN0/2/4/6/8/10/12.

        """
        # Checking for valid inputs
        if mode.lower() not in ['single-ended', 'differential']:
            raise Exception(
                "Valid reference types are 'Single-Ended' or 'Differential'.")
        # Assigning range values
        if not self._check_port(name, "AIN"):
            raise Exception('Invalid analog input port name(s).')
        # Getting corresponding port numbers
        AINpos = self._get_AINnumber(name)
        #
        if mode.lower() == 'single-ended':
            AINneg = 199
        else:
            flagport, _ = self._check_portdiff(name)
            if flagport:
                AINneg = AINpos + 1
            else:
                raise Exception('Valid analog ports are AIN0/2/4/6/8/10/12.')
        # Getting analog input voltage
        value = eGet(self._commhandle.handle,
                     LJ_ioGET_AIN_DIFF, AINpos, 0, AINneg)
        return value

    # STREAMING METHODS
    def set_stream(self, names, scanrate=50000, readrate=0.5):
        """
        Set and start data streaming.


        :param name: The U6 port name (or list of names) to be streamed.
        :type name: str, list(str)

        :param scanrate:  The scan rate (Hz) of the data streaming. 
            The default (and maximum) value is ``50000`` Hz. The effective scan
            frequency of each port is the scan rate divided by the number of
            scanned ports.
        :type scanrate: int

        :param readrate:
            The rate in seconds at which blocks of data are retrieved from the
            data buffer. The default value is ``0.5`` seconds.
        :type readrate: float


        Set data streaming on port ``'AIN0'`` at ``25000`` Hz, every ``0.5`` s:
            
            >>> lju6.set_stream('AIN0', scanrate=25000, readrate=0.5)

        Set data streaming on ports ``'AIN0'`` and ``'AIN1'`` at ``50000`` Hz,
        every ``1.0`` s:

            >>> lju6.set_stream(['AIN0', 'AIN1'], scanrate=50000, readrate=1.0)


        .. note::
            Only analog input ports ``'AIN0'`` to ``'AIN13'`` can be
            streamed. While it's possible to stream digital ports,
            that hasn't been implemented in this release.

        """
        # Assigning streamed data block read-in rate (s)
        self._streamreadrate = readrate
        # Assigning buffer size as 2 times block length (s)
        self._streambuffersize = 2*self._streamreadrate
        # Assigning data block scan rate (Hz)
        self._scanrate = scanrate
        # Checking for valid input port names
        if not self._check_port(names, "AIN"):
            raise Exception('Invalid input port name.')
        # Getting port numbers
        if type(names) != list:
            names = [names]
        portnum = []
        for name in names:
            portnum.append(self._get_AINnumber(name))
        # Assigning number of streaming ports
        self._streamnumchannels = len(portnum)
        # Updating scan frequency per port (Hz)
        self._streamscanfreq = int(self._scanrate/self._streamnumchannels)
        # Clearing streaming ports
        ePut(self._commhandle.handle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0)
        # Adding ports to scan list
        for num in portnum:
            ePut(self._commhandle.handle, LJ_ioADD_STREAM_CHANNEL, num, 0, 0)
        # Assigning scanning frequency
        ePut(self._commhandle.handle, LJ_ioPUT_CONFIG,
             LJ_chSTREAM_SCAN_FREQUENCY, self._streamscanfreq, 0)
        # Assigning buffer size
        ePut(self._commhandle.handle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE,
             self._scanrate*self._streambuffersize, 0)
        # Configuring reads to retrieve whatever data is available without waiting
        ePut(self._commhandle.handle, LJ_ioPUT_CONFIG,
             LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0)
        # Starting streaming
        ePut(self._commhandle.handle, LJ_ioSTART_STREAM, 0, 0, 0)

    def stop_stream(self):
        """
        Stop data streaming.

            >>> lju6.stop_stream()

        """
        ePut(self._commhandle.handle, LJ_ioSTOP_STREAM, 0, 0, 0)

    def get_stream(self):
        """
        Get streaming data block.


        :returns: 5-tuple

            * dt
            
                The sampling period (s) between each data point.

            * data

                The numpy `m-by-n` array containing the streamed data where
                `m` is the number of samples per port in the block and `n`
                is the number of ports defined in `set_stream`

            * numscans
            
                The actual number of scans per port in the data block.

            * commbacklog
            
                The communication backlog in % (increasing values indicate that
                the computer cannot keep up with the data download from the U6
                driver)

            * devbacklog
            
                The U6 device backlog in % (increasing values indicate that the
                device cannot keep up with the data streaming - usually not
                the case)

        :rtype: (float, ndarray, int, float, float)


        Retrieve scan period, data, and scan info:

            >>> dt, datablock, numscans, commbacklog, U3backlog = lju6.get_stream()

        """
        # Defining initial number of samples per channnel
        # (double the size for safety)
        numscans = 2 * self._streamreadrate * self._streamscanfreq
        # Preallocating output array
        datasamples = np.zeros(int(numscans)*self._streamnumchannels)
        numscansactual, datalong = self._eGetArray(
            self._commhandle.handle, LJ_ioGET_STREAM_DATA,
            LJ_chALL_CHANNELS, numscans, datasamples)
        # Separating ports
        numscansactual = int(numscansactual)
        nrow = numscansactual
        ncol = self._streamnumchannels
        data = np.array(datalong)
        data = data[0:nrow*ncol].reshape(nrow, ncol)
        # Calculating sample period (s)
        dt = 1/self._streamscanfreq
        # Calculating communication backlog (%)
        commbacklog = eGet(self._commhandle.handle,
                           LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, 0, 0)
        commbacklog = 100 * commbacklog/numscansactual
        # Calculating LabJack device backlog (%)
        devbacklog = eGet(self._commhandle.handle,
                          LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_UD, 0, 0)
        devbacklog = 100 * devbacklog/numscansactual
        # Returning streamed data block parameters
        return dt, data, numscansactual, commbacklog, devbacklog

    # PWM METHODS
    def set_pwm(self, pwmnum=1, dirport1=None, dirport2=None, frequency=366):
        """
        Configure PWM output.


        :param pwmnum: The number of PWM output signals.
            ``1`` or ``2`` PWMs can be used. For one PWM, the output port is
            ``'FIO0'``. For two PWMs, the output ports are ``'FIO0'`` and 
            ``'FIO1'``.
        :type pwmnum: int

        :param dirport1: The type of ports that control the PWM `direction`
            for electric motor control. There are three options:

            * ``None``  - Default value (no direction ports are used)
            * ``'DAC'`` - Uses analog ports ``'DAC0'`` and ``'DAC1'``
            * ``'DIO'`` - Uses digital ports ``'FIO2'`` and ``'FIO3'``

        :type dirport1: None, str

        :param dirport2: Same as `dirport1`.
            It's used when two PWM outputs are enabled. The ``'DAC'`` option
            can only be used for one set of direction ports, unless the two
            motors are running synchronously. For the ``'DIO'`` option,
            digital ports ``'FIO4'`` and ``'FIO5'`` are used.
        :type dirport2: None, str

        :param frequency: The PWM signal frequency in Hz.
            In the case of two PWMs, both will have the same frequency. Valid
            values are ``183``, ``366`` or ``732``.
        :type frequency: int


        Set 1 PWM for motor control on ``'FIO0'`` with direction ports on
        ``'DAC0'`` and ``'DAC1'``. The PWM frequency is the default ``366`` Hz:

            >>> lju6.set_pwm(dirport1='DAC')

        Set 2 PWMs on ports ``'FIO0'`` and ``'FIO1'`` with a frequency of
        ``183`` Hz:
        
            >>> lju6.set_pwm(pwmnum=2, frequency=183)

        Set 2 PWMs for motor control on ports ``'FIO0'`` and ``'FIO1'``, using
        the digital ports ``'FIO2'`` and ``'FIO3'`` for motor 1 direction, and
        ``'FIO4'`` and ``'FIO5'`` for motor 2 direction. The PWM frequency is
        ``732`` Hz:
        
            >>> lju6.set_pwm(pwmnum=2, dirport1='DIO', dirport2='DIO', frequency=732)


        .. note::
            When using digital ports, a 10 kOhm resistor has to be connected from
            the LabJack `VS` port to each one of the `DIO` ports to ensure true
            `high` and `low` states.

        """
        # Setting flag for simultaneous PWN and quadrature setup
        self._pwmquad = False
        # Defining PWM frequency divisors and checking input
        pwmfreq = {
            '183': 4,
            '366': 2,
            '732': 1
        }
        if str(frequency) not in list(pwmfreq.keys()):
            raise Exception(
                "Valid PWM frequencies are 183, 366, and 732 Hz.")
        # Checking number of PWM outputs and direction ports
        dirport = [dirport1, dirport2]
        pwmname = ['FIO0']
        pwmdir = [None]
        self._pwmtype = [1]
        if dirport[0] == 'DAC':
            pwmdir[0] = ['DAC0', 'DAC1']
            self._pwmtype[0] = 0
        elif dirport[0] == 'DIO':
            pwmdir[0] = ['FIO2', 'FIO3']
        if pwmnum == 2:
            self._pwmtype.append(1)
            pwmname.append('FIO1')
            if dirport[1] is None:
                pwmdir.append(None)
            if dirport[1] == 'DAC':
                pwmdir.append(['DAC0', 'DAC1'])
                self._pwmtype[1] = 0
            elif dirport[1] == 'DIO':
                pwmdir.append(['FIO4', 'FIO5'])
        # Assigning PWM attributes
        self._pwmnum = pwmnum
        self._pwmname = pwmname
        self._pwmdir = pwmdir
        self._pwmfreq = frequency
        # Assinging frequency divisor
        divisor = pwmfreq[str(frequency)]
        # Assigning PWM pin offset
        # (always 0 since the timers always start on FIO0)
        pinoffset = 0
        # Resetting pin configuration
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)
        # Setting the pin offset for the timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_COUNTER_PIN_OFFSET, pinoffset, 0, 0)
        # Configuring the timer clock to 48 MHz
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ_DIV, 0, 0)
        # Setting clock Divisor
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_DIVISOR, divisor, 0, 0)
        # Enabling 1 or 2 timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chNUMBER_TIMERS_ENABLED, pwmnum, 0, 0)
        for num in range(pwmnum):
            # Setting timer to 16 bits
            # Frequency = (48MHz/Divisor ) / 2^16 Hz
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                       num, LJ_tmPWM16, 0, 0)
            # Setting timer duty cycle to 0%
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                       num, 0, 0, 0)
        # Sending command sequence
        GoOne(self._commhandle.handle)

    def set_dutycycle(self, value1=None, value2=None, brake1=False, brake2=False):
        """
        Set PWM duty cycle value.


        :param value1: The PWM 1 duty cycle percent value between ``-100``
            and ``100``.
        :type value1: float

        :param value2: The PWM 2 duty cycle percent value between ``-100``
            and ``100``.
        :type value2: float

        :param brake1: The motor 1 brake option used when dutycycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake1: bool

        :param brake2: The motor 2 brake option used when dutycycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake2: bool


        Set duty cycle to ``50`` % on PWM 1:

            >>> lju6.set_dutycycle(value1=50)

        Set duty cycle to ``25`` % (reverse rotation) on PWM 2:

            >>> lju6.set_dutycycle(value2=-25)

        Set duty cycle to ``20`` % and ``40`` % on PWMs 1 and 2:
  
            >>> lju6.set_dutycycle(value1=20, value2=40)

        Stop motor 2 and apply brake:

            >>> lju6.set_dutycycle(value2=0, brake2=True)


        .. note::
            1. Avoid suddenly switching the direction of rotation to avoid damaging the motor.
            2. You can use the brake option True to hold the motor in position.

        .. note::
            If the method `set_pwm_quad` was used to configure both a PWM and
            a quadrature encoder, use only `value1` and `brake1` to control
            the motor output.

        """
        values = [value1, value2]
        brakes = [brake1, brake2]
        for pwmnum, (value, brake, pwmdir, pwmtype) in enumerate(zip(
            values, brakes, self._pwmdir, self._pwmtype)):

            if value is not None:
                # Applying bounds to inputs
                if value > 100:
                    value = 100
                if value < -100:
                    value = -100
                # Applying PWM direction
                if pwmdir:
                    if value > 0:
                        # Forward rotation
                        if pwmtype == 0:
                            self.set_analog(pwmdir[0], 4.5)
                            self.set_analog(pwmdir[1], 0)
                        else:
                            self.set_digital(pwmdir[0], 0)
                            self.get_digital(pwmdir[1])
                    elif value < 0:
                        # Reverse rotation
                        if pwmtype == 0:
                            self.set_analog(pwmdir[0], 0)
                            self.set_analog(pwmdir[1], 4.5)
                        else:
                            self.get_digital(pwmdir[0])
                            self.set_digital(pwmdir[1], 0)
                    elif value == 0:
                        # Brake stop
                        if brake:
                            if pwmtype == 0:
                                self.set_analog(pwmdir[0], 0)
                                self.set_analog(pwmdir[1], 0)
                            else:
                                self.set_digital(pwmdir[0], 0)
                                self.set_digital(pwmdir[1], 0)
                        else:
                            if pwmtype == 0:
                                self.set_analog(pwmdir[0], 4.5)
                                self.set_analog(pwmdir[1], 4.5)
                            else:
                                self.get_digital(pwmdir[0])
                                self.get_digital(pwmdir[1])
                # Calculating duty cycle
                dutycycle = np.ceil(65535*(1-np.abs(value)/100))

                # Setting timer duty cycle to desired value
                if self._pwmquad:
                    AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                               2, dutycycle, 0, 0)
                else:
                    AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                               pwmnum, dutycycle, 0, 0)

        # Sending command sequence
        GoOne(self._commhandle.handle)

    # QUADRATURE ENCODER METHODS
    def set_quadrature(self, quadnum=1, zphase1=False, zphase2=False):
        """
        Configure quadrature encoder input on ports ``'FIO4'`` and ``'FIO5'``.


        :param quadnum: The number of quadrature input signals.
            ``1`` or ``2`` encoders can be used. For one encoder, the input
            ports are ``'FIO0'`` and ``'FIO1'``. For two encoders, the input
            ports for the second one are ``'FIO2'`` and ``'FIO3'``. 
        :type quadnum: int

        :param zphase1: The logic value indicating if a `Z` phase reference
            pulse is used for the first encoder. Port ``'FIO2'`` is used if
            `quadnum` = ``1``. Port ``'FIO4'`` is used if `quadnum` = ``2``. 
        :type zphase1: bool

        :param zphase2: The logic value indicating if a `Z` phase reference
            pulse is used for the second encoder. Port ``'FIO4'`` is used for
            the first encoder and port ``'FIO5'`` is used for the second 
            encoder. `zphase2` is ignored if `quadnum` = ``1``.
        :type zphase1: bool


        Set ports ``'FIO0'`` and ``'FIO1'`` for encoder with phase `A` and `B`
        signals only:
        
            >>> lju6.set_quadrature()

        Set ports ``'FIO0'`` and ``'FIO1'`` for encoder phase `A` and `B`
        signals, and port ``'FIO2'`` for the reference `Z` phase:
        
            >>> lju6.set_quadrature(zphase1=True)

        Set 2 encoders with `Z` phase. `A` and `B` phases are on ports
        ``'FIO0'`` and ``'FIO1'`` for encoder 1, and ``'FIO2'`` and ``'FIO3'``
        for encoder 2. The `Z` phase ports are respectively ``'FIO4'`` and
        ``'FIO5'``:

            >>> lju6.set_quadrature(quadnum=2, zphase1=True, zphase2=True)

        """
        # Setting flag for simultaneous PWN and quadrature setup
        self._pwmquad = False
        # Checking input arguments
        if quadnum == 1:
            quadnameAB = ['FIO0', 'FIO1']
            portnumZ = [0]
            if zphase1:
                portnumZ[0] = 32768 + self._get_DIOnumber('FIO2')
        elif quadnum == 2:
            quadnameAB = ['FIO0', 'FIO1', 'FIO2', 'FIO3']
            portnumZ = [0, 0]
            if zphase1:
                portnumZ[0] = 32768 + self._get_DIOnumber('FIO4')
            if zphase2:
                portnumZ[1] = 32768 + self._get_DIOnumber('FIO5')
        else:
            raise Exception('Only 1 or 2 quadrature inputs can be assigned.')
        # Assigning quadrature attributes
        self._zphase = [zphase1, zphase2]
        self._quadnum = quadnum
        self._quadnameAB = quadnameAB
        # Assigning quadrature port pin offset
        pinoffset = 0
        # Resetting pin configuration
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)
        # Setting the pin offset for the timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_COUNTER_PIN_OFFSET, pinoffset, 0, 0)
        # Configuring the timer clock to 48 MHz
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ, 0, 0)
        # Enabling 2 or 4 timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chNUMBER_TIMERS_ENABLED, 2*quadnum, 0, 0)
        for num in range(quadnum):
            # Setting quadrature mode
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                    (2*num)+0, LJ_tmQUAD, 0, 0)
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                    (2*num)+1, LJ_tmQUAD, 0, 0)
            # Setting timer values to add Z port (or reset in case there's no Z)
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                    (2*num)+0, portnumZ[num], 0, 0)
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                    (2*num)+1, portnumZ[num], 0, 0)
        # Sending command sequence
        GoOne(self._commhandle.handle)

    def get_counter(self):
        """
        Get current quadrature counter value.


        :returns: The counter value or a list with 2 values for 2 encoders.
        :rtype: int, list(int)

        
        >>> lju6.get_counter()


        .. note::
            Because the qudrature counter counts rising and falling edges
            of phases `A` and `B`, a 1024 pulse/rev encoder will generate 4096
            counts for a full shaft turn.

        """
        value = [int(eGet(self._commhandle.handle, LJ_ioGET_TIMER, 0, 0, 0))]
        if self._quadnum == 2:
            value.append(int(eGet(self._commhandle.handle, LJ_ioGET_TIMER, 2, 0, 0)))
        else:
            value = value[0]
        return value

    def reset_counter(self, counter1=True, counter2=True):
        """
        Reset quadrature counter value.


        :param counter1: The flag indicating whether to reset counter 1 or not.
            The default value is ``True`` and it resets the counter.
        :type counter1: bool

        :param counter2: The flag indicating whether to reset counter 2 or not.
            The default value is ``True`` and it resets the counter.
        :type counter2: bool


        Resets current counter value of all encoders.

            >>> lju6.reset_counter()

        Resets current counter value only for second encoder.
        
            >>> lju6.reset_counter(counter1=False)


        .. note::
            The count is only reset when a `Z` phase isn't being used.

        .. note::
            If the method `set_pwm_quad` was used to configure both a PWM and
            a quadrature encoder, use only `counter1` to reset the counter.

        """
        if not self._zphase[0] and counter1:
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                       0, 0, 0, 0)
            AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                       1, 0, 0, 0)
            # Sending command sequence
            GoOne(self._commhandle.handle)
        if self._quadnum == 2:
            if not self._zphase[1] and counter2:
                AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                        2, 0, 0, 0)
                AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                        3, 0, 0, 0)
                # Sending command sequence
                GoOne(self._commhandle.handle)

    # OTHER METHODS
    def set_pwm_quad(self, dirport='DAC', zphase=False):
        """
        Configure 1 PWM output and 1 quadrature input.
        The PWM port is ``'FIO2'`` and the phases `A` and `B` ports are
        respectively ``'FIO0'`` and ``'FIO1'``.


        :param dirport: The type of ports that control the PWM `direction`
            for electric motor control. There are three options:

            * ``None``  - Default value (no direction ports are used)
            * ``'DAC'`` - Uses analog ports ``'DAC0'`` and ``'DAC1'``
            * ``'DIO'`` - Uses digital ports ``'FIO4'`` and ``'FIO5'``

        :type dirport1: None, str

        :param zphase: The logic value indicating if a `Z` phase reference
            pulse is used on port ``'FIO3'``. 
        :type zphase: bool


        Set a PWM for motor control on ``'FIO2'`` with direction ports on
        ``'DAC0'`` and ``'DAC1'``. The encoder `A` and `B` ports are ``'FIO0'``
        and ``'FIO1'``:

            >>> lju6.set_pwm_quad(dirport='DAC')

        Set a PWM for motor control on ``'FIO2'`` with direction ports on
        ``'FIO4'`` and ``'FIO5'``. The A-B-Z encoder `A` and `B` ports are
        ``'FIO0'`` and ``'FIO1'``. The `Z` phase is on port ``'FIO3'``:

            >>> lju6.set_pwm_quad(dirport='DIO', zphase=True)


        .. note::
            Due to limitations with internal clocks under this configuration,
            the PWM frequency is fixed at 732 Hz.

        """
        # Setting flag for simultaneous PWN and quadrature setup
        self._pwmquad = True
        # Assigning A-B port names
        quadnameAB = ['FIO0', 'FIO1']
        # Assigning Z port number
        portnumZ = 0
        if zphase:
            portnumZ = 32768 + self._get_DIOnumber('FIO3')
        # Assigning PWM ports
        pwmname = ['FIO2']
        if dirport == 'DAC':
            self._pwmtype = [0]
            pwmdir = [['DAC0', 'DAC1']]
        elif dirport == 'DIO':
            self._pwmtype = [1]
            pwmdir = [['FIO4', 'FIO5']]
        else:
            raise Exception("Valid direction port types are 'DAC' or 'DIO'.")
        # Setting some attributes
        self._quadnum = 1
        self._zphase = [zphase]
        self._quadnameAB = quadnameAB
        self._pwmnum = 1
        self._pwmname = pwmname
        self._pwmdir = pwmdir
        #
        # Setting timers for quadrature and PWM
        #
        # Assigning timer pin offset
        pinoffset = 0
        # Resetting pin configuration
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)
        # Setting the pin offset for the timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_COUNTER_PIN_OFFSET, pinoffset, 0, 0)
        # Configuring the timer clock to 48 MHz
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ, 0, 0)
        # Enabling 3 timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_CONFIG,
                   LJ_chNUMBER_TIMERS_ENABLED, 3, 0, 0)
        #
        # Setting up quadrature input configuration
        #
        # Setting quadrature timers
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                   0, LJ_tmQUAD, 0, 0)
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                   1, LJ_tmQUAD, 0, 0)
        # Setting timer values to add Z port (or reset in case there's no Z)
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                   0, portnumZ, 0, 0)
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                   1, portnumZ, 0, 0)
        #
        # Setting up PWM output configuration
        #
        # Setting third timer to 16-bit PWM
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_MODE,
                   2, LJ_tmPWM16, 0, 0)
        # Setting timer duty cycle to 0%
        AddRequest(self._commhandle.handle, LJ_ioPUT_TIMER_VALUE,
                   2, 0, 0, 0)

        # Sending command sequence
        GoOne(self._commhandle.handle)

    def set_range(self, names, ranges):
        """
        Set analog input voltage range.


        :param names: The analog port(s) that will have their ranges modified.
            Use ``'ALL'`` to set all analog input ports to the same range.
        :type names: str, list(str)

        :param ranges: The voltage range value to be used.
            Valid ranges are +/- ``10``, ``1``, ``0.1``, ``0.01`` V. If a
            single value is used, it will be applied to all ports in
            `names`.
        :type ranges: float, list(float)


        Set port ``'AIN0'`` with a range of +/- ``1`` V:
       
            >>> lju6.set_range('AIN0', 1)

        Set port ``'AIN0'`` and ``'AIN2'`` with a range of +/- ``0.1`` and
        ``0.01`` V:

            >>> lju6.set_range(['AIN0', 'AIN2'], [0.1, 0.01])

        Set all ports with a (default) range of +/- ``10`` V:
        
            >>> lju6.set_range('ALL', 10)

        """
        #
        rcode = {
            '10': LJ_rgBIP10V,
            '1': LJ_rgBIP1V,
            '0.1': LJ_rgBIPP1V,
            '0.01': LJ_rgBIPP01V
        }
        # Checking for valid inputs
        if type(names) != list: names = [names]
        if type(ranges) != list: ranges = [ranges]
        if (len(names) != len(ranges)) and (len(ranges) > 1):
            raise Exception('Number of ports must match number of ranges.')
        if len(ranges) == 1:
            ranges = ranges * len(names)
        for r in ranges:
            if r not in [10, 1, 0.1, 0.01]:
                raise Exception('Valid ranges are 10, 1, 0.1, or 0.01 V.')
        if names[0].lower() == 'all':
            names = self._portAIN
        else:
            if not self._check_port(names, "AIN"):
                raise Exception('Invalid analog input port name(s).')
        # Assigning range values
        for name, r in zip(names, ranges):
            AddRequest(self._commhandle.handle, LJ_ioPUT_AIN_RANGE,
                       self._get_AINnumber(name), rcode[str(r)], 0, 0)
        # Sending command sequence
        GoOne(self._commhandle.handle)

    def get_labjacktemp(self, unit='C'):
        """
        Get ambient temperature from LabJack's internal sensor.


        :param unit: The temperature measurement unit.
            Valid values are ``'C'`` or ``'F'``. Default unit is ``'C'``.
        :type unit: str

        :returns: The internal sensor temperature reading.
        :rtype: float


        Get temperature reading in `Celsius`:

            >>> lju6.get_labjacktemp()

        Get temperature reading in `Fahrenheit`:
        
            >>> lju6.get_labjacktemp(unit='F')

        """
        tempabs = eGet(self._commhandle.handle, LJ_ioGET_AIN, 30, 0, 0)
        if unit == 'C':
            temp = tempabs - 273.15
        elif unit == 'F':
            temp = 9/5*(tempabs-273.15) + 32
        else:
            raise Exception(
                "Temperature units must be either 'degC' or 'degF'.")
        return temp

    # HELPER METHODS (PRIVATE)
    def _eGetArray(self, Handle, IOType, Port, pValue, x1):
        """
        Perform one call to the LabJack Device returning a data array.
        
        This method was created to complement the eGet() function in
        LabJackPyhon. It's used primarily in data streaming and like all
        the other C library functions, it is limited to a Windows platform.

        """
        pv = ctypes.c_double(pValue)
        xv = (ctypes.c_double * len(x1))()
        ec = self._staticlib.eGet_DblArray(
            Handle, IOType, Port, ctypes.byref(pv), ctypes.byref(xv))
        if ec != 0:
            raise LabJackException(ec)
        return pv.value, xv

    def _assign_info(self, info):
        # Assigning info collected from LabJack handle to private attributes
        self._type = info['DeviceName']
        self._serialnum = info['SerialNumber']
        self._connection = 'USB'
        self._hardware = info['HardwareVersion']
        self._firmware = info['FirmwareVersion']

    def _assign_ports(self):
        """
        Creates lists with valid LabJAck T7 port names
        """
        # Assigning analog output port names
        self._portDAC = [
            'DAC0', 'DAC1']
        # Assigning analog inout port names
        self._portAIN = [
            'AIN0', 'AIN1', 'AIN2', 'AIN3', 'AIN4', 'AIN5', 'AIN6', 'AIN7',
            'AIN8', 'AIN9', 'AIN10', 'AIN11', 'AIN12', 'AIN13', 'AIN14']
        # Assigning digital I/O port names
        self._portDIO = [
            'DIO0', 'DIO1', 'DIO2', 'DIO3', 'DIO4', 'DIO5',
            'DIO6', 'DIO7', 'DIO8', 'DIO9', 'DIO10', 'DIO11',
            'DIO12', 'DIO13', 'DIO14', 'DIO15', 'DI16', 'DI17',
            'DIO18', 'DIO19', 'DIO20', 'DIO21', 'DIO22']
        # Assigning alternate digital I/O port names
        self._portDIOalt = [
            'FIO0', 'FIO1', 'FIO2', 'FIO3', 'FIO4', 'FIO5',
            'FIO6', 'FIO7', 'EIO0', 'EIO1', 'EIO2', 'EIO3',
            'EIO4', 'EIO5', 'EIO6', 'EIO7', 'CIO0', 'CIO1',
            'CIO2', 'CIO3', 'MIO0', 'MIO1', 'MIO2']

    def _reset_config(self):
        """
        Reset the flexible IO ports to default all digital.

        Parameters
        ----------
        None.

        Returns
        -------
        None.

        Example
        -------
        Reset flexible port configuration
        >>> lju3.reset_config()

        Notes
        -----
        On the LabJack U3-HV the first 4 ports AIN0 to AIN3 are always analog.

        """
        ePut(self._commhandle.handle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0)

    def _check_port(self, namelist, portype, *args):
        """
        Checks if the port NAMES belong to the possible values in
        the correposnding PORTTYPE list of valid names. If all NAMES
        are valid then FLAGPORT = 1, otherwise FLAGPORT = 0

        A user defined list can be passed with PORTYPE = 'USER'
        through an additional argument VARARGIN

        """
        # Making sure inputs are lists
        if type(namelist) != list:
            namelist = [namelist]
        # Assining valid names list based on port type
        if portype == "DAC":
            validnames = self._portDAC
        elif portype == "AIN":
            validnames = self._portAIN
        elif portype == "DIO":
            validnames = self._portDIO + self._portDIOalt
        elif portype == "ALLIN":
            validnames = self._portAIN + self._portDIO + self._portDIOalt
        elif portype == "QUAD":
            validnames = [
                'DIO0', 'DIO1', 'DIO2', 'DIO3', 'DIO4', 'DIO5', 'DIO6', 'DIO7', 'DIO8',
                'FIO0', 'FIO1', 'FIO2', 'FIO3', 'FIO4', 'FIO5', 'FIO6', 'FIO7', 'EIO0']
        elif portype == "USER":
            validnames = args[0]
        # Returns true if all names in namelist are validnames
        return all([name in validnames for name in namelist])

    def _check_portdiff(self, names):
        # Making sure inputs are lists
        if type(names) != list:
            names = [names]
        # Getting port name digits
        portnum = [int(re.findall(r'\d+', name)[0]) for name in names]
        # Checking for all even digits
        flagport = all([(num % 2)==0 for num in portnum])
        # Checking for AIN14
        if 14 in portnum:
            flagport = False
        # Returning flag and port numbers
        return flagport, portnum

    def _get_DACnumber(self, name):
        if name.upper() == 'DAC0':
            DACnumber = 0
        elif name.upper() == 'DAC1':
            DACnumber = 1
        return DACnumber

    def _get_AINnumber(self, name):
        try:
            AINnumber = self._portAIN.index(name)
        except ValueError:
            AINnumber = self._portAINalt.index(name)
        return AINnumber

    def _get_DIOnumber(self, name):
        try:
            DIOnumber = self._portDIO.index(name)
        except ValueError:
            DIOnumber = self._portDIOalt.index(name)
        return DIOnumber


class LabJackT7:
    """
    The class to represent the LabJack T7


    :param serialnum: The device serial number.
    :type serialnum: int

    Ports that are made available with this class are:

        * Analog Output (0 to 5V) : ``'DAC0'`` , ``'DAC1'``
        * Analog Input (+/-10V)   : ``'AIN0'`` , ``'AIN1'``, ... , ``'AIN13'``
        * Digital I/O             : ``'FIO0'`` , ``'FIO1'``, ... , ``'FIO7'``
        * Digital I/O             : ``'EIO0'`` , ``'EIO1'``, ... , ``'EIO7'``

    Device-specific methods:

        * `set_range` - Sets analog input voltage range
        * `set_reference` - Sets analog input reference voltage point
        * `set_TC` - Sets LabJack configuration for thermocouple input
        * `get_TCtemp` - Gets thermocouple temperature reading


    Connect to the first found U6:

        >>> from labjack_unified.devices import LabJackT7
        >>> ljt7 = LabJackT7()
        >>> ljt7.display_info()
        >>> ljt7.close()

    You can also connect to a specific device using its serial number.

        >>> ljt7 = LabJackT7(370012345)

    """
    # CONSTRUCTOR METHODS
    def __init__(self, serialnum=None):
        """
        Class constructor

        """
        try:
            if serialnum:
                # Opens LabJack with specific serial number
                self._commhandle = ljm.openS("ANY", "ANY", serialnum)
            else:
                # Opens first available LabJack
                self._commhandle = ljm.openS("ANY", "ANY", "ANY")
        except LJMError as error:
            print(error)
        else:
            # Assigning internal attributes
            self._corefreq = 80e6
            self._clockdivisor = 1
            self._assign_info(ljm.getHandleInfo(self._commhandle))
            self._assign_ports()
            self.set_reference('all', mode='Single-Ended')
            self.set_range('all', 10)
            print('Opened LabJack', self._serialnum)

    def close(self):
        """
        Close the T7 device connection.

        >>> ljt7.close()

        """
        ljm.close(self._commhandle)
        print('Closed LabJack', self._serialnum)

    def display_info(self):
        """
        Displays a summary with the T7 device information.
        
        """
        print('____________________________________________________________')
        print('Device Name........', self._type)
        print('Serial Number......', self._serialnum)
        print('Hardware Version...', self._hardware)
        print('Firmware Version...', self._firmware)
        print('Connection Type....', self._connection)
        print('IP Address.........', self._ipaddress)
        print('Port...............', self._port)
        print('____________________________________________________________')

    # I/O METHODS
    def set_analog(self, names, values):
        """
        Set analog output voltage.


        :param name: The port name to set the output voltage.
            Available ports are ``'DAC0'`` and ``'DAC1'``. Both ports can be
            set at the same time using a list containing the two names.
        :type name: str, list(str)

        :param value: The output voltage between ``0`` and ``5`` V. A list
            containing values can be used in conjunction with a list with the
            two port names.
        :type value: float, list(float)


        Set port ``'DAC1'`` output voltage to ``2.2`` V:

            >>> ljt7.set_analog('DAC1', 2.2)

        Set port ``'DAC0'`` output voltage to ``2.5`` V and ``'DAC1'`` to
        ``3.2`` V:

            >>> ljt7.set_analog(['DAC0', 'DAC1'], [2.5, 3.2])

        """
        # Making sure inputs are lists
        if type(names) != list:
            names = [names]
        # Making sure values are ndarray of float64
        values = np.double(values)
        if type(values) == np.float64:
            values = [values]
        # Doing other input checks
        if len(names) != len(values):
            raise Exception('Number of ports must match number of values.')
        if not self._check_port(names, "DAC"):
            raise Exception('Invalid analog output port name(s).')
        # Limiting analog values
        values = [max(value, 0) for value in values]
        values = [min(value, 5) for value in values]
        # Setting port values
        self._set_port_values(names, values)

    def get_analog(self, names):
        """
        Get analog input voltage.


        :param name: THe port name (or list of names) to get the input voltage.
            Ports ``'AIN0'`` to ``'AIN13'`` are possible names and can read a
            range between -10 and +10V.
        :type name: str, list(str)

        :returns: The input voltage value.
        :rtype: float, list(float)


        Get input voltage on port ``'AIN0'``:
        
            >>> ljt7.get_analog('AIN0')

        Get input voltages on ports ``'AIN0'``, ``'AIN2'`` and ``'AIN3'``:
        
            >>> ljt7.get_analog(['AIN0', 'AIN2', 'AIN3'])


        .. note::
            1.  Ports that are not connected may have erratic readings.
            2.  See `set_range` and `set_reference` for more options on analog inputs.

        """
        # Making sure inputs are lists
        if type(names) != list:
            names = [names]
        # Checking for valid analog input names
        if not self._check_port(names, "AIN"):
            raise Exception('Invalid analog input port name(s).')
        # Getting values
        return self._get_port_values(names)

    def set_digital(self, names, values):
        """
        Write the digital state to an output port.
        It also sets the port direction to output.


        :param name: The port name (or a list of names) to set the state.
        :type name: str, list(str)

        :param state: The digital state (or a list of states)
            `0 = Low`, `1 = High`.
        :type state: int, list(int)


        Set port ``'FIO0'`` bit to high:
        
            >>> ljt7.set_digital('FIO0', 1)

        Set ports ``'FIO0'``, ``'FIO1'`` and ``'FIO6'`` bits to high,
        low and high:

            >>> ljt7.set_digital(['FIO0', 'FIO1', 'FIO6'], [1, 0, 1])

        """
        # Making sure inputs are lists
        if type(names) != list:
            names = [names]
        if type(values) != list:
            values = [values]
        # Doing other input checks
        if len(names) != len(values):
            raise Exception('Number of ports must match number of values.')
        if not self._check_port(names, "DIO"):
            raise Exception('Invalid digital I/O port name(s).')
        # Limiting digital values
        values = [max(value, 0) for value in values]
        values = [min(value, 1) for value in values]
        # Setting port values
        self._set_port_values(names, values)

    def get_digital(self, names):
        """
        Read the digital state from an input port.
        It also sets the port direction to input.


        :param name: The port name (or list of port names) to get the state.
        :type name: str, list(str)

        :returns: The state of the digital port. `0 = Low`, `1 = High`.
        :rtype: int, list(int)


        Get logic state on port ``'FIO2'``:

            >>> ljt7.get_digital('FIO2')

        Get logic states on ports ``'FIO2'``, ``'FIO3'`` and ``'FIO7'``:

            >>> ljt7.get_digital(['FIO2', 'FIO3', 'FIO7'])

        """
        if type(names) != list:
            names = [names]
        # Checking for valid analog input names
        if not self._check_port(names, "DIO"):
            raise Exception('Invalid digital I/O port name(s).')
        # Getting values
        states = self._get_port_values(names)
        if len(names) > 1:
            state = [int(value) for value in states]
        else:
            state = int(states)
        return state

    # STREAMING METHODS
    def set_stream(self, names, scanrate=100000, readrate=0.5,
                   clocksource='INT', exttrigger=None):
        """
        Set and start data streaming.


        :param name: The T7 port name (or list of names) to be streamed.
            Any analog and/or digital port can be used.
        :type name: str, list(str)

        :param scanrate:  The scan rate (Hz) of the data streaming. 
            The default (and maximum) value is ``100000`` Hz. The effective
            scan frequency of each port is the scan rate divided by the
            number of scanned ports. When `clocksource` is equal to
            ``'EXT'``, `scanrate` is interpreted as the number of pulses per
            block of data.
        :type scanrate: int

        :param readrate: The rate in seconds at which blocks of data are
            retrieved from the data buffer by `get_stream`. The default value
            is ``0.5`` seconds. When `clocksource` is equal to ``'EXT'``,
            `readrate` is interpreted as the number of blocks of data to be
            retrieved each time.
        :type readrate: float

        :param clocksource: The source of the streaming clock.
            It indicates whether the LabJack internal clock or an external
            clock (pulse train) will be used. In the case of an external clock,
            the digital signal must be connected to ``'CIO3'`` (``'DIO16'``).
            The default value is ``'INT'``.
        :type clocksource: str

        :param exttrigger: The `DIO` port name containing an external trigger
            used to start the streaming.
        :type exttrigger: str


        Configure ``100000`` Hz streaming with ``0.5`` s data blocks from port
        ``'AIN0'``:

            >>> ljt7.set_stream('AIN0')

        Configure streaming for ``1`` s data blocks from analog and digital
        ports.

            >>> ljt7.set_stream(['AIN0', 'AIN1', 'DIO6', 'DIO7'], readrate=1)

        Configure streaming for ``0.5`` s data blocks from analog ports.
        The scan rate is ``50000`` Samples/s (25000 S/s for each port):

            >>> ljt7.set_stream(['AIN0','AIN1'], scanrate=50000)

        Configure streaming for 1 block of 1024 Samples/port per data retrieve.
        The external clock signal must be connected to ``'CIO3'``
        (``'DIO16'``):

            >>> ljt7.set_stream(['AIN0','AIN1'], scanrate=1024, readrate=1, clocksource='EXT')

        Configure streaming for 3 blocks of 600 Samples/port.
        Port ``'DIO1'`` is connected to an external trigger that will start the
        streaming. The external clock signal must be connected to ``'CIO3'``
        (``'DIO16'``):

            >>> ljt7.set_stream(['AIN0','DIO0'],
                                scanrate=600, readrate=3, clocksource='EXT',
                                exttrigger='DIO1')


        .. note::
            `exttrigger` port must use the `DIO` naming convention, i.e.: ports
            ``'DIO0'`` to ``'DIO16'``.

        .. note::
            Data streaming starts immediatelly after `set_stream` is invoked,
            unless `exttrigger` is used.

        """
        # Making sure inputs are lists
        names = names.copy()
        if type(names) != list:
            names = [names]
        # Checking for valid analog input names
        if not self._check_port(names, "ALLIN"):
            raise Exception('Invalid input port name(s).')
        # Limiting and interpreting scan rate
        if clocksource.upper() == 'INT':
            maxrate = min(scanrate, 100000)
        elif clocksource.upper() == 'EXT':
            maxrate = min(scanrate*(len(names)+1), 100000)
        else:
            raise Exception(
                'Stream clock source must be either "INT" or "EXT"')
        # Adding core timer register name to shallow copy of port list
        if clocksource.upper() == "EXT":
            names.append("CORE_TIMER")
        # Assining external clock trigger parameters
        if exttrigger:
            if not self._check_port(exttrigger, "USER", self._portDIO):
                raise Exception('Trigger must use DIO port naming.')
            self._streamtrigger = exttrigger
            triggernum = 2000 + int(self._streamtrigger[-1])
        # Configuring clock source in LabJack
        if clocksource.upper() == "INT":
            ljm.eWriteName(self._commhandle, 'STREAM_CLOCK_SOURCE', 0)
        else:
            ljm.eWriteName(self._commhandle, 'STREAM_CLOCK_SOURCE', 2)
        # Doing additional configuration changes for triggered external clock
        if exttrigger:
            ljm.writeLibraryConfigS(
                'LJM_STREAM_SCANS_RETURN',
                ljm.constants.STREAM_SCANS_RETURN_ALL)
            ljm.writeLibraryConfigS(
                'LJM_STREAM_RECEIVE_TIMEOUT_MS', 0)
            ljm.eWriteName(
                self._commhandle, 'STREAM_TRIGGER_INDEX', triggernum)
            ljm.eWriteName(
                self._commhandle, self._streamtrigger + '_EF_ENABLE', 0)
            ljm.eWriteName(
                self._commhandle, self._streamtrigger + '_EF_INDEX', 3)
            ljm.eWriteName(
                self._commhandle, self._streamtrigger + '_EF_CONFIG_A', 2)
            ljm.eWriteName(
                self._commhandle, self._streamtrigger + '_EF_ENABLE', 1)
        else:
            ljm.writeLibraryConfigS(
                'LJM_STREAM_SCANS_RETURN',
                ljm.constants.STREAM_SCANS_RETURN_ALL)
            ljm.writeLibraryConfigS(
                'LJM_STREAM_RECEIVE_TIMEOUT_MODE',
                ljm.constants.STREAM_RECEIVE_TIMEOUT_MODE_CALCULATED)
            ljm.eWriteName(self._commhandle, 'STREAM_TRIGGER_INDEX', 0)
        # Making sure read period is an integer for external clock
        if clocksource.upper() == 'EXT':
            readrate = np.ceil(readrate)
        # Assigning streaming parameters
        self._streamclock = clocksource
        self._streamnumports = len(names)
        self._streamreadperiod = readrate
        self._streamscanrate = maxrate/self._streamnumports
        self._streamscansperread = int(
            self._streamreadperiod*self._streamscanrate)
        self._streamTs = 1/self._streamscanrate
        # Configuring and starting stream
        addresses = self._get_port_address(names)
        ljm.eStreamStart(self._commhandle, self._streamscansperread,
                         self._streamnumports, addresses, self._streamscanrate)
        # Ditching first block of data
        if clocksource.upper() == 'INT':
            time.sleep(readrate)
            _, _, _, _, _ = self.get_stream()

    def get_stream(self):
        """
        Get streaming data block.


        :returns: 5-tuple

            * dt
            
                The sampling period (s) between each data point.
                When the streaming is configured with an external clock, `dt`
                contains the delta times between two consecutive samples.

            * data

                The numpy `m-by-n` array containing the streamed data where
                `m` is the number of samples per port in the block and `n`
                is the number of ports defined in `set_stream`

            * numscans
            
                The actual number of scans per port in the data block.

            * commbacklog
            
                The communication backlog in % (increasing values indicate that
                the computer cannot keep up with the data download from the T7
                driver)

            * devbacklog
            
                The T7 device backlog in % (increasing values indicate that the
                device cannot keep up with the data streaming - usually not
                the case)

        :rtype: (float, ndarray, int, float, float)


        Retrieve one data block:

            >>> dt, datablock, commbacklog, T7backlog = ljt7.get_stream()

        Create the time array for the acquired block (internal clock):

            >>> t = dt * np.linspace(0, datablock.shape[0]-1, datablock.shape[0])

        """
        # Reading data block from LabJack
        data, commbacklog, devbacklog = ljm.eStreamRead(self._commhandle)
        # Creating array from data
        data = np.array(data)
        # Separating interweaved ports into columns
        ncol = self._streamnumports
        nrow = len(data)//ncol
        valueaux = data[0:nrow*ncol].reshape((nrow, ncol))
        # Checking for external clock
        if self._streamclock == "EXT":
            value = valueaux[:, 0:-1]
            dAux = np.diff(valueaux[:, -1])
            iaux = np.nonzero(dAux < 0)[0]
            dAux[iaux] = 65535+valueaux[iaux+1, -1] - valueaux[iaux, -1]
            dt = dAux/40e6
        else:
            value = valueaux
            dt = self._streamTs
        # Assigning number of actual scans
        numscans = nrow
        # Converting backlog outputs to % values
        commbacklog = 100*commbacklog/self._streamscanrate
        devbacklog = 100*devbacklog/self._streamscanrate
        # Returning outputs
        return dt, value, numscans, commbacklog, devbacklog

    def stop_stream(self):
        """
        Stop data streaming.

            >>> ljt7.stop_stream()

        """
        ljm.eStreamStop(self._commhandle)

    # PWM METHODS
    def set_pwm(self, pwmnum=1, dirport1=None, dirport2=None, frequency=250):
        """
        Configure PWM output.


        :param pwmnum: The number of PWM output signals.
            ``1`` or ``2`` PWMs can be used. For one PWM, the output port is
            ``'FIO0'``. For two PWMs, the output ports are ``'FIO0'`` and 
            ``'FIO4'``.
        :type pwmnum: int

        :param dirport1: The type of ports that control the PWM `direction`
            for electric motor control. There are three options:

            * ``None``  - Default value (no direction ports are used)
            * ``'DAC'`` - Uses analog ports ``'DAC0'`` and ``'DAC1'``
            * ``'DIO'`` - Uses digital ports ``'EIO0'`` and ``'EIO1'``

        :type dirport1: None, str

        :param dirport2: Same as `dirport1`.
            It's used when two PWM outputs are enabled. The ``'DAC'`` option
            can only be used for one set of direction ports, unless the two
            motors are running synchronously. For the ``'DIO'`` option,
            digital ports ``'EIO2'`` and ``'EIO3'`` are used.
        :type dirport2: None, str

        :param frequency: The PWM signal frequency in Hz.
            In the case of two PWMs, both will have the same frequency
        :type frequency: int


        Set 1 PWM for motor control on ``'FIO0'`` with direction ports on
        ``'DAC0'`` and ``'DAC1'``. The PWM frequency is the default ``250`` Hz:

            >>> ljt7.set_pwm(dirport1='DAC')

        Set 2 PWMs on ports ``'FIO0'`` and ``'FIO4'`` with a frequency of
        ``500`` Hz:
        
            >>> ljt7.set_pwm(pwmnum=2, frequency=500)

        Set 2 PWMs for motor control on ports ``'FIO0'`` and ``'FIO4'``, using
        the digital ports ``'EIO0'`` and ``'EIO1'`` for motor 1 direction, and
        ``'EIO2'`` and ``'EIO3'`` for motor 2 direction. The PWM frequency is
        ``750`` Hz:
        
            >>> ljt7.set_pwm(pwmnum=2, dirport1='DIO', dirport2='DIO', frequency=750)


        .. note::
            When using digital ports, a 10 kOhm resistor has to be connected from
            the LabJack `VS` port to each one of the `DIO` ports to ensure true
            `high` and `low` states.

        """
        # Checking number of PWM outputs and direction ports
        dirport = [dirport1, dirport2]
        pwmname = ['DIO0']
        pwmdir = [None]
        self._pwmtype = [1]
        if dirport[0] == 'DAC':
            pwmdir[0] = ['DAC0', 'DAC1']
            self._pwmtype[0] = 0
        elif dirport[0] == 'DIO':
            pwmdir[0] = ['EIO0', 'EIO1']
        if pwmnum == 2:
            self._pwmtype.append(1)
            pwmname.append('DIO4')
            if dirport[1] is None:
                pwmdir.append(None)
            if dirport[1] == 'DAC':
                pwmdir.append[0] = ['DAC0', 'DAC1']
                self._pwmtype[1] = 0
            elif dirport[1] == 'DIO':
                pwmdir.append(['EIO2', 'EIO3'])
        # Assigning PWM attributes
        self._pwmnum = pwmnum
        self._pwmname = pwmname
        self._pwmdir = pwmdir
        self._pwmfreq = frequency
        # Calculating clock roll value
        self._clockrollval = self._corefreq/(self._clockdivisor*self._pwmfreq)
        # Configuring clock registers
        ljm.eWriteName(
            self._commhandle, 'DIO_EF_CLOCK0_ENABLE', 0)
        ljm.eWriteName(
            self._commhandle, 'DIO_EF_CLOCK0_DIVISOR', self._clockdivisor)
        ljm.eWriteName(
            self._commhandle, 'DIO_EF_CLOCK0_ROLL_VALUE', self._clockrollval)
        ljm.eWriteName(
            self._commhandle, 'DIO_EF_CLOCK0_ENABLE', 1)
        # Configuring extended feature (EF) registers for selected PWM port
        for name in pwmname:
            # Disable the EF system for initial configuration
            ljm.eWriteName(self._commhandle, name + '_EF_ENABLE', 0)
            # Configure EF system for PWM
            ljm.eWriteName(self._commhandle, name + '_EF_INDEX', 0)
            # Configure what clock source to use: Clock0
            ljm.eWriteName(self._commhandle, name + '_EF_OPTIONS', 0)
            # Configure duty cycle to be: 0%
            ljm.eWriteName(self._commhandle, name + '_EF_CONFIG_A', 0)
            # Enable the EF system, PWM wave is now being outputted
            ljm.eWriteName(self._commhandle, name + '_EF_ENABLE', 1)

    def set_dutycycle(self, value1=None, value2=None, brake1=False, brake2=False):
        """
        Set PWM duty cycle value.


        :param value1: The PWM 1 duty cycle percent value between ``-100``
            and ``100``.
        :type value1: float

        :param value2: The PWM 2 duty cycle percent value between ``-100``
            and ``100``.
        :type value2: float

        :param brake1: The motor 1 brake option used when dutycycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake1: bool

        :param brake2: The motor 2 brake option used when dutycycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake2: bool


        Set duty cycle to ``50`` % on PWM 1:

            >>> ljt7.set_dutycycle(value1=50)

        Set duty cycle to ``25`` % (reverse rotation) on PWM 2:

            >>> ljt7.set_dutycycle(value2=-25)

        Set duty cycle to ``20`` % and ``40`` % on PWMs 1 and 2:
  
            >>> ljt7.set_dutycycle(value1=20, value2=40)

        Stop motor 2 and apply brake:

            >>> ljt7.set_dutycycle(value2=0, brake2=True)


        .. note::
            1. Avoid suddenly switching the direction of rotation to avoid damaging the motor.
            2. You can use the brake option True to hold the motor in position.
        
        """
        values = [value1, value2]
        brakes = [brake1, brake2]
        for value, brake, pwmdir, pwmtype, pwmname in zip(
            values, brakes, self._pwmdir, self._pwmtype, self._pwmname):

            if value is not None:
                # Applying bounds to inputs
                if value > 100:
                    value = 100
                if value < -100:
                    value = -100
                # Applying PWM direction
                if pwmdir:
                    if value > 0:
                        # Forward rotation
                        if pwmtype == 0:
                            self.set_analog(pwmdir, [4.5, 0])
                        else:
                            self.set_digital(pwmdir[0], 0)
                            self.get_digital(pwmdir[1])
                    elif value < 0:
                        # Reverse rotation
                        if pwmtype == 0:
                            self.set_analog(pwmdir, [0, 4.5])
                        else:
                            self.get_digital(pwmdir[0])
                            self.set_digital(pwmdir[1], 0)
                    elif value == 0:
                        # Brake stop
                        if brake:
                            if pwmtype == 0:
                                self.set_analog(pwmdir, [0, 0])
                            else:
                                self.set_digital(pwmdir[0], 0)
                                self.set_digital(pwmdir[1], 0)
                        else:
                            if pwmtype == 0:
                                self.set_analog(pwmdir, [4.5, 4.5])
                            else:
                                self.get_digital(pwmdir[0])
                                self.get_digital(pwmdir[1])
                # Calculating duty cycle
                dutycycle = round(abs(value)/100*self._clockrollval)
                # Setting timer duty cycle to desired value
                ljm.eWriteName(
                    self._commhandle, pwmname + '_EF_CONFIG_A', dutycycle)

    # QUADRATURE ENCODER METHODS
    def set_quadrature(self, quadnum=1, zphase1=False, zphase2=False):
        """
        Configure quadrature encoder input.


        :param quadnum: The number of quadrature input signals.
            ``1`` or ``2`` encoders can be used. For one encoder, the input
            ports are ``'FIO2'`` and ``'FIO3'``. For two encoders, the input
            ports for the second one are ``'FIO6'`` and ``'FIO7'``. 
        :type quadnum: int

        :param zphase1: The logic value indicating if a `Z` phase reference
            pulse is used for the first encoder. Port ``'FIO1'`` is used. 
        :type zphase1: bool

        :param zphase2: The logic value indicating if a `Z` phase reference
            pulse is used for the second encoder. Port ``'FIO1'`` is used for
            the first encoder and port ``'FIO5'`` is used for the second 
            encoder. `zphase2` is ignored if `quadnum` = ``1``.
        :type zphase1: bool


        Set ports ``'FIO2'`` and ``'FIO3'`` for encoder with phase `A` and `B`
        signals only:
        
            >>> ljt7.set_quadrature()

        Set ports ``'FIO2'`` and ``'FIO3'`` for encoder phase `A` and `B`
        signals, and port ``'FIO1'`` for the reference `Z` phase:
        
            >>> ljt7.set_quadrature(zphase1=True)

        Set 2 encoders with `Z` phase. `A` and `B` phases are on ports
        ``'FIO2'`` and ``'FIO3'`` for encoder 1, and ``'FIO6'`` and ``'FIO7'``
        for encoder 2. The `Z` phase ports are respectively ``'FIO1'`` and
        ``'FIO5'``:

            >>> ljt7.set_quadrature(quadnum=2, zphase1=True, zphase2=True)

        """
        # Selecting port numbers based on input options
        if quadnum == 1:
            quadnameAB = [['DIO2', 'DIO3']]
            portnumZ = [0]
            if zphase1:
                portnumZ[0] = 1
        elif quadnum == 2:
            quadnameAB = [['DIO2', 'DIO3'], ['DIO6', 'DIO7']]
            portnumZ = [0, 0]
            if zphase1:
                portnumZ[0] = 1
            if zphase2:
                portnumZ[1] = 5
        else:
            raise Exception('Only 1 or 2 quadrature inputs can be assigned.')
        # Assigning quadrature properties
        self._zphase = [zphase1, zphase2]
        self._quadnum = quadnum
        self._quadnameAB = quadnameAB
        #
        # Configuring extended feature (EF) registers for
        # selected A-B ports and optional Z phase port
        #
        # Disabling the EF system for initial configuration
        for name, num in zip(quadnameAB, portnumZ):
            ljm.eWriteName(self._commhandle, name[0] + '_EF_ENABLE', 0)
            ljm.eWriteName(self._commhandle, name[1] + '_EF_ENABLE', 0)
            # Configure EF system for qudrature
            ljm.eWriteName(self._commhandle, name[0] + '_EF_INDEX', 10)
            ljm.eWriteName(self._commhandle, name[1] + '_EF_INDEX', 10)
            # Configuring port Z
            if num > 0:
                ljm.eWriteName(
                    self._commhandle, name[0] + '_EF_CONFIG_A', 1)
                ljm.eWriteName(
                    self._commhandle, name[0] + '_EF_CONFIG_B', num)
                ljm.eWriteName(
                    self._commhandle, name[1] + '_EF_CONFIG_A', 1)
                ljm.eWriteName(
                    self._commhandle, name[1] + '_EF_CONFIG_B', num)
            else:
                ljm.eWriteName(
                    self._commhandle, name[0] + '_EF_CONFIG_A', 0)
                ljm.eWriteName(
                    self._commhandle, name[1] + '_EF_CONFIG_A', 0)
            # Enabling the EF system
            ljm.eWriteName(self._commhandle, name[0] + '_EF_ENABLE', 1)
            ljm.eWriteName(self._commhandle, name[1] + '_EF_ENABLE', 1)

    def get_counter(self):
        """
        Get current quadrature counter value.


        :returns: The counter value or a list with 2 values for 2 encoders.
        :rtype: int, list(int)

        
        >>> ljt7.get_counter()


        .. note::
            Because the qudrature counter counts rising and falling edges
            of phases `A` and `B`, a 1024 pulse/rev encoder will generate 4096
            counts for a full shaft turn.

        """
        value = [self._get_port_values([self._quadnameAB[0][0] + '_EF_READ_A_F'])]
        if self._quadnum == 2:
            value.append(self._get_port_values([self._quadnameAB[1][0] + '_EF_READ_A_F']))
        else:
            value = value[0]
        return value

    def reset_counter(self, counter1=True, counter2=True):
        """
        Reset quadrature counter value.


        :param counter1: The flag indicating whether to reset counter 1 or not.
            The default value is ``True`` and it resets the counter.
        :type counter1: bool

        :param counter2: The flag indicating whether to reset counter 2 or not.
            The default value is ``True`` and it resets the counter.
        :type counter2: bool


        Resets current counter value of all encoders.

            >>> ljt7.reset_counter()

        Resets current counter value only for second encoder.
        
            >>> ljt7.reset_counter(counter1=False)


        .. note::
            The count is only reset when a `Z` phase isn't being used.

        """
        if not self._zphase[0] and counter1:
            self._get_port_values([self._quadnameAB[0][0] + '_EF_READ_A_AND_RESET'])
        if self._quadnum == 2:
            if not self._zphase[1] and counter2:
                self._get_port_values([self._quadnameAB[1][0] + '_EF_READ_A_AND_RESET'])

    # THERMOCOUPLE METHODS
    def get_labjacktemp(self, unit='C'):
        """
        Get ambient temperature from LabJack's internal sensor.


        :param unit: The temperature measurement unit.
            Valid values are ``'C'`` or ``'F'``. Default unit is ``'C'``.
        :type unit: str

        :returns: The internal sensor temperature reading.
        :rtype: float


        Get temperature reading in `Celsius`:

            >>> ljt7.get_labjacktemp()

        Get temperature reading in `Fahrenheit`:
        
            >>> ljt7.get_labjacktemp(unit='F')

        """
        v = ljm.eReadName(self._commhandle, 'AIN14')
        tempabs = -92.6*v + 467.6
        if unit == 'K':
            temp = tempabs
        elif unit == 'C':
            temp = tempabs - 273.15
        elif unit == 'F':
            temp = 9/5*(tempabs-273.15) + 32
        else:
            raise Exception(
                "Temperature units must be either 'C' or 'F'.")
        return temp

    def set_TC(self, names, types, unit='C'):
        """
        Set configuration for thermocouple input.

        :param names: The analog port(s) that will be used for thermocouple
            input. Ports ``'AIN0'`` to ``'AIN3'`` are recommended for higher
            measurement accuracy. The negative thermocouple wire should be
            connected to the LabJack `GND`.
        :type names: str, list(str)

        :param types: The thermocouple type.
            It can be a single string or a list with same length as `names`.
            Valid types are: ``'B'``, ``'E'``, ``'J'``, ``'K'``, ``'N'``,
            ``'R'``, ``'S'``, ``'T'``, and ``'C'``.
        :type types: str, list(str)

        :param unit:  The temperature measurement unit.
            Valid values are ``'C'`` or ``'F'``. Default unit is ``'C'``.
        :type unit: str


        Set port ``'AIN0'`` for thermocouple type ``'K'``:
        
            >>> ljt7.set_TC('AIN0', 'K')

        Set ports ``'AIN0'``, ``'AIN2'``, and ``'AIN3'`` for thermocouples
        type ``'K'`` and ``'J'`` with measurement in Fahrenheit:

            >>> ljt7.set_TC(['AIN0', 'AIN2', 'AIN3'], ['K', 'J', 'J'], unit='F')

        """
        self._TCoptions = ['B', 'E', 'J', 'K', 'N', 'R', 'S', 'T', 'C']
        # Checking for valid inputs
        if type(names) != list: names = [names]
        if type(types) != list: types = [types]
        if (len(names) != len(types)) and (len(types) > 1):
            raise Exception('Number of ports must match number of types.')
        if not self._check_port(names, "AIN"):
            raise Exception('Analog input port(s) are required for TC setup.')
        if (unit != 'C') and (unit != 'F'):
            raise Exception("Temperature units must be either 'degC' or 'degF'.")
        for tipe in types:
            if tipe not in self._TCoptions:
                raise Exception('Invalid TC type.')
        # Assining TC attributes
        self._TCnames = names
        if len(types) == 1:
            self._TCtypes = types * len(names)
        else:
            self._TCtypes = types
        self._TCunit = unit
        # Assigning TC type number
        self._TCtypenum = [6001+self._TCoptions.index(tipe) for tipe in types]
        # Setting TC analog port ranges to +/- 0.1V
        for name in names:
            ljm.eWriteName(self._commhandle, name + '_RANGE', 0.1)

    def get_TCtemp(self):
        """
        Get thermocouple temperature reading.


        :returns: The temperature readings for the thermocouples defined
            using `set_TC`.
        :rtype: float, list(float)


        Get temperature from thermocouples:

            >>> ljt7.get_TCtemp()

        """
        # Getting LabJack cold joint temperature
        TCJ = self.get_labjacktemp(unit='K')
        # Getting TC voltage and convertng it to temperature
        v = self.get_analog(self._TCnames)
        if type(v) != list:
            v = [v]
        tempabs = []
        for typ, val in zip(self._TCtypenum, v):
            tempabs.append(ljm.tcVoltsToTemp(typ, val, TCJ))
        # Converting to TC unit
        tempabs = np.array(tempabs)
        if self._TCunit == 'K':
            temp = tempabs
        elif self._TCunit == 'C':
            temp = tempabs - 273.15
        elif self._TCunit == 'F':
            temp = 9/5*(tempabs-273.15) + 32
        if len(temp) == 1:
            temp = temp[0]
        else:
            temp = list(temp)
        return temp

    # OTHER METHODS
    def set_reference(self, names, mode='Single-Ended'):
        """
        Set reference point for analog input voltage.


        :param names: The analog port(s) that will that will have the voltage
            reference point modified.
        :type names: str, list(str)

        :param mode: The reference mode for the analog ports.
            It and can be either ``'single-ended'`` or ``'differential'``.
            Default is ``'single-ended'``. Use ``'ALL'`` to set all analog
            input ports to the same mode.
        :type mode: str


        Set port ``'AIN0'`` for differential reading with port ``'AIN1'``:

            >>> ljt7.set_reference('AIN0', differential')

        Set ports ``'AIN0'``, ``'AIN2'``, and ``'AIN6'`` for differential
        reading respecitvely with ports ``'AIN1'``, ``'AIN3'``, and ``'AIN7'``:

            >>> ljt7.set_reference(['AIN0', 'AIN2', 'AIN6'], 'differential')

        Set all ports for single-ended reading:
        
            >>> ljt7.set_reference('ALL', 'single-ended')


        .. note::
            Differential reading uses two consecutive even-odd ports.
            Valid ports for differential reading are AIN0/2/4/6/8/10/12.

        """
        # Checking for valid inputs
        if type(names) != list: names = [names]
        if mode.lower() not in ['single-ended', 'differential']:
            raise Exception(
                "Valid reference types are 'single-ended' or 'differential'.")
        # Assigning range values
        if (names[0].lower() == 'all') and (len(names) == 1):
            numref = 199
            if mode.lower() == 'differential':
                numref = 1
            ljm.eWriteName(self._commhandle, 'AIN_ALL_NEGATIVE_CH', numref)
        else:
            if not self._check_port(names, "AIN"):
                raise Exception('Invalid analog input port name(s).')
            flagport, portnum = self._check_portdiff(names)
            if flagport:
                numref = 199
                for name, num in zip(names, portnum):
                    if mode.lower() == 'differential':
                        numref = num + 1
                    ljm.eWriteName(self._commhandle, name + '_NEGATIVE_CH', numref)
            else:
                raise Exception('Valid analog ports are AIN0/2/4/6/8/10/12.')

    def set_range(self, names, ranges):
        """
        Set analog input voltage range.


        :param names: The analog port(s) that will have their ranges modified.
            Use ``'ALL'`` to set all analog input ports to the same range.
        :type names: str, list(str)

        :param ranges: The voltage range value to be used.
            Valid ranges are +/- ``10``, ``1``, ``0.1``, ``0.01`` V. If a
            single value is used, it will be applied to all ports in
            `names`.
        :type ranges: float, list(float)


        Set port ``'AIN0'`` with a range of +/- ``1`` V:
       
            >>> ljt7.set_range('AIN0', 1)

        Set port ``'AIN0'`` and ``'AIN2'`` with a range of +/- ``0.1`` and
        ``0.01`` V:

            >>> ljt7.set_range(['AIN0', 'AIN2'], [0.1, 0.01])

        Set all ports with a (default) range of +/- ``10`` V:
        
            >>> ljt7.set_range('ALL', 10)

        """
        # Checking for valid inputs
        if type(names) != list: names = [names]
        if type(ranges) != list: ranges = [ranges]
        if (len(names) != len(ranges)) and (len(ranges) > 1):
            raise Exception('Number of ports must match number of ranges.')
        if len(ranges) == 1:
            ranges = ranges * len(names)
        for r in ranges:
            if r not in [10, 1, 0.1, 0.01]:
                raise Exception('Valid ranges are 10, 1, 0.1, or 0.01 V.')
        # Assigning range values
        if (names[0].lower() == 'all') and (len(names) == 1):
            ljm.eWriteName(self._commhandle, 'AIN_ALL_RANGE', ranges[0])
        else:
            if not self._check_port(names, "AIN"):
                raise Exception('Invalid analog input port name(s).')
            for name, r in zip(names, ranges):
                ljm.eWriteName(self._commhandle, name + '_RANGE', r)

    # HELPER METHODS (PRIVATE)
    def _assign_info(self, info):
        # Assigning info collected from LabJack handle to private attributes
        self._type = "T" + str(info[0])
        self._serialnum = str(info[2])
        if info[1] == 1:
            self._connection = "USB"
            self._ipaddress = "N/A"
            self._port = "N/A"
        elif info[1] == 2:
            self._connection = "TCP"
            self._ipaddress = ljm.numberToIP(info[3])
            self._port = str(info[4])
        elif info[1] == 3:
            self._connection = "ETHERNET"
            self._ipaddress = "N/A"
            self._port = "N/A"
        elif info[1] == 4:
            self._connection = "WIFI"
            self._ipaddress = ljm.numberToIP(info[3])
            self._port = str(info[4])
        else:
            self._connection = "Unknown"
            self._ipaddress = "Unknown"
            self._port = "Unknown"
        # More stuff
        value = ljm.eReadName(self._commhandle, 'HARDWARE_VERSION')
        self._hardware = np.round(value, 2)
        value = ljm.eReadName(self._commhandle, 'FIRMWARE_VERSION')
        self._firmware = np.round(value, 3)

    def _assign_ports(self):
        """
        Creates lists with valid LabJAck T7 port names
        """
        # Assigning analog output port names
        self._portDAC = [
            'DAC0', 'DAC1']
        # Assigning analog inout port names
        self._portAIN = [
            'AIN0', 'AIN1', 'AIN2', 'AIN3', 'AIN4', 'AIN5', 'AIN6', 'AIN7',
            'AIN8', 'AIN9', 'AIN10', 'AIN11', 'AIN12', 'AIN13', 'AIN14']
        # Assigning digital I/O port names
        self._portDIO = [
            'DIO0', 'DIO1', 'DIO2', 'DIO3', 'DIO4', 'DIO5',
            'DIO6', 'DIO7', 'DIO8', 'DIO9', 'DIO10', 'DIO11',
            'DIO12', 'DIO13', 'DIO14', 'DIO15', 'DI16', 'DI17',
            'DIO18', 'DIO19', 'DIO20', 'DIO21', 'DIO22']
        # Assigning alternate digital I/O port names
        self._portDIOAlt = [
            'FIO0', 'FIO1', 'FIO2', 'FIO3', 'FIO4', 'FIO5',
            'FIO6', 'FIO7', 'EIO0', 'EIO1', 'EIO2', 'EIO3',
            'EIO4', 'EIO5', 'EIO6', 'EIO7', 'CIO0', 'CIO1',
            'CIO2', 'CIO3', 'MIO0', 'MIO1', 'MIO2']

    def _check_port(self, namelist, portype, *args):
        """
        Checks if the port NAMES belong to the possible values in
        the correposnding PORTTYPE list of valid names. If all NAMES
        are valid then FLAGPORT = 1, otherwise FLAGPORT = 0

        A user defined list can be passed with PORTYPE = 'USER'
        through an additional argument VARARGIN

        """
        # Making sure inputs are lists
        if type(namelist) != list:
            namelist = [namelist]
        # Assining valid names list based on port type
        if portype == "DAC":
            validnames = self._portDAC
        elif portype == "AIN":
            validnames = self._portAIN
        elif portype == "DIO":
            validnames = self._portDIO + self._portDIOAlt
        elif portype == "ALLIN":
            validnames = self._portAIN + self._portDIO + self._portDIOAlt
        elif portype == "PWM":
            validnames = [
                'DIO0', 'DIO2', 'DIO3', 'DIO4', 'DIO5',
                'FIO0', 'FIO2', 'FIO3', 'FIO4', 'FIO5']
        elif portype == "QUAD":
            validnames = [
                'DIO0', 'DIO1', 'DIO2', 'DIO3', 'DIO6', 'DIO7',
                'FIO0', 'FIO1', 'FIO2', 'FIO3', 'FIO6', 'FIO7']
        elif portype == "USER":
            validnames = args[0]
        # Returns true if all names in namelist are validnames
        return all([name in validnames for name in namelist])

    def _check_portdiff(self, names):
        # Making sure inputs are lists
        if type(names) != list:
            names = [names]
        # Getting port name digits
        portnum = [int(re.findall(r'\d+', name)[0]) for name in names]
        # Checking for all even digits
        flagport = all([(num % 2)==0 for num in portnum])
        # Checking for AIN14
        if 14 in portnum:
            flagport = False
        # Returning flag and port numbers
        return flagport, portnum

    def _get_portalt(self, name):
        """
        Returns the alternate DIO port name corresponding to the input NAME

        """
        # Making sure input is a not list
        if type(name) == list:
            name = name[0]
        try:
            portnum = self._portDIO.index(name)
            portalt = self._portDIOAlt[portnum]
        except ValueError:
            portalt = "none"
        return portalt

    def _set_port_values(self, names, values):
        """
        Calls LJM method to write name values to LabJack

        """
        ljm.eWriteNames(self._commhandle, len(names), names, values)

    def _get_port_values(self, names):
        """
        Calls LJM method to read name values from LabJack

        """
        values = ljm.eReadNames(self._commhandle, len(names), names)
        if len(values) == 1:
            values = values[0]
        return values

    def _get_port_address(self, names):
        """
        Gets port addresses corresponding to the ports in NAMES from LabJack

        """
        return ljm.namesToAddresses(len(names), names)[0]


