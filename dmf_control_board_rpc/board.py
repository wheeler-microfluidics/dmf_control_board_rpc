import time

from nadamq.command_proxy import (NodeProxy, CommandRequestManager,
                                  CommandRequestManagerDebug, SerialStream)
from .requests import (REQUEST_TYPES, CommandResponse, CommandRequest,
                       CommandType)
from serial_device import get_serial_ports


class DMFControlBoard(object):
    def __init__(self, auto_connect=False, port=None, baudrate=115200):
        self._board = None
        self._connected = False
        self._auto_amplifier_gain_initialized = False
        if auto_connect:
            self.connect(port=port, baudrate=baudrate)

    def connect(self, port=None, baudrate=115200, debug=False):
        # Attempt to connect to the board.  If no port was specified, try each
        # available serial port until a successful connection is established.
        self._board = None
        if not debug:
            request_manager = CommandRequestManager(REQUEST_TYPES,
                                                    CommandRequest,
                                                    CommandResponse,
                                                    CommandType)
        else:
            request_manager = CommandRequestManagerDebug(REQUEST_TYPES,
                                                         CommandRequest,
                                                         CommandResponse,
                                                         CommandType)
        if port is not None:
            ports = [port]
        else:
            ports = get_serial_ports()
        for port in ports:
            stream = SerialStream(port, baudrate=baudrate)
            proxy = NodeProxy(request_manager, stream)
            stream._serial.setDTR(False)
            time.sleep(.2)
            stream._serial.setDTR(True)
            time.sleep(1.0)
            if self._test_connection(proxy):
                break
        if not self.connected:
            raise IOError('Could not connect to control board.')
        self._auto_amplifier_gain_initialized = False

    @property
    def auto_amplifier_gain_initialized(self):
        return self._auto_amplifier_gain_initialized

    def _test_connection(self, proxy):
        try:
            proxy.ram_free()
            self._board = proxy
            self._connected = True
        except:
            self._connected = False
        return self.connected

    @property
    def port(self):
        return self.board._stream._serial.port

    @property
    def baudrate(self):
        return self.board._stream._serial.baudrate

    @property
    def board(self):
        return self._board

    @property
    def connected(self):
        return self._connected

    def measure_voltages(self, sampling_time_ms=2, n_samples=10,
                         delay_between_samples_ms=0):
        import pandas as pd

        sample_count = self.board.measure_impedance(sampling_time_ms=
                                                    sampling_time_ms,
                                                    n_samples=n_samples,
                                                    delay_between_samples_ms=
                                                    delay_between_samples_ms)
        measurements = pd.DataFrame(dtype=float, index=range(sample_count))
        measurements['high-voltage',
                     'voltage'] = [self.board
                                   .read_sample_voltage(analog_input_key=1,
                                                        index=j)
                                   for j in range(sample_count)]
        measurements['high-voltage', 'resistor_index'] = [
            self.board.read_sample_resistor_index(analog_input_key=1, index=j)
            for j in range(sample_count)]
        measurements['feedback', 'voltage'] = [
            self.board.read_sample_voltage(analog_input_key=2, index=j)
            for j in range(sample_count)]
        measurements['feedback', 'resistor_index'] = [
            self.board.read_sample_resistor_index(analog_input_key=2, index=j)
            for j in range(sample_count)]

        # The auto-amplifier gain is updated after measuring high-voltage
        # signal on the control board.
        self._auto_amplifier_gain_initialized = True
        return measurements
