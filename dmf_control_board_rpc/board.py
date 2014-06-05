import sys
import time

from nadamq.command_proxy import (NodeProxy, CommandRequestManager,
                                  CommandRequestManagerDebug, SerialStream)
from .requests import (REQUEST_TYPES, CommandResponse, CommandRequest,
                       CommandType)


class DMFControlBoard(NodeProxy):
    def __init__(self, port, baudrate=115200, debug=False):
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
        stream = SerialStream(port, baudrate=baudrate)
        super(DMFControlBoard, self).__init__(request_manager, stream)
        self._stream._serial.setDTR(False)
        time.sleep(0.5)
        self._stream._serial.setDTR(True)
        time.sleep(1)
        print 'total memory:', self.total_ram_size()
        print 'free memory:', self.ram_free()

    def measure_voltages(self, sampling_time_ms=2, n_samples=10,
                         delay_between_samples_ms=0):
        import pandas as pd
        sample_count = self.measure_impedance(sampling_time_ms=
                                              sampling_time_ms,
                                              n_samples=n_samples,
                                              delay_between_samples_ms=
                                              delay_between_samples_ms)
        measurements = pd.DataFrame(dtype=float)
        measurements['high-voltage',
                     'voltage'] = [self.read_sample_voltage(analog_input_key=1,
                                                            index=j)
                                   for j in range(sample_count)]
        measurements['high-voltage', 'resistor_index'] = [
            self.read_sample_resistor_index(analog_input_key=1, index=j)
            for j in range(sample_count)]
        measurements['feedback', 'voltage'] = [
            self.read_sample_voltage(analog_input_key=2, index=j)
            for j in range(sample_count)]
        measurements['feedback', 'resistor_index'] = [
            self.read_sample_resistor_index(analog_input_key=2, index=j)
            for j in range(sample_count)]
        return measurements
