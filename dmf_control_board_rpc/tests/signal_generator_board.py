from nose.tools import ok_, nottest
from dmf_control_board_rpc import DMFControlBoard
import numpy as np
import os


@nottest
def test_single_waveform_voltage(control_board, rms_voltage):
    # Set the waveform-voltage of the signal-generator and confirm the value
    # was set correctly.
    control_board.set_waveform_voltage(output_vrms=rms_voltage,
                                       wait_for_reply=True)
    confirmed_v = control_board.waveform_voltage()
    ok_(np.abs(confirmed_v - rms_voltage) < 0.01 * rms_voltage)


def test_waveform_voltage():
    if not 'ARDUINO_PORT' in os.environ:
        raise RuntimeError('`ARDUINO_PORT` must be set to the COM port of the '
                           'control board.')
    control_board = DMFControlBoard(os.environ['ARDUINO_PORT'])

    for rms_voltage in np.arange(0, 1.5, 0.01):
        yield test_single_waveform_voltage, control_board, rms_voltage
