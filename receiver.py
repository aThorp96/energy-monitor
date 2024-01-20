#!/usr/bin/env python
import datetime as dt
import serial
import signal
import sys
import io
import math
import time


sample_count = 0


class Receiver:
    offset_i: int
    offset_v: int
    i_calibration: float
    v_calibration: float
    ser: serial.Serial
    adc_bits: int
    adc_counts: int

    def __init__(self, port_path: str, i_calibration: float, v_calibration: float, adc_bits: int = 10):
        self.adc_bits = adc_bits
        self.adc_counts = 1 << adc_bits

        self.offset_i = self.adc_counts >> 1
        self.offset_v = self.adc_counts >> 1
        self.i_calibration = i_calibration
        self.v_calibration = v_calibration

        self.ser = serial.Serial(
            port=port_path,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        )

    def _read(self) -> (int, int, int):
        """ returns (V_read, I_read, VCC)"""
        line = self.ser.readline()
        values = line.split(b',')
        if len(values) != 3 or values[0] == b'VREAD':
            return (0, 0, 0)

        global sample_count
        sample_count += 1
        return (int(v) for v in values)

    def calc_i_rms(self, n_samples: int) -> int:
        sum_i = 0
        voltage = 0

        for _ in range(n_samples):
            _, sample_i, vcc = self._read()
            voltage = vcc

            # Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
            # then subtract this - signal is now centered on 0 counts.
            self.offset_i = self.offset_i + (sample_i - self.offset_i) / 1024
            filtered_i = sample_i - self.offset_i

            # RMS
            sq_i = filtered_i * filtered_i
            sum_i += sq_i

        i_ratio = self.i_calibration * ((voltage/1000.0) / self.adc_counts)

        return i_ratio * math.sqrt(sum_i / n_samples)


MICROS = -1
start_time = dt.datetime.now()
current_time = dt.datetime.now()
filename = f"output_{start_time.isoformat()}.txt"


def signal_handler(signal, frame):
    duration = dt.datetime.now() - start_time
    if duration.seconds > 0:
        print(f"---\nCaptured {sample_count / duration.seconds} data points per second")

    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    receiver = Receiver("/dev/ttyACM0", 111.11, 110.0)
    receiver.ser.readline()

    # with io.open(filename, "w") as file:
    #     receiver.run(file)

    while 1:
        print(receiver.calc_i_rms(25))
