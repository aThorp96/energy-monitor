#!/usr/bin/env python
import datetime as dt
import serial
import signal
import sys
import math
import time
import datetime
from time import sleep
import typing as t


def get_now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


class CircularBuffer:
    _buffer: list[float]
    _buffer_size: int
    _front: int
    _back: int
    _full: bool
    _sum: float

    def __init__(self, buffer_size: int):
        self._buffer_size = buffer_size
        self._buffer = [0] * buffer_size
        self._front = 0
        self._back = 0
        self._sum = 0
        self._full = False

    def from_buffer(buff: list[float]) -> "CircularBuffer":
        self = CircularBuffer(len(buff))
        self._buffer = buff
        self._front = 0
        self._back = len(buff) - 1
        self._sum = sum(buff)
        self._full = True

        return self

    def append(self, value: float):
        self._incrememnt_index()

        self._sum -= self._buffer[self._front]
        self._buffer[self._front] = value
        self._sum += value

    def mean(self) -> float:
        if self._full:
            count = self._buffer_size
        else:
            count = self._front
        return self._sum / count

    def _incrememnt_index(self):
        self._front = (self._front + 1) % self._buffer_size

        if self._front == self._back:
            self._full = True
            self._back = (self._back + 1) % self._buffer_size

    def full(self) -> bool:
        return self._full


T = t.TypeVar("T")


class Reading(t.Generic[T]):
    start_ns: int
    stop_ns: int
    value: T

    def __init__(self, start: datetime.datetime, stop: datetime.datetime, value: T):
        self.start_ns = start
        self.stop_ns = stop
        self.value = value

    @classmethod
    def csv_header(cls, value_title: str, units: str) -> str:
        return f"sample start (ns), sample end (ns), {value_title} ({units})"

    def __str__(self) -> str:
        return f"{self.start_ns}, {self.stop_ns}, {self.value}"


class Cmd:
    STOP = b"X"
    START = b"S"
    ACK = b"A"
    KAY = b"K"


class Receiver:
    ser: serial.Serial
    sample_count: int = 0

    offset_i: float
    offset_v: float
    i_calibration: float
    v_calibration: float

    cvv: float

    phase_shift: int

    adc_bits: int
    adc_counts: int

    def __init__(
        self,
        port_path: str,
        i_calibration: float,
        v_calibration: float,
        phase_shift: int,
        adc_bits: int = 10,
    ):
        self.adc_bits = adc_bits
        self.adc_counts = 1 << adc_bits

        self.phase_shift = phase_shift

        self.offset_i = float(self.adc_counts >> 1)
        self.offset_v = float(self.adc_counts >> 1)
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

        self.vcc = self._handshake()

    def _handshake(self) -> float:
        self.ser.write(Cmd.STOP)
        sleep(1)
        self.ser.read_all()

        self.ser.write(Cmd.START)
        self.ser.read_until(Cmd.ACK)
        vcc = self._read_vcc()
        self.ser.write(Cmd.KAY)

        return vcc

    def _read_vcc(self) -> int:
        return int.from_bytes(self.ser.read(4), byteorder="little")

    def _read(self) -> int:
        """returns (V_read, I_read, VCC)"""
        # voltage: int = int.from_bytes(self.ser.read(2), byteorder='little')
        current: int = int.from_bytes(self.ser.read(2), byteorder="little")
        self.sample_count += 1

        return current

    def stream_i_rms_with_count(
        self, n_samples: int, samples_per_second: float
    ) -> t.Iterable[Reading[float]]:
        return self.stream_i_rms_with_buffer(
            CircularBuffer(n_samples), samples_per_second
        )

    def stream_i_rms_luis_llamas(self, factor: float, i_peak: float, r: float) -> Reading:
        # based on https://www.luisllamas.es/en/arduino-current-sensor-sct-013/#assembly-with-resistors-and-midpoint
        duration_s = 0.5

        def fmap(x: float, in_min: float, in_max: float, out_min: float, out_max) -> float:
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        v_peak = math.sqrt(2) * r * i_peak
        # In the example ADCV is 5.0, but we have read it in as ~5000
        adc_v = self.vcc / 1000

        midpoint = adc_v / 2
        v_min = midpoint - v_peak
        v_max = midpoint + v_peak

        def read() -> float:
            voltage = self._read() * adc_v / self.adc_counts
            return fmap(voltage, v_min, v_max, factor * -1, factor)

        def generate_buffer() -> t.Iterable[float]:
            end = get_now_ns() + (duration_s * 1000 * 1000 * 1000)
            while get_now_ns() <= end:
                current = read()
                yield current * current

        buff = CircularBuffer.from_buffer(list(generate_buffer()))

        last_read = get_now_ns()
        while True:
            current = read()
            buff.append(current * current)
            this_read = get_now_ns()
            if buff.full():
                yield Reading(last_read, this_read, math.sqrt(buff.mean()))
                last_read = this_read

    def stream_i_rms_with_duration(
        self, buffer_duration_s: float, samples_per_second: float
    ) -> t.Iterable[Reading[float]]:
        def sample_generator():
            start = get_now_ns()
            end = start + (buffer_duration_s * 1000 * 1000 * 1000)
            while end >= get_now_ns():
                # Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
                # then subtract this - signal is now centered on 0 counts.
                sample_i = float(self._read())
                self.offset_i = self.offset_i + (sample_i - self.offset_i) / 1024
                yield sample_i

        buff = list(sample_generator())

        return self.stream_i_rms_with_buffer(
            CircularBuffer.from_buffer(buff), samples_per_second
        )

    def stream_i_rms_with_buffer(
        self, squares_buff: CircularBuffer, samples_per_second: float
    ) -> t.Iterable[Reading[float]]:
        last_sample_time = get_now_ns()
        last_emit = get_now_ns()
        ns_per_sample = (1.0 / samples_per_second) * 1000 * 1000 * 1000

        while True:
            try:
                sample_i = float(self._read())
            except ValueError:
                print("error reading, continuing...", file=sys.stderr)
                continue

            # Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
            # then subtract this - signal is now centered on 0 counts.
            self.offset_i = self.offset_i + (sample_i - round(self.offset_i)) / 1024
            # Round the offset since the integer sample and
            # the floating point offset introduces error
            filtered_i = sample_i - self.offset_i

            # RMS
            sq_i: float = filtered_i * filtered_i
            squares_buff.append(sq_i)

            if squares_buff.full():
                i_ratio = self.i_calibration * ((self.vcc / 1024.0) / self.adc_counts)

                now = get_now_ns()
                time_difference_ns = now - last_sample_time

                if time_difference_ns >= ns_per_sample:
                    irms = i_ratio * math.sqrt(squares_buff.mean())
                    yield Reading(last_sample_time, now, irms)
                    # yield Reading(last_sample_time, now, filtered_i)
                    last_sample_time = get_now_ns()

                    if get_now_ns() - last_emit > 1000 * 1000 * 1000:
                        last_emit = get_now_ns()
                        print(f"irms: {irms}, last_sample: {sample_i}, offset: {round(self.offset_i)}, ratio: {i_ratio}, mean: {squares_buff.mean()}", file=sys.stderr)


    def calc_i_rms(self, n_samples: int) -> int:
        sum_raw = 0
        sum_i = 0

        for _ in range(n_samples):
            try:
                sample_i = self._read()
            except ValueError:
                print("error reading, continuing...", file=sys.stderr)
                continue

            # Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
            # then subtract this - signal is now centered on 0 counts.
            self.offset_i = self.offset_i + (sample_i - self.offset_i) / 1024
            filtered_i = sample_i - self.offset_i

            # RMS
            sum_raw += filtered_i
            sq_i = filtered_i * filtered_i
            sum_i += sq_i

        i_ratio = self.i_calibration * ((self.vcc / 1000.0) / self.adc_counts)

        irms = i_ratio * math.sqrt(sum_i / n_samples)

        print(
            f"{datetime.datetime.now()},{sum_raw / n_samples}, {self.offset_i}, {i_ratio}, {irms*120}"
        )

        return irms

    # emon_calc procedure
    # Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
    # From a sample window of the mains AC voltage and current.
    # The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
    def calc_voltage_current(self, crossings: int, timeout_seconds: int):
        check_cross = False
        last_sample_crossed = False
        cross_count = 0
        sample_count = 0
        filtered_v = 0
        filtered_i = 0

        sum_p = 0
        sum_v = 0
        sum_i = 0

        # 1) Waits for the waveform to be close to 'zero' (mid-scale adc)
        # part in sin curve.
        start = dt.datetime.now()

        while True:
            start_v, _, supply_voltage = self._read()
            if (start_v < (self.adc_counts * 0.55)) and (
                start_v > (self.adc_counts * 0.45)
            ):
                break
            if (dt.datetime.now() - start).seconds > timeout_seconds:
                break

        # Main measurement loop
        start = dt.datetime.now()

        while (
            cross_count < crossings
            and (dt.datetime.now() - start).seconds < timeout_seconds
        ):
            # Take measurement
            try:
                sample_v, sample_i, _ = self._read()
            except ValueError:
                print("error reading inputs", file=sys.stderr)
                continue

            sample_count += 1
            last_filtered_v = filtered_v

            # Low pass filter to extract the 2.5 V dc offset, then subtract it
            self.offset_v = self.offset_v + ((sample_v - self.offset_v) / 1024)
            filtered_v = sample_v - self.offset_v
            self.offset_i = self.offset_i + ((sample_i - self.offset_i) / 1024)
            filtered_i = sample_i - round(self.offset_i)

            # RMS the voltage and current
            sq_v = filtered_v**2
            sum_v += sq_v
            sq_i = filtered_i**2
            sum_i += sq_i

            # Phase calibration
            phase_shifted_v = last_filtered_v + self.phase_shift * (
                filtered_v - last_filtered_v
            )

            # Instantanious power
            inst_p = phase_shifted_v * filtered_i
            sum_p += inst_p

            # Find the number of times the voltage has crossed the initial voltage
            #  - every 2 crosses we will have sampled 1 wavelength
            #  - so this method allows us to sample an integer number of half wavelengths which increases accuracy
            last_sample_crossed = check_cross
            check_cross = sample_v > start_v

            if sample_count == 1:
                last_sample_crossed = check_cross

            if last_sample_crossed != check_cross:
                cross_count += 1

        v_ratio = self.v_calibration * ((supply_voltage / 1000.0) / (self.adc_counts))
        vrms = v_ratio * math.sqrt(sum_v / sample_count)
        print(f"voltage: {vrms}, ratio: {v_ratio}, offset: {self.offset_v}")

        i_ratio = self.i_calibration * ((supply_voltage / 1000.0) / (self.adc_counts))
        irms = i_ratio * math.sqrt(sum_v / sample_count)

        real_power = v_ratio * i_ratio * sum_p / sample_count
        apparent_power = vrms * irms
        power_factor = real_power / apparent_power

        return real_power


def __main__():
    tty = sys.argv[1]
    ct_ratio = 1250
    if len(sys.argv) > 2:
        ct_ratio = int(sys.argv[2])
    # Current calibration is Rt (current transformer ratio, 2k) / Rb (burden resistor resistance)
    receiver = Receiver(tty, ct_ratio / 200, 120.0, 1.0)
    start_time = dt.datetime.now()

    closed = False

    def close():
        nonlocal closed
        if not closed:
            closed = True
            duration = dt.datetime.now() - start_time
            if duration.seconds > 0:
                print(
                    f"---\nCaptured {receiver.sample_count / duration.seconds} data points per second",
                    file=sys.stderr,
                )
                receiver.ser.write(Cmd.STOP)
                receiver.ser.read_all()

    def signal_handler(signal, frame):
        close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    print(
        f"Reading with VCC {receiver.vcc}",
        file=sys.stderr,
    )
    print(Reading.csv_header("apparent power", "W"))

    try:
        # while True:
        #     print(receiver._read())
        #     time.sleep(0.5)
        for i_rms in receiver.stream_i_rms_with_duration(0.5, 5000):
            i_rms.value *= 123.5
            print(i_rms)
    finally:
        close()


if __name__ == "__main__":
    __main__()
