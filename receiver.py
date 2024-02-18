#!/usr/bin/env python
import datetime as dt
import serial
import signal
import sys
import io
import math
import datetime


sample_count = 0


class Receiver:
    ser: serial.Serial

    offset_i: float
    offset_v: float
    i_calibration: float
    v_calibration: float

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

    def _read(self) -> (int, int, int):
        """returns (V_read, I_read, VCC)"""
        line = self.ser.readline()
        values = line.split(b",")
        if len(values) != 3 or values[0] == b"VREAD":
            raise ValueError("Error reading input")
        global sample_count
        sample_count += 1
        return (int(v) for v in values)

    def calc_i_rms(self, n_samples: int) -> int:
        sum_raw = 0
        sum_i = 0
        voltage = 0

        for _ in range(n_samples):
            try:
                _, sample_i, vcc = self._read()
            except ValueError:
                print("error reading, continuing...", file=sys.stderr)
                continue
            voltage = vcc

            # Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
            # then subtract this - signal is now centered on 0 counts.
            self.offset_i = self.offset_i + (sample_i - self.offset_i) / 1024
            filtered_i = sample_i - self.offset_i

            # RMS
            sum_raw += filtered_i
            sq_i = filtered_i * filtered_i
            sum_i += sq_i

        i_ratio = self.i_calibration * ((voltage / 1000.0) / self.adc_counts)

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
            try:
                start_v, _, supply_voltage = self._read()
            except ValueError:
                print("error reading inputs", file=sys.stderr)
                continue
            sample_count += 1
            if (start_v < (self.adc_counts * 0.55)) and (
                start_v > (self.adc_counts * 0.45)
            ):
                break
            if (dt.datetime.now() - start).seconds > timeout_seconds:
                break

        # Main measurement loop
        start = dt.datetime.now()

        while cross_count < crossings and (dt.datetime.now() - start).seconds < timeout_seconds:
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
            filtered_i = sample_i - self.offset_i

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

            # print("---")
            # print(f"sample_count: {sample_count}")
            # print(f"sumv: {sum_v}")
            # print(f"sumi: {sum_i}")
            # print(f"cross_count: {cross_count}")
            # print(f"crossed?: {check_cross}")

        v_ratio = self.v_calibration * ((supply_voltage/1000.0) / (self.adc_counts))
        vrms = v_ratio * math.sqrt(sum_v / sample_count)

        i_ratio = self.i_calibration * ((supply_voltage/1000.0) / (self.adc_counts))
        irms = i_ratio * math.sqrt(sum_v / sample_count)

        real_power = v_ratio * i_ratio * sum_p / sample_count
        apparent_power = vrms * irms
        power_factor = real_power / apparent_power

        breakpoint()
        print(f"{real_power} {apparent_power} {vrms} {irms} {power_factor}")

        return real_power


MICROS = -1
start_time = dt.datetime.now()
current_time = dt.datetime.now()
filename = f"output_{start_time.isoformat()}.txt"


def signal_handler(signal, frame):
    duration = dt.datetime.now() - start_time
    if duration.seconds > 0:
        print(
            f"---\nCaptured {sample_count / duration.seconds} data points per second",
            file=sys.stderr,
        )

    sys.exit(0)


if __name__ == "__main__":
    tty = sys.argv[1]
    signal.signal(signal.SIGINT, signal_handler)

    receiver = Receiver(tty, 54.5, 110.0, 1.7)
    receiver.ser.readline()

    # with io.open(filename, "w") as file:
    #     receiver.run(file)
    print("sample_number,value")

    while 1:
        # receiver.calc_i_rms(300)
        # print(receiver.offset_i)
        real_power = receiver.calc_voltage_current(20, 2)
        print(f"{datetime.datetime.now()},{real_power}")
