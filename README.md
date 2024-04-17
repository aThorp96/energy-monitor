# Energy Monitor
Based off of [Emonlib](https://github.com/openenergymonitor/EmonLib/tree/master), and the [OpenEnergyMonitor project](https://openenergymonitor.org/).
Licensed accordingly.

# Status
This project is currently in beta.

# Setup
Before starting, it's recommended that you read through the documentation
off of which this project is based: [Open Energy Monitor: Learn: Electricity Monitoring](https://docs.openenergymonitor.org/electricity-monitoring/index.html)

Following OpenEnergyMonitor's guide to building an Arduino Energy Monitor,
setup the following circuit:
![circuit diagram](schematic.png)

With the circuit set up, flash `./energy_monitor.ino` onto your Mega2560
and connect it to the circuit.

With the Mega2560 connected to a computer, the computer should begin
receiving serial messages with readings from the CT and the Arduino VCC.

In a terminal, run `./harmon-e <path/to/arduino/tty>`. This should start reading the
CT's readings and outputting calculations of Apparent Power.
The output is in CSV format, with headers included.

# Help
```
usage: harmon-e [-h] [-V VOLTAGE] [-r CT_RATIO] [-R RESISTANCE] [-b ADC_BITS]
                [-v] [--samples-per-second SAMPLES_PER_SECOND]
                [--buffer-duration BUFFER_DURATION]
                tty

harmon-e (HARdware-MONitoring Energy). Read from an Arduino to stream high-
resolution energy usage readings in Watt-Wiser compatible CSV format.

positional arguments:
  tty                   Path to TTY connected to hardware. Usually
                        /dev/ttyAMC* or /dev/ttyUSB*

options:
  -h, --help            show this help message and exit
  -V VOLTAGE, --voltage VOLTAGE
                        Expected voltage of measured wire (default: 120)
  -r CT_RATIO, --ct-ratio CT_RATIO
                        Ratio for the hardware's Current Transformer (default:
                        1250)
  -R RESISTANCE, --resistance RESISTANCE
                        Burden resistance for hardware circuit in olms
                        (default: 200)
  -b ADC_BITS, --adc-bits ADC_BITS
                        Bits of precision in hardware Analog to Digital
                        Converter (default: 10)
  -v, --verbose         Log verbose output to stderr (default: 0)

Algorithm tuning:
  Arguments for tuning the IRMS algorithm for fidelity. You likely will not
  need to change these.

  --samples-per-second SAMPLES_PER_SECOND
                        Maximum samples per second (default: 5000)
  --buffer-duration BUFFER_DURATION
                        How many ms of data to store in reading-buffer
                        (default: 500)

Part of the Watt Wise game jam in conjuction with Watt Wiser
(https://wattwise.games)
```
