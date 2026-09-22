"""ST LSM9DS1 IMU on I2C: accelerometer, gyroscope and magnetometer.

The I2C bus is passed in, so tests can use a fake. Anything with the smbus2.SMBus methods
read_byte_data, write_byte_data and read_i2c_block_data works.

Register addresses, bit fields and scale factors come from the ST datasheet, DocID025715
Rev 3 (March 2015), https://www.st.com/resource/en/datasheet/lsm9ds1.pdf, cited as [DS]
with its section and table numbers.

Axes
----
The chip holds two sensors with their own axes ([DS] Figure 1, drawn from the top with the
pin 1 dot):

- Accelerometer and gyroscope (X_AG, Y_AG, Z_AG). With the chip seen from above and the
  pin 1 dot at the top left corner, X_AG points right, Y_AG points down and Z_AG out of
  the top. That set is left-handed; the gyroscope reports the right-hand rotation rate
  about each of these axes ([DS] Figure 1, "+Omega" arrows).
- Magnetometer: x = -X_AG (left in the same view), y = Y_AG, z = Z_AG, which is
  right-handed. Figure 1 draws the chip turned by 90 degrees for this sensor (pin 1 dot
  at the far corner), which makes its axes look rotated against X_AG and Y_AG; they are
  not, only x is reversed.

ROS frames must be right-handed (REP 103), so this module reports all three sensors in the
magnetometer's axes: acceleration (-x, y, z) and angular rate (-x, y, z) of the
accelerometer and gyroscope outputs, magnetic field as read. This matches the axis handling
of RTIMULib (the Raspberry Pi Sense HAT library, which uses the LSM9DS1) up to a
rotation, and of github.com/jremington/LSM9DS1-AHRS. The ROS frame of these readings is
imu_link; iot_robot_description places it on the robot.
"""

import math
import time

# 7-bit I2C addresses. SDO_A/G and SDO_M select the lower or upper one ([DS] 5.1.1,
# Tables 19 and 20); most breakout boards pull both high
AG_ADDRESSES = (0x6A, 0x6B)
MAG_ADDRESSES = (0x1C, 0x1E)
DEFAULT_AG_ADDRESS = 0x6B
DEFAULT_MAG_ADDRESS = 0x1E

# Accelerometer and gyroscope registers ([DS] Table 21)
WHO_AM_I = 0x0F
CTRL_REG1_G = 0x10
STATUS_REG = 0x17
OUT_X_L_G = 0x18
CTRL_REG6_XL = 0x20
CTRL_REG8 = 0x22
OUT_X_L_XL = 0x28
AG_ID = 0x68  # [DS] 7.11, Table 43

# Magnetometer registers ([DS] Table 22)
WHO_AM_I_M = 0x0F
CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
CTRL_REG4_M = 0x23
CTRL_REG5_M = 0x24
STATUS_REG_M = 0x27
MAG_ID = 0x3D  # [DS] 8.4, Table 107
# The magnetometer only auto-increments the register address in a multi-byte read when
# the sub-address MSB is set ([DS] 5.1.1). The accelerometer and gyroscope do it by
# default (CTRL_REG8 IF_ADD_INC)
MAG_AUTO_INCREMENT = 0x80

# CTRL_REG8 bits ([DS] 7.26, Table 73)
CTRL_REG8_BDU = 0x40
CTRL_REG8_IF_ADD_INC = 0x04
CTRL_REG8_SW_RESET = 0x01
# CTRL_REG2_M bit ([DS] 8.6, Table 113)
CTRL_REG2_M_SOFT_RST = 0x04
# STATUS_REG bits ([DS] 7.18, Table 61) and STATUS_REG_M bit ([DS] 8.10, Table 124)
STATUS_GDA = 0x02
STATUS_XLDA = 0x01
STATUS_M_ZYXDA = 0x08

# Magnetometer configuration, fixed: temperature compensation on ([DS] 3.1), X/Y and Z in
# ultra-high performance mode, 80 Hz, continuous conversion, block data update
# ([DS] 8.5 to 8.9, Tables 109 to 122)
CTRL_REG1_M_VALUE = 0x80 | (0b11 << 5) | (0b111 << 2)
CTRL_REG3_M_CONTINUOUS = 0x00
CTRL_REG4_M_VALUE = 0b11 << 2
CTRL_REG5_M_BDU = 0x40
MAG_ODR_HZ = 80.0

# Output data rate of the accelerometer and gyroscope together -> ODR_G bits
# ([DS] 7.12, Table 46). With both sensors on, ODR_G sets the rate of both ([DS] 3.1)
ODR_BITS = {59.5: 0b010, 119.0: 0b011, 238.0: 0b100, 476.0: 0b101, 952.0: 0b110}
# Samples to drop after power-down to normal mode: the gyroscope column (LPF1 only) of
# [DS] Table 12, which is never less than the accelerometer's in [DS] Table 11 with the
# ODR-dependent anti-aliasing bandwidth configured here
TURN_ON_SAMPLES = {59.5: 3, 119.0: 3, 238.0: 4, 476.0: 5, 952.0: 8}

# Full scale -> (register bits, sensitivity) from [DS] Table 3 (sensitivity) and the
# register descriptions. Note the non-monotonic FS_XL encoding
# Accelerometer: g -> (FS_XL, mg/LSB), [DS] 7.24 Table 67
ACCEL_RANGES = {2: (0b00, 0.061), 4: (0b10, 0.122), 8: (0b11, 0.244), 16: (0b01, 0.732)}
# Gyroscope: dps -> (FS_G, mdps/LSB), [DS] 7.12 Table 45
GYRO_RANGES = {245: (0b00, 8.75), 500: (0b01, 17.50), 2000: (0b11, 70.0)}
# Magnetometer: gauss -> (FS, mgauss/LSB), [DS] 8.6 Table 114
MAG_RANGES = {4: (0b00, 0.14), 8: (0b01, 0.29), 12: (0b10, 0.43), 16: (0b11, 0.58)}

STANDARD_GRAVITY = 9.80665  # m/s^2 per g
TESLA_PER_GAUSS = 1e-4

# SW_RESET clears itself when the reset is done ([DS] Table 73); ST's reference driver
# polls it. The datasheet gives no duration, so allow generously
RESET_POLL_INTERVAL = 0.002
RESET_TIMEOUT = 0.1


class Lsm9ds1Error(Exception):
    """The device is missing, is a different chip, or does not respond as expected."""


def _int16(low, high):
    value = low | (high << 8)
    return value - 0x10000 if value & 0x8000 else value


def _vector(data, scale):
    # Little-endian two's complement X, Y, Z (CTRL_REG8 BLE = 0, CTRL_REG4_M BLE = 0)
    return tuple(_int16(data[i], data[i + 1]) * scale for i in (0, 2, 4))


def _check_choice(name, value, choices, unit="", fmt="g"):
    if value not in choices:
        options = ", ".join(format(c, fmt) for c in sorted(choices))
        raise ValueError(f"{name} must be one of {options}{unit}, got {format(value, fmt)}")


class Lsm9ds1:
    """Configure the LSM9DS1 and read it in SI units in the imu_link axes."""

    def __init__(self, bus, ag_address=DEFAULT_AG_ADDRESS, mag_address=DEFAULT_MAG_ADDRESS,
                 accel_range=4, gyro_range=500, mag_range=4, odr=119.0, sleep=time.sleep):
        _check_choice("ag_address", ag_address, AG_ADDRESSES, fmt="#04x")
        _check_choice("mag_address", mag_address, MAG_ADDRESSES, fmt="#04x")
        _check_choice("accel_range", accel_range, ACCEL_RANGES, " g")
        _check_choice("gyro_range", gyro_range, GYRO_RANGES, " dps")
        _check_choice("mag_range", mag_range, MAG_RANGES, " gauss")
        _check_choice("odr", odr, ODR_BITS, " Hz")
        self.bus = bus
        self.ag_address = ag_address
        self.mag_address = mag_address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.mag_range = mag_range
        self.odr = float(odr)
        self.sleep = sleep
        self.accel_scale = ACCEL_RANGES[accel_range][1] * 1e-3 * STANDARD_GRAVITY
        self.gyro_scale = math.radians(GYRO_RANGES[gyro_range][1] * 1e-3)
        self.mag_scale = MAG_RANGES[mag_range][1] * 1e-3 * TESLA_PER_GAUSS
        self.discard = 0

    def begin(self):
        """Check both device IDs, reset and configure. Raises Lsm9ds1Error or OSError."""
        self._check_id(self.ag_address, WHO_AM_I, AG_ID, "accelerometer/gyroscope")
        self._check_id(self.mag_address, WHO_AM_I_M, MAG_ID, "magnetometer")

        # Reset the configuration so that settings left by another program do not apply
        self.bus.write_byte_data(
            self.ag_address, CTRL_REG8, CTRL_REG8_IF_ADD_INC | CTRL_REG8_SW_RESET)
        self.bus.write_byte_data(self.mag_address, CTRL_REG2_M, CTRL_REG2_M_SOFT_RST)
        self._wait_for_reset()

        # Block data update: a sample's low and high bytes always belong together
        self.bus.write_byte_data(
            self.ag_address, CTRL_REG8, CTRL_REG8_BDU | CTRL_REG8_IF_ADD_INC)
        odr = ODR_BITS[self.odr]
        # The gyroscope output passes LPF1 only (CTRL_REG2_G OUT_SEL = 00 after the reset,
        # [DS] Figure 28): cutoff 38 Hz at 119 Hz ([DS] Table 46). BW_G, which only sets
        # the LPF2 cutoff, stays 00
        self.bus.write_byte_data(
            self.ag_address, CTRL_REG1_G, (odr << 5) | (GYRO_RANGES[self.gyro_range][0] << 3))
        # Same ODR code for the accelerometer. BW_SCAL_ODR = 0 ties its anti-aliasing
        # filter to the ODR (50 Hz at 119 Hz, [DS] Table 67)
        self.bus.write_byte_data(
            self.ag_address, CTRL_REG6_XL,
            (odr << 5) | (ACCEL_RANGES[self.accel_range][0] << 3))

        self.bus.write_byte_data(self.mag_address, CTRL_REG1_M, CTRL_REG1_M_VALUE)
        self.bus.write_byte_data(
            self.mag_address, CTRL_REG2_M, MAG_RANGES[self.mag_range][0] << 5)
        self.bus.write_byte_data(self.mag_address, CTRL_REG4_M, CTRL_REG4_M_VALUE)
        self.bus.write_byte_data(self.mag_address, CTRL_REG5_M, CTRL_REG5_M_BDU)
        self.bus.write_byte_data(self.mag_address, CTRL_REG3_M, CTRL_REG3_M_CONTINUOUS)

        self.discard = TURN_ON_SAMPLES[self.odr]

    def _check_id(self, address, register, expected, name):
        try:
            found = self.bus.read_byte_data(address, register)
        except OSError as err:
            raise Lsm9ds1Error(
                f"No LSM9DS1 {name} answers at I2C address 0x{address:02X} "
                f"({err.strerror or err})") from None
        if found != expected:
            raise Lsm9ds1Error(
                f"The device at I2C address 0x{address:02X} reports WHO_AM_I 0x{found:02X}, "
                f"not 0x{expected:02X} for an LSM9DS1 {name}. Check the address parameters")

    def _wait_for_reset(self):
        # The datasheet does not say whether the chip answers on I2C while it resets, so a
        # failed read only counts as "not finished yet" until the timeout
        deadline = RESET_TIMEOUT
        while True:
            error = None
            try:
                ag = self.bus.read_byte_data(self.ag_address, CTRL_REG8) & CTRL_REG8_SW_RESET
                mag = (self.bus.read_byte_data(self.mag_address, CTRL_REG2_M)
                       & CTRL_REG2_M_SOFT_RST)
                if not ag and not mag:
                    return
            except OSError as err:
                error = err
            if deadline <= 0:
                if error is not None:
                    raise error
                raise Lsm9ds1Error("The LSM9DS1 did not finish its software reset")
            self.sleep(RESET_POLL_INTERVAL)
            deadline -= RESET_POLL_INTERVAL

    def read_accel_gyro(self):
        """Return (acceleration m/s^2, angular rate rad/s), or None without a new sample.

        None is also returned for the samples dropped after start-up.
        """
        # STATUS_REG and the gyroscope output are adjacent: one burst read
        data = self.bus.read_i2c_block_data(self.ag_address, STATUS_REG, 7)
        if not data[0] & STATUS_GDA:
            return None
        accel = self.bus.read_i2c_block_data(self.ag_address, OUT_X_L_XL, 6)
        if self.discard > 0:
            self.discard -= 1
            return None
        gx, gy, gz = _vector(data[1:], self.gyro_scale)
        ax, ay, az = _vector(accel, self.accel_scale)
        # Accelerometer/gyroscope axes -> magnetometer (imu_link) axes, see the module
        # docstring
        return (-ax, ay, az), (-gx, gy, gz)

    def read_mag(self):
        """Return the magnetic field in tesla, or None without a new sample."""
        data = self.bus.read_i2c_block_data(
            self.mag_address, STATUS_REG_M | MAG_AUTO_INCREMENT, 7)
        if not data[0] & STATUS_M_ZYXDA:
            return None
        return _vector(data[1:], self.mag_scale)


class GyroBiasEstimator:
    """Average the angular rate over a window in which the robot stands still.

    A window counts as still when every sample stays within `tolerance` (rad/s) of the
    window's mean on every axis. A window with motion is dropped and a new one started.
    """

    def __init__(self, samples, tolerance):
        if samples < 1:
            raise ValueError("samples must be at least 1")
        self.samples = samples
        self.tolerance = tolerance
        self.window = []
        self.rejected = 0

    def add(self, rate):
        """Add one angular rate (x, y, z). Returns the bias once a still window is full."""
        self.window.append(rate)
        if len(self.window) < self.samples:
            return None
        window, self.window = self.window, []
        mean = tuple(sum(axis) / len(window) for axis in zip(*window))
        if any(abs(value - m) > self.tolerance for sample in window
               for value, m in zip(sample, mean)):
            self.rejected += 1
            return None
        return mean
