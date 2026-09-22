"""LSM9DS1 driver and node against a fake SMBus that follows the datasheet register map."""

import errno
import math
import time

import pytest

from iot_robot_drivers import lsm9ds1
from iot_robot_drivers.lsm9ds1 import GyroBiasEstimator, Lsm9ds1, Lsm9ds1Error

AG, MAG = 0x6B, 0x1E

# Register access per [DS] Tables 21 and 22: anything else is reserved and must not be
# touched. OUT_TEMP, status and outputs are read-only
AG_WRITABLE = {*range(0x04, 0x0E), *range(0x10, 0x14), *range(0x1E, 0x25), 0x2E,
               *range(0x30, 0x38)}
AG_READABLE = AG_WRITABLE | {0x0F, *range(0x14, 0x1E), *range(0x26, 0x2E), 0x2F}
MAG_WRITABLE = {*range(0x05, 0x0B), *range(0x20, 0x25), 0x30, 0x32, 0x33}
MAG_READABLE = MAG_WRITABLE | {0x0F, *range(0x27, 0x2E), 0x31}

# Power-on values of the control registers ([DS] Tables 21 and 22)
AG_DEFAULTS = {0x0F: 0x68, 0x1E: 0x38, 0x1F: 0x38, 0x22: 0x04}
MAG_DEFAULTS = {0x0F: 0x3D, 0x20: 0x10, 0x22: 0x03}


class FakeDevice:
    def __init__(self, defaults, writable, readable):
        self.defaults = defaults
        self.writable = writable
        self.readable = readable
        self.regs = [0] * 0x80
        self.reset()

    def reset(self):
        for register in self.writable:
            self.regs[register] = 0
        for register, value in self.defaults.items():
            self.regs[register] = value


class FakeSMBus:
    """smbus2.SMBus stand-in with an LSM9DS1 at the given addresses.

    Models what the driver depends on: WHO_AM_I, the self-clearing reset bits, register
    auto-increment (IF_ADD_INC for the accelerometer/gyroscope, sub-address MSB for the
    magnetometer), and a NACK (OSError) from absent addresses.
    """

    def __init__(self, ag=AG, mag=MAG, reset_polls=1, reset_nacks=0):
        self.devices = {}
        if ag is not None:
            self.devices[ag] = FakeDevice(AG_DEFAULTS, AG_WRITABLE, AG_READABLE)
        if mag is not None:
            self.devices[mag] = FakeDevice(MAG_DEFAULTS, MAG_WRITABLE, MAG_READABLE)
        self.ag, self.mag = ag, mag
        # Reads of the reset register before the reset bit clears; None never clears
        self.reset_polls = reset_polls
        # Register reads NACKed after a reset request; None NACKs them all
        self.reset_nacks = reset_nacks
        self.nacks_left = 0
        self.pending_reset = {}
        self.writes = []
        self.closed = False
        if ag is not None:
            self.set_ready(True, True)

    def device(self, address):
        assert not self.closed, "bus used after close()"
        if address not in self.devices:
            raise OSError(errno.EREMOTEIO, "Remote I/O error")
        return self.devices[address]

    def write_byte_data(self, address, register, value):
        device = self.device(address)
        assert register in device.writable, f"write to register 0x{register:02X}"
        self.writes.append((address, register, value))
        device.regs[register] = value
        if address == self.ag and register == lsm9ds1.CTRL_REG8 and value & 0x01:
            self.pending_reset[(address, register, 0x01)] = self.reset_polls
            self.nacks_left = self.reset_nacks
        if address == self.mag and register == lsm9ds1.CTRL_REG2_M and value & 0x04:
            self.pending_reset[(address, register, 0x04)] = self.reset_polls
            self.nacks_left = self.reset_nacks

    def read_byte_data(self, address, register):
        device = self.device(address)
        if self.nacks_left is None or self.nacks_left > 0:
            if self.nacks_left is not None:
                self.nacks_left -= 1
            raise OSError(errno.EREMOTEIO, "Remote I/O error")
        assert register in device.readable, f"read of register 0x{register:02X}"
        value = device.regs[register]
        for key in [k for k in self.pending_reset if k[:2] == (address, register)]:
            polls = self.pending_reset[key]
            if polls is not None and polls <= 0:
                del self.pending_reset[key]
                device.reset()
                value = device.regs[register]
            elif polls is not None:
                self.pending_reset[key] = polls - 1
        return value

    def read_i2c_block_data(self, address, register, length):
        device = self.device(address)
        if address == self.mag:
            increment = bool(register & 0x80)
            register &= 0x7F
        else:
            increment = bool(device.regs[lsm9ds1.CTRL_REG8] & 0x04)
        registers = [register + i if increment else register for i in range(length)]
        for r in registers:
            assert r in device.readable, f"read of register 0x{r:02X}"
        return [device.regs[r] for r in registers]

    def close(self):
        self.closed = True

    # Test helpers ---------------------------------------------------------------------

    def set_vector(self, address, register, raw):
        for i, value in enumerate(raw):
            value &= 0xFFFF
            self.devices[address].regs[register + 2 * i] = value & 0xFF
            self.devices[address].regs[register + 2 * i + 1] = value >> 8

    def set_raw(self, accel=None, gyro=None, mag=None):
        if accel is not None:
            self.set_vector(self.ag, lsm9ds1.OUT_X_L_XL, accel)
        if gyro is not None:
            self.set_vector(self.ag, lsm9ds1.OUT_X_L_G, gyro)
        if mag is not None:
            self.set_vector(self.mag, 0x28, mag)

    def set_ready(self, accel_gyro, mag):
        status = lsm9ds1.STATUS_GDA | lsm9ds1.STATUS_XLDA if accel_gyro else 0
        self.devices[self.ag].regs[lsm9ds1.STATUS_REG] = status
        if self.mag in self.devices:
            self.devices[self.mag].regs[lsm9ds1.STATUS_REG_M] = 0x0F if mag else 0

    def register(self, address, register):
        return self.devices[address].regs[register]


def started(bus=None, **kwargs):
    bus = bus or FakeSMBus()
    imu = Lsm9ds1(bus, sleep=lambda _: None, **kwargs)
    imu.begin()
    imu.discard = 0
    return bus, imu


class TestConfiguration:
    def test_default_configuration(self):
        bus, _ = started()
        reg = bus.register
        assert reg(AG, 0x22) == 0x44          # CTRL_REG8: BDU, IF_ADD_INC
        assert reg(AG, 0x10) == 0x68          # CTRL_REG1_G: 119 Hz, 500 dps, BW 00
        assert reg(AG, 0x20) == 0x70          # CTRL_REG6_XL: 119 Hz, +-4 g
        assert reg(MAG, 0x20) == 0xFC         # CTRL_REG1_M: TEMP_COMP, UHP X/Y, 80 Hz
        assert reg(MAG, 0x21) == 0x00         # CTRL_REG2_M: +-4 gauss
        assert reg(MAG, 0x22) == 0x00         # CTRL_REG3_M: continuous conversion
        assert reg(MAG, 0x23) == 0x0C         # CTRL_REG4_M: UHP Z
        assert reg(MAG, 0x24) == 0x40         # CTRL_REG5_M: BDU

    def test_resets_first_and_starts_the_magnetometer_last(self):
        bus, _ = started()
        assert bus.writes[:2] == [(AG, 0x22, 0x05), (MAG, 0x21, 0x04)]
        assert bus.writes[-1] == (MAG, 0x22, 0x00)

    def test_waits_for_the_reset_bits_to_clear(self):
        bus = FakeSMBus(reset_polls=5)
        sleeps = []
        Lsm9ds1(bus, sleep=sleeps.append).begin()
        assert sleeps and all(s == lsm9ds1.RESET_POLL_INTERVAL for s in sleeps)
        assert bus.register(AG, 0x22) == 0x44

    def test_reset_that_never_finishes_is_an_error(self):
        with pytest.raises(Lsm9ds1Error, match="software reset"):
            Lsm9ds1(FakeSMBus(reset_polls=None), sleep=lambda _: None).begin()

    def test_no_answer_during_the_reset_is_tolerated(self):
        bus = FakeSMBus(reset_nacks=3)
        Lsm9ds1(bus, sleep=lambda _: None).begin()
        assert bus.register(AG, 0x22) == 0x44

    def test_no_answer_after_the_reset_is_an_io_error(self):
        with pytest.raises(OSError):
            Lsm9ds1(FakeSMBus(reset_nacks=None), sleep=lambda _: None).begin()

    @pytest.mark.parametrize("g, bits", [(2, 0b00), (4, 0b10), (8, 0b11), (16, 0b01)])
    def test_accel_range_bits(self, g, bits):
        bus, _ = started(accel_range=g)
        assert bus.register(AG, 0x20) >> 3 & 0b11 == bits

    @pytest.mark.parametrize("dps, bits", [(245, 0b00), (500, 0b01), (2000, 0b11)])
    def test_gyro_range_bits(self, dps, bits):
        bus, _ = started(gyro_range=dps)
        assert bus.register(AG, 0x10) >> 3 & 0b11 == bits

    @pytest.mark.parametrize("gauss, bits", [(4, 0b00), (8, 0b01), (12, 0b10), (16, 0b11)])
    def test_mag_range_bits(self, gauss, bits):
        bus, _ = started(mag_range=gauss)
        assert bus.register(MAG, 0x21) == bits << 5

    @pytest.mark.parametrize("odr, bits", [(119.0, 0b011), (238.0, 0b100), (952.0, 0b110)])
    def test_same_odr_for_gyro_and_accel(self, odr, bits):
        bus, _ = started(odr=odr)
        assert bus.register(AG, 0x10) >> 5 == bits
        assert bus.register(AG, 0x20) >> 5 == bits

    def test_alternate_addresses(self):
        bus, imu = started(FakeSMBus(ag=0x6A, mag=0x1C), ag_address=0x6A, mag_address=0x1C)
        bus.set_raw(accel=(0, 0, 1000), gyro=(0, 0, 0), mag=(0, 0, 1000))
        assert imu.read_accel_gyro() is not None
        assert imu.read_mag() is not None

    @pytest.mark.parametrize("kwargs, message", [
        ({"accel_range": 3}, "accel_range must be one of 2, 4, 8, 16 g, got 3"),
        ({"gyro_range": 250}, "gyro_range must be one of 245, 500, 2000 dps"),
        ({"mag_range": 6}, "mag_range"),
        ({"odr": 100.0}, "odr must be one of 59.5, 119, 238, 476, 952 Hz"),
        ({"ag_address": 0x68}, "ag_address must be one of 0x6a, 0x6b, got 0x68"),
        ({"mag_address": 0x1D}, "mag_address"),
    ])
    def test_unsupported_settings_are_rejected(self, kwargs, message):
        with pytest.raises(ValueError, match=message):
            Lsm9ds1(FakeSMBus(), **kwargs)


class TestMissingDevice:
    def test_no_accel_gyro(self):
        with pytest.raises(Lsm9ds1Error, match="accelerometer/gyroscope answers at I2C "
                                               "address 0x6B"):
            Lsm9ds1(FakeSMBus(ag=None)).begin()

    def test_no_magnetometer(self):
        with pytest.raises(Lsm9ds1Error, match="magnetometer answers at I2C address 0x1E"):
            Lsm9ds1(FakeSMBus(mag=None)).begin()

    def test_other_chip_at_the_address(self):
        bus = FakeSMBus()
        bus.devices[AG].regs[0x0F] = 0x6A    # e.g. an LSM6DS3
        with pytest.raises(Lsm9ds1Error, match="WHO_AM_I 0x6A, not 0x68"):
            Lsm9ds1(bus).begin()
        assert bus.writes == []

    def test_read_errors_propagate_as_oserror(self):
        bus, imu = started()
        del bus.devices[AG]
        with pytest.raises(OSError):
            imu.read_accel_gyro()


class TestReadings:
    def test_si_scaling(self):
        # [DS] Table 3: 0.122 mg, 17.50 mdps and 0.14 mgauss per LSB at the defaults
        bus, imu = started()
        bus.set_raw(accel=(0, 0, 8197), gyro=(0, 1000, 0), mag=(1000, 0, 0))
        accel, gyro = imu.read_accel_gyro()
        assert accel[2] == pytest.approx(8197 * 0.122e-3 * 9.80665)
        assert accel[2] == pytest.approx(9.807, abs=1e-3)
        assert gyro[1] == pytest.approx(math.radians(17.5))
        assert imu.read_mag()[0] == pytest.approx(0.14e-4)

    @pytest.mark.parametrize("g, mg_per_lsb", [(2, 0.061), (8, 0.244), (16, 0.732)])
    def test_accel_sensitivity_per_range(self, g, mg_per_lsb):
        bus, imu = started(accel_range=g)
        bus.set_raw(accel=(0, 0x4000, 0), gyro=(0, 0, 0))
        assert imu.read_accel_gyro()[0][1] == pytest.approx(0x4000 * mg_per_lsb * 9.80665e-3)

    def test_negative_values_are_twos_complement(self):
        bus, imu = started()
        bus.set_raw(accel=(0, -8197, 0), gyro=(0, 0, -32768), mag=(0, -1, 0))
        accel, gyro = imu.read_accel_gyro()
        assert accel[1] == pytest.approx(-9.807, abs=1e-3)
        assert gyro[2] == pytest.approx(math.radians(-32768 * 17.5e-3))
        assert imu.read_mag()[1] == pytest.approx(-0.14e-7)

    def test_axes_are_reported_in_the_magnetometer_frame(self):
        # X of the accelerometer and gyroscope is negated, Y and Z kept; the
        # magnetometer is passed through (module docstring)
        bus, imu = started()
        bus.set_raw(accel=(1000, 2000, 3000), gyro=(100, 200, 300), mag=(10, 20, 30))
        accel, gyro = imu.read_accel_gyro()
        assert [round(a / imu.accel_scale) for a in accel] == [-1000, 2000, 3000]
        assert [round(w / imu.gyro_scale) for w in gyro] == [-100, 200, 300]
        assert [round(m / imu.mag_scale) for m in imu.read_mag()] == [10, 20, 30]

    def test_resulting_frame_is_right_handed(self):
        # A frame is right-handed when x cross y = z. The accelerometer/gyroscope triad of
        # [DS] Figure 1, top view with pin 1 at the top left: X right, Y down, Z up out of
        # the page. Its unit vectors in page coordinates (right, up, out):
        x_ag, y_ag, z_ag = (1, 0, 0), (0, -1, 0), (0, 0, 1)

        def cross(a, b):
            return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0])

        assert cross(x_ag, y_ag) != z_ag     # left-handed as drawn
        x_link = tuple(-v for v in x_ag)     # imu_link: -X_AG, Y_AG, Z_AG
        assert cross(x_link, y_ag) == z_ag

    def test_no_new_sample(self):
        bus, imu = started()
        bus.set_ready(False, False)
        assert imu.read_accel_gyro() is None
        assert imu.read_mag() is None

    def test_turn_on_samples_are_dropped(self):
        bus = FakeSMBus()
        imu = Lsm9ds1(bus, sleep=lambda _: None)
        imu.begin()
        bus.set_raw(accel=(0, 0, 8197), gyro=(0, 0, 0))
        results = [imu.read_accel_gyro() for _ in range(lsm9ds1.TURN_ON_SAMPLES[119.0] + 1)]
        assert results[:-1] == [None] * lsm9ds1.TURN_ON_SAMPLES[119.0]
        assert results[-1] is not None

    def test_magnetometer_reads_use_auto_increment(self):
        # Without the sub-address MSB the fake, like the chip, repeats STATUS_REG_M
        bus, imu = started()
        bus.set_raw(mag=(1, 2, 3))
        assert [round(m / imu.mag_scale) for m in imu.read_mag()] == [1, 2, 3]


class TestGyroBias:
    def test_still_window_gives_the_mean(self):
        estimator = GyroBiasEstimator(4, 0.05)
        rates = [(0.01, -0.02, 0.03), (0.02, -0.01, 0.03), (0.01, -0.02, 0.04),
                 (0.02, -0.01, 0.04)]
        assert [estimator.add(r) for r in rates[:3]] == [None, None, None]
        assert estimator.add(rates[3]) == pytest.approx((0.015, -0.015, 0.035))

    def test_motion_restarts_the_window(self):
        estimator = GyroBiasEstimator(3, 0.05)
        for rate in [(0.0, 0.0, 0.0), (0.0, 0.0, 0.5), (0.0, 0.0, 0.0)]:
            assert estimator.add(rate) is None
        assert estimator.rejected == 1
        assert estimator.window == []
        for rate in [(0.01, 0.0, 0.0)] * 2:
            assert estimator.add(rate) is None
        assert estimator.add((0.01, 0.0, 0.0)) == pytest.approx((0.01, 0.0, 0.0))

    def test_needs_at_least_one_sample(self):
        with pytest.raises(ValueError):
            GyroBiasEstimator(0, 0.05)


# Node -------------------------------------------------------------------------------


class BusFactory:
    """Hands out buses in order; an exception in the list is raised instead."""

    def __init__(self, *buses):
        self.buses = list(buses)
        self.opened = []

    def __call__(self, number):
        assert number == 1
        item = self.buses.pop(0) if len(self.buses) > 1 else self.buses[0]
        if isinstance(item, Exception):
            raise item
        self.opened.append(item)
        return item


def wait_for(condition, timeout, message, spin):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            pytest.fail(message)
        spin()


@pytest.fixture
def ros():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Imu, MagneticField

    from iot_robot_drivers.lsm9ds1_node import Lsm9ds1Node

    context = rclpy.context.Context()
    rclpy.init(context=context)
    created = []

    def start(factory, **params):
        node = Lsm9ds1Node(bus_factory=factory, context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()])
        probe = rclpy.create_node("probe", context=context)
        received = {"imu": [], "mag": []}
        probe.create_subscription(Imu, "/imu/data_raw", received["imu"].append, 10)
        probe.create_subscription(MagneticField, "/imu/mag", received["mag"].append, 10)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(probe)
        created.append((executor, node, probe))
        return node, received, lambda: executor.spin_once(timeout_sec=0.05)

    yield start
    for executor, node, probe in created:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
    rclpy.try_shutdown(context=context)


def test_node_publishes_imu_and_mag_with_gyro_bias_removed(ros):
    bus = FakeSMBus()
    bus.set_raw(accel=(100, 0, 8197), gyro=(57, -57, 114), mag=(1000, -2000, 3000))
    node, received, spin = ros(BusFactory(bus), gyro_bias_samples=5)

    wait_for(lambda: len(received["imu"]) >= 3 and received["mag"], 10.0,
             "no imu/data_raw and imu/mag", spin)
    imu = received["imu"][-1]
    assert imu.header.frame_id == "imu_link"
    assert imu.orientation_covariance[0] == -1.0
    assert imu.linear_acceleration.x == pytest.approx(-100 * 0.122e-3 * 9.80665)
    assert imu.linear_acceleration.z == pytest.approx(9.807, abs=1e-3)
    assert imu.linear_acceleration_covariance[0] == pytest.approx(0.03 ** 2)
    assert imu.angular_velocity_covariance[4] == pytest.approx(0.005 ** 2)
    # The still start-up window is the bias
    w = imu.angular_velocity
    assert (w.x, w.y, w.z) == pytest.approx((0.0, 0.0, 0.0))

    mag = received["mag"][-1]
    assert mag.header.frame_id == "imu_link"
    assert (mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z) == \
        pytest.approx((1.4e-5, -2.8e-5, 4.2e-5))
    assert list(mag.magnetic_field_covariance) == [0.0] * 9

    # A rotation after start-up: raw +1000 about X_AG is negative about imu_link x
    bus.set_raw(gyro=(57 + 1000, -57, 114))
    received["imu"].clear()
    wait_for(lambda: received["imu"], 5.0, "no imu/data_raw after the rotation", spin)
    w = received["imu"][-1].angular_velocity
    assert (w.x, w.y, w.z) == pytest.approx((-math.radians(17.5), 0.0, 0.0))


def test_node_retries_until_the_device_answers(ros):
    working = FakeSMBus()
    working.set_raw(accel=(0, 0, 8197), gyro=(0, 0, 0))
    missing_mag = FakeSMBus(mag=None)
    factory = BusFactory(FileNotFoundError(errno.ENOENT, "No such file"),
                         PermissionError(errno.EACCES, "Permission denied"),
                         missing_mag, working)
    node, received, spin = ros(factory, reconnect_period=0.05, gyro_bias_samples=0)

    wait_for(lambda: received["imu"], 10.0, "no data after the device appeared", spin)
    assert factory.opened == [missing_mag, working]
    assert missing_mag.closed and not working.closed
    assert "i2cdetect -y 1" in node.explain(Lsm9ds1Error("No LSM9DS1"))
    assert "Enable I2C" in node.explain(FileNotFoundError(errno.ENOENT, "x"))
    assert "i2c group" in node.explain(PermissionError(errno.EACCES, "x"))


def test_node_reinitialises_a_sensor_that_stops_sampling(ros):
    first, second = FakeSMBus(), FakeSMBus()
    node, received, spin = ros(BusFactory(first, second), gyro_bias_samples=0,
                               reconnect_period=0.05, stale_timeout=0.2)
    wait_for(lambda: received["imu"], 10.0, "no data from the first bus", spin)

    # E.g. a brown-out reset the configuration: no new data any more
    first.set_ready(False, False)
    wait_for(lambda: second.writes, 5.0, "the sensor was not reinitialised", spin)
    assert first.closed


def test_node_closes_the_bus_on_shutdown(ros):
    bus = FakeSMBus()
    node, received, spin = ros(BusFactory(bus), gyro_bias_samples=0)
    wait_for(lambda: bus.writes, 5.0, "the node did not open the bus", spin)
    node.destroy_node()
    assert bus.closed


def test_node_rejects_an_unsupported_range(ros):
    with pytest.raises(ValueError, match="accel_range"):
        ros(BusFactory(FakeSMBus()), accel_range=3)
