"""
Stand-in for MicroPython's `machine` module so `src/` imports and runs under
CPython in CI, with no board attached.

These are behavioural fakes, not just silent no-ops: Pin records its level, PWM
records its duty, and I2C serves bytes from a register dict. That is enough for
tests to assert things that actually matter - "was SLP driven low before the PWMs
were touched", "did disarm() zero every channel".
"""


class Pin:
    OUT = "OUT"
    IN = "IN"
    PULL_UP = "PULL_UP"
    PULL_DOWN = "PULL_DOWN"

    # Every Pin ever constructed, so tests can inspect what the code did.
    instances = {}

    def __init__(self, pin, mode=None, pull=None, value=0):
        self.id = pin
        self.mode = mode
        self.pull = pull
        self._value = 1 if pull == Pin.PULL_UP else value
        self.history = [self._value]
        Pin.instances[pin] = self

    def value(self, val=None):
        if val is None:
            return self._value
        self._value = 1 if val else 0
        self.history.append(self._value)
        return None

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)

    def high(self):
        self.value(1)

    def low(self):
        self.value(0)

    def toggle(self):
        self.value(0 if self._value else 1)

    @classmethod
    def reset(cls):
        cls.instances = {}


class PWM:
    instances = []

    def __init__(self, pin, freq=None, duty_u16=None):
        self.pin = pin
        self._freq = freq or 0
        self._duty = duty_u16 or 0
        self.duty_history = [self._duty]
        PWM.instances.append(self)

    def freq(self, value=None):
        if value is None:
            return self._freq
        self._freq = value
        return None

    def duty_u16(self, value=None):
        if value is None:
            return self._duty
        if not 0 <= value <= 65535:
            raise ValueError("duty_u16 out of range: %r" % value)
        self._duty = value
        self.duty_history.append(value)
        return None

    def deinit(self):
        self._duty = 0

    @classmethod
    def reset(cls):
        cls.instances = []


class I2C:
    """
    Fake bus. Seed `registers` as {addr: {reg: bytes}} and `scan()` reports the
    addresses present.
    """

    def __init__(self, bus_id=0, sda=None, scl=None, freq=400_000, registers=None):
        self.bus_id = bus_id
        self.sda = sda
        self.scl = scl
        self._freq = freq
        self.registers = registers if registers is not None else {}
        self.writes = []

    def scan(self):
        return sorted(self.registers.keys())

    def readfrom_mem(self, addr, reg, length):
        if addr not in self.registers:
            raise OSError("ENODEV: nothing at 0x%02X" % addr)
        data = self.registers[addr].get(reg, b"\x00" * length)
        if len(data) < length:
            data = data + b"\x00" * (length - len(data))
        return data[:length]

    def writeto_mem(self, addr, reg, data):
        if addr not in self.registers:
            raise OSError("ENODEV: nothing at 0x%02X" % addr)
        self.registers[addr][reg] = bytes(data)
        self.writes.append((addr, reg, bytes(data)))


class UART:
    """Fake serial port. Push bytes in with `feed()`; code under test reads them."""

    def __init__(self, uart_id=0, baudrate=9600, tx=None, rx=None, **kwargs):
        self.uart_id = uart_id
        self.baudrate = baudrate
        self.tx = tx
        self.rx = rx
        self._rx_buffer = b""
        self.written = b""

    def feed(self, data):
        self._rx_buffer += data

    def any(self):
        return len(self._rx_buffer)

    def read(self, size=None):
        if not self._rx_buffer:
            return None
        if size is None:
            data, self._rx_buffer = self._rx_buffer, b""
        else:
            data, self._rx_buffer = self._rx_buffer[:size], self._rx_buffer[size:]
        return data

    def write(self, data):
        self.written += bytes(data)
        return len(data)


def reset_all():
    Pin.reset()
    PWM.reset()
