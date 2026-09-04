"""
Single source of truth for the airframe's pin map and tuning constants.

Every module and every script under testing/ imports from here, so a wiring
change is a one-line edit in one file rather than a hunt through the tree.

Board: Raspberry Pi Pico 2 W (RP2350).

  GP23, GP24, GP25 and GP29 are wired to the CYW43 WiFi/Bluetooth chip on the
  "W" boards and are NOT free GPIO. Never assign them here.
"""

# --- Motors -----------------------------------------------------------------
# Two DRV8833s, two motors each. Each motor is driven from one channel with the
# second input of that channel tied to GND on the breakout, which makes it
# unidirectional (fast decay). Rotation direction is set by which way round the
# motor leads are soldered, never in software.
#
#   M1  DRV#1 AIN1      M3  DRV#2 AIN1
#   M2  DRV#1 BIN1      M4  DRV#2 BIN1
MOTOR_PINS = {
    1: 10,  # physical pin 14 - front-right, CCW
    2: 11,  # physical pin 15 - rear-left,   CCW
    3: 12,  # physical pin 16 - front-left,  CW
    4: 13,  # physical pin 17 - rear-right,  CW
}

# Pulled low = both DRV8833s asleep, outputs high-impedance, motors dead
# regardless of what the PWM registers hold. This is the hardware disarm.
MOTOR_SLEEP_PIN = 15  # physical pin 20 -> SLP on both drivers

# Optional: DRV8833 nFAULT is open-drain, needs a pull-up. Low = over-current
# or thermal shutdown has tripped.
MOTOR_FAULT_PIN = 14  # physical pin 19, or None if not wired

MOTOR_PWM_FREQ = 20_000  # Hz, above hearing range

# Two DRV8833s means two motors per package. Each dissipates roughly 0.8 W per
# motor at 1.5 A, and the cheap breakouts have no heatsinking, so sustained full
# throttle trips the 150 C thermal shutdown. Cap it.
MAX_DUTY = 0.70

# Below this a coreless motor buzzes without turning. Anything lower is wasted
# range at the bottom of the throttle curve.
MIN_START = 0.20

# --- IMU: GY-521 / MPU6050 (I2C0) -------------------------------------------
IMU_I2C_ID = 0
IMU_SDA_PIN = 4  # physical pin 6
IMU_SCL_PIN = 5  # physical pin 7
IMU_ADDR = 0x68  # 0x69 if AD0 is pulled high

# --- Magnetometer: HMC5883L (shares I2C0 with the IMU) ----------------------
# QMC5883L clones answer at 0x0D instead - see testing/05_hmc5883l_compass.
MAG_ADDR = 0x1E

I2C_FREQ = 400_000

# --- GPS: GY-GPS6MV2 / u-blox NEO-6M (UART0) --------------------------------
# Crossover: module TX -> Pico RX, module RX -> Pico TX.
GPS_UART_ID = 0
GPS_TX_PIN = 0  # physical pin 1 -> module RX
GPS_RX_PIN = 1  # physical pin 2 -> module TX
GPS_BAUD = 9600  # u-blox factory default

# --- Onboard LED ------------------------------------------------------------
# "LED" is the portable name. On the Pico 2 W it is behind the CYW43 chip, so it
# is on/off only and cannot be PWM'd.
LED_PIN = "LED"
