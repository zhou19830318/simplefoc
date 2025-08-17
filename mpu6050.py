from math import sqrt, atan2
from machine import Pin, SoftI2C
from time import sleep_ms


class MPU6050:
    # Constants
    _GRAVITY_MS2 = 9.80665
    _I2C_ERR_STR = "ESP32 could not communicate with module at address 0x{:02X}, check wiring"
    _MAX_FAILS = 3
    _MPU6050_ADDRESS = 0x68

    # Register addresses
    _PWR_MGMT_1 = 0x6B
    _ACCEL_XOUT0 = 0x3B
    _TEMP_OUT0 = 0x41
    _GYRO_XOUT0 = 0x43
    _ACCEL_CONFIG = 0x1C
    _GYRO_CONFIG = 0x1B

    # Scale modifiers
    _ACCEL_SCALERS = {
        0x00: 16384.0,  # 2g
        0x08: 8192.0,   # 4g
        0x10: 4096.0,   # 8g
        0x18: 2048.0    # 16g
    }
    _GYRO_SCALERS = {
        0x00: 131.0,    # 250 deg/s
        0x08: 65.5,     # 500 deg/s
        0x10: 32.8,     # 1000 deg/s
        0x18: 16.4      # 2000 deg/s
    }

    def __init__(self, sda_pin=39, scl_pin=38, freq=100000, addr=_MPU6050_ADDRESS):
        """Initialize MPU6050 with I2C communication."""
        self.addr = addr
        self._fail_count = 0
        self._terminating_fail_count = 0
        self._accel_range = None
        self._gyro_range = None
        
        # Initialize I2C
        self.i2c = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        
        # Wake up MPU6050 from sleep mode
        try:
            self.i2c.writeto_mem(self.addr, self._PWR_MGMT_1, bytes([0x00]))
            sleep_ms(5)
        except Exception as e:
            raise RuntimeError(self._I2C_ERR_STR.format(self.addr)) from e
            
        # Set initial ranges
        self._accel_range = self._get_raw_accel_range()
        self._gyro_range = self._get_raw_gyro_range()

    @staticmethod
    def _signed_int_from_bytes(data, endian="big"):
        """Convert bytes to signed integer."""
        value = int.from_bytes(data, endian)
        return -(65535 - value + 1) if value >= 0x8000 else value

    def _read_raw_data(self, register):
        """Read 6 bytes of raw data from specified register."""
        fail_count = 0
        while fail_count < self._MAX_FAILS:
            try:
                sleep_ms(10)
                data = self.i2c.readfrom_mem(self.addr, register, 6)
                return {
                    "x": self._signed_int_from_bytes(data[0:2]),
                    "y": self._signed_int_from_bytes(data[2:4]),
                    "z": self._signed_int_from_bytes(data[4:6])
                }
            except:
                fail_count += 1
                self._fail_count += 1
                if fail_count >= self._MAX_FAILS:
                    self._terminating_fail_count += 1
                    raise RuntimeError(self._I2C_ERR_STR.format(self.addr))
        return {"x": float("NaN"), "y": float("NaN"), "z": float("NaN")}

    # Accelerometer methods
    def set_accel_range(self, accel_range):
        """Set accelerometer range (0x00, 0x08, 0x10, 0x18 for 2g, 4g, 8g, 16g)."""
        self.i2c.writeto_mem(self.addr, self._ACCEL_CONFIG, bytes([accel_range]))
        self._accel_range = accel_range

    def _get_raw_accel_range(self):
        """Get raw accelerometer range value."""
        return self.i2c.readfrom_mem(self.addr, self._ACCEL_CONFIG, 1)[0]

    def get_accel_range(self, raw=False):
        """Get accelerometer range in g (2, 4, 8, 16) or raw value."""
        raw_value = self._get_raw_accel_range()
        if raw:
            return raw_value
        return {0x00: 2, 0x08: 4, 0x10: 8, 0x18: 16}.get(raw_value, -1)

    def read_accel_data(self, g=True):
        """Read accelerometer data in g (True) or m/s^2 (False)."""
        data = self._read_raw_data(self._ACCEL_XOUT0)
        scaler = self._ACCEL_SCALERS.get(self._accel_range, self._ACCEL_SCALERS[0x00])
        
        result = {
            "x": data["x"] / scaler,
            "y": data["y"] / scaler,
            "z": data["z"] / scaler
        }
        
        if not g:
            result = {k: v * self._GRAVITY_MS2 for k, v in result.items()}
        return result

    def read_accel_abs(self, g=True):
        """Calculate absolute acceleration magnitude."""
        data = self.read_accel_data(g)
        return sqrt(data["x"]**2 + data["y"]**2 + data["z"]**2)

    def read_angle(self):
        """Calculate tilt angles in radians based on accelerometer data."""
        accel = self.read_accel_data()
        return {
            "x": atan2(accel["y"], accel["z"]),
            "y": atan2(-accel["x"], accel["z"])
        }

    # Gyroscope methods
    def set_gyro_range(self, gyro_range):
        """Set gyroscope range (0x00, 0x08, 0x10, 0x18 for 250, 500, 1000, 2000 deg/s)."""
        self.i2c.writeto_mem(self.addr, self._GYRO_CONFIG, bytes([gyro_range]))
        self._gyro_range = gyro_range

    def _get_raw_gyro_range(self):
        """Get raw gyroscope range value."""
        return self.i2c.readfrom_mem(self.addr, self._GYRO_CONFIG, 1)[0]

    def get_gyro_range(self, raw=False):
        """Get gyroscope range in deg/s (250, 500, 1000, 2000) or raw value."""
        raw_value = self._get_raw_gyro_range()
        if raw:
            return raw_value
        return {0x00: 250, 0x08: 500, 0x10: 1000, 0x18: 2000}.get(raw_value, -1)

    def read_gyro_data(self):
        """Read gyroscope data in deg/s."""
        data = self._read_raw_data(self._GYRO_XOUT0)
        scaler = self._GYRO_SCALERS.get(self._gyro_range, self._GYRO_SCALERS[0x00])
        return {
            "x": data["x"] / scaler,
            "y": data["y"] / scaler,
            "z": data["z"] / scaler
        }

    # Temperature methods
    def read_temperature(self):
        """Read temperature in degrees Celsius."""
        try:
            raw_data = self.i2c.readfrom_mem(self.addr, self._TEMP_OUT0, 2)
            raw_temp = self._signed_int_from_bytes(raw_data)
            return (raw_temp / 340) + 36.53
        except:
            raise RuntimeError(self._I2C_ERR_STR.format(self.addr))
        
#-----------------------------------------------------------------------
# Initialize MPU6050 with default pins (SDA: GPIO 39, SCL: GPIO 38)
try:
    mpu = MPU6050(sda_pin=39, scl_pin=38, freq=100000)
except RuntimeError as e:
    print(f"Initialization failed: {e}")
    while True:
        pass  # Halt execution if initialization fails

# Function to print sensor data
def print_sensor_data():
    # Read accelerometer data (in g)
    accel = mpu.read_accel_data(g=True)
    print(f"Accelerometer (g): X={accel['x']:.2f}, Y={accel['y']:.2f}, Z={accel['z']:.2f}")
    
    # Read accelerometer data (in m/s^2)
    accel_ms2 = mpu.read_accel_data(g=False)
    print(f"Accelerometer (m/s²): X={accel_ms2['x']:.2f}, Y={accel_ms2['y']:.2f}, Z={accel_ms2['z']:.2f}")
    
    # Read absolute acceleration
    accel_abs = mpu.read_accel_abs(g=True)
    print(f"Absolute Acceleration (g): {accel_abs:.2f}")
    
    # Read gyroscope data (deg/s)
    gyro = mpu.read_gyro_data()
    print(f"Gyroscope (deg/s): X={gyro['x']:.2f}, Y={gyro['y']:.2f}, Z={gyro['z']:.2f}")
    
    # Read temperature
    temp = mpu.read_temperature()
    print(f"Temperature (°C): {temp:.2f}")
    
    # Read tilt angles
    angles = mpu.read_angle()
    print(f"Tilt Angles (radians): X={angles['x']:.2f}, Y={angles['y']:.2f}")
    
    # Print accelerometer range
    accel_range = mpu.get_accel_range()
    print(f"Accelerometer Range: ±{accel_range}g")
    
    # Print gyroscope range
    gyro_range = mpu.get_gyro_range()
    print(f"Gyroscope Range: ±{gyro_range}deg/s")

# Main loop to continuously read and display data
while True:
    try:
        print("\n--- MPU6050 Sensor Readings ---")
        print_sensor_data()
        print("-----------------------------")
        sleep_ms(1000)  # Wait 1 second between readings
    except RuntimeError as e:
        print(f"Error reading sensor data: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        break
#-----------------------------------------------------------------------
