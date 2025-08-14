import utime
from machine import I2C, Pin
import _thread
from collections import deque

class MT6701:
    COUNTS_PER_REVOLUTION = 16384
    COUNTS_TO_RADIANS = 2 * 3.141592653589793 / COUNTS_PER_REVOLUTION
    COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REVOLUTION
    SECONDS_PER_MINUTE = 60.0
    RPM_FILTER_SIZE = 10  # Assuming this constant from original code

    def __init__(self, device_address, update_interval, rpm_threshold, rpm_filter_size):
        self.address = device_address
        self.updateIntervalMillis = update_interval
        self.rpmThreshold = rpm_threshold
        self.rpmFilterSize = rpm_filter_size
        self.rpmFilter = deque(maxlen=rpm_filter_size)
        self.count = 0
        self.accumulator = 0
        self.lastUpdateTime = 0
        self.rpm = 0
        self.rpmFilterIndex = 0
        self._lock = _thread.allocate_lock()

    def begin(self):
        self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)  # Adjust pins as needed
        for _ in range(self.rpmFilterSize):
            self.rpmFilter.append(0.0)
        _thread.start_new_thread(self.updateTask, ())

    def getAngleRadians(self):
        return self.count * self.COUNTS_TO_RADIANS

    def getAngleDegrees(self):
        return float(self.count) * self.COUNTS_TO_DEGREES

    def getFullTurns(self):
        return self.accumulator // self.COUNTS_PER_REVOLUTION

    def getTurns(self):
        return float(self.accumulator) / float(self.COUNTS_PER_REVOLUTION)

    def getAccumulator(self):
        return self.accumulator

    def getRPM(self):
        with self._lock:
            sum_rpm = sum(self.rpmFilter)
            rpm = sum_rpm / len(self.rpmFilter)
        return rpm

    def getCount(self):
        return self.count

    def updateCount(self):
        newCount = self.readCount()
        for _ in range(3):
            if newCount >= 0:
                break
            newCount = self.readCount()

        if newCount < 0:
            return

        diff = newCount - self.count
        if diff > self.COUNTS_PER_REVOLUTION // 2:
            diff -= self.COUNTS_PER_REVOLUTION
        elif diff < -self.COUNTS_PER_REVOLUTION // 2:
            diff += self.COUNTS_PER_REVOLUTION

        currentTime = utime.ticks_ms()
        timeElapsed = utime.ticks_diff(currentTime, self.lastUpdateTime)
        if timeElapsed > 0:
            self.rpm = (diff / float(self.COUNTS_PER_REVOLUTION)) * (self.SECONDS_PER_MINUTE * 1000 / float(timeElapsed))
            if abs(self.rpm) < self.rpmThreshold:
                self.updateRPMFilter(self.rpm)

        self.accumulator += diff
        self.count = newCount
        self.lastUpdateTime = currentTime

    def updateRPMFilter(self, newRPM):
        with self._lock:
            self.rpmFilter.append(newRPM)
            self.rpmFilterIndex = (self.rpmFilterIndex + 1) % self.RPM_FILTER_SIZE

    def readCount(self):
        try:
            self.i2c.writeto(self.address, bytearray([0x03]))
            data = self.i2c.readfrom(self.address, 2)
            if len(data) < 2:
                return -1
            angle_h = data[0]
            angle_l = data[1]
            return (angle_h << 6) | (angle_l >> 2)
        except OSError:
            return -1

    def updateTask(self):
        while True:
            self.updateCount()
            utime.sleep_ms(self.updateIntervalMillis)


# Example usage of the MT6701 class
def main():
    # Initialize the MT6701 encoder
    # Parameters: device_address, update_interval (ms), rpm_threshold, rpm_filter_size
    encoder = MT6701(device_address=0x06, update_interval=100, rpm_threshold=10, rpm_filter_size=10)
    
    # Start the encoder (initializes I2C and starts the update task)
    encoder.begin()
    
    # Allow some time for the encoder to initialize
    utime.sleep_ms(1000)
    
    # Main loop to read and print encoder data
    while True:
        # Get the current angle in degrees
        angle_deg = encoder.getAngleDegrees()
        # Get the current angle in radians
        angle_rad = encoder.getAngleRadians()
        # Get the current RPM
        rpm = encoder.getRPM()
        # Get the number of full turns
        full_turns = encoder.getFullTurns()
        # Get the accumulated count
        accumulator = encoder.getAccumulator()
        # Get the raw count
        count = encoder.getCount()
        
        # Print the results
        print(f"Angle: {angle_deg:.2f} deg, {angle_rad:.2f} rad")
        print(f"RPM: {rpm:.2f}")
        print(f"Full Turns: {full_turns}")
        print(f"Accumulator: {accumulator}")
        print(f"Raw Count: {count}")
        print("---")
        
        # Wait for 500ms before the next reading
        utime.sleep_ms(500)

# Run the example
if __name__ == "__main__":
    main()
