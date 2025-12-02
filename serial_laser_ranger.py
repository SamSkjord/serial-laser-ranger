"""
Laser Ranging Module Communication Library

A Python interface for laser ranging modules using serial communication.
Protocol: 9600 baud, 8 data bits, 1 stop bit, no parity

Basic Usage
-----------
    from laser_ranger import LaserRanger

    with LaserRanger(port="/dev/cu.usbserial-2210") as ranger:
        distance = ranger.measure_single()
        print(f"{distance:.3f} m")

Continuous Measurement
----------------------
    with LaserRanger() as ranger:
        for distance in ranger.measure_continuous(count=10):
            print(f"{distance:.3f} m")

Quick One-liner
---------------
    from laser_ranger import measure
    print(f"{measure():.3f} m")

Error Handling
--------------
    from laser_ranger import LaserRanger, LaserRangerError

    with LaserRanger() as ranger:
        try:
            distance = ranger.measure_single()
        except LaserRangerError as e:
            print(f"Measurement failed: {e}")

Configuration
-------------
    with LaserRanger() as ranger:
        ranger.set_resolution(LaserRanger.RESOLUTION_01MM)  # 0.1mm resolution
        ranger.set_range(LaserRanger.RANGE_30M)             # 30m range
        ranger.set_frequency(20)                             # 20 Hz

Error Codes
-----------
    ERR-1e: Low power
    ERR-14: Calculation error
    ERR-15: Laser low power
    ERR-18: Weak signal or measurement time too long
    ERR-20: Strong ambient light
    ERR-74: Out of range
"""

import serial
import time
from typing import Optional, Generator, Callable


class LaserRangerError(Exception):
    """Base exception for laser ranger errors."""

    ERROR_CODES = {
        "ERR-1e": "Low power",
        "ERR-14": "Calculation error",
        "ERR-15": "Laser low power",
        "ERR-18": "Weak signal or measurement time too long",
        "ERR-20": "Strong ambient light",
        "ERR-74": "Out of range",
    }

    def __init__(self, code: str):
        self.code = code
        self.message = self.ERROR_CODES.get(code, f"Unknown error: {code}")
        super().__init__(self.message)


class LaserRanger:
    """
    Interface for laser ranging module communication.

    This class provides methods to communicate with laser distance measurement
    modules over serial. It supports single and continuous measurements,
    laser control, and device configuration.

    Attributes:
        DEFAULT_ADDR: Default device address (0x80 / 128)
        RESOLUTION_1MM: 1mm resolution setting
        RESOLUTION_01MM: 0.1mm resolution setting
        RANGE_5M: 5 meter range setting
        RANGE_10M: 10 meter range setting
        RANGE_30M: 30 meter range setting
        RANGE_50M: 50 meter range setting
        RANGE_80M: 80 meter range setting

    Example:
        >>> with LaserRanger("/dev/ttyUSB0") as ranger:
        ...     print(f"{ranger.measure_single():.3f} m")
        1.234 m
    """

    # Default address (128 decimal = 0x80)
    DEFAULT_ADDR = 0x80

    # Resolution options
    RESOLUTION_1MM = 0x01
    RESOLUTION_01MM = 0x02

    # Range options (in meters)
    RANGE_5M = 0x05
    RANGE_10M = 0x0A
    RANGE_30M = 0x12
    RANGE_50M = 0x32
    RANGE_80M = 0x50

    def __init__(
        self,
        port: str = "/dev/cu.usbserial-2210",
        baudrate: int = 9600,
        timeout: float = 2.0,
        address: int = DEFAULT_ADDR
    ):
        """
        Initialize the laser ranger.

        Args:
            port: Serial port path
            baudrate: Baud rate (default 9600)
            timeout: Read timeout in seconds
            address: Device address (default 0x80 / 128)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.address = address
        self._serial: Optional[serial.Serial] = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        """Open the serial connection and reset device state."""
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        time.sleep(0.1)  # Allow device to initialize

        # Stop any ongoing continuous measurement and clear buffers
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self.laser_off()
        time.sleep(0.05)
        self._serial.reset_input_buffer()

    def disconnect(self):
        """Close the serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    @staticmethod
    def _calculate_checksum(data: bytes) -> int:
        """
        Calculate checksum: sum of all bytes, inverted, plus 1.

        Args:
            data: Bytes to calculate checksum for

        Returns:
            Single byte checksum
        """
        total = sum(data) & 0xFF
        return ((~total) + 1) & 0xFF

    def _send_command(
        self,
        command: bytes,
        expect_response: bool = True,
        wait_time: float = 0.3
    ) -> Optional[bytes]:
        """
        Send a command and optionally read the response.

        Args:
            command: Command bytes (without checksum)
            expect_response: Whether to wait for a response
            wait_time: Time to wait for response in seconds

        Returns:
            Response bytes or None
        """
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Serial connection not open")

        # Add checksum
        cs = self._calculate_checksum(command)
        full_command = command + bytes([cs])

        # Clear buffers
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        # Send command
        self._serial.write(full_command)
        self._serial.flush()

        if not expect_response:
            return None

        # Read response
        time.sleep(wait_time)
        response = self._serial.read(self._serial.in_waiting or 64)
        return response if response else None

    # Frame size: ADDR(1) + CMD(1) + STATUS(1) + DATA(7) + CS(1) = 11 bytes
    FRAME_SIZE = 11

    def _parse_measurement(self, response: bytes) -> float:
        """
        Parse a measurement response and return the distance.

        Response format: ADDR CMD STATUS DATA... CS
        Example: 80 06 83 30 30 30 2E 36 31 39 99
                 |  |  |  |______________|  |
                 |  |  |  ASCII "000.619"   checksum
                 |  |  status (0x80 | cmd = success)
                 |  command echo
                 address

        Args:
            response: Raw response bytes

        Returns:
            Distance in meters

        Raises:
            LaserRangerError: If response indicates an error
        """
        if not response or len(response) < 4:
            raise LaserRangerError("No response")

        # Check for error responses (contains "ERR" in ASCII)
        try:
            text = response.decode('ascii', errors='ignore')
            if 'ERR' in text:
                import re
                match = re.search(r'ERR-\w+', text)
                if match:
                    raise LaserRangerError(match.group())
                raise LaserRangerError(text)
        except UnicodeDecodeError:
            pass

        # Find frame start (address byte)
        start = 0
        for i, b in enumerate(response):
            if b == self.address:
                start = i
                break

        # Extract single frame (11 bytes)
        if len(response) >= start + self.FRAME_SIZE:
            frame = response[start:start + self.FRAME_SIZE]
            data_bytes = frame[3:-1]  # Skip header, remove checksum
        else:
            data_bytes = response[3:-1] if len(response) > 4 else response[3:]

        try:
            text = data_bytes.decode('ascii', errors='ignore').strip()
            # Extract numeric value (handles "000.619" format)
            distance_str = ''.join(c for c in text if c.isdigit() or c in '.-')
            if distance_str:
                return float(distance_str)
        except (ValueError, IndexError):
            pass

        raise LaserRangerError("Failed to parse measurement")

    # --- Measurement Commands ---

    def measure_single(self) -> float:
        """
        Perform a single distance measurement.

        Returns:
            Distance in meters
        """
        command = bytes([self.address, 0x06, 0x02])
        response = self._send_command(command, wait_time=0.5)
        return self._parse_measurement(response)

    def measure_continuous(
        self,
        callback: Optional[Callable[[float], None]] = None,
        count: int = 0
    ) -> Generator[float, None, None]:
        """
        Start continuous measurement mode.

        Streams distance measurements from the device. Use Ctrl+C or specify
        a count to stop. The laser_off() method also stops continuous mode.

        Args:
            callback: Optional function called with each distance (meters)
            count: Number of measurements to take (0 = infinite)

        Yields:
            Distance in meters for each measurement

        Example:
            # Take 10 measurements
            for d in ranger.measure_continuous(count=10):
                print(f"{d:.3f} m")

            # Continuous with callback
            def log(d):
                print(f"Distance: {d:.3f}")
            for _ in ranger.measure_continuous(callback=log):
                pass  # Runs until Ctrl+C
        """
        command = bytes([self.address, 0x06, 0x03])
        cs = self._calculate_checksum(command)
        full_command = command + bytes([cs])

        self._serial.reset_input_buffer()
        self._serial.write(full_command)
        self._serial.flush()

        measurements = 0
        buffer = b''

        while count == 0 or measurements < count:
            try:
                # Read available data
                chunk = self._serial.read(self._serial.in_waiting or self.FRAME_SIZE)
                if chunk:
                    buffer += chunk

                # Process complete frames
                while len(buffer) >= self.FRAME_SIZE:
                    # Find frame start
                    try:
                        start = buffer.index(self.address)
                        if start > 0:
                            buffer = buffer[start:]
                        if len(buffer) < self.FRAME_SIZE:
                            break

                        frame = buffer[:self.FRAME_SIZE]
                        buffer = buffer[self.FRAME_SIZE:]

                        try:
                            distance = self._parse_measurement(frame)
                            measurements += 1
                            if callback:
                                callback(distance)
                            yield distance
                            if count > 0 and measurements >= count:
                                return
                        except LaserRangerError:
                            continue
                    except ValueError:
                        buffer = b''
                        break

                time.sleep(0.01)
            except KeyboardInterrupt:
                break

    def stop_continuous(self):
        """Stop continuous measurement mode by sending power command."""
        self.laser_off()

    # --- Laser Control ---

    def laser_on(self) -> bool:
        """
        Turn the laser on.

        Returns:
            True if successful
        """
        command = bytes([self.address, 0x06, 0x05, 0x01])
        response = self._send_command(command)
        return response is not None and len(response) >= 4

    def laser_off(self) -> bool:
        """
        Turn the laser off.

        Returns:
            True if successful
        """
        command = bytes([self.address, 0x06, 0x05, 0x00])
        response = self._send_command(command)
        return response is not None and len(response) >= 4

    # --- Configuration Commands ---

    def set_resolution(self, resolution: int) -> bool:
        """
        Set measurement resolution.

        Args:
            resolution: RESOLUTION_1MM or RESOLUTION_01MM

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x0C, resolution])
        response = self._send_command(command)
        return response is not None and 0x80 not in response[3:4]

    def set_range(self, range_m: int) -> bool:
        """
        Set measurement range.

        Args:
            range_m: One of RANGE_5M, RANGE_10M, RANGE_30M, RANGE_50M, RANGE_80M

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x05, range_m])
        response = self._send_command(command)
        return response is not None

    def set_frequency(self, freq: int) -> bool:
        """
        Set measurement frequency.

        Args:
            freq: Frequency value (0=minimum ~3Hz, 5, 10, 20)

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x04, freq])
        response = self._send_command(command)
        return response is not None

    def set_measurement_interval(self, interval: int) -> bool:
        """
        Set the measurement return interval.

        Args:
            interval: Interval value in device units

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x35, interval])
        response = self._send_command(command)
        return response is not None

    def set_power_on_start(self, enabled: bool) -> bool:
        """
        Configure whether device starts measuring on power-on.

        Args:
            enabled: True to enable, False to disable

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x8D, 0x01 if enabled else 0x00])
        response = self._send_command(command)
        return response is not None

    def set_start_point(self, from_top: bool) -> bool:
        """
        Set the measurement starting point.

        Args:
            from_top: True for top, False for bottom/end

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x06, 0x01 if from_top else 0x00])
        response = self._send_command(command)
        return response is not None

    def set_distance_offset(self, offset_mm: float, negative: bool = False) -> bool:
        """
        Set distance offset/modification.

        Args:
            offset_mm: Offset value in millimeters
            negative: True for negative offset

        Returns:
            True if successful
        """
        offset_int = int(abs(offset_mm))
        offset_h = (offset_int >> 8) & 0xFF
        offset_l = offset_int & 0xFF
        symbol = 0xFF if negative else 0x00

        command = bytes([0xFA, 0x04, 0x34, symbol, offset_h, offset_l])
        response = self._send_command(command)
        return response is not None and b'\x88\x77' in response

    # --- Utility Commands ---

    def read_address(self) -> Optional[int]:
        """
        Read the device address.

        Returns:
            Device address or None if failed
        """
        command = bytes([0xFA, 0x04, 0x0A])
        response = self._send_command(command)
        if response and len(response) >= 3:
            return response[1]
        return None

    def set_address(self, new_address: int) -> bool:
        """
        Set a new device address.

        Args:
            new_address: New address (0-255)

        Returns:
            True if successful
        """
        command = bytes([0xFA, 0x04, 0x01, new_address])
        response = self._send_command(command)
        if response is not None:
            self.address = new_address
            return True
        return False

    def reset_cache(self) -> bool:
        """
        Reset the measurement cache.

        Returns:
            True if successful
        """
        command = bytes([self.address, 0x06, 0x07])
        response = self._send_command(command)
        return response is not None

    def read_cache(self) -> Optional[float]:
        """
        Read the cached measurement.

        Returns:
            Cached distance in meters or None
        """
        command = bytes([self.address, 0x06, 0x02])
        response = self._send_command(command)
        if response:
            try:
                return self._parse_measurement(response)
            except LaserRangerError:
                return None
        return None

    def power_off(self):
        """Power off the device."""
        command = bytes([self.address, 0x00, 0x02])
        self._send_command(command, expect_response=False)


# Convenience function for quick measurements
def measure(port: str = "/dev/cu.usbserial-2210") -> float:
    """
    Quick single measurement.

    Args:
        port: Serial port path

    Returns:
        Distance in meters
    """
    with LaserRanger(port=port) as ranger:
        return ranger.measure_single()


if __name__ == "__main__":
    # Example usage
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-2210"

    print(f"Connecting to laser ranger on {port}...")

    with LaserRanger(port=port) as ranger:
        print("Taking single measurement...")
        try:
            distance = ranger.measure_single()
            print(f"  Distance: {distance:.3f} m")
        except LaserRangerError as e:
            print(f"  Error: {e}")

        print("\nTaking 5 continuous measurements...")
        try:
            for i, distance in enumerate(ranger.measure_continuous(count=5)):
                print(f"  [{i+1}] {distance:.3f} m")
        except LaserRangerError as e:
            print(f"  Error: {e}")

        print("\nTurning laser off...")
        ranger.laser_off()
        print("Done.")
