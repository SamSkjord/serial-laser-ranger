#!/usr/bin/env python3
"""
Laser Ranger Examples

Practical examples for using the laser ranging module.
Run with: python examples.py [example_name]
"""

import sys
import time
from serial_laser_ranger import LaserRanger, LaserRangerError, measure


# Default serial port - change this to match your setup
PORT = "/dev/cu.usbserial-2210"


def example_basic():
    """Basic single measurement."""
    print("=== Basic Measurement ===\n")

    with LaserRanger(PORT) as ranger:
        distance = ranger.measure_single()
        print(f"Distance: {distance:.3f} m")
        print(f"          {distance * 100:.1f} cm")
        print(f"          {distance * 1000:.0f} mm")


def example_quick():
    """Quick one-liner measurement."""
    print("=== Quick Measurement ===\n")

    # Simplest possible usage
    print(f"Distance: {measure(PORT):.3f} m")


def example_continuous():
    """Continuous measurement stream."""
    print("=== Continuous Measurement (10 readings) ===\n")

    with LaserRanger(PORT) as ranger:
        for i, distance in enumerate(ranger.measure_continuous(count=10)):
            print(f"  [{i+1:2d}] {distance:.3f} m")


def example_statistics():
    """Collect measurements and compute statistics."""
    print("=== Measurement Statistics (20 samples) ===\n")

    with LaserRanger(PORT) as ranger:
        measurements = list(ranger.measure_continuous(count=20))

    avg = sum(measurements) / len(measurements)
    min_val = min(measurements)
    max_val = max(measurements)
    spread = max_val - min_val

    print(f"  Samples:  {len(measurements)}")
    print(f"  Average:  {avg:.4f} m")
    print(f"  Min:      {min_val:.4f} m")
    print(f"  Max:      {max_val:.4f} m")
    print(f"  Spread:   {spread * 1000:.2f} mm")


def example_monitor():
    """Real-time distance monitor with threshold alert."""
    print("=== Distance Monitor (Ctrl+C to stop) ===")
    print("    Alert threshold: 0.5 m\n")

    THRESHOLD = 0.5  # meters

    with LaserRanger(PORT) as ranger:
        try:
            for distance in ranger.measure_continuous():
                if distance < THRESHOLD:
                    status = "!! ALERT !!"
                else:
                    status = "OK"

                # Simple bar graph
                bar_len = int(min(distance * 20, 40))
                bar = "#" * bar_len

                print(f"\r  {distance:.3f} m [{bar:<40}] {status}  ", end="")
                sys.stdout.flush()
        except KeyboardInterrupt:
            print("\n\nStopped.")


def example_logger():
    """Log measurements to CSV file."""
    print("=== CSV Logger (20 samples) ===\n")

    filename = "measurements.csv"

    with LaserRanger(PORT) as ranger:
        with open(filename, "w") as f:
            f.write("timestamp,distance_m\n")

            for distance in ranger.measure_continuous(count=20):
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"{timestamp},{distance:.4f}\n")
                print(f"  {timestamp} -> {distance:.3f} m")

    print(f"\nSaved to {filename}")


def example_calibration():
    """Measure a known distance for calibration check."""
    print("=== Calibration Check ===")
    print("    Place target at a known distance.\n")

    known_distance = 1.0  # meters - adjust this!

    with LaserRanger(PORT) as ranger:
        # Take multiple samples for accuracy
        samples = list(ranger.measure_continuous(count=10))
        measured = sum(samples) / len(samples)

        error = measured - known_distance
        error_pct = (error / known_distance) * 100

        print(f"  Known distance:    {known_distance:.3f} m")
        print(f"  Measured average:  {measured:.4f} m")
        print(f"  Error:             {error * 1000:+.2f} mm ({error_pct:+.2f}%)")

        if abs(error) < 0.002:
            print("\n  Status: EXCELLENT (< 2mm error)")
        elif abs(error) < 0.005:
            print("\n  Status: GOOD (< 5mm error)")
        else:
            print("\n  Status: Consider recalibration")


def example_configuration():
    """Show device configuration options."""
    print("=== Configuration Example ===\n")

    with LaserRanger(PORT) as ranger:
        # Set high resolution
        print("  Setting 0.1mm resolution...")
        ranger.set_resolution(LaserRanger.RESOLUTION_01MM)

        # Set range
        print("  Setting 30m range...")
        ranger.set_range(LaserRanger.RANGE_30M)

        # Set frequency
        print("  Setting 20 Hz frequency...")
        ranger.set_frequency(20)

        # Take measurement with new settings
        print("\n  Measurement with new settings:")
        distance = ranger.measure_single()
        print(f"    Distance: {distance:.4f} m")


def example_error_handling():
    """Demonstrate error handling."""
    print("=== Error Handling ===\n")

    with LaserRanger(PORT) as ranger:
        try:
            distance = ranger.measure_single()
            print(f"  Distance: {distance:.3f} m")
        except LaserRangerError as e:
            print(f"  Measurement failed: {e.code}")
            print(f"  Description: {e.message}")

            # Handle specific errors
            if e.code == "ERR-18":
                print("  Suggestion: Check if target is reflective enough")
            elif e.code == "ERR-74":
                print("  Suggestion: Target may be out of range")


def example_motion_detect():
    """Simple motion detection by measuring distance changes."""
    print("=== Motion Detection (Ctrl+C to stop) ===")
    print("    Sensitivity: 5mm\n")

    SENSITIVITY = 0.005  # 5mm

    with LaserRanger(PORT) as ranger:
        last_distance = None

        try:
            for distance in ranger.measure_continuous():
                if last_distance is not None:
                    change = abs(distance - last_distance)
                    if change > SENSITIVITY:
                        direction = "CLOSER" if distance < last_distance else "FARTHER"
                        print(f"  Motion detected! {direction} ({change*1000:.1f}mm)")

                last_distance = distance
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped.")


# Available examples
EXAMPLES = {
    "basic": example_basic,
    "quick": example_quick,
    "continuous": example_continuous,
    "statistics": example_statistics,
    "monitor": example_monitor,
    "logger": example_logger,
    "calibration": example_calibration,
    "config": example_configuration,
    "errors": example_error_handling,
    "motion": example_motion_detect,
}


def main():
    if len(sys.argv) < 2:
        print("Laser Ranger Examples")
        print("=" * 40)
        print(f"\nUsage: python {sys.argv[0]} <example>\n")
        print("Available examples:")
        for name, func in EXAMPLES.items():
            doc = func.__doc__.split("\n")[0] if func.__doc__ else ""
            print(f"  {name:12} - {doc}")
        print(f"\nExample: python {sys.argv[0]} basic")
        return

    example_name = sys.argv[1].lower()

    if example_name not in EXAMPLES:
        print(f"Unknown example: {example_name}")
        print(f"Available: {', '.join(EXAMPLES.keys())}")
        return

    try:
        EXAMPLES[example_name]()
    except Exception as e:
        print(f"\nError: {e}")
        raise


if __name__ == "__main__":
    main()
