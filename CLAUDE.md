# serial-laser-ranger

Python library for serial laser distance measurement modules.

## Project Structure

```
serial_laser_ranger.py   # Main library module
examples.py              # Usage examples (run with: python examples.py <name>)
pyproject.toml           # Package configuration for PyPI
docs.txt                 # Protocol reference (transcribed from docs.png)
```

## Hardware

- Generic infrared laser ranging modules (AliExpress, DFRobot SEN0366, etc.)
- Serial connection: 9600 baud, 8N1
- Default device address: 0x80 (128)
- Frame size: 11 bytes (ADDR + CMD + STATUS + 7 bytes data + CS)

## Key Implementation Details

- Checksum calculation: `(~sum(bytes) + 1) & 0xFF`
- Commands use format: `[ADDR, 0x06, CMD, ...params]`
- Responses contain ASCII-encoded distance (e.g., "000.619" for 0.619m)
- Continuous mode streams data; `connect()` resets device state to avoid buffer issues

## Hardware Limitations

- **Max sample rate: ~4 Hz** (235ms per measurement) despite marketing claims of 1-20Hz
- `set_frequency()` has no effect; `set_data_return_interval()` only adds delay
- **Motion limit: ~25 cm/s** - faster target movement causes sensor reset/errors

## Testing

Device connected at: `/dev/cu.usbserial-2210`

```bash
python examples.py basic       # Single measurement
python examples.py continuous  # Stream measurements
python examples.py statistics  # Collect and analyze samples
```

## Publishing to PyPI

```bash
pip install build twine
python -m build
twine upload dist/*
```
