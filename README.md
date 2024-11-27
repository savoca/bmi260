## Compile

- `make`
- `insmod bmi260.ko`

## Initialization

- If the device has a `bosch,bmi260` definition in the device tree, the driver's `probe()` method is called automatically.
- For kernel hacking or forcing `probe()` on a specific I2C bus with custom GPIO/IRQ, use `fakedevice.ko`:
  - `insmod fakedevice.ko`
  - `modprobe fakedevice.ko`
- Devices enumerated by BIOS/ACPI are not currently supported, but this can be added with minimal changes.

## Usage
- Right now you can change modes by writing them, e.g.
  - `echo 1 > /sys/bus/i2c/devices/3-0018/mode` will enter normal mode with BMI260 on i2c bus 3, address 0x18
- Without IIO, a simple processed or raw data value could look like:
  - `cat /sys/bus/i2c/devices/3-0018/accel_processed` or `cat /sys/bus/i2c/devices/3-0018/gyro_raw`


## TODO

- Add a method for users to read raw or processed accelerometer/gyroscope data from the FIFO.
- Implement all device modes (e.g., performance mode).
- Register with the IIO subsystem to enable testing the sensor with better userspace tools and visualizers.
