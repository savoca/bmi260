## Compile

- `make`
- `insmod bmi260.ko`

## Initialization

- If the device has a `bosch,bmi260` definition in the device tree, the driver's `probe()` method is called automatically.
- For kernel hacking or forcing `probe()` on a specific I2C bus with custom GPIO/IRQ, use `fakedevice.ko`:
  - `insmod fakedevice.ko`
  - `modprobe fakedevice.ko`
- Devices enumerated by BIOS/ACPI are not currently supported, but this can be added with minimal changes.

## TODO

- Add a method for users to read raw or processed accelerometer/gyroscope data from the FIFO.
- Implement all device modes (e.g., performance mode).
- Register with the IIO subsystem to enable testing the sensor with better userspace tools and visualizers.
