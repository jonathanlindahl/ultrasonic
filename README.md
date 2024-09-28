# ultrasonic
Linux device driver for ultrasonic sensor HC-SR04

The module works by triggering an ultrasonic pulse, the echo from which then causes an interrupt.
Time between the trigger and echo is measured and is used to calculate distance in the following way: uS / 58

Future improvements:
- Trigger and read measurements through a userspace application.
- Another userspace application could be used to regularly trigger measurements and only output when the distance changes from the last reading, resulting in a simple motion sensor.

Raspberry pi pinout can be found here: https://pinout.xyz/
Datasheet for the sensor can be found here: https://www.alldatasheet.com/datasheet-pdf/pdf/1132203/ETC2/HC-SR04.html
TODO: Add diagram

Build using make.

Insert the module with: sudo insmod ultrasonic.ko

Check that the module has been inserted: sudo lsmod | grep ultrasonic

To trigger a measurement use: echo "1" | sudo tee /dev/usnc_device

To read the result of the measurement use: dmesg | grep ultrasonic

To remove the module use: sudo rmmod ultrasonic
