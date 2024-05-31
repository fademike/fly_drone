# fly_drone

My example drone. (in development)

![alt text](https://github.com/fademike/fly_drone/blob/main/4.png)


How to connect:
Android phone (AppFlyControl) <-> usb-uart(example: cp2102) <-> uart->rf (STM8S003_AND_SI4463) <-> fly_drone
<br>
or:
<br>
Android phone (AppFlyControl) <-wifi-> Linux PC (flyLinuxServer) <-> uart->rf (STM8S003_AND_SI4463) <-> fly_drone

on board:
- DC-DC for 2,3,..S battery operation. To operate on a 1S battery, it is necessary to replace the components to turn off the DC-DC.
- RF SI4463 433MHz radio transceiver. when changing the frequency, you can to change components and programmatically set a different frequency.
- imu mpu6050
- 4pcs 3-pin connector for servo motor/ESC (IMP 1-2 ms) for 4pcs brushed motors need to set mosfet or esc for brushed motors.
- i2c connector, for example: for VL53L0X
- connector for debugging by swdio/uart

![alt text](https://github.com/fademike/fly_drone/blob/main/fly_drone.png)

For connect brushed motors with esc or servo motor, you can use convertor on attiny13a. example: kicad->tiny_2nmosfet.
For connect 4pcs brushed motors, you can use n-mosfet on every channel.

<br>
Code:
<br>
bootloader (update firmware via RF by FlyLinuxServer)<br>
fly_drone<br>
- mavlink protocol:<br>
-- params list settings(PID, mux chan, motor settings, orientation)<br>
-- RC_CHANNELS<br>
-- COMMAND_LONG - ARM_DISARM/CALIBRATION/REBOOT<br>
-- MSG_STATUSTEXT - logs<br>
- imu, rf, vl53l0x<br>
- acc and gyro calibrate<br>
- for save params and acc calibrate on flash, need to change "flash_params"<br>
<br>
And in repository:<br>
kicad:<br>
- ESC_atmega8 - esc for brashless motor. (need to changed frequency by fuses)<br>
- stm32f103_Fly.. - fly_drone project<br>
- tiny_2nmosfet - esc for connection 4 brashed motors(independent 4 in, 4 out)<br>
<br>
video fly_drone.mpg in video repository

