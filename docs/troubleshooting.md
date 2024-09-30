---
title: Troubleshooting
has_children: false
nav_order: 9
---


# Troubleshooting

## Problem 1: I2C device not activate or wrong device configured in the *.env

If you have configured a device that exists but cannot be used due to missing registers, you will get the error:
```
motor_right-dtp-1           |     raise IOError("Can't send stuff.")
motor_right-dtp-1           | OSError: Can't send stuff.
```

**Solution:** Reboot, and check first with `ls /dev/i2c*` whether there is already an existing device. The device that already exists, cannot be used for the DTP.
Create a new one and adjust the *.env as described in [Getting Started]({{ site.baseurl }}{% link getting_started.md %}).

## Problem 2: /dev/I2C-X is busy**

If you started the Docker compose file before you activated I2C, than a folder named /dev/I2C-X was created on your system, due to the mounting of volumes.

**Solution:** Remove the folder via 

```console
sudo rm -r /dev/i2c-x
```

And [activate I2C](#activate-gpio-and-i2c-on-your-system)

## Several Active I2C devices:
If there are several active I2C devices, you have to find the one with the active register 0x14.

### Case 1: no active register:

```console
$ sudo i2cdetect -r 0
WARNING! This program can confuse your I2C bus, cause data loss and worse!
I will probe file /dev/i2c-0 using receive byte commands.
I will probe address range 0x03-0x77.
Continue? [Y/n] Y
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: 30 -- -- -- -- -- UU UU -- -- -- -- -- -- -- -- 
40: -- -- -- -- 44 -- -- -- -- -- -- -- -- -- -- -- 
50: -- UU -- UU -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

### Case 2: active register:

```console
$ sudo i2cdetect -r 3
WARNING! This program can confuse your I2C bus, cause data loss and worse!
I will probe file /dev/i2c-3 using receive byte commands.
I will probe address range 0x03-0x77.
Continue? [Y/n] Y
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- 14 -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

`/dev/i2c-3` is the correct device.


**Problem 3: GPIO ports exist after the containers crashed/were killed.**

If you use CTRL+C more than once, you kill the containers instead of stopping them. Although deleting the GPIO pins is part of the shutdown routine, this is skipped if the containers crash or get killed.
The error message is something like

```
OSError: [Errno 16] Device or resource busy
```

**Solution:** 
Try to unexport the pins manually.

```console
echo 24 > /sys/class/gpio/unexport
echo 23 > /sys/class/gpio/unexport
```

If this does not work due to permission issues, than reboot your WSL2 or Linux System or create a script with the following commands and execute it.

```console
# create the script with nano editor
nano unexport-pins.sh

# write the command to the script
echo "echo 23 > /sys/class/gpio/unexport" >> unexport-pins.sh
echo "echo 24 > /sys/class/gpio/unexport" >> unexport-pins.sh

# make it executeable
chmod +x unexport-pins.sh

# execute
sudo ./unexport-pins.sh
```

Check with `ls /sys/class/gpio` whether the pins were removed.