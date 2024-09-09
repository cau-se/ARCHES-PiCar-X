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