---
title: Troubleshooting
has_children: false
nav_order: 9
---


# Troubleshooting

**Problem 1: /dev/I2C-X is busy:**

If you started the Docker compose file before you activated I2C, than a folder named /dev/I2C-X was created on your system, due to the mounting of volumes.

<em>Solution:</em> Remove the folder via 

```console
sudo rm -r /dev/i2c-x
```

And [activate I2C](#activate-gpio-and-i2c-on-your-system)

**Problem 2: GPIO ports exist after the containers crashed/were killed.**

If you use CTRL+C more than once, you kill the containers instead of stopping them. Although deleting the GPIO pins is part of the shutdown routine, this is skipped if the containers crash or get killed.

<em>Solution:</em> 
Try to unexport the pins manually.

```console
echo 24 > /sys/class/gpio/unexport
echo 23 > /sys/class/gpio/unexport
```
If this does not work, due to permission issues, than reboot your WSL2 or Linux System.