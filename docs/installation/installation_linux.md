---
title: Installation Linux
has_children: false
parent: Installation
nav_order: 1
---

# Installation on Linux Systems
This project was developed on Ubuntu 20.04 but also runs on Ubuntu 22.04.

**!!! The ARCHES PiCar-X will not work on Ubuntu 24.04, since the character devices fully replace the sysfs GPIO functionallity.**

## Build new Kernel if not all required modules are installed

```console
uname -r

# POSSIBLE RESULT:
On Ubuntu 20.04: 5.13.0-48-generic

sudo apt-get install wget unzip build-essential flex bison libssl-dev libelf-dev dwarves ncurses-dev zstd xorg

wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.13.tar.gz

tar xf linux-5.13.tar.gz

cd linux-5.13/

make clean && make mrproper

cp /usr/lib/modules/$(uname -r)/build/.config ./

# ALTERNATIVE 1 (activate the modules via GUI):
make menuconfig

# ALTERNATIVE 2 (write the modules to file):
echo CONFIG_GPIOLIB=y >> .config
echo CONFIG_GPIO_SYSFS=y >> .config 
echo CONFIG_GPIO_CDEV=y >> .config
echo CONFIG_GPIO_CDEV_V1=y >> .config
echo CONFIG_GPIO_MOCKUP=m >> .config   
echo CONFIG_I2C_CHARDEV=m >> .config
echo CONFIG_I2C_STUB=m >> .config

make scripts

# Disable the key options, otherwise the kernel may wont be built properly
scripts/config --disable CONFIG_SYSTEM_REVOCATION_KEYS
scripts/config --disable CONFIG_SYSTEM_TRUSTED_KEYS


# This will take quite some time, with more cores you are faster
make -j #NumberOfCores
make bzImage
sudo make modules_install
sudo make install

sudo update-grub

sudo reboot
```

After reboot you can check if your new kernel is activated by:

```console
uname -r

# Result should be the Kernel version you wanted to install, e.g.:
linux-5.13
```

If you built and installed the new kernel properly, you should now be able to [activate GPIO and I2C]({{ site.baseurl }}{% link getting_started.md %}). 