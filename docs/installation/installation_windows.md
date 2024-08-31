---
title: Installation Windows
has_children: false
parent: Installation
nav_order: 2
---

# Installation on Windows with WSL2
The goal of this project is to exemplify my research on digital twins. All concepts presented and formalized in the paper:



## XLaunch for Windows

[Download the VcXsrv X Server for Windows](https://sourceforge.net/projects/vcxsrv/).
Start the application with following configuration:

1. First Screen: Multiple Windows
2. Second Screen: Start no client
3. Third Screen: Deactivate "<em>Native opengl</em>"
4. Fourth Screen: Start


# Build new Windows WSL2 Kernel

WSL2 Kernel can be found on [the official WSL2 Linux Kernel GitHub page](https://github.com/microsoft/WSL2-Linux-Kernel).

```console
uname -r

# Possible Output: 
5.10.102.1-microsoft-standard-WSL2


sudo apt-install wget unzip build-essential flex bison libssl-dev libelf-dev dwarves

wget https://github.com/microsoft/WSL2-Linux-Kernel/releases/tag/linux-msft-wsl-5.10.102.1.zip
unzip linux-msft-wsl-5.10.102.1.zip

cp ./Microsoft/config-wsl .config

# ALTERNATIVE 1 (activate the modules via GUI):
make menuconfig

# ALTERNATIVE 2 (write the modules to file):
echo CONFIG_GPIOLIB=y >> .config
echo CONFIG_GPIO_SYSFS=y >> .config             
echo CONFIG_GPIO_CDEV=y >> .config
echo CONFIG_GPIO_CDEV_V1=y >> .config
echo CONFIG_GPIO_MOCKUP=m   >> .config           
echo CONFIG_I2C_CHARDEV=m >> .config
echo CONFIG_I2C_STUB=m >> .config


make KCONFIG_CONFIG=.config -j $NumberOfCores    # if you have 4 cores, just type 4 (the more the better)
```

## Activate new WSL2 Kernel:

Create a folder where you will copy the Kernel from the WSL2 VM to Windows, for example C:\WSLKernel. Then
go back to your Linux console and copy the bzImage and shutdown the WSL2 VM:

```console
# IN WSL:
cp arch/x86/boot/bzImage /mnt/c/WSLKernel

# SHUTDOWN WSL2 VIA POWERSHELL
wsl --shutdown
```

Back on Windows you know have to copy the <em>bzImage</em> to the kernel folder <em>C:\Windows\System32\lxss\tools</em>
Then rename the <em>kernel</em> file to <em>kernel.old</em>, **do not delete it**! If something went wrong, you can just restore the old working kernel. 

After you copied the <em>bzImage</em> into the folder, rename <em>bzImage</em> to <em>kernel</em>. Afterwards, start Ubuntu again and go into the WSL2 Kernel folder from the privious steps and install the modules via:

```console
make modules_install
```

If you built and installed the new kernel properly, you should now be able to [activate GPIO and I2C]({{ site.baseurl }}{% link getting_started.md %}). 