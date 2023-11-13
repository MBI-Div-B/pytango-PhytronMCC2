
# Phytron MCC2 Tango device server

This a Tango device server written in PyTango for a Phytron MCC2 stepper motor
controller using the RS485 serial interface.
It consits of a `PhytronMCC2Ctrl` device server that handles the communication
with the RS485 bus and one to many `PhytronMCC2Axis` device servers implementing
the actual interface to the stepper axis.

## Installation

### USB Serial converter

Create a udev rule in order to mount the USB Serial converter always under the same link, e.g. `/dev/ttyMCC`

First check the VendorID, ProductID, and SerialNumber using `dmesg`
Then add a new udev rule
    
    sudo nano /etc/udev/rules.d/55-usbcom.rules
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A106K4W0", SYMLINK+="ttyMCC", MODE="0666"

Relaod and apply the udev rule by

    sudo udevadm control --reload
    sudo udevadm trigger --action=add

## Authors

* Dirk Rohloff
* Daniel Schick


