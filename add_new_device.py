# -*- coding: utf-8 -*-

# add a Phytron-MCC TangoDevice

from PyTango import Database, DbDevInfo, DeviceProxy
import sys
import re

NAME = "sxr/MCC2/Ctrl"

ctrl, addr, axis = "", "", ""
device_name = ""


def usage():
    print("\nusage: add_new_device /sxr/MCC2/Ctrl[nof]_[addr].[motor]")
    print("example: add_new_device /sxr/MCC2/Ctrl1_0.0")
    print("   add a device with controller:1, address 0 and axis 0\n")
    print("   - nof: number of controller")
    print("   - addr: address of module (0..15)")
    print("   - motor: motor axis on modul (0/1)")
    sys.exit(255)


def create_device(ctrl, addr, axis):
    db = Database()
    new_device_info_mcc = DbDevInfo()
    new_device_info_mcc._class = "PhytronMcc2"
    new_device_info_mcc.server = "PhytronMcc2/raspi14"

    new_device = NAME + ctrl + "_" + addr + "_" + axis

    print("Creating device: %s" % new_device)
    new_device_info_mcc.name = new_device
    db.add_device(new_device_info_mcc)
    return new_device


def create_properties(device_name, addr, axis):
    CTRL_DEVICE = "sxr/SerialDS/1"
    MOTOR_NAME = "mcc_"
    property_names = ["Address", "Motor", "CtrlDevice"]

    mcc = DeviceProxy(device_name)

    prop_dict = mcc.get_property(property_names)
    prop_dict["CtrlDevice"] = CTRL_DEVICE
    prop_dict["Motor"] = int(axis)
    prop_dict["Address"] = int(addr)
    prop_dict["Alias"] = MOTOR_NAME + (addr) + "_" + axis

    mcc.put_property(prop_dict)


def check_input():
    if not (NAME in sys.argv[1]):
        usage()
    tmp = re.split("[.l_]", sys.argv[1])
    axis = tmp[-1]
    if not axis.isdigit():
        print("\naxis not an integer!")
        usage()
    addr = tmp[-2]
    if not addr.isdigit():
        print("\naddr not an integer!")
        usage()
    ctrl = tmp[-3]
    if not ctrl.isdigit():
        print("\nctrl not an integer!")
        usage()
    return (ctrl, addr, axis)


def main():
    if len(sys.argv) < 2:
        usage()

    ctrl, addr, axis = check_input()
    device_name = create_device(ctrl, addr, axis)
    create_properties(device_name, addr, axis)


if __name__ == "__main__":
    main()
