#!/usr/bin/python3 -u
# coding: utf8
# PhytronMCC2Ctrl
from tango import DevState, AttrWriteType, DispLevel
from tango.server import Device, attribute, command, device_property
import time
import sys
import serial


class PhytronMCC2Ctrl(Device):
    # device properties
    Port = device_property(
        dtype="str",
        default_value="/dev/ttyMCC",
    )

    Baudrate = device_property(
        dtype="int",
        default_value=115200,
    )

    # device attributes
    port = attribute(
        dtype="str",
        label="port",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    baudrate = attribute(
        dtype="int",
        label="baudrate",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    # connection settings
    PARITY = (
        serial.PARITY_NONE
    )  # serial.PARITY_NONE, serial.PARITY_ODD, serial.PARITY_EVEN
    FLOWCONTROL = "none"  # "none", "software", "hardware", "sw/hw"
    TIMEOUT = 0
    BYTESIZE = 8
    STOPBITS = 1

    # definition some constants
    __STX = chr(2)  # Start of text
    __ACK = chr(6)  # Command ok
    __NACK = chr(0x15)  # command failed
    __ETX = chr(3)  # end of text

    def init_device(self):
        self.info_stream("init_device()")
        self.get_device_properties(self.get_device_class())
        self.set_state(DevState.OFF)

        # configure serial
        self.serial = serial.Serial()
        self.serial.baudrate = self.Baudrate
        self.serial.port = self.Port
        self.serial.parity = self.PARITY
        self.serial.bytesize = self.BYTESIZE
        self.serial.stopbits = self.STOPBITS
        self.serial.timeout = self.TIMEOUT

        if self.FLOWCONTROL == "none":
            self.serial.xonxoff = 0
            self.serial.rtscts = 0
        elif self.FLOWCONTROL == "software":
            self.serial.xonxoff = 1
            self.serial.rtscts = 0
        elif self.FLOWCONTROL == "hardware":
            self.serial.xonxoff = 0
            self.serial.rtscts = 1
        elif self.FLOWCONTROL == "sw/hw":
            self.serial.xonxoff = 1
            self.serial.rtscts = 1

        self.info_stream("port: {:s}".format(self.Port))
        self.info_stream("baudrate = {:d}".format(self.Baudrate))

        # open serial port
        self.open()

    def delete_device(self):
        self.close()

    # attribute read/write methods
    def read_port(self):
        return self.Port

    def read_baudrate(self):
        return int(self.Baudrate)

    # commands
    @command
    def open(self):
        self.info_stream("open()")

        try:
            self.serial.open()
            self.set_state(DevState.ON)
            self.info_stream("connected to {:s}".format(self.Port))
        except Exception:
            self.error_stream("failed to open {:s}".format(self.Port))
            sys.exit(255)

    def is_open_allowed(self):
        if self.get_state() in [DevState.ON, DevState.FAULT]:
            return False
        return True

    @command
    def close(self):
        try:
            self.serial.close()
            self.set_state(DevState.OFF)
            self.info_stream("closed connection on {:s}".format(self.Port))
        except Exception:
            self.warn_stream("could not close connection on {:s}".format(self.Port))

    def is_close_allowed(self):
        if self.get_state() in [DevState.OFF]:
            return False
        return True

    @command(dtype_in=str, dtype_out=str)
    def write_read(self, cmd):
        cmd = self.__STX + cmd + self.__ETX
        self.debug_stream("write command: {:s}".format(cmd))
        self.serial.write(cmd.encode("utf-8"))
        self.serial.flush()
        time.sleep(0.02)  # 20ms wait time
        res = self.serial.readline().decode("utf-8")
        self.debug_stream("read response: {:s}".format(res))
        if self.__ACK in res:
            return res.lstrip(self.__STX).lstrip(self.__ACK).rstrip(self.__ETX)
        else:
            # no acknowledgment in response
            return self.__NACK

    def is_write_allowed(self):
        if self.get_state() in [DevState.FAULT, DevState.OFF]:
            return False
        return True


if __name__ == "__main__":
    PhytronMCC2Ctrl.run_server()
