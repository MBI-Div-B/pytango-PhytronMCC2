#!/usr/bin/python3 -u
# coding: utf8
# PhytronMCC2Ctrl

from tango import DevState
from tango.server import Device, command, device_property

import serial


class PhytronMCC2Ctrl(Device):
    # device properties
    Port = device_property(
        dtype="str",
        default_value="/dev/ttyMCC",
    )

    Baudrate = device_property(
        dtype="int",
        default_value=57600,
    )

    # definition some constants
    __STX = chr(2)  # Start of text
    __ACK = chr(6)  # Command ok
    __NACK = chr(0x15)  # command failed
    __ETX = chr(3)  # end of text

    def init_device(self):
        super().init_device()
        self.set_state(DevState.INIT)
        self.info_stream("init_device()")

        # configure serial
        self.serial = serial.Serial()
        self.serial.baudrate = self.Baudrate
        self.serial.port = self.Port
        self.serial.parity = serial.PARITY_NONE
        self.serial.bytesize = 8
        self.serial.stopbits = 1
        self.serial.timeout = 0

        # open serial connection
        try:
            self.serial.open()
            self.info_stream("Connected to {:s}".format(self.Port))
            self.set_state(DevState.ON)
        except Exception:
            self.error_stream("Failed to open {:s}".format(self.Port))
            self.set_state(DevState.FAULT)

    def delete_device(self):
        self.serial.close()
        self.set_state(DevState.OFF)

    # commands
    @command(dtype_in=str, dtype_out=str)
    def write_read(self, cmd):
        cmd = self.__STX + cmd + self.__ETX
        self.debug_stream("write command: {:s}".format(cmd))
        
        self.serial.write(cmd.encode("utf-8"))
        res = ""
        while not self.__ETX in res:
            self.serial.flush()
            line = self.serial.readline().decode("utf-8")
            res += line

        self.debug_stream("read response: {:s}".format(res))
        if self.__ACK in res:
            return res.lstrip(self.__STX).lstrip(self.__ACK).rstrip(self.__ETX)
        else:
            # no acknowledgment in response
            self.debug_stream("read response: NACK")
            return self.__NACK

    def is_write_allowed(self):
        if self.get_state() in [DevState.FAULT, DevState.OFF]:
            return False
        return True


if __name__ == "__main__":
    PhytronMCC2Ctrl.run_server()
