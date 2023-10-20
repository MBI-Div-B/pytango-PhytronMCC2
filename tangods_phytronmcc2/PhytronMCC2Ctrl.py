#!/usr/bin/python3 -u
# coding: utf8
# PhytronMCC2Ctrl
from tango import DevState, AttrWriteType, DispLevel
from tango.server import Device, attribute, command, device_property
import time
import sys
import socket


class PhytronMCC2Ctrl(Device):
    # device properties
    Address = device_property(
        dtype="str",
        default_value="phymotion.sxr.lab",
    )

    Port = device_property(
        dtype="int",
        default_value=22222,
    )

    # definition some constants
    __STX = chr(2)  # Start of text
    __ACK = chr(6)  # Command ok
    __NACK = chr(0x15)  # command failed
    __ETX = chr(3)  # end of text

    def init_device(self):
        Device.init_device(self)
        self.info_stream("init_device()")
        self.set_state(DevState.OFF)

        # configure serial
        self.con = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.con.connect((self.Address, self.Port))

        self.info_stream("Address: {:s}".format(self.Address))
        self.info_stream("Port = {:d}".format(self.Port))

        self.set_state(DevState.ON)

    def delete_device(self):
        self.con.close()

    @command(dtype_in=str, dtype_out=str)
    def write_read(self, cmd):
        cmd = self.__STX + cmd + ':XX' + self.__ETX
        self.debug_stream("write command: {:s}".format(cmd))
        self.con.send(cmd.encode("utf-8"))
        res = self.con.recv(1024).decode("utf-8")
        self.debug_stream("read response: {:s}".format(res))
        if self.__ACK in res:
            return res.lstrip(self.__STX).lstrip(self.__ACK).rstrip(self.__ETX).split(':')[0]
        else:
            # no acknowledgment in response
            return self.__NACK

    def is_write_allowed(self):
        if self.get_state() in [DevState.FAULT, DevState.OFF]:
            return False
        return True


if __name__ == "__main__":
    PhytronMCC2Ctrl.run_server()
