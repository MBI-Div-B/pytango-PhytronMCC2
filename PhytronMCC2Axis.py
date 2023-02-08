#!/usr/bin/python3 -u
# coding: utf8
# PhytronMCC2Axis
from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import sys
from enum import IntEnum


class MovementType(IntEnum):
    rotational = 0
    linear = 1


class MovementUnit(IntEnum):
    step = 0
    mm = 1
    inch = 2
    degree = 3


class InitiatorType(IntEnum):
    NCC = 0
    NOC = 1


class EncoderType(IntEnum):
    none = 0
    incremental = 1
    SSI_binary = 2
    SSI_Gray = 3


class PhytronMCC2Axis(Device):
    # device properties
    CtrlDevice = device_property(
        dtype="str", default_value="domain/family/member"
    )

    Axis = device_property(
        dtype="int16"
    )

    Address = device_property(
        dtype="int16"
    )

    Alias = device_property(
        dtype="str"
    )

    # device attributes
    hw_limit_minus = attribute(
        dtype="bool",
        label="HW limit -",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    hw_limit_plus = attribute(
        dtype="bool",
        label="HW limit +",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    sw_limit_minus = attribute(
        dtype="float",
        format="%8.3f",
        label="SW limit -",
        unit="steps",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    sw_limit_plus = attribute(
        dtype="float",
        format="%8.3f",
        label="SW limit +",
        unit="steps",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    position = attribute(
        dtype="float",
        format="%8.3f",
        label="position",
        unit="steps",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.OPERATOR,
    )

    pos_P19 = attribute(
        dtype="float",
        format="%8.3f",
        label="position E (P19)",
        access=AttrWriteType.READ,
        display_level=DispLevel.EXPERT
    )

    pos_P20 = attribute(
        dtype="float",
        format="%8.3f",
        label="position M (P20)",
        access=AttrWriteType.READ,
        display_level=DispLevel.EXPERT
    )

    pos_P21 = attribute(
        dtype="float",
        format="%8.3f",
        label="position enc (P21)",
        access=AttrWriteType.READ,
        display_level=DispLevel.EXPERT
    )

    alias = attribute(
        dtype="string",
        label="alias",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR,
    )

    inverted = attribute(
        dtype="bool",
        label="inverted",
        memorized=True,
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    acceleration = attribute(
        dtype="int",
        label="acceleration",
        unit="Hz",
        min_value=4000,
        max_value=500000,
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    velocity = attribute(
        dtype="int",
        label="velocity",
        unit="Hz",
        min_value=0,
        max_value=40000,
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    homing_velocity = attribute(
        dtype="int",
        label="homing velocity",
        unit="Hz",
        min_value=0,
        max_value=40000,
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )
    hold_current = attribute(
        dtype="float",
        label="hold current",
        unit="A",
        min_value=0,
        max_value=2.5,
        format="%2.1f",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    run_current = attribute(
        dtype="float",
        label="run current",
        unit="A",
        min_value=0,
        max_value=2.5,
        format="%2.1f",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    initiator_type = attribute(
        dtype=InitiatorType,
        label="initiator type",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    steps_per_unit = attribute(
        dtype="float",
        format="%10.1f",
        label="steps per unit",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    step_resolution = attribute(
        dtype="int",
        label="step resolution",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="""Step resolution 1 to 256
1 = Full step
2 = Half step
4 = 1/4 step
8 = 1/8 step
10 = 1/10 step
16 = 1/16 step
128 = 1/128 step
256 = 1/256 step"""
    )

    backlash_compensation = attribute(
        dtype="int",
        label="backlash compensation",
        unit="step",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    type_of_movement = attribute(
        dtype=MovementType,
        label="type of movement",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="""0 = rotational
Rotating table, 1 limit switch for mechanical zero
(referencing)
1 = linear
for XY tables or other linear systems,
2 limit switches:
Mechanical zero and limit direction -
Limit direction +"""
    )

    movement_unit = attribute(
        dtype=MovementUnit,
        label="unit",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="Allowed unit values are step, mm, inch, degree"
    )

    encoder_type = attribute(
        dtype=EncoderType,
        label="encoder type",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="type of installed encoder"
    )

    encoder_resolution = attribute(
        dtype="int",
        label="encoder resolution",
        unit="bit",
        min_value=10, max_value=31,
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="absolute (SSI) encoder resolution in bit (max. 31)"
    )

    encoder_conversion = attribute(
        dtype="float",
        format="%10.1f",
        label="encoder steps per unit",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    # private class properties
    __NACK = chr(0x15)     # command failed
    __LIM_PLUS = 2
    __LIM_MINUS = 1
    __Axis_Name = ''
    __HW_Limit_Minus = False
    __HW_Limit_Plus = False
    __Inverted = False
    __Unit = MovementUnit.step
    __Steps_Per_Unit = 1.0
    __Encoder = EncoderType.none

    def init_device(self):
        super().init_device()
        self.info_stream("init_device()")

        if self.Axis == 0:
            self.__Axis_Name = "X"
        else:
            self.__Axis_Name = "Y"

        self.info_stream("module address: {:d}".format(self.Address))
        self.info_stream("module axis: {:d}".format(self.Axis))
        self.info_stream("alias: {:s}".format(self.Alias))

        try:
            self.ctrl = DeviceProxy(self.CtrlDevice)
            self.info_stream("ctrl. device: {:s}".format(self.CtrlDevice))
        except DevFailed as df:
            self.error_stream("failed to create proxy to {:s}".format(df))
            sys.exit(255)

        # check if the CrlDevice ON, if not open the serial port
        if str(self.ctrl.state()) == "OFF":
            self.ctrl.open()
            self.info_stream("controller sucessfully opened")
        else:
            self.info_stream("controller was already open")

        if ("MCC" in self.read_firmware_version()):
            # read memorized attributes from Database
            self.db = Database()
            try:
                attr = self.db.get_device_attribute_property(self.get_name(), ["inverted"])
                if attr["inverted"]["__value"][0] == "true":
                    self.__Inverted = True
                else:
                    self.__Inverted = False
            except Exception:
                self.__Inverted = False

            self.__Encoder = self.read_encoder_type()
            self.set_state(DevState.ON)
        else:
            self.set_state(DevState.OFF)

        self.info_stream("HW limit-: {0}".format(self.__HW_Limit_Minus))
        self.info_stream("HW limit+: {0}".format(self.__HW_Limit_Plus))

    def delete_device(self):
        self.set_state(DevState.OFF)

    def always_executed_hook(self):
        answer = self._send_cmd("SE")
        if (self.Axis == 0):
            if self.__Inverted:
                self.__HW_Limit_Minus = bool(int(answer[2]) & self.__LIM_PLUS)
                self.__HW_Limit_Plus = bool(int(answer[2]) & self.__LIM_MINUS)
            else:
                self.__HW_Limit_Minus = bool(int(answer[2]) & self.__LIM_MINUS)
                self.__HW_Limit_Plus = bool(int(answer[2]) & self.__LIM_PLUS)
            moving = not(bool(int(answer[1]) & 1))
        else:
            if self.__Inverted:
                self.__HW_Limit_Minus = bool(int(answer[6]) & self.__LIM_PLUS)
                self.__HW_Limit_Plus = bool(int(answer[6]) & self.__LIM_MINUS)
            else:
                self.__HW_Limit_Minus = bool(int(answer[6]) & self.__LIM_MINUS)
                self.__HW_Limit_Plus = bool(int(answer[6]) & self.__LIM_PLUS)
            moving = not(bool(int(answer[5]) & 1))
        self.debug_stream("HW limit-: {0}".format(self.__HW_Limit_Minus))
        self.debug_stream("HW limit+: {0}".format(self.__HW_Limit_Plus))
        if moving is False:
            self.set_status("Device in ON")
            self.set_state(DevState.ON)
            self.debug_stream("device is: ON")
        else:
            self.set_status("Device is MOVING")
            self.set_state(DevState.MOVING)
            self.debug_stream("device is: MOVING")

    # attribute read/write methods
    def read_hw_limit_minus(self):
        return self.__HW_Limit_Minus

    def read_hw_limit_plus(self):
        return self.__HW_Limit_Plus

    def read_sw_limit_minus(self):
        ret = float(self.send_cmd("P24R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def write_sw_limit_minus(self, value):
        if self.__Inverted:
            value = -1*value
        self.send_cmd("P24S{:f}".format(value))

    def read_sw_limit_plus(self):
        ret = float(self.send_cmd("P23R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def write_sw_limit_plus(self, value):
        if self.__Inverted:
            value = -1*value
        self.send_cmd("P23S{:f}".format(value))

    def read_position(self):
        param_pos = 20 if self.__Encoder == EncoderType.none else 21
        ret = float(self.send_cmd(f"P{param_pos}R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def read_pos_P19(self):
        ret = float(self.send_cmd(f"P19R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def read_pos_P20(self):
        ret = float(self.send_cmd(f"P20R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def read_pos_P21(self):
        ret = float(self.send_cmd(f"P21R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def write_position(self, value):
        if self.__Inverted:
            value = -1*value
        if self.__Encoder == EncoderType.none:
            answer = self.send_cmd("A{:.10f}".format(value))
        elif self.__Encoder == EncoderType.incremental:
            ### v1: free run with stop on encoder position (P21)
            ### typo in manual? Doesn't seem to check P21, but P20 -> no good
            # current_pos = self.read_position()
            # ax = self.__Axis_Name
            # mv, comp = '+>' if current_pos < value else '-<'
            # answer = self._send_cmd(f"{ax}L{mv} {ax}{comp}{value} {ax}S")

            ### v2: relative move -> better
            delta = value - self.read_position()
            answer = self.send_cmd(f"{delta:+.10f}")
        else:  # absolute encoder, TODO: decide what to do
            answer = self.send_cmd("A{:.10f}".format(value))

        if answer != self.__NACK:
            self.set_state(DevState.MOVING)

    def read_alias(self):
        return self.Alias

    def read_inverted(self):
        return self.__Inverted

    def write_inverted(self, value):
        self.__Inverted = bool(value)

    def read_acceleration(self):
        return int(self.send_cmd("P15R"))

    def write_acceleration(self, value):
        self.send_cmd("P15S{:d}".format(value))

    def read_velocity(self):
        return int(self.send_cmd("P14R"))

    def write_velocity(self, value):
        self.send_cmd("P14S{:d}".format(value))

    def read_homing_velocity(self):
        return int(self.send_cmd("P08R"))

    def write_homing_velocity(self, value):
        self.send_cmd("P08S{:d}".format(value))

    def read_run_current(self):
        return float(self.send_cmd("P41R"))/10

    def write_run_current(self, value):
        value = int(value*10)
        if value not in range(0, 26):
            return "input not in range 0..25"
        self.send_cmd("P41S{:d}".format(value))

    def read_hold_current(self):
        return float(self.send_cmd("P40R"))/10

    def write_hold_current(self, value):
        value = int(value*10)
        if value not in range(0, 26):
            return "input not in range 0..25"
        self.send_cmd("P40S{:d}".format(value))

    def read_initiator_type(self):
        return InitiatorType.NOC if bool(int(self.send_cmd("P27R"))) else InitiatorType.NCC

    def write_initiator_type(self, value):
        self.send_cmd("P27S{:d}".format(int(value)))

    def read_steps_per_unit(self):
        # inverse of spindle pitch (see manual page 50)
        self.__Steps_Per_Unit = 1/float(self.send_cmd("P03R"))
        return self.__Steps_Per_Unit

    def write_steps_per_unit(self, value):
        # inverse of spindle pitch (see manual page 50)
        self.send_cmd("P03S{:10.8f}".format(1/value))
        # update display unit
        self.set_display_unit()

    def read_step_resolution(self):
        return int(self.send_cmd("P45R"))

    def write_step_resolution(self, value):
        if value not in [1, 2, 4, 8, 10, 16, 128, 256]:
            return "input not in [1, 2, 4, 8, 10, 16, 128, 256]"
        self.send_cmd("P45S{:d}".format(value))

    def read_backlash_compensation(self):
        ret = int(self.send_cmd("P25R"))
        if self.__Inverted:
            return -1*ret
        else:
            return ret

    def write_backlash_compensation(self, value):
        if self.__Inverted:
            value = -1*value
        self.send_cmd("P25S{:d}".format(int(value)))

    def read_type_of_movement(self):
        return MovementType.linear if bool(int(self.send_cmd("P01R"))) else MovementType.rotational

    def write_type_of_movement(self, value):
        self.send_cmd("P01S{:d}".format(int(value)))

    def read_encoder_type(self):
        return EncoderType(int(self.send_cmd("P34R")))

    def write_encoder_type(self, value):
        ans = self.send_cmd(f"P34S{value:d}")
        if ans != self.__NACK:
            self.__Encoder = EncoderType(value)

    def read_encoder_resolution(self):
        return int(self.send_cmd("P35R"))

    def write_encoder_resolution(self, value):
        self.send_cmd(f"P35S{value:d}")

    def read_encoder_conversion(self):
        return 1 / float(self.send_cmd("P39R"))

    def write_encoder_conversion(self, value):
        self.send_cmd(f"P39S{1 / value}")

    def read_movement_unit(self):
        res = int(self.send_cmd("P02R"))
        if res == 1:
            self.__Unit = MovementUnit.step
        elif res == 2:
            self.__Unit = MovementUnit.mm
        elif res == 3:
            self.__Unit = MovementUnit.inch
        elif res == 4:
            self.__Unit = MovementUnit.degree
        return self.__Unit

    def write_movement_unit(self, value):
        self.send_cmd("P02S{:d}".format(int(value+1)))
        self.read_movement_unit()
        self.set_display_unit()

    # internal methods
    def set_display_unit(self):
        attributes = [b"position", b"sw_limit_minus", b"sw_limit_plus"]
        for attr in attributes:
            ac3 = self.get_attribute_config_3(attr)
            ac3[0].unit = self.__Unit.name.encode("utf-8")
            if (1/self.__Steps_Per_Unit % 1) == 0.0:
                ac3[0].format = b"%8d"
            else:
                ac3[0].format = b"%8.3f"
            self.set_attribute_config_3(ac3)

    def _send_cmd(self, cmd):
        # add module address to beginning of command
        cmd = str(self.Address) + cmd
        res = self.ctrl.write_read(cmd)
        if res == self.__NACK:
            self.set_state(DevState.FAULT)
            self.warn_stream("command not acknowledged from controller "
                             "-> Fault State")
        return res

    # commands
    @command(dtype_in=str, dtype_out=str, doc_in="enter a command", doc_out="the response")
    def send_cmd(self, cmd):
        # add axis name (X, Y) to beginning of command
        return self._send_cmd(str(self.__Axis_Name) + cmd)

    @command(dtype_out=str, doc_out="the firmware version")
    def read_firmware_version(self):
        version = self._send_cmd("IVR")
        return version

    @command(dtype_in=float, doc_in="position")
    def set_position(self, value):
        if self.__Inverted:
            value = -1*value
        self.send_cmd("P20S{:.4f}".format(value))
        self.send_cmd("P21S{:.4f}".format(value))

    # @command(dtype_in=float, doc_in="position")
    # def set_encoder_position(self, value):
    #     if self.__Inverted:
    #         value = -value
    #     self.send_cmd(f"P21S{value:.4f}")

    @command
    def jog_plus(self):
        if self.__Inverted:
            self.send_cmd("L-")
        else:
            self.send_cmd("L+")
        self.set_state(DevState.MOVING)

    @command
    def jog_minus(self):
        if self.__Inverted:
            self.send_cmd("L+")
        else:
            self.send_cmd("L-")
        self.set_state(DevState.MOVING)

    @command
    def homing_plus(self):
        flag_dir = '-' if self.__Inverted else '+'
        flag_enc = '^I' if self.__Encoder == EncoderType.incremental else ''
        self.send_cmd(f"0{flag_dir}{flag_enc}")
        self.set_state(DevState.MOVING)

    @command
    def homing_minus(self):
        flag_dir = '+' if self.__Inverted else '-'
        flag_enc = '^I' if self.__Encoder == EncoderType.incremental else ''
        self.send_cmd(f"0{flag_dir}{flag_enc}")
        self.set_state(DevState.MOVING)

    @command
    def stop(self):
        self.send_cmd("S")
        self.set_state(DevState.ON)

    @command
    def abort(self):
        self.send_cmd("SN")
        self.set_state(DevState.ON)

    @command(dtype_in=str)
    def set_alias(self, name):
        self.Alias = name
        self.db.put_device_property(self.get_name(), {"Alias": name})

    @command(dtype_out=str)
    def write_to_eeprom(self):
        self._send_cmd("SA")
        self.info_stream("parameters written to EEPROM")
        return "parameters written to EEPROM"

    @command(dtype_out=str)
    def dump_config(self):
        parameters = range(1, 50)
        res = ""
        for par in parameters:
            cmd = "P{:02d}R".format(par)
            res = res + "P{:02d}: {:s}\n".format(par, str(self.send_cmd(cmd)))
        return res


if __name__ == "__main__":
    PhytronMCC2Axis.run_server()
