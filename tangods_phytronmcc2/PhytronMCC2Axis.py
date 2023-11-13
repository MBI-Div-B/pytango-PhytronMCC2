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
    steps = 0
    mm = 1
    inch = 2
    degree = 3


class InitiatorType(IntEnum):
    NCC = 0
    NOC = 1


class StepResolution(IntEnum):
    step_1_1 = 1
    step_1_2 = 2
    step_1_4 = 4
    step_1_8 = 8
    step_1_10 = 10
    step_1_16 = 16
    step_1_128 = 128
    step_1_256 = 256


_PHY_AXIS_STATUS_CODES = [
    "Power stage error",  # 0
    "Power stage under voltage",  # 1
    "Power stage overtemperature",  # 2
    "Power stage is actived",  # 3
    "limit– is activated (emergency stop)",  # 4
    "limit+ is activated",  # 5
    "Step failure",  # 6
    "Encoder error",  # 7
    "Motor stands still",  # 8
    "Reference point is driven and OK",  # 9
]


class PhytronMCC2Axis(Device):
    # device properties
    CtrlDevice = device_property(
        dtype="str",
        default_value="domain/family/member"
    )

    Address = device_property(
        dtype="int16",
        default_value=0,
        doc="Module address (starts at 0)."
    )

    Axis = device_property(
        dtype="int16",
        default_value=0,
        doc="Axis number per module (0 or 1)."
    )

    TimeOut = device_property(
        dtype="float",
        default_value=0.2,
        doc=(
            "Timeout in seconds between status requests\n"
            "to reduce communication traffic."
        )
    )

    # device attributes

    position = attribute(
        dtype="float",
        format="%8.3f",
        label="position",
        unit="steps",
        access=AttrWriteType.READ_WRITE,
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
        dtype="StepResolution",
        label="step resolution",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT
    )

    backlash_compensation = attribute(
        dtype="int",
        label="backlash compensation",
        unit="steps",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
    )

    type_of_movement = attribute(
        dtype=MovementType,
        label="type of movement",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc=(
            "0 = rotation; limit switches are ignored.\n"
            "1 = linear; limit switches are monitored.\n",
        )
    )

    movement_unit = attribute(
        dtype=MovementUnit,
        label="unit",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="Allowed unit values are steps, mm, inch, degree",
    )

    # private class properties
    __NACK = chr(0x15)  # command failed
    _inverted = False
    _unit = MovementUnit.steps
    _steps_Per_Unit = 1.0
    _last_status_query = 0

    # decorators
    def update_parameters(func):
        """update_parameters

        decorator for setter-methods of attributes in order to update
        the values of all parameters/attributes of the DS.
        """
        def inner(self, value):
            func(self, value)
            self.read_all_parameters()
        return inner

    def init_device(self):
        super().init_device()
        self.info_stream("init_device()")

        if self.Axis == 0:
            self.__Axis_Name = "X"
        else:
            self.__Axis_Name = "Y"

        self.info_stream("module address: {:d}".format(self.Address))
        self.info_stream("module axis: {:d}".format(self.Axis))

        try:
            self.ctrl = DeviceProxy(self.CtrlDevice)
            self.info_stream("ctrl. device: {:s}".format(self.CtrlDevice))
        except DevFailed as df:
            self.error_stream("failed to create proxy to {:s}".format(df))
            self.set_state(DevState.FAULT)
            return

        # read all parameters
        self.read_all_parameters()

        # read memorized attributes from Database
        self.db = Database()
        try:
            attr = self.db.get_device_attribute_property(
                self.get_name(), ["inverted"]
            )
            if attr["inverted"]["__value"][0] == "true":
                self._inverted = True
            else:
                self._inverted = False
        except Exception:
            self._inverted = False
        self.set_state(DevState.ON)

    def delete_device(self):
        self.set_state(DevState.OFF)

    def _decode_status(self, status, ndigits):
        """Decode status number to bit list.

        Status codes are returned as decimal, but should be interpreted
        as hex representation of packed 4-bit nibbles. In other words, the
        decimal status code needs to be formatted as a hexadecimal number,
        where each hex digit represents 4 status bits.
        To get the correct bit status length, leading zeros are required.
        Hence the maximum number of digits must be provided as a parameter.

        Documentation:
        https://www.phytron.de/fileadmin/user_upload/produkte/endstufen_controller/pdf/phylogic-de.pdf
        page 41
        """
        hexstring = f"{int(status):0{ndigits}X}"
        statusbits = []
        for digit in hexstring[::-1]:
            digit_num = int(digit, base=16)
            statusbits += [(digit_num >> i) & 1 for i in range(4)]
        return statusbits

    def always_executed_hook(self):
        # axis state query takes 10 ms (per axis!) on phymotion over TCP
        # -> limit max. query rate to 5 Hz
        now = time.time()
        if now - self._last_status_query > self.TimeOut:
            status, position = self.send_cmd(["SE", "P20R"])
            self.debug_stream(f"position: {position}")
            self._last_status_query = now
            self._statusbits = self._decode_status(int(status), 7)
            self.debug_stream(f"status bits: {self._statusbits}")
            # set current position
            self._all_parameters['P20R'] = position

            status_list = []
            for n, bit_value in enumerate(self._statusbits):
                if bit_value:
                    if (n == 4 or n == 7) and self._inverted:
                        status_list.append(_PHY_AXIS_STATUS_CODES[n+1])
                    elif (n == 5 or n == 8) and self._inverted:
                        status_list.append(_PHY_AXIS_STATUS_CODES[n-1])
                    else:
                        status_list.append(_PHY_AXIS_STATUS_CODES[n])
            self.set_status("\n".join(status_list))

            if any([self._statusbits[n] for n in [12]]):
                # reset limit switch error on module
                self.reset_errors()

            if any([self._statusbits[n] for n in [3, 19]]):
                self.set_state(DevState.ON)

            if any([self._statusbits[n] for n in [0, 16, 21, 22, 23]]):
                self.set_state(DevState.MOVING)
            elif any([self._statusbits[n] for n in [4, 5, 6, 7, 8, 12]]) and int(self._all_parameters["P01R"]) > 0:
                # no alarm for rotational stages
                self.set_state(DevState.ALARM)
            elif any([self._statusbits[n] for n in [1, 11, 13, 14, 15]]):
                self.set_state(DevState.FAULT)

    # def always_executed_hook(self):
    #     answer = self._send_cmd("SE")
    #     if self.Axis == 0:
    #         if self._inverted:
    #             self.__HW_Limit_Minus = bool(int(answer[2]) & self.__LIM_PLUS)
    #             self.__HW_Limit_Plus = bool(int(answer[2]) & self.__LIM_MINUS)
    #         else:
    #             self.__HW_Limit_Minus = bool(int(answer[2]) & self.__LIM_MINUS)
    #             self.__HW_Limit_Plus = bool(int(answer[2]) & self.__LIM_PLUS)
    #         moving = not (bool(int(answer[1]) & 1))
    #     else:
    #         if self._inverted:
    #             self.__HW_Limit_Minus = bool(int(answer[6]) & self.__LIM_PLUS)
    #             self.__HW_Limit_Plus = bool(int(answer[6]) & self.__LIM_MINUS)
    #         else:
    #             self.__HW_Limit_Minus = bool(int(answer[6]) & self.__LIM_MINUS)
    #             self.__HW_Limit_Plus = bool(int(answer[6]) & self.__LIM_PLUS)
    #         moving = not (bool(int(answer[5]) & 1))
    #     self.debug_stream("HW limit-: {0}".format(self.__HW_Limit_Minus))
    #     self.debug_stream("HW limit+: {0}".format(self.__HW_Limit_Plus))
    #     if moving is False:
    #         self.set_status("Device in ON")
    #         self.set_state(DevState.ON)
    #         self.debug_stream("device is: ON")
    #     else:
    #         self.set_status("Device is MOVING")
    #         self.set_state(DevState.MOVING)
    #         self.debug_stream("device is: MOVING")

    # attribute read/write methods
    def read_position(self):
        ret = float(self._all_parameters['P20R'])
        if self._inverted:
            return -1 * ret
        else:
            return ret

    def write_position(self, value):
        memorize_value = value
        if self._inverted:
            value = -1 * value
        answer = self.send_cmd("A{:.10f}".format(value))
        if answer != self.__NACK:
            self.set_state(DevState.MOVING)
            DeviceProxy(self.get_name()).write_attribute(
                "last_position", memorize_value)

    def read_last_position(self):
        return self._last_position

    def write_last_position(self, value):
        self._last_position = value

    def read_inverted(self):
        return self._inverted

    def write_inverted(self, value):
        self._inverted = bool(value)

    def read_acceleration(self):
        return int(self._all_parameters["P15R"])

    @update_parameters
    def write_acceleration(self, value):
        self.send_cmd("P15S{:d}".format(value))

    def read_velocity(self):
        return int(self._all_parameters["P14R"])

    @update_parameters
    def write_velocity(self, value):
        self.send_cmd("P14S{:d}".format(value))

    def read_homing_velocity(self):
        return int(self._all_parameters["P08R"])

    @update_parameters
    def write_homing_velocity(self, value):
        self.send_cmd("P08S{:d}".format(value))

    def read_run_current(self):
        return float(self._all_parameters["P41R"]) / 10

    @update_parameters
    def write_run_current(self, value):
        value = int(value * 10)
        if value not in range(0, 26):
            return "input not in range 0..25"
        self.send_cmd("P41S{:d}".format(value))

    def read_hold_current(self):
        return float(self._all_parameters["P40R"]) / 10

    @update_parameters
    def write_hold_current(self, value):
        value = int(value * 10)
        if value not in range(0, 26):
            return "input not in range 0..25"
        self.send_cmd("P40S{:d}".format(value))

    def read_initiator_type(self):
        return InitiatorType(int(self._all_parameters["P27R"]))

    @update_parameters
    def write_initiator_type(self, value):
        self.send_cmd("P27S{:d}".format(int(value)))

    def read_steps_per_unit(self):
        # inverse of spindle pitch (see manual page 50)
        self._steps_Per_Unit = 1 / float(self._all_parameters["P03R"])
        return self._steps_Per_Unit

    @update_parameters
    def write_steps_per_unit(self, value):
        # inverse of spindle pitch (see manual page 50)
        self.send_cmd("P03S{:10.8f}".format(1 / value))
        # update display unit
        self.set_display_unit()

    def read_step_resolution(self):
        return int(self._all_parameters["P45R"])

    @update_parameters
    def write_step_resolution(self, value):
        if value not in [1, 2, 4, 8, 10, 16, 128, 256]:
            return "input not in [1, 2, 4, 8, 10, 16, 128, 256]"
        self.send_cmd("P45S{:d}".format(value))

    def read_backlash_compensation(self):
        ret = int(self._all_parameters["P25R"])
        if self._inverted:
            return -1 * ret
        else:
            return ret

    @update_parameters
    def write_backlash_compensation(self, value):
        if self._inverted:
            value = -1 * value
        self.send_cmd("P25S{:d}".format(int(value)))

    def read_type_of_movement(self):
        return MovementType(int(self._all_parameters["P01R"]))

    @update_parameters
    def write_type_of_movement(self, value):
        self.send_cmd("P01S{:d}".format(int(value)))

    def read_movement_unit(self):
        res = int(self._all_parameters["P02R"])
        if res == 1:
            self._unit = MovementUnit.steps
        elif res == 2:
            self._unit = MovementUnit.mm
        elif res == 3:
            self._unit = MovementUnit.inch
        elif res == 4:
            self._unit = MovementUnit.degree
        return self._unit

    @update_parameters
    def write_movement_unit(self, value):
        self.send_cmd("P02S{:d}".format(int(value + 1)))
        self.read_movement_unit()
        self.set_display_unit()

    # internal methods
    def set_display_unit(self):
        attributes = ["position", "last_position"]
        for attr in attributes:
            ac3 = self.get_attribute_config_3(attr)
            ac3[0].unit = self.__Unit.name.encode("utf-8")
            if (1 / self.__Steps_Per_Unit % 1) == 0.0:
                ac3[0].format = b"%8d"
            else:
                ac3[0].format = b"%8.3f"
            self.set_attribute_config_3(ac3)

    def _send_cmd(self, cmd_str):
        # add module address to beginning of command
        if isinstance(cmd_str, list):
            cmd = ""
            for sub_cmd in cmd_str:
                cmd = cmd + ' ' + '{:d}.1{:s}'.format(self.Axis, sub_cmd)
            res = self.ctrl.write_read(cmd).split(chr(6))
        else:
            cmd = str(self.Address) + cmd
            res = self.ctrl.write_read(cmd)
        if res == self.__NACK:
            self.set_state(DevState.FAULT)
            self.warn_stream(
                "command not acknowledged from controller " "-> Fault State"
            )
            return ""
        return res

    # commands
    @command(
        dtype_in=str, dtype_out=str,
        doc_in="enter a command",
        doc_out="the response"
    )
    def send_cmd(self, cmd):
        # add axis name (X, Y) to beginning of command
        return self._send_cmd(str(self.__Axis_Name) + cmd)

    @command(dtype_out=str, doc_out="the firmware version")
    def read_firmware_version(self):
        version = self._send_cmd("IVR")
        return version

    @command(dtype_in=float, doc_in="position")
    def set_position(self, value):
        if self._inverted:
            value = -1 * value
        self.send_cmd("P20S{:.4f}".format(value))

    @command
    def jog_plus(self):
        if self._inverted:
            self.send_cmd("L-")
        else:
            self.send_cmd("L+")
        self.set_state(DevState.MOVING)

    @command
    def jog_minus(self):
        if self._inverted:
            self.send_cmd("L+")
        else:
            self.send_cmd("L-")
        self.set_state(DevState.MOVING)

    @command
    def homing_plus(self):
        if self._inverted:
            self.send_cmd("0-")
        else:
            self.send_cmd("0+")
        self.set_state(DevState.MOVING)

    @command
    def homing_minus(self):
        if self._inverted:
            self.send_cmd("0+")
        else:
            self.send_cmd("0-")
        self.set_state(DevState.MOVING)

    @command
    def stop(self):
        self.send_cmd("S")
        self.set_state(DevState.ON)

    @command
    def abort(self):
        self.send_cmd("SN")
        self.set_state(DevState.ON)

    @command(dtype_out=str)
    def write_to_eeprom(self):
        self._send_cmd("SA")
        self.info_stream("parameters written to EEPROM")
        return "parameters written to EEPROM"

    @command()
    def read_all_parameters(self):
        self._all_parameters = {}
        # generate list of commands
        cmd_list = []
        for par in range(1, 49):
            cmd_str = "P{:02d}R".format(par)
            cmd_list.append(cmd_str)
        # query list of commands
        ret = self.send_cmd(cmd_list)
        # parse response
        for i, cmd_str in enumerate(cmd_list):
            self._all_parameters[cmd_str] = ret[i]

    @command(dtype_out=str)
    def dump_all_parameters(self):
        self.read_all_parameters()
        res = ""
        for par in range(1, 49):
            cmd = "P{:02d}R".format(par)
            res = res + "P{:02d}: {:s}\n".format(par, str(self._all_parameters[cmd]))
        return res


if __name__ == "__main__":
    PhytronMCC2Axis.run_server()
