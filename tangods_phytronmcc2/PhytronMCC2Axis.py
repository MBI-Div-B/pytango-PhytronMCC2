#!/usr/bin/python3 -u
# coding: utf8
# PhytronMCC2Axis

from tango import Database, DevFailed, AttrWriteType, DevState, DeviceProxy, DispLevel
from tango.server import device_property
from tango.server import Device, attribute, command
import time


_MOVEMENT_UNITS = [
    "steps",
    "mm",
    "inch",
    "degree"
]

_PHY_AXIS_STATUS_CODES = [
    "Power stage error",  # 0
    "Power stage under voltage",  # 1
    "Power stage overtemperature",  # 2
    "Power stage is actived",  # 3
    "limit- is activated (emergency stop)",  # 4
    "limit+ is activated",  # 5
    "Step failure",  # 6
    "Encoder error",  # 7
    "Motor stands still",  # 8
    "Reference point is driven and OK",  # 9
]

_ALL_USED_PARAMETERS = [
    1, 2, 3, 8, 14, 15, 20, 25, 27, 40, 41, 45
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
        default_value=0.5,
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

    last_position = attribute(
        dtype="float",
        format="%8.3f",
        label="last position",
        unit="steps",
        memorized=True,
        hw_memorized=True,
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
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
        dtype="DevEnum",
        enum_labels=["NCC", "NOC"],
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
        dtype="DevEnum",
        enum_labels=[
            "1/1", # 1
            "1/2", # 2
            "1/4", # 4
            "1/8", # 8
            "1/10", # 10
            "1/16", # 16
            "1/128", # 128
            "1/256", # 256
            ],
        label="step resolution",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT
    )

    backlash_compensation = attribute(
        dtype="float",
        label="backlash compensation",
        unit="steps",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc=(
            "positive values result in compensation\n"
            "only for negative moves and vice versa"
        )
    )

    type_of_movement = attribute(
        dtype="DevEnum",
        enum_labels=["rotational", "linear"],
        label="type of movement",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc=(
            "0 = rotation; limit switches are ignored.\n"
            "1 = linear; limit switches are monitored.\n",
        )
    )

    movement_unit = attribute(
        dtype="DevEnum",
        enum_labels=_MOVEMENT_UNITS,
        label="unit",
        access=AttrWriteType.READ_WRITE,
        display_level=DispLevel.EXPERT,
        doc="Allowed unit values are steps, mm, inch, degree",
    )

    # private class properties
    __NACK = chr(0x15)  # command failed

    # decorators
    def update_parameters(parameter=-1):
        """update_parameters

        decorator for setter-methods of attributes in order to update
        the values of all parameters/attributes of the DS.
        """
        def wrap(func):
            def inner(self, value):
                func(self, value)
                self.read_all_parameters(parameter)
            return inner
        return wrap

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

        self._inverted = False
        self._last_status_query = 0
        self._all_parameters = {}

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

    def always_executed_hook(self):
        # axis state query takes 10 ms (per axis!) on phymotion over TCP
        # -> limit max. query rate to 5 Hz
        now = time.time()
        if now - self._last_status_query > self.TimeOut:
            position = self.send_cmd("P20R")
            # set current position
            self._all_parameters['P20R'] = position
            self.debug_stream(f"position: {position}")
            self._last_status_query = now
            status_str = self._send_cmd("SE")
            self.debug_stream(f"status: {status_str}")

            status_num = int(status_str, base=16)
            if self.Axis == 0:
                self._statusbits = [status_num >> i & 1 for i in range(16, 26)]
            else:
                self._statusbits = [status_num >> i & 1 for i in range(10)]

            status_list = []
            for n, bit_value in enumerate(self._statusbits):
                if bit_value:
                    if (n == 4) and self._inverted:
                        status_list.append(_PHY_AXIS_STATUS_CODES[n+1])
                    elif (n == 5) and self._inverted:
                        status_list.append(_PHY_AXIS_STATUS_CODES[n-1])
                    else:
                        status_list.append(_PHY_AXIS_STATUS_CODES[n])
            self.set_status("\n".join(status_list))

            if self._statusbits[8]:
                self.set_state(DevState.ON)
                if any([self._statusbits[n] for n in [1, 2]]):
                    self.set_state(DevState.ALARM)                
                elif (
                    any([self._statusbits[n] for n in [4, 5]])
                    and int(self._all_parameters["P01R"]) > 0
                ):
                    # no alarm for rotational stages
                    self.set_state(DevState.ALARM)

                if any([self._statusbits[n] for n in [0, 6, 7]]):
                    self.set_state(DevState.FAULT)
            else:
                self.set_state(DevState.MOVING)

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

    @update_parameters(15)
    def write_acceleration(self, value):
        self.send_cmd("P15S{:d}".format(value))

    def read_velocity(self):
        return int(self._all_parameters["P14R"])

    @update_parameters(14)
    def write_velocity(self, value):
        self.send_cmd("P14S{:d}".format(value))

    def read_homing_velocity(self):
        return int(self._all_parameters["P08R"])

    @update_parameters(8)
    def write_homing_velocity(self, value):
        self.send_cmd("P08S{:d}".format(value))

    def read_run_current(self):
        return float(self._all_parameters["P41R"]) / 10

    @update_parameters(41)
    def write_run_current(self, value):
        value = int(value * 10)
        if value not in range(0, 26):
            return "input not in range 0..25"
        self.send_cmd("P41S{:d}".format(value))

    def read_hold_current(self):
        return float(self._all_parameters["P40R"]) / 10

    @update_parameters(40)
    def write_hold_current(self, value):
        value = int(value * 10)
        if value not in range(0, 26):
            return "input not in range 0..25"
        self.send_cmd("P40S{:d}".format(value))

    def read_initiator_type(self):
        return int(self._all_parameters["P27R"])

    @update_parameters(27)
    def write_initiator_type(self, value):
        self.send_cmd("P27S{:d}".format(int(value)))

    def read_steps_per_unit(self):
        # inverse of spindle pitch (see manual page 50)
        return 1 / float(self._all_parameters["P03R"])

    @update_parameters(3)
    def write_steps_per_unit(self, value):
        # inverse of spindle pitch (see manual page 50)
        self.send_cmd("P03S{:10.8f}".format(1 / value))
        self.set_display_unit(steps_per_unit=value)

    def read_step_resolution(self):
        res = int(self._all_parameters["P45R"])
        if res == 1:
            value = 0
        elif res == 2:
            value = 1
        elif res == 4:
            value = 2
        elif res == 8:
            value = 3
        elif res == 10:
            value = 4
        elif res == 16:
            value = 5
        elif res == 128:
            value = 6
        elif res == 256:
            value = 7
        return value

    @update_parameters(45)
    def write_step_resolution(self, value):
        if value == 0:
            res = 1
        elif value == 1:
            res = 2
        elif value == 2:
            res = 4
        elif value == 3:
            res = 8
        elif value == 4:
            res = 10
        elif value == 5:
            res = 16
        elif value == 6:
            res = 128
        elif value == 7:
            res = 256

        if res not in [1, 2, 4, 8, 10, 16, 128, 256]:
            return "input not in [1, 2, 4, 8, 10, 16, 128, 256]"
        self.send_cmd("P45S{:d}".format(res))

    def read_backlash_compensation(self):
        # backlash compensation is internally stored in steps  
        ret = float(self._all_parameters["P25R"])*float(self._all_parameters["P03R"])
        if self._inverted:
            return 1 * ret
        else:
            return -1* ret

    @update_parameters(25)
    def write_backlash_compensation(self, value):
        if self._inverted:
            value = 1 * value
        else:
            value = -1 * value
        # backlash compensation is internally stored in steps
        self.send_cmd("P25S{:d}".format(int(value/float(self._all_parameters["P03R"]))))

    def read_type_of_movement(self):
        return int(self._all_parameters["P01R"])

    @update_parameters(1)
    def write_type_of_movement(self, value):
        self.send_cmd("P01S{:d}".format(int(value)))

    def read_movement_unit(self):
        return int(self._all_parameters["P02R"])-1

    @update_parameters(2)
    def write_movement_unit(self, value):
        self.send_cmd("P02S{:d}".format(int(value)+1))
        self.set_display_unit(unit=_MOVEMENT_UNITS[value])

    # internal methods
    def set_display_unit(self, unit="", steps_per_unit=0):
        attributes = [
            b"position",
            b"last_position",
            b"backlash_compensation"
            ]
        for attr in attributes:
            ac3 = self.get_attribute_config_3(attr)
            if len(unit) > 0:
                ac3[0].unit = unit.encode("utf-8")
            if steps_per_unit > 0:
                if (1 / steps_per_unit % 1) == 0.0:
                    ac3[0].format = "%8d"
                else:
                    ac3[0].format = "%8.3f"
            self.set_attribute_config_3(ac3)

    def _send_cmd(self, cmd_str):
        # add the address number in front of command
        cmd = str(self.Address) + cmd_str
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

    @command()
    def restore_position(self):
        self.set_position(self._last_position)

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
    def read_all_parameters(self, parameter=-1):
        # generate list of commands
        if parameter > 0:
            parameters = [parameter]
        else:
            parameters = _ALL_USED_PARAMETERS
        
        for par in parameters:
            cmd_str = "P{:02d}R".format(par)
            self._all_parameters[cmd_str] = self.send_cmd(cmd_str)

    @command(dtype_out=str)
    def dump_all_parameters(self):
        self.read_all_parameters()
        res = ""
        for par in _ALL_USED_PARAMETERS:
            cmd = "P{:02d}R".format(par)
            res = res + "P{:02d}: {:s}\n".format(par, str(self._all_parameters[cmd]))
        return res


if __name__ == "__main__":
    PhytronMCC2Axis.run_server()
