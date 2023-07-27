from tango import DeviceProxy


class PhytronMCC2Configurator:
    default_config = {
        1: [0, "Type of movement"],
        2: [1, "Measuring units of movement"],
        3: [1, "Conversion factor for the thread"],
        4: [400, "Start/stop frequency"],
        7: [100000, "Emergency stop ramp"],
        8: [4000, "fmaxMØP (mechanical zero point) "],
        9: [4000, "Ramp MØP"],
        10: [400, "fminMØP Run frequency for leaving the limit switch range"],
        11: [0, "MØP offset for limit switch direction +"],
        12: [0, "MØP offset for limit switch direction –"],
        13: [20, "Recovery time MØP"],
        14: [4000, "fmaxRun frequency during program operation"],
        15: [4000, "Ramp for run frequency (P14"],
        16: [20, "Recovery time position"],
        17: [0, "Boost (defined in P42)"],
        19: [0, "Electronical zero counter"],
        20: [0, "Mechanical zero counter"],
        21: [0, "Absolute counter"],
        22: [0, "Encoder counter"],
        23: [0, "Axial limitation  pos. direction +"],
        24: [0, "Axial limitation  neg. direction -"],
        25: [0, "Compensation for play"],
        27: [0, "Initiator type"],
        34: [0, "Encoder type"],
        35: [10, "Encoder resolution for SSI encode"],
        36: [0, "Encoder function"],
        38: [0, "Encoder preferential direction of rotation"],
        39: [0, "Encoder conversion factor"],
        40: [2, "Stop current"],
        41: [6, "Run current"],
        42: [10, "Boost current"],
        43: [20, "Current delay time in msec"],
        45: [4, "Step resolution 1 to 256"],
        46: [1, "Current Shaping (CS)"],
        47: [1, "Chopper frequency"],
    }

    def __init__(self, device):
        self.proxy = DeviceProxy(device)
        self.current_config = {}

    def read_current_config(self):
        self.current_config = {}
        import io

        raw_conf = self.proxy.dump_config()
        buf = io.StringIO(raw_conf)
        for line in buf:
            [param, value] = line.strip().split(":")
            self.current_config[int(param.replace("P", ""))] = float(value.strip())

    def compare_configs(self, read_current_config=True):
        if read_current_config:
            self.read_current_config()
        print("Param.\tdefault\t  current\tcompare\tdescription")
        print("======================================================================")
        for param, val_list in self.default_config.items():
            curr_value = self.current_config[param]
            print(
                "P{:02d}: {:10.2f}\t{:10.2f}"
                "\t{:s}\t{:s}".format(
                    param,
                    val_list[0],
                    curr_value,
                    "same" if curr_value == val_list[0] else "changed",
                    val_list[1],
                )
            )

    def reset_to_default(self, save_to_eeprom=False):
        print("Setting the following default parameters:")
        print("=========================================")
        for param, val_list in self.default_config.items():
            cmd = "P{:02d}S{:f}".format(param, val_list[0])
            print(
                "P{:02d}: {:10.2f}\t{:s}".format(
                    param,
                    val_list[0],
                    val_list[1],
                )
            )
            _ = self.proxy.send_cmd(cmd)
        print("=========================================")
        if save_to_eeprom:
            self.proxy.write_to_eeprom()
            print("Config permanently saved to EEPROM")
        else:
            print("Config NOT permanently saved to EEPROM")
            print("Use save_to_eeprom=True as parameter")
