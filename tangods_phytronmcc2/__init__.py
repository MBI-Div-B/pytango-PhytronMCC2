from .PhytronMCC2Axis import PhytronMCC2Axis
from .PhytronMCC2Ctrl import PhytronMCC2Ctrl


def main():
    import sys
    import tango.server

    args = ["PhytronMCC2"] + sys.argv[1:]
    tango.server.run((PhytronMCC2Ctrl, PhytronMCC2Axis), args=args)
