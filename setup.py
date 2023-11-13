from setuptools import setup

setup(
    name="tangods_phytronmcc2",
    version="0.0.1",
    description="Tango device server written in PyTango for a Phytron MCC2 stepper motor controller using the RS485 serial interface",
    author="Daniel Schick",
    author_email="schick@mbi-berlin.de",
    python_requires=">=3.6",
    entry_points={"console_scripts": ["PhytronMCC2 = tangods_phytronmcc2:main"]},
    license="MIT",
    packages=["tangods_phytronmcc2"],
    install_requires=[
        "pytango",
        "pyserial",
    ],
    url="https://github.com/MBI-Div-b/pytango-PhytronMCC2",
    keywords=[
        "tango device",
        "tango",
        "pytango",
        "phytron",
        "MCC2",
        "RS485",
    ],
)
