# This is a simple uploader which uses wireless (bluetooth) connection between computer and V5 remote.
# https://pros.cs.purdue.edu/v5/tutorials/topical/wireless-upload.html

import logging
from typing import Optional

import pros.serial.devices.vex as vex
from pros.serial.ports.v5_wireless_port import V5WirelessPort


def upload(slot: int, pname: str, bin_file: str, desc: Optional[str] = None) -> None:
    # according to upload.py, L106 and L143, we need use system port
    if not 1 <= slot <= 8:
        raise ValueError(f"Slot must be in 1 - 8: {slot}")

    slot = slot - 1  # indexed by 0 while V5 indexed by 1

    desc = desc or "Lazy team does not supply description."

    ports = [p for p in vex.find_v5_ports("user") if "Controller" in p.description]

    if len(ports) != 1:
        raise ValueError(f"Find non-unique port")

    port = ports[0].device
    ser = V5WirelessPort(port)

    device = vex.V5Device(ser)

    kwargs = {
        "slot": slot,
        # https://www.vexforum.com/t/a-guide-to-changing-program-icons/78293
        "icon": "USER901x.bmp",
        "IDE": "VSCode",
        "description": desc,
        "remote_name": pname  # Program name shown on V5 brain screen
    }

    with open(bin_file, mode='rb') as pf:
        device.write_program(pf, **kwargs)


if __name__ == "__main__":
    logging.RootLogger(logging.DEBUG)

    upload(5, "WirelessTest", r"d:\dev\robotics\VEX_47114A_Cpp\Play\InertialCoord\build\InertialCoord.bin")

    print("OK")
