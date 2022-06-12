# This is a simple uploader which uses wired (USB) connection between computer and V5 brain.
# user need ensure no other programs use such connections, or program will fail.
import logging
from typing import Optional

import pros.serial.devices.vex as vex
from pros.cli.common import resolve_v5_port
from pros.serial.ports import DirectPort


def upload(slot: int, pname: str, bin_file: str, desc: Optional[str] = None) -> None:
    # according to upload.py, L106 and L143, we need use system port
    if not 1 <= slot <= 8:
        raise ValueError(f"Slot must be in 1 - 8: {slot}")

    slot = slot - 1  # indexed by 0 while V5 indexed by 1

    desc = desc or "Lazy team does not supply description."

    port, is_v5_user_joystick = resolve_v5_port(None, "system")
    ser = DirectPort(port)

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

    upload(6, "WiredTest", r"..\..\Play\InertialCoord\build\InertialCoord.bin")

    print("OK")
