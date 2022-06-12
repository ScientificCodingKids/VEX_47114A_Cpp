# This is a simple uploader which uses wired (USB) connection between computer and V5 brain.
# user need ensure no other programs use such connections, or program will fail.
import logging

import pros.serial.devices.vex as vex
from pros.cli.common import resolve_v5_port
from pros.serial.ports import DirectPort


def upload(slot: int, bin_file: str) -> None:
    # according to upload.py, L106 and L143, we need use system port
    if not 0 <= slot < 8:
        raise ValueError(f"Slot must be in 0-7: {slot}") # indexed by 0 while V5 indexed by 1

    port, is_v5_user_joystick = resolve_v5_port(None, "system")
    ser = DirectPort(port)  # fail to open port on new desktop

    device = vex.V5Device(ser)

    kwargs = {
        "slot": 6,
        #"icon": "pizza",  # known str constas are in upload.py, but none of them working yet (? shown)
        "remote_name": "ProUp"  # Program name shown on V5 brain screen
    }

    with open(bin_file, mode='rb') as pf:
        device.write_program(pf, **kwargs)


if __name__ == "__main__":
    logging.RootLogger(logging.DEBUG)

    upload(6, r"..\..\Play\InertialCoord\build\InertialCoord.bin")

    print("OK")
