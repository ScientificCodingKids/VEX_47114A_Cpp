# This is a simple uploader which uses wired (USB) connection between computer and V5 brain.
# user need ensure no other programs use such connections, or program will fail.
import logging
import click
from typing import Optional

import pros.serial.devices.vex as vex
from pros.cli.common import resolve_v5_port
from pros.serial.ports import DirectPort
#
# USER000x.bmp: VEX logo
# USER001x.bmp: A cool looking X
# USER003x.bmp: A slice of pizza
# USER010x.bmp: Clawbot
# USER011x.bmp: Robot head
# USER012x.bmp: Power icon
# USER013x.bmp: Planet with two moons and a ring
# USER027x.bmp: Alien
# USER029x.bmp: UFO with alien in it
# USER901x.bmp: MATLAB logo
# USER902x.bmp: PROS logo
# USER903x.bmp: Robot Mesh Studio logo
# USER910x.bmp: Robot Mesh Studio logo (again)
# USER911x.bmp: Robot Mesh Studio C++ logo
# USER912x.bmp: Robot Mesh Studio Blockly logo
# USER913x.bmp: Robot Mesh Studio Flowol logo
# USER914x.bmp: Robot Mesh Studio JavaScript logo
# USER915x.bmp: Robot Mesh Studio Python logo
# USER920x.bmp: Default file logo
# USER921x.bmp: VexCode logo
# USER922x.bmp: VexCode Blocks logo
# USER923x.bmp: Default file logo
# USER924x.bmp: Default file logo
# USER925x.bmp: VexCode Python logo
# USER926x.bmp: VexCode C++ logo
# USER999x.bmp: Default file logo (A file with </> in it)


@click.command()
@click.option("--slot", default=1, help="V5 slot (1-8)")
@click.option("--pname", default="Unnamed", help="Program name")
@click.option("--bin_file", prompt="Path to bin file", help="bin file for project to upload")
@click.option("--icon", default=None, help="icon as provided by VEX")
def upload(slot: int, pname: str, bin_file: str, desc: Optional[str] = None, icon: Optional[str] = None) -> None:
    # according to upload.py, L106 and L143, we need use system port
    if not 1 <= slot <= 8:
        raise ValueError(f"Slot must be in 1 - 8: {slot}")

    icon = icon or "USER027x.bmp"  # default at alien
    slot = slot - 1  # indexed by 0 while V5 indexed by 1

    desc = desc or "Lazy team does not supply description."

    port, is_v5_user_joystick = resolve_v5_port(None, "system")
    ser = DirectPort(port)

    device = vex.V5Device(ser)

    kwargs = {
        "slot": slot,
        # https://www.vexforum.com/t/a-guide-to-changing-program-icons/78293
        "icon": icon,
        "IDE": "VSCode",
        "description": desc,
        "remote_name": pname  # Program name shown on V5 brain screen
    }

    with open(bin_file, mode='rb') as pf:
        device.write_program(pf, **kwargs)


if __name__ == "__main__":
    logging.RootLogger(logging.DEBUG)
    upload()
    #upload(7, "WiredTest", r"..\..\Play\InertialCoord\build\InertialCoord.bin")

    print("OK")
