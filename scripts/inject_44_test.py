#!/usr/bin/env python3
import os
import struct
import sys
import time

import hid
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser

from crc8 import crc8

try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore
except Exception:
    print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
    sys.exit(1)


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC

# TinyUSB vendor endpoints (see usb_descriptors.c)
EP_VENDOR_OUT = 0x03
EP_VENDOR_IN = 0x81

EXPO = 0.4


def find_device():
    devs = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID, find_all=True)

    for dev in devs:
        serial = usb.util.get_string(dev, dev.iSerialNumber)

        if not serial.startswith("picoflex"):
            continue

        try:
            if hasattr(dev, "set_configuration"):
                dev.set_configuration()  # type: ignore[attr-defined]
        except Exception:
            pass
        return dev

    return None


def build_override_payload(frame_id: int, base: int, data_bytes: bytes) -> bytes:
    if not (0 <= frame_id <= 0xFFFF):
        raise ValueError("frame_id out of range")
    if not (0 <= base <= 0xFF):
        raise ValueError("base out of range")
    if len(data_bytes) > 0xFFFF:
        raise ValueError("data too long")
    # op 0x90: [0x90][u16 id][u8 base][u16 len][len bytes]
    header = struct.pack("<BHBH", 0x90, frame_id, base, len(data_bytes))
    return header + data_bytes


# Initialize CANPacker with local DBC path
_DBC_PATH = os.path.join(os.path.dirname(__file__), "..", "dbc", "bmw_sp2018.dbc")
_PACKER = CANPacker(_DBC_PATH)
_DBC = _PACKER.dbc


STEER_REQUEST_DEFAULTS = {
    "CYCLE_COUNT": 0,
    "CHECKSUM_1": 0,
    "COUNTER_1": 0,
    "SET_ME_0xF_1": 0xF,
    "SET_ME_0xD0": 0xD0,
    "SET_ME_0xE7": 0xE7,
    "NEW_SIGNAL_2": 0,  # zero when active
    "NEW_SIGNAL_1": 0xFE,
    "NEW_SIGNAL_7": 0,
    "TORQUE_MODE": 2,
    "TORQUE_REQUEST_1": 1,
    "CHECKSUM_2": 0,
    "COUNTER_2": 0,
    "SET_ME_0xF_2": 0xF,
    "TORQUE": 0.0,
    "TORQUE_MODE_CPY": 2,
    "SET_ME_0xFF_1": 0xFF,
    "SET_ME_0xFF_2": 0xFF,
    "SET_ME_0xF_3": 0xF,
    "TORQUE_REQUEST_2": 2,
    "LKA_STATE": 0x7,  # Ramp from 0-14 on engage/disengage, 5-9 otherwise
    "TORQUE_REQUEST_3": 1,  # Only 1 while not ramping up/down and active
}


def pack_acc_payload(values: dict):
    """Pack the entire ACC payload bytes (17 bytes) using CANPacker.

    Note: crc1/cnt1 are not auto-computed for this custom DBC; set them explicitly if needed.
    """
    merged = dict(STEER_REQUEST_DEFAULTS)
    merged.update(values)
    msg = _PACKER.make_can_msg("EPS_CONTROL", 0, merged)
    return msg


def build_frame(counter: int, torque: float) -> bytes:
    values = {"TORQUE": torque, "COUNTER_1": counter, "COUNTER_2": counter}
    data = pack_acc_payload(values)[1]

    crc_1 = crc8(data[2:9], 0xA4)
    crc_2 = crc8(data[10:17], 0xDC)

    # Pack again with checksum filled in
    values = {
        "TORQUE": torque,
        "COUNTER_1": counter,
        "COUNTER_2": counter,
        "CHECKSUM_1": crc_1,
        "CHECKSUM_2": crc_2,
    }
    data = pack_acc_payload(values)[1]

    assert data[1] == crc8(data[2:9], 0xA4)  # Sanity check
    assert data[9] == crc8(data[10:17], 0xDC)  # Sanity check
    return data


def main() -> int:

    print(build_frame(0, 0.0).hex())
    dev = find_device()
    if dev is None:
        print(
            "Device not found. Is the Pico connected and running the app?",
            file=sys.stderr,
        )
        return 1

    gamepad = hid.device()
    gamepad.open(0x046D, 0xC215)
    gamepad.set_nonblocking(True)

    send_count = 0
    sends_in_second = 0

    its = 0

    enabled = False
    prev_button = False

    angle_cmd = 0
    angle_cmd_prev = 0

    while True:
        its += 1

        # Handle joystick
        report = gamepad.read(64)
        if report:
            button = report[4] == 1

            if button and not prev_button:
                enabled = False if enabled else True

            prev_button = button

            norm = np.interp(report[3], [0, 255], [-1, 1])
            norm = norm if abs(norm) > 0.03 else 0.0
            norm = EXPO * norm**3 + (1 - EXPO) * norm

            angle_cmd = norm * -1

        # rate limit
        # max_diff = 5
        # to_send = np.clip(
        #     angle_cmd, angle_cmd_prev - max_diff, angle_cmd_prev + max_diff
        # )
        # angle_cmd_prev = to_send
        #

        # Build ACC override payload via DBC packing slice
        payload = build_frame(its & 0xF, angle_cmd)
        buf = build_override_payload(0x44, 0, payload)
        print(f"Hex: {buf.hex()} Torque: {angle_cmd} Enabled: {enabled}")

        if enabled:
            try:
                dev.write(EP_VENDOR_OUT, buf, timeout=1000)
                send_count += 1
                sends_in_second += 1
            except usb.core.USBError as e:
                print(f"USB write failed: {e}", file=sys.stderr)
                # Try to reconnect
                time.sleep(0.2)
                dev = find_device()
                if dev is None:
                    print("Reconnection failed. Exiting.", file=sys.stderr)
                    break

        # pacing ~100 Hz
        time.sleep(0.01)

    return 0


if __name__ == "__main__":
    sys.exit(main())
