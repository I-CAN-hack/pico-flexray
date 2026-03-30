#!/usr/bin/env python3
import math
import os
import queue
import struct
import sys
import threading
import time

import hid
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser

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
_DBC_PATH = os.path.join(os.path.dirname(__file__), "..", "dbc", "lateral.dbc")
_PACKER = CANPacker(_DBC_PATH)
_DBC = _PACKER.dbc

# Expose full-field packing for ACC so you can tune every field
ACC_DEFAULTS = {
    "cycle_count": 0,
    "crc1": 0,
    "cnt1": 0,
    "always_0x9": 0x9,
    "steering_angle_req": 0.0,
    "steer_torque_req": 0.0,
    "TJA_ready": 0,
    "assist_mode": 0,
    # "wayback_en1_lane_keeping_trigger": 0x1F,  # Full byte, upper nibble 1 makes it go fast
    "wayback_en1_lane_keeping_trigger": 0x00,  # Full byte, upper nibble 1 makes it go fast
    "lane_keeping_triggered": 0,
    "like_assist_torque_reserve": 200,  # 160, 200, > faults
    # "constants": 0x03FF17FE,
    "SET_ME_0xFE": 0xFE,
    "SET_ME_0x17": 0x17,
    "SET_ME_0xFF": 0xFF,
    "SET_ME_0x03": 0x03,
    "wayback_en_2": 0,  # Not sure what this does
    "steering_engaged": 2,
    "maybe_assist_force_enhance": 250,  # >= 253 faults
    "maybe_assist_force_weaken": 250,
}

#          B8 61 FC 7F 02 01 00 AO FE 17 FF 23 A2 FA
#          96 64 1f 80 00 01 00 a0 fe 17 ff 23 a2 fa

# 01 48 90 a8 61 fe 7f 00 01 00 a0 fe 17 ff 23 a2 fa


def pack_acc_payload(values: dict):
    """Pack the entire ACC payload bytes (17 bytes) using CANPacker.

    Note: crc1/cnt1 are not auto-computed for this custom DBC; set them explicitly if needed.
    """
    merged = dict(ACC_DEFAULTS)
    merged.update(values)
    msg = _PACKER.make_can_msg("ACC", 0, merged)
    return msg


def crc8_checksum(data: bytes, init_value: int) -> int:
    """
    Compute CRC-8 using polynomial 0x1D (MSB-first), no final XOR.
    init_value is the initial CRC register value.
    """
    crc: int = init_value & 0xFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if (crc & 0x80) != 0:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(angle_deg: float) -> bytes:
    values = {"steering_angle_req": angle_deg}
    data = pack_acc_payload(values)[1]
    crc = crc8_checksum(data[1:], 0xF1)
    data = bytearray(data)
    data[0] = crc
    return data


def main() -> int:
    # Initial params
    mode = "A"  # A: angle-only, T: torque-only
    angle_amp = 90.0  # deg peak for triangle wave
    half_period_s = 1.0

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

            angle_cmd = norm * -360

        # Build ACC override payload via DBC packing slice
        max_diff = 5
        to_send = np.clip(
            angle_cmd, angle_cmd_prev - max_diff, angle_cmd_prev + max_diff
        )
        payload = build_frame(to_send)
        angle_cmd_prev = to_send
        buf = build_override_payload(0x48, 1, payload)
        print(f"Hex: {buf.hex()} Angle: {to_send} Enabled: {enabled}")

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

        # pacing ~50 Hz
        time.sleep(0.02)

    return 0


if __name__ == "__main__":
    sys.exit(main())
