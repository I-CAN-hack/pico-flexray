#!/usr/bin/env python3
import os
import struct
import sys
import time

import hid
import numpy as np
from opendbc.can.packer import CANPacker

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
READ_SIZE = 65536
USB_READ_TIMEOUT_MS = 50
MIN_BODY_LEN = 11
TARGET_FRAME_ID = 0x44


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


def parse_varlen_records(buffer: bytes):
    frames = []
    i = 0
    buflen = len(buffer)

    while i + 2 <= buflen:
        body_len = buffer[i] | (buffer[i + 1] << 8)
        if body_len < MIN_BODY_LEN:
            i += 1
            continue
        if i + 2 + body_len > buflen:
            break

        header = buffer[i + 3 : i + 8]
        payload_len_words = (header[2] >> 1) & 0x7F
        payload_bytes = payload_len_words * 2
        if 5 + payload_bytes + 3 != body_len - 1:
            i += 1
            continue

        frames.append(
            {
                "frame_id": ((header[0] & 0x07) << 8) | header[1],
                "cycle_count": header[4] & 0x3F,
            }
        )
        i += 2 + body_len

    return i, frames


def read_next_odd_cycle(
    dev, data_buffer: bytes, target_frame_id: int, last_odd_cycle: int | None
):
    while True:
        try:
            data = dev.read(EP_VENDOR_IN, READ_SIZE, timeout=USB_READ_TIMEOUT_MS)  # type: ignore[arg-type]
        except usb.core.USBTimeoutError:
            return data_buffer, None

        if not data:
            return data_buffer, None

        data_buffer += bytes(data)
        consumed, frames = parse_varlen_records(data_buffer)
        if consumed > 0:
            data_buffer = data_buffer[consumed:]

        for frame in frames:
            cycle_count = frame["cycle_count"]
            if frame["frame_id"] != target_frame_id:
                continue
            if (cycle_count & 1) == 0:
                continue
            if cycle_count == last_odd_cycle:
                continue
            return data_buffer, cycle_count


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


def build_frame(cycle_count: int, counter: int, torque: float) -> bytes:
    values = {
        "CYCLE_COUNT": cycle_count,
        "TORQUE": torque,
        "COUNTER_1": counter,
        "COUNTER_2": counter,
    }
    data = pack_acc_payload(values)[1]

    crc_1 = crc8(data[2:9], 0xA4)
    crc_2 = crc8(data[10:17], 0xDC)

    # Pack again with checksum filled in
    values = {
        "CYCLE_COUNT": cycle_count,
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

    print(build_frame(0, 0, 0.0).hex())
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

    enabled = False
    prev_button = False

    angle_cmd = 0
    rx_buffer = b""
    last_odd_cycle = None
    skipped_cycles = 0

    while True:
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

            angle_cmd = norm * -2

        try:
            rx_buffer, odd_cycle = read_next_odd_cycle(
                dev, rx_buffer, TARGET_FRAME_ID, last_odd_cycle
            )
        except usb.core.USBError as e:
            print(f"USB read failed: {e}", file=sys.stderr)
            time.sleep(0.2)
            dev = find_device()
            if dev is None:
                print("Reconnection failed. Exiting.", file=sys.stderr)
                break
            rx_buffer = b""
            last_odd_cycle = None
            continue

        if odd_cycle is None:
            continue

        if last_odd_cycle is not None:
            expected_odd_cycle = (last_odd_cycle + 2) & 0x3F
            if odd_cycle != expected_odd_cycle:
                skipped_cycles += (((odd_cycle - last_odd_cycle) & 0x3F) // 2) - 1
        last_odd_cycle = odd_cycle
        inject_cycle = (odd_cycle + 1) & 0x3F
        payload = build_frame(inject_cycle, send_count & 0xF, angle_cmd)
        buf = build_override_payload(TARGET_FRAME_ID, 0, payload)
        print(
            f"Seen odd cycle {odd_cycle}, queueing even cycle {inject_cycle}: "
            f"Skipped cycles {skipped_cycles} "
            f"{buf.hex()} Torque: {angle_cmd} Enabled: {enabled}"
        )

        if enabled:
            try:
                dev.write(EP_VENDOR_OUT, buf, timeout=1000)
                send_count += 1
            except usb.core.USBError as e:
                print(f"USB write failed: {e}", file=sys.stderr)
                # Try to reconnect
                time.sleep(0.2)
                dev = find_device()
                if dev is None:
                    print("Reconnection failed. Exiting.", file=sys.stderr)
                    break
                rx_buffer = b""
                last_odd_cycle = None

    return 0


if __name__ == "__main__":
    sys.exit(main())
