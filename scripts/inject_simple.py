#!/usr/bin/env python3
import sys
import struct
import time
import math
import threading
import queue
import os
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker

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


def find_device():
    devs = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID, find_all=True)

    for dev in devs:
        serial = usb.util.get_string(dev, dev.iSerialNumber)

        if not serial.startswith('picoflex'):
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
    header = struct.pack('<BHBH', 0x90, frame_id, base, len(data_bytes))
    return header + data_bytes


def main() -> int:
    dev = find_device()
    if dev is None:
        print("Device not found. Is the Pico connected and running the app?", file=sys.stderr)
        return 1


    while True:
        # payload = bytes.fromhex("904800011100f60090e457fe7f000000a0fe17ff23a2fa")
        payload = bytes.fromhex("01 48 90 a8 61 fe 7f 00 01 00 a0 fe 17 ff 23 a2 fa")
        print(len(payload), payload.hex())
        buf = build_override_payload(0x48, 1, payload)
        dev.write(EP_VENDOR_OUT, buf, timeout=1000)

        # pacing ~50 Hz
        time.sleep(0.02)

    return 0


if __name__ == "__main__":
    sys.exit(main())


