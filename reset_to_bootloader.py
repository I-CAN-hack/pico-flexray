#!/usr/bin/env python3

from panda import Panda

if __name__ == "__main__":
    pandas = Panda.list()

    for serial in pandas:
        if serial.startswith("picoflex"):
            print("Resetting panda with serial:", serial)
            panda = Panda(serial)
            panda._handle.controlWrite(Panda.REQUEST_OUT, 0xd1, 0, 0, b'')
