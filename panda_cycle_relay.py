#!/usr/bin/env python3

import time

from opendbc.car.structs import CarParams
from panda import Panda



if __name__ == "__main__":
    print(Panda.list())
    panda = Panda("0f002d000c51303136383232")

    while True:
        # Open relay
        print("Opening relay")
        panda.set_safety_mode(CarParams.SafetyModel.allOutput)
        time.sleep(0.5)

        # Close relay
        print("Closing relay")
        panda.set_safety_mode(CarParams.SafetyModel.elm327)
        time.sleep(0.5)
