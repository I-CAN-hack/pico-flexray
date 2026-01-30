#!/usr/bin/env python3

import sys

from opendbc.car.structs import CarParams
from panda import Panda



if __name__ == "__main__":
    pandas = Panda.list()

    for p in pandas:
        if p.startswith('picoflex'):
            continue

        panda = Panda(p)
        
        if sys.argv[1] == "1":
            # Open relay
            print("Opening relay")
            panda.set_safety_mode(CarParams.SafetyModel.allOutput)
        else:
            # Close relay
            print("Closing relay")
            panda.set_safety_mode(CarParams.SafetyModel.elm327)
