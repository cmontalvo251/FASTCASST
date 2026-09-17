import sys
import time
import numpy as np

class MODEL():
    def __init__(self):
        self.timestep = 0.1
        print('Running SIMONLY mode')

    def loop(self):
        print('Running 1 timestep')