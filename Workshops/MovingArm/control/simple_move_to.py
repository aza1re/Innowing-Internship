import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pydobotplus'))

from pydobotplus import dobotplus as dobot

# Example usage of the pydobot library
device = dobot.Dobot(port="COM7")
#device.home()
device.move_to(250, 0, 50, 0)
device.close()
