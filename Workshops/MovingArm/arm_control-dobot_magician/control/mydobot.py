import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pydobotplus'))

from pydobotplus import dobotplus as dobot

class MyDobot(dobot.Dobot):
    def jump_to(self, x, y, z, r, height=50, wait=False):
        print(f"Jumping to ({x}, {y}, {z}, {r}) with height {height}")
        current_pose = self.get_pose().position
        self._set_ptp_cmd(current_pose.x, current_pose.y, current_pose.z + height, current_pose.r, mode=dobot.MODE_PTP.MOVL_XYZ, wait=True)
        self._set_ptp_cmd(x, y, z + height, r, mode=dobot.MODE_PTP.MOVL_XYZ, wait=True)
        self._set_ptp_cmd(x, y, z, r, mode=dobot.MODE_PTP.MOVL_XYZ, wait=wait)

    def suck(self, enable, wait_time=0):
        self._set_end_effector_suction_cup(enable)
        time.sleep(wait_time)    
        
    def grip(self, enable, wait_time=0):
        self._set_end_effector_gripper(enable)
        time.sleep(wait_time)

