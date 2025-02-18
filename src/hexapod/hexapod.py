import time
import numpy as np
from threading import Lock
from leg import Leg
from servo import Servo


PI = np.pi
DEGTORAD = np.radians(1)
RADTODEG = np.degrees(1)
MAXITER = 250


class Hexapod:
    def __init__(self, leg_configs):
        '''
            Initialize variables/controllers
            Body-Level kinematics (gait, stability, coordination)
            modes: initialize, stand, walk, rotate, shutdown, calibration
            gaits: tripod, wave, ripple

            leftRear        Left Front
            [0]     [1]     [2]
                \    |    /
                ----------
                ----------
               /     |    \
            [5]     [4]      [3]
            rightRear       Right Front

            hook up legs 5 & 0 at pins 0 on the servoboard.
        '''
        self.max_speed = 1.0
        self._legs = {}
        
        for leg_name, config, in leg_configs.items():
            servos = [Servo(index) for index in config["servos"]]
            offsets = config["offsets"]
        
        self._legs[leg_name, *servos, offsets]

    def set_legOffsets(self, leg):
        ''''''
        self._legs[leg].set_offsets = {
            "coxa" : [0,0,0],
            "femur" : [0,0,0],
            "tibia" : [0,0,0]
        }

    def move(self):
        '''
            Move leg based on inverse kinematic angles
        '''
        
        pass


    def stand(self):
        self.set_mode("stand")

    def set_mode(self, mode):
        self.mode = mode

    def set_servo_ranges(self, min_pulse, max_pulse):
        pass

    # Gaits are below
    def tripod_gait(self, steps):
        '''
            3 legs move at the same time
            ex (0,4,2) (1,3,5)
            "no delay"
        '''
        pass

    def triple_gait(self,steps):
        '''
            3 legs at the same time
            1/6 phase offset
            phase_delay = speed/6
            sleep(phase_delay)
        '''
        pass
    def wave_gait(self, steps):
        ''' One leg is lifted at a time 0,1,2,5,4,3'''
        pass

    def ripple_gait(self, steps):
        '''
            One leg at a time 0, 3, 1, 5, 2, 4
            happens at phases see image
            phase 1/6
        '''
        pass
    
    def quad_gait(self, steps):
        pass

    def tetra_gait(self, steps):
        pass


    def walk(self, gait="tripod", direction="", speed=1.0):
        if gait == "tripod":
            self.tripod_gait(direction, speed)
        elif gait == "wave": #also known as ripple
            self.wave_gait(direction, speed)
        elif gait == "ripple":
            self.ripple_gait(direction, speed)
        elif gait == "quadrupedal":
            self.quad_gait(speed)
        elif gait == "tetrapod":
            self.tetra_gait(speed)
        else:
            pass