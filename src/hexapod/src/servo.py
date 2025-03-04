import os
import sys
sys.path.append('../')
import time
from common.pca9685 import PCA9685


class Servo:
    def __init__(self, index, offset:int, pca = 0x40, pulse_min = 500, pulse_max = 2500, freq = 50, debug=False):
        '''
            Servo Class
            input:
                index: Servo pin
                pca: PCA9685 Servoboard I2C address:
                pulse_min: min pulse of DS3225 Servo
                pulse_max: max pulse of DS3225 Servo
                freq: frequency to run the servo at (generally 50 hz or 60 hz)
                debug: Resets mode_01 in PCA9685
        '''
        if pca == 0x40:
            self.pca_index = 0
        elif pca == 0x41:
            self.pca_index = 1
            
        self.servo_index = index
        self.offset = offset # make an array for offsets of legs
        self.pca = PCA9685(pca, debug)
        
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        self.pulse_middle = (self.pulse_max + self.pulse_min)/2
        
        # set PWM
        self.pca.setPWMFreq(freq)

        # move servos to initial positions
        self.initialize()


    def initialize(self):
        # Based on number of servos 0-18+ set the correct servo index to be adjusted on the leg
        # ie [0, 1, 2] for each leg

        # Adjust the servo index within the PCA (loops between 0-8)
        servo_index = self.servo_index % 9

        kin_angle_corr = self.offset
        inverse = -1 if self.servo_index % 3 else 1
        pulse = self.angle2pulse(kin_angle_corr, inverse)
        self.pca.setServoPulse(servo_index, pulse)
        time.sleep(0.02)

    def angle2pulse(self, kin_angle, reverse,  min_angle=-90, max_angle=90):
        '''
        pulse = (max- min)/(max angle-min angle) * (angle - min angle) + min
        '''
        if reverse:
            min_pulse, max_pulse = self.pulse_max, self.pulse_min
        else:
            min_pulse, max_pulse = self.pulse_min, self.pulse_max

        return (((max_pulse - min_pulse) / (max_angle - min_angle)) * (kin_angle - min_angle) + min_pulse)

    
    def set_offset(self, offset):
        self.offset = offset

    def set_angle(self, kin_angle):
        # Based on number of servos 0-18+ set the correct servo index to be adjusted on the leg
        # ie [0, 1, 2] for each leg

        # Adjust the servo index within the PCA (loops between 0-8)
        servo_index = self.servo_index % 9

        kin_angle_corr = kin_angle + self.offset
        inverse = -1 if self.servo_index % 3 else 1
        pulse = self.angle2pulse(kin_angle_corr, inverse)
        self.pca.setServoPulse(servo_index, pulse)
        time.sleep(0.02)


if __name__ == '__main__':
    # See reset_servos to set standard positions
    legs = []
    servos = [Servo(0), Servo(1), Servo(2)]
    legs = [servos]

    for leg in range(len(legs)):
        servos[0].set_angle(0)
        time.sleep(0.02)

    time.sleep(1)

    for leg in range(len(legs)):
        servos[1].set_angle(-120)
        time.sleep(0.02)

    time.sleep(1)

    for leg in range(len(legs)):
        servos[2].set_angle(0)
        time.sleep(0.02)