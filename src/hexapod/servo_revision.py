import os
import sys
sys.path.append('../')

from common.pca9685 import PCA9685


class Servo:
    def __init__(self, index, left_pca = 0x40, right_pca = 0x41, pulse_min = 500, pulse_max = 2500, freq = 60, debug=False):
        self.servo_index = index
        self.offset = [0,0,0] # make an array for offsets of legs
        self.pca_left = PCA9685(left_pca, debug)
        #self.right_pca = PCA9685(right_pca, debug)
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        self.pulse_middle = (self.pulse_max + self.pulse_min)/2

        # set PWM
        self.pca_left.setPWMFreq(freq)
        #self.right_pca.setPWMFreq(freq)

    def angle2pulse(self, kin_angle, reverse):
        return ( (self.pulse_max + self.pulse_min) / 2 ) + reverse * kin_angle * ((self.pulse_max - self.pulse_min)/100)
    
    def set_offset(self, offset):
        self.offset = offset

    def set_angle(self, servo_index, kin_angle):
        if self.servo_index <= 8:
            pca = self.pca_left
        
        kin_angle_corr = kin_angle + self.offset
        inverse = -1 if servo_index == 1 else 1
        pulse = self.angle2pulse(kin_angle_corr, inverse)
        pca.setServoPulse(servo_index, pulse)

# Test servos by testing "legs" (Note only testing 1 leg)
'''
    To test more than 1 leg, leg_i must be changed
    for servos servo_j

'''
if __name__ == '__main__':
    from time import sleep

    servo = Servo()
    while True:
        for angle in range(-30, 50):
            for leg_i in range(1):
                for part_j in range(3):
                    servo.set_angle(leg_i, part_j, angle)
            sleep(0.02)

        for angle in range(50, -30, -1):
            for leg_i in range(1):
                for part_j in range(3):
                    servo.set_angle(leg_i, part_j, angle)
            sleep(0.02)