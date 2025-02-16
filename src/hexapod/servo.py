import os
import sys
sys.path.append('../')

from common.pca9685 import PCA9685


class Servo(object):
    def __init__(self, left_pca = 0x40, right_pca = 0x41, pulse_min = 500, pulse_max = 2500, freq = 60, debug=False):
        self.offset = [[0,0,0]] * 6 # make an array for offsets of lets
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

    def set_angle(self, leg_index, servo_index, kin_angle):
        # left rear -> left front, right front -> right rear. (clockwise)

        if leg_index == 0:
            index = 0 + servo_index
            pca = self.pca_left
        elif leg_index == 1:
            index = 3 + servo_index
            pca = self.pca_left
        elif leg_index == 2:
            index = 6 + servo_index
            pca = self.pca_left
        elif leg_index == 3:
            index = 6 + servo_index
            #pca = self.right_pca
        elif leg_index == 4:
            index = 3 + servo_index
            #pca = self.right_pca
        elif leg_index == 5:
            index = 0 + servo_index
            #pca = self.right_pca
        else:
            raise ValueError
        
        leg_index = 0
        servo_index = 0

        kin_angle_corr = kin_angle + self.offset[leg_index][servo_index]
        inverse = -1 if servo_index == 1 else 1
        pulse = self.angle2pulse(kin_angle_corr, inverse)
        pca.setServoPulse(index, pulse)

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