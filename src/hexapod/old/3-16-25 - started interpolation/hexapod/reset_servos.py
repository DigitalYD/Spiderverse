from src.pca9685 import PCA9685
import time

## Reset all leg positions
# Base position is
'''
    Servo 0: 0
    Servo 1: -90
    Servo 2: 0

'''

class Servo(object):
    def __init__(self, index, pca = 0x40, pulse_min = 500, pulse_max = 2500, freq = 60, debug=False):
        self.servo_index = index
        if pca == 0x40:
            self.pca_index = 0
        elif pca == 0x41:
            self.pca_index = 1

        self.offset = [0,0,0] # make an array for offsets of legs
        self.pca = PCA9685(pca, debug)
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        self.pulse_middle = (self.pulse_max + self.pulse_min)/2

        # Set pwm frequency
        self.pca.setPWMFreq(freq)


    def angle2pulse(self, kin_angle, reverse,  min_angle=-90, max_angle=120):
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
        if self.servo_index % 3 == 0:
            servo_index = 0
        elif self.servo_index % 3 == 1:
            servo_index = 1
        elif self.servo_index % 3 == 2:
            servo_index = 2
        else:
            return "Invalid Index"
        
        kin_angle_corr = kin_angle + self.offset[servo_index]
        inverse = -1 if servo_index == 1 else 1
        pulse = self.angle2pulse(kin_angle_corr, inverse)
        self.pca.setServoPulse(servo_index, pulse)
        time.sleep(0.02)


if __name__ == '__main__':
    from time import sleep

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

    ## Further adjust for each individual leg