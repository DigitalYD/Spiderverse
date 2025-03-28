
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16, address=0x40, frequency=60)


## Set servos to default 
kit.servo[0].set_pulse_width_range(500,2500)
kit.servo[0].angle = 90
time.sleep(1)

kit.servo[1].set_pulse_width_range(500,2500)
kit.servo[1].angle = 90
time.sleep(1)

kit.servo[2].set_pulse_width_range(500,2500)
kit.servo[2].angle = 40
time.sleep(1)

## drop body || Standing position
kit.servo[1].set_pulse_width_range(500,2500)
kit.servo[1].angle = 130

kit.servo[2].set_pulse_width_range(500,2500)
kit.servo[2].angle = 90
time.sleep(3)

for _ in range(3):
    # # Start walking procedure
    # # lift leg
    kit.servo[1].set_pulse_width_range(500,2500)
    kit.servo[1].angle = 140

    kit.servo[0].set_pulse_width_range(500,2500)
    kit.servo[0].angle = 60
    time.sleep(1)

    kit.servo[1].set_pulse_width_range(500,2500)
    kit.servo[1].angle = 120
    time.sleep(1)

    kit.servo[0].set_pulse_width_range(500,2500)
    kit.servo[0].angle = 90
    time.sleep(1)




# for i in range(len(kit.servo)):  # pylint: disable=consider-using-enumerate
#     kit.servo[i].set_pulse_width_range(500,2500)
#     kit.servo[i].angle = 90

time.sleep(1)
# for i in range(len(kit.servo)):  # pylint: disable=consider-using-enumerate
#     kit.servo[i].set_pulse_width_range(500,2500)
#     kit.servo[i].angle = 60
# time.sleep(1)



'''
pulse width range 500,2500
tibia: 0-130
femur: 30 - 160
coxa: 30-150
'''