# from src.servo import Servo
# import time

# # Create servos for LF leg (leg index 2)
# lf_coxa = Servo(6, pca=0x40)  # Servo index 6 
# lf_femur = Servo(7, pca=0x40)  # Servo index 7
# lf_tibia = Servo(8, pca=0x40)  # Servo index 8

# # Test each servo individually
# print("Testing LF Coxa")
# lf_coxa.set_angle(0)
# time.sleep(1)
# lf_coxa.set_angle(20)
# time.sleep(1)
# lf_coxa.set_angle(0)
# time.sleep(1)

# print("Testing LF Femur")
# lf_femur.set_angle(0)
# time.sleep(1)
# lf_femur.set_angle(-45)
# time.sleep(1)
# lf_femur.set_angle(-90)
# time.sleep(1)
# lf_femur.set_angle(0)
# time.sleep(1)

# print("Testing LF Tibia")
# lf_tibia.set_angle(0)
# time.sleep(1)
# lf_tibia.set_angle(45)
# time.sleep(1)
# lf_tibia.set_angle(0)
# time.sleep(1)

# print("Test complete")


# def test_lf_servo_angles():
#     """Test direct angle movement of LF servos"""
from src.servo import Servo
import time

# Create servos for LF leg
lf_coxa = Servo(6, pca=0x40)
lf_femur = Servo(7, pca=0x40)
lf_tibia = Servo(8, pca=0x40)

print("Testing LF femur angles")
angles_to_test = [-20]

for angle in angles_to_test:
    print(f"Setting femur to {angle}")
    lf_femur.set_angle(angle)
    time.sleep(1)

print("Testing LF tibia angles")
angles_to_test = [30, 40, 50, 60, 70]

for angle in angles_to_test:
    print(f"Setting tibia to {angle}")
    lf_tibia.set_angle(angle)
    time.sleep(1)
    
print("Test complete")