## Preamble
Hey everyone, as some of you may know I will be deploying. So here are the instructions for my code. I will attempt to assist when I can, I have bought a hotspot, and will hopefully get a service provider for it so I can continue work on my project. On at the very least the simulation portion, which can be ported to "real-time" physical hexapod robot.


## TODO
Adding reverse/left/right movement (Constants already created in config.py just needs imported to assist with this). 
-> It may be useful to use a specific bezier curve for forward movement, backward movement, then rotation left and right, and adjust them accordingly only as needed. That requires saving 100 points each in memory. So replacing the currently used bezier curve "bezier_curve" (current implementation) may be best option. With that said, a new function will need made for such. The function for intialization, and walking forward is completed. 
- [X] Initialization bezier curve
- [X] Walking Bezier Curve
- [ ] Reverse Bezier Curve 
- [ ] Left/Right bezier curve
- [ ] Adjusted pod.update() class for reverse/turning using CONSTANTS 
- [ ] Controller Implementated

## Potatoesss
Real-Time - This is the code that gets copied onto the hexapod to be ran.\
SIMULATION - This is the code for the simulation\
a_tests is just example tests show some of the functionality that I have, it can be mostly ignored.

## Differences in  Real-time vs Simulation

main.py (The main code to be ran on the hexapod)

test_sim.py & matplot_sim.py\
Both are "simulations" that allow the user to see the code being ran.
NOTE: there are some key differences between real-time and simulation code. For example, real time the bezier curve is.
```python
self.control_points = {
    "start": start_pos,
    "lift": start_pos + np.array([0, 10, -70]),
    "peak": start_pos + np.array([0, 50, -150]),
    "lower": start_pos + np.array([0, 75, -70]),
    "touchdown": start_pos + np.array([0, 75, 0]),
    "grounded": start_pos + np.array([0, 75, 0]),
    "sliding": start_pos + np.array([0, 75, 0]),
    "return": start_pos,
}
```

While in the simulation it is 

```python
self.control_points = {
    "start": start_pos,
    "lift": start_pos + np.array([0, 75, -20]),
    "peak": start_pos + np.array([0, 100, -35]),
    "lower": start_pos + np.array([0, 125, -20]),
    "touchdown": start_pos + np.array([0, 125, 0]),
    "grounded": start_pos + np.array([0, 125, 0]),
    "sliding": start_pos + np.array([0, 120, 0]),
    "return": start_pos,
}
```

As you can see, the curve is smaller forward and back in real-time, and the height is increased.

Within "inversekinematics.py" \
For for simulation

```python
servo_angles.Tibia = 180 - ()
```

In real-time
```python
servo_angles.Tibia =  ()
```

As you can see the tibia is not subtracted by 180 degrees. There may be some other discrepencies with this.

## Classes
**coord.py**\
This class holds all the coordinate and shifting functions. If a point needs manipulated in 3d space, it will be found here

**inversekinematics.py**\
This is the function in which allows the hexapod's legs to move along the bezier curve. Returns the angles to be used by the servos.

**config.py**\
This holds all of the constantsfor the entire hexapod. Each constant has a description. Use these as needed.

**bezier2d.py**\
This file holds the class for the bezier curve. It maintains a dictionary of all the points, and control points. Allows you to grab point x-z using the dictionary names, allows the entire curve to be accessed, or specific points.


**Servo.py**\
 Holds all the information for the servo class. Utilizes pca9685 in order to run. Note if you get ANY ERRORS dealing with PCA9685, or the servo.py class with "pca". Check your connections, power, and verify that the servos are initialized within the "leg.py" and "pod.py" (these are commented out in simulation)
```python.
    Coxa: Servo
    Femur: Servo
    Tibia: Servo
```
**leg.py**\
This class is where all the leg movement, bezier curve adjustments based off the coxa offset, etc take place.

**hex_body.py**\
This houses the hexapod body constants, which will be useful when performing different functions. The gait is stored here, among other variables. Note some of the information here is duplicated in pod. However, when making changes, to pod, ensure hex_body also has that same information for consistency.

**pod.py**\
This function manages body/gaits/movement/legs etc for the hexapod. To include the hexapod height, which when changed should be adjusted in the legs.\
Note: when making changes to height, the bezier curve for each must also be updated (may not be implemented)


### IMPORTANT FUNCTIONS
These are functions that will be key to what will need done.

##### Pod:
- update(): This manages the movement using the bezier curves. For all Modes "initialization, resetting, walking, neutral (standing)". It also uses "slide hexapod forward (for future simulation implementation *not used for anything else). Returns the bezier curve foot position.


#### Leg:
- set_initialization_control_points(): sets up the points for initialziation. Laying position to standing position. 
- set_walking_control_points(): Set walking control points, based off the coxa position,
- set_back_control_points(): (this is where reverse control points will be added. Not fully implemented yet.) NEEDS TWEAKED
- recalculate_forward_kinematics(): Calculates all the positions for the hexapod leg components. THIS WORKS DO NOT EDIT
- get_joint_origin(): helper to pull coordinate from the homogeneous transformation

## Coord
#### NOTE: TO see how some of these functions work, see the a_tests folder. Will come in very useful for hexapod bezier curve rotation. Please see how they function and read their comments. Crutial for making the hexapod rotate, and walk
- homogeneous_transform_around_center()
- homogeneous_transformation_matrix()
- get_radial_direction ()
- adjust_point_away_from_coxa ()
- rotate_bezier_curve()
- translate_point_along_leg_direction()
