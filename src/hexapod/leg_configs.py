leg_configs = {
    "left_rear": {
        "servos": [0, 1 ,2],  # coxa, femur, tibia
        "offsets": {
            "coxa": [10, 50, 0],  # coxa joint offset from body
            "femur": [0, 50, -50],  # femur offset relative to coxa
            "tibia": [0, 0, -100]  # tibia offset relative to femur
        }
    },
    "left_middle": {
        "servos": [0, 1 ,2],
        "offsets": {
            "coxa": [-10, 50, 0],
            "femur": [0, 50, -50],
            "tibia": [0, 0, -100]
        }
    },
    "left_front": {
        "servos":  [0, 1 ,2],
        "offsets": {
            "coxa":  [0, 1 ,2],
            "femur": [0, 50, -50],
            "tibia": [0, 0, -100]
        }
    },
    "right_front": {
        "servos":  [0, 1 ,2],
        "offsets": {
            "coxa": [-10, 50, 0],
            "femur": [0, 50, -50],
            "tibia": [0, 0, -100]
        }
    },
    "left_middle": {
        "servos":  [0, 1 ,2],
        "offsets": {
            "coxa": [-10, 50, 0],
            "femur": [0, 50, -50],
            "tibia": [0, 0, -100]
        }
    },
    "left_rear": {
        "servos":  [0, 1 ,2],
        "offsets": {
            "coxa": [-10, 50, 0],
            "femur": [0, 50, -50],
            "tibia": [0, 0, -100]
        }
    },
}
