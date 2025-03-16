import numpy as np

hexapod_configs = {
    "NumLegs": 6,
    "gait": "tripod",
    "Coordinate": {    # X, Y positions of coxa from center-mass
        "LR": [-60.5, -89, 0],
        "LM": [-97, 0, 0], # -97
        "LF": [-60.5, 89, 0],
        "RF": [60.5, 89, 0],
        "RM": [97, 0, 0], # 97
        "RR": [60.5, -89, 0]
        },
    "coxa_angle_offset": {
        "LR": 214.309,
        "LM": 270,
        "LF": 325.931,
        "RF": 34.0685,
        "RM": 90,
        "RR": 147.172
    },
    "Segments": {
        "Coxa": 45,
        "Femur": 110,
        "Tibia": 193
    },
    "Angles": {
        "Coxa": 80,
        "Femure": 0,
        "Tibia": 60,
    }
}