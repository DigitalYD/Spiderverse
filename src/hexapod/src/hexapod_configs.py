import numpy as np

hexapod_configs = {
    "body_pose": {
        "pose":(0.0, 0.0, 0.0, 0.0 ,0.0, 0.0),
        "coxa_offsets": {    # X, Y positions of coxa from center-mass
            "LR": [-60.5, -89],
            "LM": [-97, 0], # -97
            "LF": [-60.5, 89],
            "RF": [60.5, 89],
            "RM": [97, 0], # 97
            "RR": [60.5, -89]
            },
    }
}