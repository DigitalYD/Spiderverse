leg_configs = {
    "LR": {
        "leg_index": 0,
        "servos": [0, 1, 2],  # coxa, femur, tibia
        "pulsemin" : 500,
        "pulsemax": 2500,
        "segment_lengths": [45, 110, 193],
        "position": [-60.5, 89],
    },
    "LM": {
        "leg_index": 1,
        "servos": [3, 4, 5],
        "pulsemin" : 500,
        "pulsemax": 2500,
        "segment_lengths": [45, 110, 193],
        "position":  [-97, 0],
    },
    "LF": {
        "leg_index": 2,
        "servos": [6, 7, 8],
        "pulsemin" : 500,
        "pulsemax": 2500,
        "segment_lengths": [45, 110, 193],
        "position": [-60.5, -89],
    },
    "RF": {
        "leg_index": 3,
        "servos": [9, 10, 11],
        "pulsemin" : 500,
        "pulsemax": 2500,
        "segment_lengths": [45, 110, 193],
        "position": [60.5, 89],
    },
    "RM": {  # Corrected duplicate key issue
        "leg_index": 4,
        "servos": [12, 13, 14],
        "pulsemin" : 500,
        "pulsemax": 2500,
        "segment_lengths": [45, 110, 193],
        "position": [97, 0],
    },
    "RR": {  # Corrected duplicate key issue
        "leg_index": 5,
        "servos": [15, 16, 17],
        "pulsemin" : 500,
        "pulsemax": 2500,
        "segment_lengths": [45, 110, 193],
        "position": [60.5, -89],
    }
}