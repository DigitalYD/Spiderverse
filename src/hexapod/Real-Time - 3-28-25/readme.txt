All files are on the raspi4 in the hexapod. Does not sync with this repo.
TODO:
- Adjust bezier curve to be larger for real life, perhaps modify it's "offset" from the body based on the leg.

Figure out why the simulation works, but real life does not.
- Hexapod leg 1 (LM) tibia is angle 165, which appears to be super high and out of bounds for my servo calibration. Perhaps rotate the motor when leg is in up position after the run to bring tibia back down, and work as expected.
- Hexapod leg 2 (LF) Just moves weirdly, kinda jumps, The points should all be capable for the hexapod leg to reach.