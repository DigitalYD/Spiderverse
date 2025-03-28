
    # Gaits are below
def tripod_gait(self, steps):
    '''
        3 legs move at the same time
        ex (0,4,2) (1,3,5)
        "no delay"
    '''
    step_positions = self.generate_tripod_step_positions()
    for group in self.tripod_gait_groups:
        for leg_name in group:
            self.move_leg(leg_name, step_positions[leg_name])


    def triple_gait(self,steps):
        '''
            3 legs at the same time
            1/6 phase offset
            phase_delay = speed/6
            sleep(phase_delay)
        '''
        pass
    def wave_gait(self, steps):
        ''' One leg is lifted at a time 0,1,2,5,4,3'''
        pass

    def ripple_gait(self, steps):
        '''
            One leg at a time 0, 3, 1, 5, 2, 4
            happens at phases see image
            phase 1/6
        '''
        pass
    
    def quad_gait(self, steps):
        pass

    def tetra_gait(self, steps):
        pass



    def walk(self, gait="tripod", direction="", speed=1.0):
        if gait == "tripod":
            self.tripod_gait(direction, speed)
        elif gait == "wave": #also known as ripple
            self.wave_gait(direction, speed)
        elif gait == "ripple":
            self.ripple_gait(direction, speed)
        elif gait == "quadrupedal":
            self.quad_gait(speed)
        elif gait == "tetrapod":
            self.tetra_gait(speed)
        else:
            raise ValueError(f"Invalid gait {gait}")
        

           
        
    def update(self):
        # get current position of each leg, and the gait position of each leg add with current position
        # Do this for x,y,z positions
        
        '''
            rotio = hexapod rotation (rotio)
            RB_FeetPos = (posio)
        ''' 
        # initialize temp vars
        
        for legs in self.VALID_LEGS:
            # Get values from legs
            pass
        
    def calculate_gait(self):
        '''
            Calculate where each leg is during the gait and return x,y,z,rot
            Each leg has 3 positions that a single leg can be lifted to
        '''
        return