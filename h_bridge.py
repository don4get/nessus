__author__ = ("Antoine Drouin <poinix@gmail.com>", "Piotr Esden-Tempski <piotr@esden.net>")
__licence__ = "GPL3"

import numpy as np

import motor

import misc_utils as mu

import math

PWM_freq = 16000
PWM_cycle_time = (1./16000)
PWM_duty = 0.4
PWM_duty_time = PWM_cycle_time * PWM_duty

debug = False


class HBridge():
    def __init__(self, motor):
        self.motor_n_poles = motor.n_poles

    #
    #
    # Sp setpoint, Y output
    #
    def run_hpwm_l_on_bipol(self, Sp, Y, t):
        elec_angle = mu.norm_angle(Y[motor.OE.Theta] * self.motor_n_poles / 2)

        U = np.zeros(motor.CE.size)

        step = "none"

        # switching pattern based on the "encoder"
        # H PWM L ON pattern
        if 0. <= elec_angle <= (math.pi * (1. / 6.)):  # second half of step 1
            # U off
            # V low
            # W hpwm
            hu = 0
            lu = 0
            hv = 0
            lv = 1
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hw = 1
            else:
                hw = 0
            lw = 0
            step = "1b"
        elif (math.pi * (1.0 / 6.0)) < elec_angle <= (math.pi * (3.0 / 6.0)):  # step 2
            # U hpwm
            # V low
            # W off
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hu = 1
            else:
                hu = 0
            lu = 0
            hv = 0
            lv = 1
            hw = 0
            lw = 0
            step = "2 "
        elif (math.pi * (3.0 / 6.0)) < elec_angle <= (math.pi * (5.0 / 6.0)):  # step 3
            # U hpwm
            # V off
            # W low
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hu = 1
            else:
                hu = 0
            lu = 0
            hv = 0
            lv = 0
            hw = 0
            lw = 1
            step = "3 "
        elif (math.pi * (5.0 / 6.0)) < elec_angle <= (math.pi * (7.0 / 6.0)):  # step 4
            # U off
            # V hpwm
            # W low
            hu = 0
            lu = 0
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hv = 1
            else:
                hv = 0
            lv = 0
            hw = 0
            lw = 1
            step = "4 "
        elif (math.pi * (7.0 / 6.0)) < elec_angle <= (math.pi * (9.0 / 6.0)):  # step 5
            # U low
            # V hpwm
            # W off
            hu = 0
            lu = 1
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hv = 1
            else:
                hv = 0
            lv = 0
            hw = 0
            lw = 0
            step = "5 "
        elif (math.pi * (9.0 / 6.0)) < elec_angle <= (math.pi * (11.0 / 6.0)):  # step 6
            # U low
            # V off
            # W hpwm
            hu = 0
            lu = 1
            hv = 0
            lv = 0
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hw = 1
            else:
                hw = 0
            lw = 0
            step = "6 "
        elif (math.pi * (11.0 / 6.0)) < elec_angle <= (
                math.pi * (12.0 / 6.0)):  # first half of step 1
            # U off
            # V low
            # W hpwm
            hu = 0
            lu = 0
            hv = 0
            lv = 1
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hw = 1
            else:
                hw = 0
            lw = 0
            step = "1a"
        else:
            print('ERROR: The electrical angle is out of range!!!')

        # Assigning the scheme phase values to the simulator phases
        # "Connecting the controller wires to the motor" ^^
        # This way we can for example decide which direction we want to turn the motor
        U[motor.CE.HighU] = hu
        U[motor.CE.LowU] = lu
        U[motor.CE.HighV] = hw
        U[motor.CE.LowV] = lw
        U[motor.CE.HighW] = hv
        U[motor.CE.LowW] = lv

        if debug:
            print('time {} step {} eangle {} switches {}'.format(t, step, mu.deg_of_rad(elec_angle),
                                                                 U))

        return U

    #
    #
    # Sp setpoint, Y output
    #
    def run_hpwm_l_on(self, Sp, Y, t):
        elec_angle = mu.norm_angle(Y[motor.OE.Theta] * self.motor_n_poles / 2)

        U = np.zeros(motor.CE.size)

        step = "none"

        # switching pattern based on the "encoder"
        # H PWM L ON pattern bipolar
        if 0. <= elec_angle <= (math.pi * (1. / 6.)):  # second half of step 1
            # U off
            # V low
            # W hpwm
            hu = 0
            lu = 0
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hw = 1
                lw = 0
                hv = 0
                lv = 1
            else:
                hw = 0
                lw = 1
                hv = 1
                lv = 0
            step = "1b"
        elif (math.pi * (1.0 / 6.0)) < elec_angle <= (math.pi * (3.0 / 6.0)):  # step 2
            # U hpwm
            # V low
            # W off
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hu = 1
                lu = 0
                hv = 0
                lv = 1
            else:
                hu = 0
                lu = 1
                hv = 1
                lv = 0
            hw = 0
            lw = 0
            step = "2 "
        elif (math.pi * (3.0 / 6.0)) < elec_angle <= (math.pi * (5.0 / 6.0)):  # step 3
            # U hpwm
            # V off
            # W low
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hu = 1
                lu = 0
                hw = 0
                lw = 1
            else:
                hu = 0
                lu = 1
                hw = 1
                lw = 0
            hv = 0
            lv = 0
            step = "3 "
        elif (math.pi * (5.0 / 6.0)) < elec_angle <= (math.pi * (7.0 / 6.0)):  # step 4
            # U off
            # V hpwm
            # W low
            hu = 0
            lu = 0
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hv = 1
                lv = 0
                hw = 0
                lw = 1
            else:
                hv = 0
                lv = 1
                hw = 1
                lw = 0
            step = "4 "
        elif (math.pi * (7.0 / 6.0)) < elec_angle <= (math.pi * (9.0 / 6.0)):  # step 5
            # U low
            # V hpwm
            # W off
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hv = 1
                lv = 0
                hu = 0
                lu = 1
            else:
                hv = 0
                lv = 1
                hu = 1
                lu = 0
            hw = 0
            lw = 0
            step = "5 "
        elif (math.pi * (9.0 / 6.0)) < elec_angle <= (math.pi * (11.0 / 6.0)):  # step 6
            # U low
            # V off
            # W hpwm
            hv = 0
            lv = 0
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hw = 1
                lw = 0
                hu = 0
                lu = 1
            else:
                hw = 0
                lw = 1
                hu = 1
                lu = 0
            step = "6 "
        elif (math.pi * (11.0 / 6.0)) < elec_angle <= (
                math.pi * (12.0 / 6.0)):  # first half of step 1
            # U off
            # V low
            # W hpwm
            hu = 0
            lu = 0
            if math.fmod(t, PWM_cycle_time) <= PWM_duty_time:
                hw = 1
                lw = 0
                hv = 0
                lv = 1
            else:
                hw = 0
                lw = 1
                hv = 1
                lv = 0
            step = "1a"
        else:
            print('ERROR: The electrical angle is out of range!!!')

        # Assigning the scheme phase values to the simulator phases
        # "Connecting the controller wires to the motor" ^^
        # This way we can for example decide which direction we want to turn the motor
        U[motor.CE.HighU] = hu
        U[motor.CE.LowU] = lu
        U[motor.CE.HighV] = hw
        U[motor.CE.LowV] = lw
        U[motor.CE.HighW] = hv
        U[motor.CE.LowW] = lv


        if debug:
            print('time {} step {} eangle {} switches {}'.format(t, step, mu.deg_of_rad(elec_angle),
                                                                 U))

        return U

    # Sp setpoint, Y output
    #
    def run(self, Sp, Y, t):
        # return run_hpwm_l_on(Sp, Y, t)
        return self.run_hpwm_l_on_bipol(Sp, Y, t)
