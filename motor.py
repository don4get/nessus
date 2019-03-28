__author__ = ("Antoine Drouin <poinix@gmail.com>", "Piotr Esden-Tempski <piotr@esden.net>")
__licence__ = "GPL3"

import math
import misc_utils as mu
import yaml
from enum import Enum
import warnings


class Motor:
    def __init__(self, parsed):
        self.n_poles = parsed["n_poles"]
        self.inertia = parsed["inertia"]
        self.damping = parsed["damping"]
        self.supply_voltage = parsed["supply_voltage"]
        self.phase_resistance = parsed["phase_resistance"]
        self.coil_inductance = parsed["coil_inductance"]
        self.mutual_inductance = parsed["mutual_inductance"]
        self.velocity_constant = parsed["velocity_constant"]
        self.diode_forward_voltage = parsed["diode_forward_voltage"]

    def back_emf(self, X, thetae_offset):
        """ Calculate back_emf at a given omega offset from the current rotor position

        :param X:
        :param thetae_offset:
        :return:
        """
        phase_thetae = mu.norm_angle((X[SE.Theta] * (self.n_poles / 2.)) + thetae_offset)

        bemf_constant = mu.vpradps_of_rpmpv(self.velocity_constant)  # aka. ke in V/rad/s
        max_bemf = bemf_constant * X[SE.Omega]

        bemf = 0.
        if 0. <= phase_thetae <= (math.pi * (1. / 6.)):
            bemf = (max_bemf / (math.pi * (1. / 6.))) * phase_thetae
        elif (math.pi / 6.) < phase_thetae <= (math.pi * (5. / 6.)):
            bemf = max_bemf
        elif (math.pi * (5. / 6.)) < phase_thetae <= (math.pi * (7. / 6.)):
            bemf = -((max_bemf / (math.pi / 6.)) * (phase_thetae - math.pi))
        elif (math.pi * (7. / 6.)) < phase_thetae <= (math.pi * (11. / 6.)):
            bemf = -max_bemf
        elif (math.pi * (11. / 6.)) < phase_thetae <= (2.0 * math.pi):
            bemf = (max_bemf / (math.pi / 6.)) * (phase_thetae - (2. * math.pi))
        else:
            raise AngleBoundError("Cannot compute BEMF {}".format(phase_thetae))

        return bemf

    def voltages(self, X, U):
        """ Calculate phase voltages

        :param X:
        :param U:
        :return: vector of phase voltages in reference to the star point
        """
        eu = self.back_emf(X, 0.)
        ev = self.back_emf(X, math.pi * (2. / 3.))
        ew = self.back_emf(X, math.pi * (4. / 3.))

        # Check which phases are excited
        pux = (U[CE.HighU] == 1) or (U[CE.LowU] == 1)

        pvx = (U[CE.HighV] == 1) or (U[CE.LowV] == 1)

        pwx = (U[CE.HighW] == 1) or (U[CE.LowW] == 1)

        vu = 0.
        vv = 0.
        vw = 0.
        vm = 0.

        if pux and pvx and pwx:
            vu = self.supply_voltage / 2. if U[CE.HighU] == 1 else -self.supply_voltage / 2.
            vv = self.supply_voltage / 2. if U[CE.HighV] == 1 else -self.supply_voltage / 2.
            vw = self.supply_voltage / 2. if U[CE.HighW] == 1 else -self.supply_voltage / 2.
            vm = (vu + vv + vw - eu - ev - ew) / 3.
        elif pux and pvx:
            vu = self.supply_voltage / 2. if U[CE.HighU] == 1 else -self.supply_voltage / 2.
            vv = self.supply_voltage / 2. if U[CE.HighV] == 1 else -self.supply_voltage / 2.

            # calculate star voltage
            vm = (vu + vv - eu - ev) / 2.

            # calculate remaining phase voltage
            vw = ew + vm

            # clip the voltage to freewheeling diodes
            # if (vw > ((VDC/2) + dvf)):
            #    vw = (VDC/2) + dvf;
            #    vm = (vu + vv + vw - eu - ev - ew) / 3.
            # elif (vw < (-(VDC/2) - dvf)):
            #    vw = -(VDC/2) - dvf;
            #    vm = (vu + vv + vw - eu - ev - ew) / 3.

        elif pux and pwx:
            vu = self.supply_voltage / 2. if U[CE.HighU] == 1 else -self.supply_voltage / 2.
            vw = self.supply_voltage / 2. if U[CE.HighW] == 1 else -self.supply_voltage / 2.
            vm = (vu + vw - eu - ew) / 2.
            vv = ev + vm

            # clip the voltage to freewheeling diodes
            # if (vv > ((VDC/2) + dvf)):
            #    vv = (VDC/2) + dvf;
            #    vm = (vu + vv + vw - eu - ev - ew) / 3.
            # elif (vv < (-(VDC/2) - dvf)):
            #    vv = -(VDC/2) - dvf;
            #    vm = (vu + vv + vw - eu - ev - ew) / 3.

        elif pvx and pwx:
            vv = self.supply_voltage / 2. if U[CE.HighV] == 1 else -self.supply_voltage / 2.
            vw = self.supply_voltage / 2. if U[CE.HighW] == 1 else -self.supply_voltage / 2.
            vm = (vv + vw - ev - ew) / 2.
            vu = eu + vm

            # clip the voltage to freewheeling diodes
            # if (vu > ((VDC/2) + dvf)):
            #    vu = (VDC/2) + dvf;
            #    vm = (vu + vv + vw - eu - ev - ew) / 3.
            # elif (vu < (-(VDC/2) - dvf)):
            #    vu = -(VDC/2) - dvf;
            #    vm = (vu + vv + vw - eu - ev - ew) / 3.

        elif pux:
            vu = self.supply_voltage / 2. if U[CE.HighU] == 1 else -self.supply_voltage / 2.
            vm = (vu - eu)
            vv = ev + vm
            vw = ew + vm

        # if we want to handle diodes properly how to do that here?

        elif pvx:
            vv = self.supply_voltage / 2. if U[CE.HighV] == 1 else -self.supply_voltage / 2.
            vm = (vv - ev)
            vu = eu + vm
            vw = ew + vm
        elif pwx:
            vw = self.supply_voltage / 2. if U[CE.HighW] == 1 else -self.supply_voltage / 2.
            vm = (vw - ew)
            vu = eu + vm
            vv = ev + vm
        else:
            vm = eu
            vv = ev
            vw = ew

        V = [vu, vv, vw, vm]

        return V

    def dyn(self, X, t, U, W):
        """ Step the dynamic model

        :param X: state
        :param t: time
        :param U: input
        :param W: perturbation
        """
        Xd, _ = self.dyn_debug(X, t, U, W)

        return Xd

    def dyn_debug(self, X, t, U, W):
        """ Dynamic model with debug vector

        :param X: state
        :param t: time
        :param U: input
        :param W: perturbation
        """
        eu = self.back_emf(X, 0.)
        ev = self.back_emf(X, math.pi * (2. / 3.))
        ew = self.back_emf(X, math.pi * (4. / 3.))

        # Electromagnetic torque
        electromagnetic_torque = (eu * X[SE.IU] + ev * X[SE.IV] + ew * X[SE.IW]) / X[SE.Omega]

        # Mechanical torque
        mechanical_torque = (
                    (electromagnetic_torque * (self.n_poles / 2)) - (self.damping * X[SE.Omega]) -
                    W[PE.Torque])

        if (mechanical_torque > 0) and (mechanical_torque <= W[PE.Friction]):
            mechanical_torque = 0
        elif mechanical_torque >= W[PE.Friction]:
            mechanical_torque = mechanical_torque - W[PE.Friction]
        elif (mechanical_torque < 0) and (mechanical_torque >= (-W[PE.Friction])):
            mechanical_torque = 0
        elif mechanical_torque <= (-W[PE.Friction]):
            mechanical_torque = mechanical_torque + W[PE.Friction]

        # Acceleration of the rotor
        omega_dot = mechanical_torque / self.inertia

        V = self.voltages(X, U)

        pdt = self.supply_voltage / 2 + self.diode_forward_voltage

        inductance = self.coil_inductance - self.mutual_inductance
        iu_dot = (V[PhE.U] - (self.phase_resistance * X[SE.IU]) - eu - V[PhE.Star]) / inductance
        iv_dot = (V[PhE.V] - (self.phase_resistance * X[SE.IV]) - ev - V[PhE.Star]) / inductance
        iw_dot = (V[PhE.W] - (self.phase_resistance * X[SE.IW]) - ew - V[PhE.Star]) / inductance

        Xd = [X[SE.Omega], omega_dot, iu_dot, iv_dot, iw_dot]
        Xdebug = [eu, ev, ew, V[PhE.U], V[PhE.V], V[PhE.W], V[PhE.Star]]

        return Xd, Xdebug

    def output(self, X, U):
        V = self.voltages(X, U)

        Y = [X[SE.IU], X[SE.IV], X[SE.IW],
             V[PhE.U], V[PhE.V], V[PhE.W],
             X[SE.Theta], X[SE.Omega]]

        return Y


class StateEnum:
    size = 5
    Theta, Omega, IU, IV, IW = range(size)


class CommandEnum:
    size = 6
    LowU, HighU, LowV, HighV, LowW, HighW = range(size)


class PerturbationEnum:
    size = 2
    Torque, Friction = range(size)


class OutputEnum:
    size = 8
    IU, IV, IW, VU, VV, VW, Theta, Omega = range(size)


class PhasesEnum:
    size = 4
    U, V, W, Star = range(size)


class DebugEnum:
    size = 7
    EU, EV, EW, PhU, PhV, PhW, PhStar = range(size)


class AngleBoundError(Exception):
    """Raised when the angle of the motor is out of bounds"""
    pass


SE = StateEnum
CE = CommandEnum
PE = PerturbationEnum
OE = OutputEnum
PhE = PhasesEnum
DE = DebugEnum
