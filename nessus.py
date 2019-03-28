__author__ = ("Antoine Drouin <poinix@gmail.com>", "Piotr Esden-Tempski <piotr@esden.net>")
__licence__ = "GPL3"

import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
from scipy import integrate
import misc_utils as mu
import motor
from motor import Motor
from h_bridge import HBridge
import time_series_plot as mp
from tqdm import tqdm
import yaml


def display_state_and_command(time, X, U):
    titles_state = ['$\\theta$', '$\omega$', '$i_u$', '$i_v$', '$i_w$']
    titles_cmd = ['$u_l$', '$u_h$', '$v_l$', '$v_h$', '$w_l$', '$w_h$']
    for i in range(0, 2):
        plt.subplot(6, 2, 2 * i + 1)
        plt.plot(time, mu.deg_of_rad(X[:, i]), 'r', linewidth=3.0)
        plt.title(titles_state[i])
    for i in range(2, motor.SE.size):
        plt.subplot(6, 2, 2 * i + 1)
        plt.plot(time, X[:, i], 'r', linewidth=3.0)
        plt.title(titles_state[i])
    for i in range(0, 6):
        plt.subplot(6, 2, 2 * i + 2)
        plt.plot(time, U[:, i], 'r', linewidth=3.0)
        plt.title(titles_cmd[i])


def print_simulation_progress(count, steps, pbar):
    sim_perc_last = np.ceil(((count - 1) * 100) / steps)
    sim_perc = np.ceil((count * 100) / steps)

    if (sim_perc_last != sim_perc):
        pbar.update(1)


def drop_it(a, factor):
    new = []
    for n, x in enumerate(a):
        if ((n % factor) == 0):
            new.append(x)
    return np.array(new)


def compress(a, factor):
    return drop_it(a, factor)

def main():

    freq_sim = 1e4  # simulation frequency
    compress_factor = 3
    time = pl.arange(0.0, 1, 1. / freq_sim)  # create time slice vector
    X = np.zeros((time.size, motor.SE.size))  # allocate state vector
    Xdebug = np.zeros((time.size, motor.DE.size))  # allocate debug data vector
    Y = np.zeros((time.size, motor.OE.size))  # allocate output vector
    U = np.zeros((time.size, motor.CE.size))  # allocate input vector
    X0 = [0, mu.rad_of_deg(0.1), 0, 0, 0]  #
    X[0, :] = X0
    W = [0, 1]
    pbar = tqdm(total=100)

    with open('parameters/motor_4poles_mutual_inductance.yaml', 'r') as param_file:
        parsed_params = yaml.load(param_file)
        m = Motor(parsed_params)
        h = HBridge(m)
    for i in range(1, time.size):

        if i == 1:
            Uim2 = np.zeros(motor.CE.size)
        else:
            Uim2 = U[i - 2, :]

        Y[i - 1, :] = m.output(X[i - 1, :], Uim2)  # get the output for the last step
        U[i - 1, :] = h.run(0, Y[i - 1, :], time[i - 1])  # run the controller for the last step
        tmp = integrate.odeint(m.dyn, X[i - 1, :], [time[i - 1], time[i]],
                               args=(U[i - 1, :], W))  # integrate
        X[i, :] = tmp[1, :]  # copy integration output to the current step
        X[i, motor.SE.Theta] = mu.norm_angle(X[i, motor.SE.Theta])  # normalize the angle in the state
        tmp, Xdebug[i, :] = m.dyn_debug(X[i - 1, :], time[i - 1], U[i - 1, :], W)  # get debug data
        print_simulation_progress(i, time.size, pbar)
    pbar.close()

    Y[-1, :] = Y[-2, :]
    U[-1, :] = U[-2, :]

    if compress_factor > 1:
        time = compress(time, compress_factor)
        Y = compress(Y, compress_factor)
        X = compress(X, compress_factor)
        U = compress(U, compress_factor)
        Xdebug = compress(Xdebug, compress_factor)

    mp.plot_output(time, Y, '-')
    plt.figure("state and command", figsize=(10.24, 5.12))
    display_state_and_command(time, X, U)

    plt.figure(figsize=(10.24, 5.12))
    mp.plot_debug(time, Xdebug)

    pl.show()


if __name__ == "__main__":
    main()
