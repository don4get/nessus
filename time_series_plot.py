__author__ = ("Antoine Drouin <poinix@gmail.com>", "Piotr Esden-Tempski <piotr@esden.net>")
__licence__ = "GPL3"

import matplotlib.pyplot as plt

import motor
import misc_utils as mu

ang_unit_rad_s = 0
ang_unit_deg_s = 1
ang_unit_rpm = 2


def plot_output(time, Y, ls):
    ang_unit = ang_unit_rpm

    # Phase current
    ax = plt.subplot(4, 1, 1)
    ax.yaxis.set_label_text('A', {'color': 'k', 'fontsize': 15})
    plt.plot(time, Y[:, motor.OE.IU], ls, linewidth=1.5)
    plt.plot(time, Y[:, motor.OE.IV], ls, linewidth=1.5)
    plt.plot(time, Y[:, motor.OE.IW], ls, linewidth=1.5)
    plt.legend(['$i_u$', '$i_v$', '$i_w$'], loc='upper right')
    plt.title('Phase current')

    # Phase terminal voltage
    ax = plt.subplot(4, 1, 2)
    ax.yaxis.set_label_text('V', {'color': 'k', 'fontsize': 15})
    plt.plot(time, Y[:, motor.OE.VU], ls, linewidth=1.5)
    plt.plot(time, Y[:, motor.OE.VV], ls, linewidth=1.5)
    plt.plot(time, Y[:, motor.OE.VW], ls, linewidth=1.5)
    plt.legend(['$v_u$', '$v_v$', '$v_w$'], loc='upper right')
    plt.title('Phase terminal voltage')

    # Rotor mechanical position
    ax = plt.subplot(4, 1, 3)
    ax.yaxis.set_label_text('Deg', {'color': 'k', 'fontsize': 15})
    plt.plot(time, mu.deg_of_rad(Y[:, motor.OE.Theta]), ls, linewidth=1.5)
    #    plt.plot(time, Y[:,dm.ov_theta], ls, linewidth=1.5)
    plt.title('Rotor angular position')

    # Rotor mechanical angular speed
    ax = plt.subplot(4, 1, 4)

    if (ang_unit == ang_unit_rad_s):
        ax.yaxis.set_label_text('Rad/s', {'color': 'k', 'fontsize': 15})
        plt.plot(time, Y[:, motor.OE.Omega], ls, linewidth=1.5)
    elif (ang_unit == ang_unit_deg_s):
        ax.yaxis.set_label_text('Deg/s', {'color': 'k', 'fontsize': 15})
        plt.plot(time, mu.degps_of_radps(Y[:, motor.OE.Omega]), ls, linewidth=1.5)
    elif (ang_unit == ang_unit_rpm):
        ax.yaxis.set_label_text('RPM', {'color': 'k', 'fontsize': 15})
        plt.plot(time, mu.rpm_of_radps(Y[:, motor.OE.Omega]), ls, linewidth=1.5)

    plt.title('Rotor Rotational Velocity')


def plot_debug(time, Xdebug):
    plt.subplot(4, 1, 1)

    plt.plot(time, Xdebug[:, motor.DE.EU], linewidth=1.5)
    plt.plot(time, Xdebug[:, motor.DE.EV], linewidth=1.5)
    plt.plot(time, Xdebug[:, motor.DE.EW], linewidth=1.5)
    plt.legend(['$U_{BEMF}$', '$V_{BEMF}$', '$W_{BEMF}$'], loc='upper right')

    plt.subplot(4, 1, 2)

    plt.plot(time, Xdebug[:, motor.DE.PhU], linewidth=1.5)
    plt.plot(time, Xdebug[:, motor.DE.PhV], linewidth=1.5)
    plt.plot(time, Xdebug[:, motor.DE.PhW], linewidth=1.5)
    plt.legend(['$U$', '$V$', '$W$'], loc='upper right')

    plt.subplot(4, 1, 3)

    plt.plot(time, Xdebug[:, motor.DE.PhStar], linewidth=1.5)
    plt.legend(['$star$'], loc='upper right')


def plot_diodes(time, D):
    titles_diodes = ['$dhu$', '$dlu$', '$dhv$', '$dlv$', '$dhw$', '$dlw$']

    for i in range(0, motor.CE.size):
        plt.subplot(6, 2, 2 * i + 1)
        plt.plot(time, D[:, i], 'r', linewidth=1.5)
        plt.title(titles_diodes[i])
