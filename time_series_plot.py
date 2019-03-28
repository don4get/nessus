__author__ = ("Antoine Drouin <poinix@gmail.com>", "Piotr Esden-Tempski <piotr@esden.net>")
__licence__ = "GPL3"

import matplotlib.pyplot as plt

import motor  as dm
import misc_utils as mu

ang_unit_rad_s = 0
ang_unit_deg_s = 1
ang_unit_rpm = 2


def plot_output(time, Y, ls):
    ang_unit = ang_unit_rpm

    # Phase current
    ax = plt.subplot(4, 1, 1)
    ax.yaxis.set_label_text('A', {'color': 'k', 'fontsize': 15})
    plt.plot(time, Y[:, dm.ov_iu], ls, linewidth=1.5)
    plt.plot(time, Y[:, dm.ov_iv], ls, linewidth=1.5)
    plt.plot(time, Y[:, dm.ov_iw], ls, linewidth=1.5)
    plt.legend(['$i_u$', '$i_v$', '$i_w$'], loc='upper right')
    plt.title('Phase current')

    # Phase terminal voltage
    ax = plt.subplot(4, 1, 2)
    ax.yaxis.set_label_text('V', {'color': 'k', 'fontsize': 15})
    plt.plot(time, Y[:, dm.ov_vu], ls, linewidth=1.5)
    plt.plot(time, Y[:, dm.ov_vv], ls, linewidth=1.5)
    plt.plot(time, Y[:, dm.ov_vw], ls, linewidth=1.5)
    plt.legend(['$v_u$', '$v_v$', '$v_w$'], loc='upper right')
    plt.title('Phase terminal voltage')

    # Rotor mechanical position
    ax = plt.subplot(4, 1, 3)
    ax.yaxis.set_label_text('Deg', {'color': 'k', 'fontsize': 15})
    plt.plot(time, mu.deg_of_rad(Y[:, dm.ov_theta]), ls, linewidth=1.5)
    #    plt.plot(time, Y[:,dm.ov_theta], ls, linewidth=1.5)
    plt.title('Rotor angular position')

    # Rotor mechanical angular speed
    ax = plt.subplot(4, 1, 4)

    if (ang_unit == ang_unit_rad_s):
        ax.yaxis.set_label_text('Rad/s', {'color': 'k', 'fontsize': 15})
        plt.plot(time, Y[:, dm.ov_omega], ls, linewidth=1.5)
    elif (ang_unit == ang_unit_deg_s):
        ax.yaxis.set_label_text('Deg/s', {'color': 'k', 'fontsize': 15})
        plt.plot(time, mu.degps_of_radps(Y[:, dm.ov_omega]), ls, linewidth=1.5)
    elif (ang_unit == ang_unit_rpm):
        ax.yaxis.set_label_text('RPM', {'color': 'k', 'fontsize': 15})
        plt.plot(time, mu.rpm_of_radps(Y[:, dm.ov_omega]), ls, linewidth=1.5)

    plt.title('Rotor Rotational Velocity')


def plot_debug(time, Xdebug):
    plt.subplot(4, 1, 1)

    plt.plot(time, Xdebug[:, dm.dv_eu], linewidth=1.5)
    plt.plot(time, Xdebug[:, dm.dv_ev], linewidth=1.5)
    plt.plot(time, Xdebug[:, dm.dv_ew], linewidth=1.5)
    plt.legend(['$U_{BEMF}$', '$V_{BEMF}$', '$W_{BEMF}$'], loc='upper right')

    plt.subplot(4, 1, 2)

    plt.plot(time, Xdebug[:, dm.dv_ph_U], linewidth=1.5)
    plt.plot(time, Xdebug[:, dm.dv_ph_V], linewidth=1.5)
    plt.plot(time, Xdebug[:, dm.dv_ph_W], linewidth=1.5)
    plt.legend(['$U$', '$V$', '$W$'], loc='upper right')

    plt.subplot(4, 1, 3)

    plt.plot(time, Xdebug[:, dm.dv_ph_star], linewidth=1.5)
    plt.legend(['$star$'], loc='upper right')


def plot_diodes(time, D):
    titles_diodes = ['$dhu$', '$dlu$', '$dhv$', '$dlv$', '$dhw$', '$dlw$']

    for i in range(0, dm.adc_size):
        plt.subplot(6, 2, 2 * i + 1)
        plt.plot(time, D[:, i], 'r', linewidth=1.5)
        plt.title(titles_diodes[i])
