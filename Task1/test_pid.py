#!/usr/bin/env python3

from pid import PID
import time
from  matplotlib import pyplot, patches

if __name__ == "__main__":
    pid = PID()
    pid.set_kp(0.2)
    pid.set_ki(1)
    pid.set_kd(0.1)
    pid.set_lim_up(1)
    pid.set_lim_low(-1)

    error = [0,0,0,0.4,0.4,0.4,0.4,0.4,0.5,0.6,0.7,0.8,0.9, 0, 0, 0, 0, 0,
             -0.4,-0.4,-0.4,-0.4,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,0,0,0,0,0]
    iter = 0
    up = []
    ui = []
    ud = []
    u = []
    t = []
    dt = 0.25
    count = 0
    for e in error:
        ctl_value = pid.compute(e, 0)
        print('u =',  ctl_value)
        u_values = pid.get_pid_values()
        up.append(u_values[0])
        ui.append(u_values[1])
        ud.append(u_values[2])
        u.append(u_values[3])
        t.append(dt*count)
        count = count + 1
        time.sleep(dt)

    line1 = pyplot.plot(t, error, 'c', linewidth=2, label='error')
    #pyplot.hold(True)
    line2 = pyplot.plot(t, up, 'b', linewidth=2, label='up')
    line3 = pyplot.plot(t, ui, 'g', linewidth=2, label='ui')
    line4 = pyplot.plot(t, ud, 'm', linewidth=2, label='ud')
    line5 = pyplot.plot(t, u, 'r', linewidth=2, label='u')
    e_leg = patches.Patch(color='cyan', label='error')
    up_leg = patches.Patch(color='blue', label='up')
    ui_leg = patches.Patch(color='green', label='ui')
    ud_leg = patches.Patch(color='magenta', label='ud')
    u_leg = patches.Patch(color='red', label='u_total')
    pyplot.legend(loc='upper center', ncol=5)
    pyplot.xlabel('t [s]')
    pyplot.grid(True)
    pyplot.show()

