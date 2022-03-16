import numpy as np

from src.evtol import eVTOL

import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Model inputs
    m = 2200  #kg
    n = 6
    r = 0.65  # meters
    x_init = [0,0, 100]
    # Model initialize
    evtol = eVTOL(m, n, r)
    evtol.initialize_state(x_init)

    print(evtol.calculate_power(mode='hover'))
    print(evtol.calculate_power(mode='cruise'))

    print(evtol.battery.get_energy_remaining())

    evtol.fly(30, 'taxi')
    evtol.fly(5, 'hover')
    evtol.fly(45, 'vertical climb')
    evtol.fly(15, 'vertical climb') #TODO: TRANSITION
    evtol.fly(105, 'climb')
    evtol.fly(1500-105, 'cruise')
    evtol.fly(105, 'descent')
    evtol.fly(15, 'vertical descent')   #TODO: Transition reverse
    evtol.fly(45, 'vertical descent')
    evtol.fly(5, 'hover')
    evtol.fly(30, 'taxi')

    final_range_reamining = evtol.calculate_range()
    print(final_range_reamining)
    print(evtol.mission)
    print('Total Energy Used in kWh: {}'.format(evtol.battery.get_energy_used()))
    print('Predicted Range Remaining in km: {}'.format(evtol.calculate_range()))
    #TODO: ARE WE IGNORING THE ENERGY BELOW 20% OR NOT??? I THOUGHT I WAS!?

    # (tuple([mode, round(time, 2), round(power, 0),
    #                        round(soc_remaining, 2), round(range_remaining, 2)]))

    fig = plt.figure(10)
    times = [x[1] for x in evtol.mission]
    states = [x[5] for x in evtol.mission]
    xs = [x[0] for x in states]
    ys = [x[1] for x in states]
    zs = [x[2] for x in states]

    plt.plot(xs, ys)

    fig2 = plt.figure(11)
    plt.plot(xs,zs)

    predicted_ranges = [p[4] for p in evtol.mission]

    ts=[0]
    xs_new = [x_init[0]]
    ys_new = [x_init[1]]
    zs_new = [x_init[2]]
    plot_ranges = [0]

    index = 0
    for time in times:
        new_times = np.linspace(ts[-1], ts[-1]+time,time+1)
        for i in new_times:
            ts.append(i)
        new_xs = [xs[index]] * len(new_times)
        new_ys = [ys[index]] * len(new_times)
        new_zs = [zs[index]] * len(new_times)
        ranges = [predicted_ranges[index]] * len(new_times)
        #TODO: SMOOTHING USING NP.LINSPACE
        # new_xs = np.linspace(xs[index], xs[index+1], len(new_times))
        # new_ys = np.linspace(ys[index], ys[index+1], len(new_times))
        # new_zs = np.linspace(zs[index], zs[index+1], len(new_times))
        for j in new_xs:
            xs_new.append(j)
        for j in new_ys:
            ys_new.append(j)
        for j in new_zs:
            zs_new.append(j)
        for j in ranges:
            plot_ranges.append(j)
        index+=1

    fig3 = plt.figure(12)
    plt.plot(ts, xs_new)
    fig4 = plt.figure(13)
    plt.plot(ts, zs_new)
    fig5 = plt.figure(14)
    plt.plot(ts, plot_ranges)

    plt.show()

