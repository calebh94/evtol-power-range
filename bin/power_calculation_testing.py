import numpy as np

from src.evtol import eVTOL

import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Model inputs
    m = 2200  #kg
    n = 6
    r = 0.65  # meters
    x_init = [0,0, 100]
    heading = 0
    # Model initialize
    evtol = eVTOL(m, n, r)
    evtol.initialize_state(x_init, heading)

    # print(evtol.calculate_power(mode='hover'))
    # print(evtol.calculate_power(mode='cruise'))
    #
    # print(evtol.battery.get_energy_remaining())

    mission = []
    evtol.fly(30, 'taxi')
    evtol.fly(5, 'hover')
    # (z_alt_now - z_alt_des) / evtol.ROC_climb

    evtol.fly(45, 'vertical climb')
    # evtol.fly(15, 'vertical climb')
    evtol.fly(45, 'transition forward')
    evtol.fly(105, 'climb')
    # evtol.fly(1500-105, 'cruise')
    cruise_time = 1500-105
    evtol.fly(cruise_time*0.333, 'cruise')
    evtol.fly(cruise_time*0.333, 'cruise')
    evtol.fly(cruise_time*0.333, 'cruise')
    evtol.fly(105, 'descent')
    # evtol.fly(15, 'vertical descent')
    evtol.fly(45, 'transition reverse')
    evtol.fly(45, 'vertical descent')
    evtol.fly(5, 'hover')
    evtol.fly(30, 'taxi')

    final_range_remaining = evtol.calculate_range()
    print(final_range_remaining)
    print(evtol.mission)
    print('Total Energy Used in kWh: {}'.format(evtol.battery.get_energy_used()))
    print('Predicted Range Remaining in km: {}'.format(evtol.calculate_range()))
    #TODO: ARE WE IGNORING THE ENERGY BELOW 20% OR NOT??? I THOUGHT I WAS!?

    # (tuple([mode, round(time, 2), round(power, 0),
    #                        round(soc_remaining, 2), round(range_remaining, 2)]))

    fig = plt.figure(10)
    times = [x[1] for x in evtol.mission]
    socs = [x[3] for x in evtol.mission]
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
    predicted_ranges.insert(0,0)
    socs.insert(0,100)
    xs.insert(0,x_init[0])
    ys.insert(0,x_init[1])
    zs.insert(0,x_init[2])

    plot_socs = [100]

    for time in times:
        new_times = np.linspace(ts[-1], ts[-1]+time,time+1)
        for i in new_times:
            ts.append(i)
        # new_xs = [xs[index]] * len(new_times)
        # new_ys = [ys[index]] * len(new_times)
        # new_zs = [zs[index]] * len(new_times)
        ranges = np.linspace(predicted_ranges[index], predicted_ranges[index+1], len(new_times))
        # ranges = [predicted_ranges[index+1]] * len(new_times)
        #TODO: SMOOTHING USING NP.LINSPACE
        new_xs = np.linspace(xs[index], xs[index+1], len(new_times))
        new_ys = np.linspace(ys[index], ys[index+1], len(new_times))
        new_zs = np.linspace(zs[index], zs[index+1], len(new_times))

        new_socs = np.linspace(socs[index], socs[index+1], len(new_times))

        for j in new_xs:
            xs_new.append(j)
        for j in new_ys:
            ys_new.append(j)
        for j in new_zs:
            zs_new.append(j)
        for j in ranges:
            plot_ranges.append(j)
        for j in new_socs:
            plot_socs.append(j)
        index+=1

    fig3 = plt.figure(12)
    plt.plot(ts, xs_new)
    plt.title("X Position over Flight")
    plt.xlabel("Time")
    fig4 = plt.figure(16)
    plt.plot(ts, ys_new)
    plt.title("Y Position over Flight")
    plt.xlabel("Time")
    fig6 = plt.figure(13)
    plt.plot(ts, zs_new)
    plt.title("Altitude over Flight")
    plt.xlabel("Time")
    fig5 = plt.figure(14)
    plt.plot(ts, plot_ranges)
    plt.title("Predicted Range over Flight")
    plt.xlabel("Time")
    fig7 = plt.figure(17)
    plt.plot(ts, plot_socs)
    plt.title("SOC over Flight")
    plt.xlabel("Time")

    plt.show()

