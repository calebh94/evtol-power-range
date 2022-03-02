from evtol import eVTOL





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

    evtol.fly(20, 'hover')
    evtol.fly(200, 'cruise')
    evtol.fly(500, 'cruise')
    evtol.fly(2000, 'cruise')

    final_range_reamining = evtol.calculate_range()
    print(final_range_reamining)
    print(evtol.mission)

    # (tuple([mode, round(time, 2), round(power, 0),
    #                        round(soc_remaining, 2), round(range_remaining, 2)]))

