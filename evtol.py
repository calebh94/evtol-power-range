from math import pi, sqrt


class battery:
    def __init__(self, m: float, total_energy: float):
        self.m = m
        self.E = total_energy  # Energy in kiloWatt-Hours (kWH)
        self.E_total = total_energy
        self.soc_limit = 20
        self.bat_eff = 0.95
        self.eff_total = 0.65
        self.DOD = 1.00 - (self.soc_limit / 100)  # Depth of Discharge
        self.SOH = 1.00  # State of Health #TODO: not used for now

    def run(self, power, time):
        energy_used = self._calculate_energy_use(power, time)
        self.E = self.E - energy_used
        if self.get_SOC() < self.soc_limit:
            print('SOC is below the limit! {} < {}'.format(self.get_SOC(), self.soc_limit))
            #TODO: replace with warning or return a 1 vs. 0
        return energy_used

    def get_energy_remaining(self):
        return self.E

    def get_SOC(self):
        return round(self.E / self.E_total * 100, 2)  # SOC in percentage


    def _calculate_energy_use(self, power, time):
        """ Power in kWH and time in hours"""
        power_bat = power / self.eff_total
        energy_used = power_bat * time
        return energy_used


class eVTOL:
    def __init__(self, m: float, n_rotors: int, r_rotors: float):
        self.g = 9.81
        self.m = m
        self.W = m * self.g
        self.DL = self.W / (n_rotors * 2*pi*pow(r_rotors, 2))
        self.x = tuple([0,0,0])

        self.battery = battery(0.33 * m, total_energy=0.33 * m * 260 / 1000)

        # Flight Performance Parameters to set later
        self.eff_hover = 0.70
        self.eff_cruise = 0.80
        self.V_cruise = 72  # m/s
        self.LD_cruise = 16

        self.mission = []

    def initialize_state(self, x_init: tuple):
        self.x = x_init
        self.density = self._get_density(self.x[2])

    def calculate_power(self, mode: str) -> float:
        """ Calculates the power requred in kW"""

        if mode == 'hover':
            power = self._hover_power()
        elif mode == 'cruise':
            power = self._cruise_power()
        else:
            power = 0

        power = power / 1000  # Watts to kW
        return power

    def fly(self, time: float, mode: str):
        """ Fly a flight mode for a amount of time (in seconds)"""
        time = time / 60 / 60  # conversion of seconds to hours
        power = self.calculate_power(mode)
        # energy_used = power * time
        self.battery.run(power, time)
        soc_remaining = self.battery.get_SOC()
        print('SOC remaining: {}%'.format(soc_remaining))
        range_remaining = self.calculate_range()
        print('Range remaining: {} km'.format(range_remaining))

        self.mission.append(tuple([mode, round(time,2), round(power, 0),
                                   round(soc_remaining,2), round(range_remaining,2)]))

        #TODO: update state
        self.x = self.x
        return self.x

    def calculate_range(self):
        range = self._calculate_range()  # already in km
        # range = range / 1000  # conversion from meters to kilometers
        return range

    def _calculate_range(self):
        # temp_batt = battery(self.battery.m, self.battery.E_total)
        power_cruise_left = self.calculate_power(mode='cruise')
        #TODO: time_left an input
        time_left_min = 30
        time_left = time_left_min * 60 / 60 / 60  # 5 minute cruise left
        # energy_use_est = self.battery._calculate_energy_use(power_cruise_left, time_left)
        # energy_use_est = 121.40
        # energy_use_est = 25
        energy_use_est = self.battery.get_energy_remaining()
        energy_density_est = energy_use_est / \
                             (self.battery.m * self.battery.bat_eff * self.battery.DOD) * 1000  #TODO: duoble check power and energy units everywhere
        # energy_density_est = 91.19
        range = energy_density_est * self.battery.eff_total * \
                (1/self.g) * (self.LD_cruise) * (self.battery.m / self.m)
        #TODO: change varaible name range since already a function range()

        return range

    def _hover_power(self):
        power = self.W / self.eff_hover * sqrt(self.DL / 2 * self._get_density(self.x[2]))
        return power

    def _cruise_power(self):
        """ Cruise power assuming thrust = drag and weight = lift"""
        power = self.W * self.V_cruise / (self.LD_cruise * self.eff_cruise)
        return power


    def _get_density(self, h: float) -> float:
        """ Calculation density from Earth Atmosphere Model (NASA)
        density in kg/m^3
        Calculation is accurate in the Troposphere (h < 11000)
        h: altitude in meters
        p: Pressure in K-Pa
        t: Temperature in degrees Celsius
        """
        if h > 11000:
            print("Vehicle altitude {} is lower than 11000 and the Earth Atmosphere Model is no longer accurate".format(h))
        t = 15.04 - 0.00649 * h
        p = 101.29 * pow((t + 273.1) / 288.08, 5.256)
        density = p / (0.2869 * (t+273.1))
        return density

    def set_hover_efficiency(self, eff):
        self.eff_hover = eff

    def get_mission(self):
        return self.mission

