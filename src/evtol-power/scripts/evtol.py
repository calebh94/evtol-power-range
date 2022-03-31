""" eVTOL Model for Power and Energy Range Prediction
Power calculations from Bruhl, Fricke, Schultz, "Air taxi flight performance modeling and
application", 2021.
"""
import warnings
from math import pi, sqrt, sin, cos
from Battery import Battery

RAD2DEG = 180 / pi
DEG2RAD = pi / 180


class eVTOL:
    def __init__(self, m: float, n_rotors: int, r_rotors: float):
        self.g = 9.81
        self.m = m
        self.W = m * self.g
        self.DL = self.W / (n_rotors * 2*pi*pow(r_rotors, 2))
        self.x = [0,0,0]
        self.heading = 0
        self.density = self._get_density(h=0)

        self.CD = 0.039
        self.Cd = 0.015
        self.A = 8.0  # m^2
        self.N = 5
        self.n = 6
        self.c = 0.3
        self.d_rotor = 2*r_rotors
        self.S = 11.0  # m^2

        self.battery = Battery(0.33 * m, total_energy=0.33 * m * 260 / 1000)

        # Flight Performance Parameters to set later #TODO: READ FROM models/ INPUT
        self.eff_hover = 0.70
        self.eff_cruise = 0.80
        self.eff_climb = 0.75
        self.V_cruise = 72  # m/s
        self.LD_cruise = 16
        self.ROC_TO = 0.5  # m/s
        self.ROC_LD = -0.5

        # Climb
        self.ROC_climb = 4.5
        self.climb_angle = 8*DEG2RAD  # radians
        self.V_climb = self.ROC_climb / sin(self.climb_angle)  # m/s
        self.LD_climb = 15

        # Descend
        self.ROC_descend = -4.5
        self.descend_angle = self.climb_angle
        self.V_descend = abs(self.ROC_descend) / sin(self.descend_angle)  # m/s

        # Transition
        self.V_transition = self.V_cruise-self.V_climb #TODO: CHECK THIS, SHOULD BE A TRANSITION FROM V_TALL TO CRUISE (or climb for Bruhl) (SEE TEM WORK)
        self.theta_transition = 82 * DEG2RAD  # radians
        self.eff_transition = 0.65
        self.M_tip = 0.55

        self.mission = []

        self.planned_mission =[(30, 'taxi'),
                                (5, 'hover'),
                                (45, 'vertical climb'),
                                (15, 'vertical climb'),  # TODO: TRANSITION
                                (105, 'climb'),
                                (1500 - 105, 'cruise'),
                                (105, 'descent'),
                                (15, 'vertical descent'),  # TODO: Transition reverse
                                (45, 'vertical descent'),
                                (5, 'hover'),
                                (30, 'taxi')]

    def initialize_state(self, x_init: tuple, heading: float):
        self.x = x_init
        self.heading = heading
        self.density = self._get_density(self.x[2])

    def calculate_power(self, mode: str) -> float:
        """ Calculates the power requred in kW"""

        if mode == 'hover':
            power = self._hover_power()
        elif mode == 'cruise':
            power = self._cruise_power()
        elif mode == 'vertical climb':
            power = self._vertical_climb_power()
        elif mode == 'climb':
            power = self._climb_power()
        elif mode == 'descent':
            power = self._cruise_power()
        elif mode == 'vertical descent':
            power = self._vertical_descent_power()
        elif mode == 'taxi':
            power = self._taxi_power()
        elif mode == 'transition forward':
            power = self._transition_forward_power()
        elif 'transition reverse':
            power = self._transition_reverse_power()
        else:
            power = 0

        power = power / 1000  # Watts to kW
        return power

    def fly(self, time: float, mode: str):
        """ Fly a flight mode for a amount of time (in seconds)"""
        time_hrs = time / 60 / 60  # conversion of seconds to hours
        power = self.calculate_power(mode)
        # energy_used = power * time
        self.battery.run(power, time_hrs)
        soc_remaining = self.battery.get_SOC()
        print('SOC remaining: {}%'.format(soc_remaining))
        range_remaining = self.calculate_range()
        print('Range remaining: {} km'.format(range_remaining))

        self.x = self._update_state(mode, time)
        self.density = self._get_density(self.x[2])

        self.mission.append(tuple([mode, round(time,2), round(power, 0),
                                   round(soc_remaining,2), round(range_remaining,2),
                                   self.x]))
        #TODO: SELF.X is not copying it is being attached using a pointer and syaing the same throughout appending
        return self.x

    def calculate_range(self):
        range = self._calculate_range()  # already in km
        # range = range / 1000  # conversion from meters to kilometers
        return range

    def _calculate_range(self):
        method = 2
        # temp_batt = battery(self.battery.m, self.battery.E_total)
        power_cruise_left = self.calculate_power(mode='cruise')
        #TODO: time_left an input
        time_left_min = 30
        time_left = time_left_min * 60 / 60 / 60  # 5 minute cruise left
        # energy_use_est = self.battery._calculate_enefrgy_use(power_cruise_left, time_left)
        # energy_use_est = 121.40
        # energy_use_est = 25
        #TODO: MAKE INPUTS FOR ALTITUDE AND TIMING TO CHANGE MISSION TYPE
        mission_to_land = [(105, 'descent'),
                                (45, 'transition reverse'),  # TODO: Transition reverse
                                (45, 'vertical descent'),
                                (5, 'hover'),
                                (30, 'taxi')]
        #TODO: CREATE TEMP BATTERY  MODEL AND SIM THESE POWERS/ENERGY REQUIREMENTS
        # solve this using surrogatge instead?
        temp_battery = Battery(self.battery.m, self.battery.E_total)
        temp_battery.E = self.battery.E
        for mission in mission_to_land:
            time = mission[0]
            time_hrs = time / 60 / 60  # conversion of seconds to hours
            mode = mission[1]
            power = self.calculate_power(mode)
            temp_battery.run(power, time_hrs)
        if method == 1:
            # energy_use_est = self.battery.get_energy_remaining()
            energy_use_est = self.battery.get_useful_energy_remaining()

        elif method == 2:
            # energy_use_est = temp_battery.get_energy_remaining()
            energy_use_est = temp_battery.get_useful_energy_remaining()
        else:
            energy_use_est = 0  # OTHER OPTIONS FROM ABOVE MOVE HERE
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

    def _taxi_power(self):
        power = 0.1 * self._cruise_power()
        return power

    def _vertical_climb_power(self):
        """ Estimated from helicopter theory (ex. W. Johnson, Helicopter Theory, 1980.
        ) - Momentum Theory"""
        vh = sqrt(self.DL / (2*self.density))
        power_factor = (self.ROC_TO / (2 * vh) + sqrt((self.ROC_TO/(2*vh))**2+1))
        power = self._hover_power() * power_factor
        return power

    def _climb_power(self):
        power = self.W / self.eff_climb * (self.ROC_climb + self.V_climb / self.LD_climb)
        return power

    def _vertical_descent_power(self):
        """ Estimated from helicopter theory (ex. W. Johnson, Helicopter Theory, 1980.
        ) - Momentum Theory"""
        if not -2 <= self.ROC_LD <= 0:
            warnings.warn("The desent power calculation is only valid for ROC Landing to be between "
                          "-2 and 0, it is set at {}".format(self.ROC_LD))

        vh = sqrt(self.DL / (2*self.density))
        k = [0.974, -1.125, -1.372, -1.718, -0.655]
        vivh = k[0] + sum([k[i]*(self.ROC_LD/vh)**i for i in range(1,5)])
        power_factor = (self.ROC_LD / vh + vivh)
        power = self._hover_power() * power_factor
        return power

    def _transition_forward_power(self):
        """ stability has to be preserved while avoiding any
            loss of altitude
            using theory from A. M. Kamal and A. Ramirez-Serrano, “Design methodology for hybrid
            (vtol + fixed wing) unmanned aerial vehicles,” in Aeronautics and
            Aerospace Open Access Journal, 2018."""
        self.V_transition = self.V_climb
        V_tip = self.V_transition / self.M_tip  #TODO: CHECK THIS
        solidity = self.N * self.c / (pi * self.d_rotor/2)*self.n
        mu = self.V_climb * cos(self.theta_transition) / V_tip
        Cd_transition = self.Cd
        CD_transition = self.CD
        rho = self._get_density(self.x[2])

        v_term = -(self.V_transition**2 / 2) + \
                 sqrt((self.V_transition**2 / 2)**2 + (self.W / (sin(self.theta_transition)*2*rho*self.A))**2)
        induced = self.W / (self.eff_transition * sin(self.theta_transition)) * sqrt(v_term)

        profile = rho*self.A*V_tip**3 * (solidity*Cd_transition / 8 * (1+4.6*mu**2))

        drag = 1/2 * rho * self.S * self.V_transition**2 * CD_transition

        power = induced + drag + profile
        return power #target 1431 kw

    def _transition_reverse_power(self):  #TODO: MERGE WITH OTHER TRANSITOIN WITH CHANGING VELOCITY
        self.V_transition = abs(self.V_descend)
        V_tip = self.V_transition / self.M_tip  # TODO: CHECK THIS
        solidity = self.N * self.c / (pi * self.d_rotor / 2) * self.n
        mu = self.V_climb * cos(self.theta_transition) / V_tip
        Cd_transition = self.Cd
        CD_transition = self.CD
        rho = self._get_density(self.x[2])

        v_term = -(self.V_transition ** 2 / 2) + \
                 sqrt((self.V_transition ** 2 / 2) ** 2 + (
                             self.W / (sin(self.theta_transition) * 2 * rho * self.A)) ** 2)
        induced = self.W / (self.eff_transition * sin(self.theta_transition)) * sqrt(v_term)

        profile = rho * self.A * V_tip ** 3 * (solidity * Cd_transition / 8 * (1 + 4.6 * mu ** 2))

        drag = 1 / 2 * rho * self.S * self.V_transition ** 2 * CD_transition

        power = induced + drag + profile
        return power  # target ??

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

    def _update_state(self, mode, time):
        x_last = self.x.copy()
        if mode == 'hover' or mode =='taxi':
            x = x_last.copy()
        elif mode == 'cruise':
            x = x_last.copy()
            x[0] = x_last[0] + self.V_cruise * time * cos(self.heading)
            x[1] = x_last[1] + self.V_cruise * time * sin(self.heading)
        elif mode == 'vertical climb':
            x = x_last.copy()
            x[2] = x_last[2] + self.ROC_TO * time
        elif mode == 'climb':
            x = x_last.copy()
            x[0] = x_last[0] + self.V_climb * cos(self.climb_angle) * time * cos(self.heading)
            x[1] = x_last[1] + self.V_climb * cos(self.climb_angle) * time * sin(self.heading)
            x[2] = x_last[2] + self.ROC_climb * time
        elif mode == 'descent':
            x = x_last.copy()
            x[0] = x_last[0] + self.V_climb * cos(self.climb_angle) * time * cos(self.heading)
            x[1] = x_last[1] + self.V_climb * cos(self.climb_angle) * time * sin(self.heading)
            x[2] = x_last[2] + self.ROC_descend * time
        elif mode == 'vertical descent':
            x = x_last.copy()
            x[2] = x_last[2] + self.ROC_LD * time
        elif mode == 'transition forward':
            x = x_last.copy()
            x[0] = x_last[0] + self.V_climb * time * cos(self.heading)
            x[1] = x_last[1] + self.V_climb * time * sin(self.heading)
        elif mode == 'transition reverse':
            x = x_last.copy()
            x[0] = x_last[0] + self.V_descend * time * cos(self.heading)
            x[1] = x_last[1] + self.V_descend * time * sin(self.heading)
        else:
            x = x_last.copy()
            warnings.warn("The provided mode has not been setup yet in update_state(): {}".format(mode))
        return x

    def set_hover_efficiency(self, eff):
        self.eff_hover = eff

    def get_mission(self):
        return self.mission

    def set_cruise_velocity(self, var1, var2):
        print('NOT SETUP YET')

