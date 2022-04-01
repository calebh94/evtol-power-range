""" Battery Model """
import numpy as np
import warnings


class Battery:
    def __init__(self, m: float, total_energy: float, soc_init: float, soc_limit: float):
        if type(soc_limit) == np.array:
            soc_limit = soc_limit[0]
        self.m = m
        self.E = total_energy  # Energy in kiloWatt-Hours (kWH)
        self.E_total = total_energy * (soc_init / 100)  # Energy in kiloWatt-Hours (kWH) at initial SOC
        self.soc_init = soc_init
        self.soc_limit = soc_limit
        self.bat_eff = 0.95
        self.total_eff = 0.65 # product of several efficiency (battery, electric motors, controllers, gearbox, propellers, etc.)
        self.DOD = 1.00 - (self.soc_limit / 100)  # Depth of Discharge
        self.SOH = 1.00  # State of Health #TODO: not used for now

    def calculate_usage(self, power, time):
        """ Power in kWH and time in hours"""
        energy_used = self._calculate_energy_use(power, time)
        soc_used = self.get_SOC(energy_used)  # returns the chagne in SOC since delta energy
        return energy_used, soc_used

    def run(self, power, time):
        energy_used = self._calculate_energy_use(power, time)
        self.E = self.E - energy_used
        if self.get_SOC() < self.soc_limit:
            warnings.warn('SOC is below the limit! {} < {}'.format(self.get_SOC(), self.soc_limit))
        return energy_used

    def get_energy_remaining(self):
        return self.E

    def get_useful_energy_remaining(self):
        return self.E - self.E_total*(1-self.DOD)

    def get_energy_used(self):
        return self.E_total - self.E

    def get_SOC(self, energy_used=None):
        if energy_used is None:
            energy_used = self.get_energy_remaining()
        return energy_used / self.E_total * 100  # SOC in percentage

    def _calculate_energy_use(self, power, time):
        """ Power in kWH and time in hours"""
        power_bat = power / self.total_eff
        energy_used = power_bat * time
        return energy_used
