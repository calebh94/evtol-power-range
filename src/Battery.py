""" Battery Model """


class Battery:
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

    def get_useful_energy_remaining(self):
        return self.E - self.E_total*(1-self.DOD)

    def get_energy_used(self):
        return self.E_total - self.E

    def get_SOC(self):
        return round(self.E / self.E_total * 100, 2)  # SOC in percentage

    def _calculate_energy_use(self, power, time):
        """ Power in kWH and time in hours"""
        power_bat = power / self.eff_total
        energy_used = power_bat * time
        return energy_used
