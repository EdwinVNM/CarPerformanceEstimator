import math
#Vehicle Database
VEHICLES = {
    1: {
        "name": "992.1 Carrera 4S",
        "mass": 1570,  # kg
        "cd": 0.29,
        "frontal_area": 2.02,  # m²
        "drivetrain_efficiency": 0.83,
        "redline_rpm": 7500,
        "gear_limited_vmax": 315 / 3.6,  # m/s
        "brake_mu": 1.25
    },
    2: {
        "name": "F82 M4 Comp",
        "mass": 1600,
        "cd": 0.34,
        "frontal_area": 2.23,
        "drivetrain_efficiency": 0.85,
        "redline_rpm": 7800,
        "gear_limited_vmax": 310 / 3.6,
        "brake_mu": 1.2
    },
    3: {
        "name": "992.1 Turbo S",
        "mass": 1640,
        "cd": 0.31,
        "frontal_area": 2.05,
        "drivetrain_efficiency": 0.82,
        "redline_rpm": 7500,
        "gear_limited_vmax": 330 / 3.6,
        "brake_mu": 1.25
    }
}
#Constants
AIR_DENSITY_SEA_LEVEL = 1.225
G = 9.81
ROLLING_RESISTANCE = 0.015
DT = 0.05
#Better fuel gives better performance, most sports cars figures are based off of 93 OCTANE / 98 RON
FUEL_MAP = {
    "91": {"torque": 0.90, "power": 0.92},
    "93": {"torque": 1.00, "power": 1.00},
    "100": {"torque": 1.07, "power": 1.08},
    "E85": {"torque": 1.15, "power": 1.18}
}
#Torque Curve Model
def torque_curve(rpm, peak_torque_nm):
    """Turbo torque curve approximation"""
    if rpm < 2000:
        return peak_torque_nm * 0.6
    elif rpm < 3000:
        return peak_torque_nm * (0.6 + 0.4 * (rpm - 2000) / 1000)
    elif rpm <= 5500:
        return peak_torque_nm
    else:
        falloff = max(0.75, 1 - (rpm - 5500) / 4000)
        return peak_torque_nm * falloff
#Traction Efficiency
def traction_efficiency(kmh):
    """Speed-dependent traction loss"""
    if kmh < 50:
        return 0.85 + 0.10 * kmh / 50
    elif kmh < 150:
        return 0.95 + 0.05 * (kmh - 50) / 100
    else:
        return 1.0
#Inputs
print("Select vehicle:")
for idx, v in VEHICLES.items():
    print(f"{idx}. {v['name']}")
vehicle_idx = int(input("Choice: "))
vehicle = VEHICLES[vehicle_idx]

input_type = input("Input type (crank / wheel): ").lower()
units = input("Units (metric / imperial): ").lower()
fuel = input("Fuel (91 / 93 / 100 / E85): ")

ideal_mode = input("Ideal conditions? (y/n): ").lower() == "y"
if ideal_mode:
    elevation = 0
    temp_c = 20
else:
    elevation = float(input("Elevation (m): "))
    temp_c = float(input("Ambient temp (°C): "))

hp_input = float(input("Enter horsepower: "))
tq_input = float(input("Enter torque (Nm or ft-lb): "))
#Convert Units
if units == "imperial":
    tq_input_nm = tq_input * 1.356
else:
    tq_input_nm = tq_input

if input_type == "wheel":
    crank_hp = hp_input / vehicle["drivetrain_efficiency"]
    crank_torque = tq_input_nm / vehicle["drivetrain_efficiency"]
else:
    crank_hp = hp_input
    crank_torque = tq_input_nm
#Fuel Multiplier
peak_torque = crank_torque * FUEL_MAP[fuel]["torque"]
peak_power_hp = crank_hp * FUEL_MAP[fuel]["power"]
peak_power_watts = peak_power_hp * 745.7
#Density Altitude Correction
def air_density(elevation_m, temp_c):
    """Simplified ISA correction"""
    temp_k = temp_c + 273.15
    pressure = 101325 * (1 - 0.0000225577 * elevation_m) ** 5.25588  # Pa
    return pressure / (287.05 * temp_k)

rho_actual = air_density(elevation, temp_c)
power_correction = rho_actual / AIR_DENSITY_SEA_LEVEL
peak_power_watts *= power_correction
peak_torque *= power_correction
#Simulation
def simulate():
    v = 0
    s = 0
    t = 0
    rpm = 2500
    times = {}
    traps = {}

    while v < vehicle["gear_limited_vmax"]:
        drag = 0.5 * rho_actual * vehicle["cd"] * vehicle["frontal_area"] * v**2
        rolling = vehicle["mass"] * G * ROLLING_RESISTANCE
        resistive = drag + rolling

        rpm = min(vehicle["redline_rpm"], 2500 + v * 95)
        torque = torque_curve(rpm, peak_torque)
        power = torque * rpm * 2 * math.pi / 60
        power = min(power, peak_power_watts)

        drive_force = power / max(v, 1)
        traction_limit = vehicle["mass"] * G * 1.2
        effective_force = min(drive_force, traction_limit) * traction_efficiency(v * 3.6)

        accel = max((effective_force - resistive) / vehicle["mass"], 0)
        v += accel * DT
        s += v * DT
        t += DT

        kmh = v * 3.6

        for target in [100, 200, 300]:
            if kmh >= target and f"0-{target}" not in times:
                times[f"0-{target}"] = t

        if s >= 402 and "quarter" not in times:
            times["quarter"] = t
            traps["quarter"] = kmh
        if s >= 804 and "half" not in times:
            times["half"] = t
            traps["half"] = kmh

        if t > 120:
            break

    return times, traps, v * 3.6
#Braking
def braking_distance(v_kmh):
    v_mps = v_kmh / 3.6
    mu = vehicle["brake_mu"]
    return v_mps**2 / (2 * mu * G)
#Run
times, traps, vmax = simulate()

print("\n=== Performance ===")
for k, v in times.items():
    if k in traps:
        print(f"{k}: {v:.2f} s @ {traps[k]:.1f} km/h")
    else:
        print(f"{k}: {v:.2f} s")

print(f"Vmax: {vmax:.1f} km/h")
print("\n=== Braking ===")
print(f"100–0 km/h: {braking_distance(100):.1f} m")
print(f"200–0 km/h: {braking_distance(200):.1f} m")
