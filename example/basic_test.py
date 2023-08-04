"""
Just tests to see whether relevant bindings are readable.
"""

import digit_controller_pybind as dc

print("Digit Controller")
print(dir(dc.Digit_Controller))

# Agility LLAPI types

print("Observation")
print(dir(dc.Observation))
obs = dc.Observation()
print(f"Time: {obs.time}")
print(f"Error: {obs.error}")
print(f"Base: {obs.base}")
print(f"IMU: {obs.imu}")
print(f"Motor: {obs.motor}")
print(f"Joint: {obs.joint}")
print(f"Battery Charge: {obs.battery_charge}")

print("Limits")
print(dir(dc.Limits))
limits = dc.Limits()
print(f"Torque: {limits.torque_limit}")
print(f"Velocity: {limits.velocity_limit}")
print(f"Damping: {limits.damping_limit}")

print("Motor")
print(dir(dc.Motor))
motor = dc.Motor()
print(f"Torque: {motor.torque}")
print(f"Velocity: {motor.velocity}")
print(f"Damping: {motor.damping}")

print("Command")
print(dir(dc.Command))
command = dc.Command()
print(f"Apply command: {command.apply_command}")
print(f"Fallback opmode: {command.fallback_opmode}")
print(f"Motors: {command.motors}")