import math

# State
cur_x = -0.344
cur_y = -0.029
cur_yaw = math.radians(4.9)
cx = 0.0
cy = 0.0
speed = 0.08
k_gain = 1.0
radius = 0.3
dir_sign = 1.0
K_RADIAL = 1.0
omega_max = 2.0

ex = cx - cur_x
ey = cy - cur_y
dist_to_centre = math.hypot(ex, ey)

term = (cur_x - cx) * math.cos(cur_yaw) + (cur_y - cy) * math.sin(cur_yaw)
u = (speed + k_gain * term) / radius

radial_error = dist_to_centre - radius
effective_k_radial = K_RADIAL * (1.0 + 50.0 * abs(radial_error))
u += effective_k_radial * radial_error

omega = dir_sign * u
omega = max(-omega_max, min(omega_max, omega))

print(f"dist: {dist_to_centre:.3f}, radial_err: {radial_error:.3f}, eff_k: {effective_k_radial:.3f}")
print(f"term: {term:.3f}, u_base: {(speed + k_gain * term) / radius:.3f}, u_final: {u:.3f}, omega: {omega:.3f}")
