##################### BASIC IDEA #####################
# We will use the numerical method RK4 to solve for forces.
# Using derived equations of motion, interate through a drop scenario of set initial velocity.
# We can manually set initial drop velocity if we know hight of drop and weight of car.

from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
import math

# %%  VARIABLES AND PARAMETERS 

# unit funcitons 
g = 32.174 # ft/s^2
def lbf_to_slugs(x): return x / 32.174
def in_to_ft(x): return x / 12.0
def lbf_in_to_lbf_ft(x): return 12.0 * x
def mph_to_fts(x): return x * 1.4666666667
def lbs_to_kg(x): return x * 0.453592

# weight calcs
total_vehicle_weight     = 510.0  # fully loaded (driver, gas, etc.) designed = 520.0
rear_weight_distribution = 0.515  # designed = 0.55, actual = 0.515
corner_unsprung_weight   = 29.18  # designed = 16.0, actual = 29.18
# corner_total_weight      = (total_vehicle_weight * rear_weight_distribution) / 2.0
corner_total_weight      = 118+27 # sprung + unsprung
# corner_sprung_weight     = corner_total_weight - corner_unsprung_weight
corner_sprung_weight     = 118
# corner_sprung_weight     = 125    # hardcoded because something was off

corner_gs = 0.65 # unitless
corner_force = (lbs_to_kg(corner_total_weight))*(9.81)*corner_gs
# print("cornering force: ", corner_force)

# hardcoded values 
mph = 25.0                                     # not currently used for anything
longitudinal_velocity_fts = mph_to_fts(mph)    # longitudinal velocity; ft/s
m_s = lbf_to_slugs(corner_sprung_weight)       # mass sprung; INPUT IN --> weight | output is slugs
m_u = lbf_to_slugs(corner_unsprung_weight)     # mass unsprung; INPUT IN --> weight | output is slugs
k_t = 4800.0                                   # tire stiffness; lbf/ft; might be 7800.0(?)
damp_ratio = 0.7                               # damping ratio; unitless
ride_freq = 1.5

# motion ratio calcs (specific for trailing arm system)
dim_a = 18.0 # inches
dim_b = 24.0 # inches
spring_angle = 60 # degrees

m_r = (dim_a/dim_b)*math.sin(math.radians(spring_angle))

#  wheel_rate = 0.102236*corner_sprung_weight*ride_freq**2 # dont know what that hardcoded value is (never used in calcs)

spring_rate_ft = 100*12 # designed = 90, actual = 100

c = damp_ratio*2*(math.sqrt(spring_rate_ft*m_s)) # damping coefficient; (lbf*s)/ft

k_s = spring_rate_ft*(m_r**2)  # effective spring stiffness -> motion ratio applied to the spring 
c_d = c*(m_r**2)               # effective damper stiffness -> motion ratio applied to the damper

''' debugging above parameters
print(f"motion ratio: {m_r:.2f}")
print(f"damping coeffient: {c:.2f}")
print(f"wheel_rate: {wheel_rate:.2f}")
print(f"spring_rate_ft: {spring_rate_ft:.2f}")
print(f"effective spring stiffness: {k_s:.2f}")
'''
# %%  CALCULATIONS 

# drop height input
print("")
drop_h = float(input("Enter desired drop height: "))
v0 = math.sqrt(2.0*g*drop_h)

# ground profile (function for more profiles later on)
def road_profile(t):
    return 0.0

# EOMs 
def quarter_car_calc (t, y): 
    zs, vs, zu, vu = y # EOMs; z variables are postions of masses; v variables are velocites of masses
    zg = road_profile(t)
    
    tire_deflection = zg - zu # only apply tire force if tire is in contact with ground
    if tire_deflection >= 0:
        f_tire = k_t*tire_deflection
    else:
        f_tire = 0.0

    d1_zs = vs
    d2_zs = (k_s*(zu-zs)+c_d*(vu-vs))/m_s - g
    
    d1_zu = vu
    d2_zu = (-k_s*(zu-zs)-c_d*(vu-vs)+f_tire)/m_u - g
    
    return [d1_zs, d2_zs, d1_zu, d2_zu]

# initial conditions
z_s0 = drop_h + (4/12) # sprung mass starts higher for length of suspension
v_s0 = -v0
z_u0 = drop_h
v_u0 = -v0

y0 = [z_s0, v_s0, z_u0, v_u0]

# simulation span
t_span = (0, 2)
t_eval = np.linspace(0, 2, 1000)

# solving
print("")
print("Running simulation...")
sol = solve_ivp(quarter_car_calc, t_span, y0, t_eval=t_eval, 
                method='RK45', dense_output=True, max_step=0.01) # 100 Hz --> sample at 200 Hz b/c of Nyquist

# results
t  = sol.t
zs = sol.y[0]  # sprung mass position
vs = sol.y[1]  # sprung mass velocity
zu = sol.y[2]  # unsprung mass position
vu = sol.y[3]  # unsprung mass velocity

# forces throughout simulation
spring_force = k_s * (zu - zs)
damper_force = c_d * (vu - vs)
tire_force = np.where(zu <= road_profile(t), k_t * (road_profile(t) - zu), 0)

# net forces
net_force_sprung = spring_force + damper_force
net_force_unsprung = tire_force - spring_force - damper_force
force_through_wheel = tire_force

# max forces
max_force_through_wheel = np.max(force_through_wheel)
max_net_unsprung = np.max(np.abs(net_force_unsprung))
max_net_sprung = np.max(np.abs(net_force_sprung))

# %%  OUTPUT/RESULTS 

print("")

print("================== Impact ==================")
print(f"Drop height: {drop_h} ft")
print(f"Impact velocity: {v0:.2f} ft/s ({v0/1.467:.1f} mph)")
print("Maximum Forces:")
print(f"  Through Wheel: {max_force_through_wheel:.1f} lbf")
print(f"  Unsprung Mass: {max_net_unsprung:.1f} lbf")
print(f"  Sprung Mass:   {max_net_sprung:.1f} lbf")
print(" ")
print("G-forces:")
print(f"  Unsprung mass: {max_net_unsprung/corner_unsprung_weight:.2f} g")
print(f"  Sprung mass:   {max_net_sprung/corner_sprung_weight:.2f} g")
print(" ")
print(" ")
print(" ")

# position plot
plt.figure(figsize=(10, 6))
plt.plot(t, zs, label='Sprung mass (zs)', linewidth=2)
plt.plot(t, zu, label='Unsprung mass (zu)', linewidth=2)
plt.axhline(y=0, color='k', linestyle='-', label='Contact Surface')
plt.title('Quarter Car Drop Test Simulation - Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (ft)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# velocity plot
plt.figure(figsize=(10, 6))
plt.plot(t, vs, label='Sprung mass velocity', linewidth=2)
plt.plot(t, vu, label='Unsprung mass velocity', linewidth=2)
plt.title('Quarter Car Drop Test Simulation - Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (ft/s)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# force plot
plt.figure(figsize=(10, 6))
plt.plot(t, force_through_wheel, label='Force Through Wheel', linewidth=2.5, color='green')
plt.plot(t, net_force_unsprung, label='Net Force on Unsprung Mass', linewidth=2, color='orange')
plt.plot(t, net_force_sprung, label='Net Force on Sprung Mass', linewidth=2, color='blue')
plt.title('Quarter Car Drop Test Simulation - Forces')
plt.xlabel('Time (s)')
plt.ylabel('Force (lbf)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

