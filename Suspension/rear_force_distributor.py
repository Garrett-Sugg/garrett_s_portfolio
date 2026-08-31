# force distribution calc 
# coordinate system is the same as Lotus & CAD (+x forward, -y right, +z up) on right hand

import numpy as np
import math as math
import matplotlib.pyplot as plt

def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("Error: Zero-length vector")
    return v / n

def rotate_force(F_wheel, yaw, pitch, roll):
    F_wheel = np.asarray(F_wheel, dtype = float)
    sin = np.sin; cos = np.cos
    
    # negative on the gamma component to adjust the rotation matrix
    alpha = math.radians(yaw)
    beta = math.radians(pitch)
    gamma = -math.radians(roll) 
    
    if None not in (alpha, beta, gamma): 
        R = np.array([
            [
                cos(alpha)*cos(beta),
                cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma),
                cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma)
            ],
            [
                sin(alpha)*cos(beta),
                sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma),
                sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma)
            ],
            [
                -sin(beta),
                cos(beta)*sin(gamma),
                cos(beta)*cos(gamma)
            ]
        ])
    else:
        raise ValueError("Error: Value of rotation missing")
        
    return R @ F_wheel # @ is short for matrix multiplication

def equilibrium(F_wheel, rAB, rAC, rAD, rAE, dD, dU, dL):
    F = np.asarray(F_wheel, float)
    dD = unit(dD); dU = unit(dU); dL = unit(dL)

    # force balance rows
    A_top = np.c_[np.eye(3), dU.reshape(3,1), dL.reshape(3,1), dD.reshape(3,1)]
    b_top = -F

    # moment balance rows (about A)
    colU = np.cross(rAC, dU)
    colL = np.cross(rAD, dL)
    colD = np.cross(rAB, dD)
    A_bot = np.c_[np.zeros((3,3)), colU.reshape(3,1), colL.reshape(3,1), colD.reshape(3,1)]
    b_bot = -np.cross(rAE, F)

    A = np.r_[A_top, A_bot]
    b = np.r_[b_top, b_bot]
    return A, b

def solve(F_wheel, A, B_arm, C_arm, D_arm, E, B_ch, C_ch, D_ch):
    # moment arms from pivot
    rAB = B_arm - A; rAC = C_arm - A; rAD = D_arm - A; rAE = E - A

    # member axes (tension is +) damper, upper link, lower link (respectivly)
    dD = unit(B_ch - B_arm); dU = unit(C_ch - C_arm); dL = unit(D_ch - D_arm)  

    M, rhs = equilibrium(F_wheel, rAB, rAC, rAD, rAE, dD, dU, dL)
    x = np.linalg.solve(M, rhs)  

    FT = x[0:3] # pivot reaction [Fx,Fy,Fz]
    fU, fL, fD = x[3], x[4], x[5]  # scalars

    F_U = fU * dU
    F_L = fL * dL
    F_D = fD * dD

    res_F = FT + F_U + F_L + F_D + np.asarray(F_wheel)
    res_M = (np.cross(rAB, F_D) + np.cross(rAC, F_U) +
             np.cross(rAD, F_L) + np.cross(rAE, F_wheel))

    return {
        "F_pivot": FT,
        "f_upper": fU, "F_upper": F_U,
        "f_lower": fL, "F_lower": F_L,
        "f_damper": fD,"F_damper": F_D,
        "residual_force": res_F,
        "residual_moment": res_M,
        "A_matrix": M, "b_vec": rhs,
        "dirs": {"dU": dU, "dL": dL, "dD": dD},
    }

F_wheel_drop = np.array([0.0, 0.0, 1799.9], dtype = float)
corner_force = 413.6031024840001 # calcualted outside of this script
# negative on the y component to adjust for the correct input force
F_wheel_corner = np.array([0.0, -corner_force, 0.0], dtype = float)

###################### test inputs #####################
F_test_x = np.array([1799.9, 0.0, 0.0], dtype = float)
F_test_y = np.array([0.0, 1799.9, 0.0], dtype = float)
########################################################

scenario = str(input("Input which scenario you would like to analyze (drop or corner): "))
yaw, pitch, roll = map(float, input("Enter desired yaw, pitch, and roll (relative to the initial vector): ").split())

if scenario == 'drop':
    F_base = F_wheel_drop
elif scenario == 'corner':
    F_base = F_wheel_corner
elif scenario == 'testx':
    F_base = F_test_x
elif scenario == 'testy':
    F_base = F_test_y
else:
    raise ValueError("Error: Invalid scenario")

F_applied = rotate_force(F_base, yaw, pitch, roll)

A     = np.array([-26.53,    14.68,     5.59])  # Trailing-arm chassis pivot

B_ch  = np.array([-5.58,      9.51,    16.27])  # Damper upper (CHASSIS)
B_arm = np.array([-9.25,     19.60,     2.76])  # Damper lower (ARM)

C_ch  = np.array([0.63,       3.00,     5.10])  # Upper link inboard (CHASSIS)
C_arm = np.array([-0.88,     26.00,     2.37])  # Upper link outboard (ARM)

D_ch  = np.array([0.63,       3.35,     1.70])  # Lower link inboard (CHASSIS)
D_arm = np.array([-0.88,     24.50,    -2.75])  # Lower link outboard (ARM)

E     = np.array([-1.75,     24.00,    -0.50])  # Wheel spindle (inboard)

if __name__ == "__main__":
    out = solve(F_applied, A, B_arm, C_arm, D_arm, E, B_ch, C_ch, D_ch)
    
    np.set_printoptions(precision=3, suppress=True)
    print("\n=========== Force Distribution ===========")
    print("Baseline impact force =", F_base)
    print("Rotated impact force  =", F_applied)
    print("\nPivot reaction:", out["F_pivot"])
    print("\nAxial loads (tension +):")
    print("  Upper link | fU =", out["f_upper"])
    print("  Lower link | fL =", out["f_lower"])
    print("  Damper     | fD =", out["f_damper"])
    print("\nMember 3D forces (Lotus):")
    print("  Upper link | F_upper  =", out["F_upper"])
    print("  Lower link | F_lower  =", out["F_lower"])
    print("  Damper     | F_damper =", out["F_damper"])
    print("\nEquilibrium residuals (≈0):")
    print("  Force  =", out["residual_force"])
    print("  Moment =", out["residual_moment"])
    
    # for visualization, imagine the iso view of the Lotus model
    # with the values on the y axis reversed (math is handled in the calc)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.quiver(0, 0, 0, 
        F_base[0], F_base[1], F_base[2], # original vector
        color='b', 
        arrow_length_ratio=0.2)
    
    ax.quiver(0, 0, 0, 
        F_applied[0], F_applied[1], F_applied[2], # rotated vector
        color='r',
        arrow_length_ratio=0.2)
    
    max_val = max(abs(F_base))
    
    ax.set_xlim(-max_val, max_val)
    ax.set_ylim(-max_val, max_val)
    ax.set_zlim(-max_val, max_val)
    
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    zmin, zmax = ax.get_zlim()
    
    ax.plot([-max_val, max_val], [0, 0], [0, 0], color = 'black') # x axis
    ax.plot([0, 0], [-max_val, max_val], [0, 0], color = 'black') # y axis
    ax.plot([0, 0], [0, 0], [-max_val, max_val], color = 'black') # z axis

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Original impact vector vs Rotated impact vector")
    
    plt.show()

