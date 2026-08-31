# arb blade geometry calc
# assumption: the blade can be assessed as a rectangle

k_stiff_target = 3000
k_soft_target = 500

material = "Ti-6Al-4V"
e = 16000000 # young's modulus (change per material) # 29e6
sigma_y = 120000

force_applied = 85

# current geometry or new geometry
h = 0.59 # < 1
b = 0.24  # < 1
L = 4.00  # < 4

i_stiff = (b * h**3) / 12
i_soft = (h * b**3) / 12

k_stiff = (3 * e * i_stiff) / (L**3)
k_soft = (3 * e * i_soft) / (L**3)

c = h / 2
M = force_applied * L

sigma_max = (M * c) / i_stiff

FOS = sigma_y / sigma_max

print(f"\nResults for {material} blade")

print(f"Material Properties of blade:")
print(f"    Young's Modulus: {e}")
print(f"    Sigma y:         {sigma_y}")

print(f"Geometry of blade:")
print(f"    Length: {L:.3f} in")
print(f"    Width:  {b:.3f} in")
print(f"    Height: {h:.3f} in")

print(f"Stiffness of blade:")
print(f"    Stiff position: {k_stiff:.1f} lbf/in")
print(f"    Soft position:  {k_soft:.1f} lbf/in")

print(f"Stresses & Factor of Safety:")
print(f"    Max stress: {sigma_max:.1f} psi")
print(f"    FOS:        {FOS:.2f}") 