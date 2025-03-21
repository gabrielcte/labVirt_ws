# Utilização da técnica de controle de alocação dos polos 
# para parte do sistema acoplada, sistema MIMO
# sendo os estado phi, psi, dotphi, dotpsi
# e entradas omega_rdr_x, dotomega_rdr_x e omega_rdr_z dotomega_rdr_z

import matplotlib.pyplot as plt
import numpy as np
import control
from scipy import signal


def osv2eko(r_SCGI, v_SCGI):
    mu_Terra = 3.98600441e+014
    # versores sistema de coordenadas inercial centrado na terra J2000
    vec_k = np.array([0, 0, 1])
    vec_i = np.array([1, 0, 0])
    # Elementos orbitais clássicos
    h = np.cross(r_SCGI, v_SCGI) # [m**2/s]
    p = np.dot(h, h)/mu_Terra # [m]
    B = np.cross(v_SCGI,h) - mu_Terra * r_SCGI / np.linalg.norm(r_SCGI) # [m^3/s**2]
    e = B/mu_Terra
    a = p/(1-np.dot(e, e)) # [m]
    vec_N = np.cross(vec_k, h) # [m**2/s]
    i = np.arccos(np.dot(vec_k, h)/np.linalg.norm(h))
    RAAN = np.arccos(np.dot(vec_i, vec_N)/np.linalg.norm(vec_N)) # [rad]
    omega = np.arccos(np.dot(e, vec_N)/(np.linalg.norm(e)*np.linalg.norm(vec_N))) # [rad]
    f = np.real(np.arccos(np.dot(r_SCGI, e)/(np.linalg.norm(e)*np.linalg.norm(r_SCGI)))) # [rad]
    return  [a, e, i, RAAN, omega, f]

# For a CubeSat 6U
cubeSatMass = 5 #  [kg] 
cubeSatLength = 0.1 # [m] -> x
cubeSatWidth = 0.2 # [m] -> y
cubeSatHeight = 0.3 # [m] -> z
Ixx = 1 /12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
Iyy = 1 / 12 * cubeSatMass * ((cubeSatLength ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
Izz = 1 / 12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatLength ** 2)) # slug * ft ^ 2

# Parâmetros Roda de Reação 
V_RDR_Idle = 0.225 # [V]
V_RDR_Operacao = 4.5 # [V]
wRDR_max = 8000/9.5492965964254 # [rad/s]
wRDR_otm = 1000/9.5492965964254 # [rad/s]
alpha = 1-wRDR_otm/wRDR_max
r = 1
No = 3.84e-003 # [N.m]
N_C = 7.06e-004 # [N.m]
f = 1.21e-008*9.5492965964254 # [N.m/rad/s]
m_RDR = 0.137 # [kg]
r_RDR = 0.0435 # [m]
I_rdr = 0.5*m_RDR*r_RDR**2 # [kg*m²]
alpha = 1-wRDR_otm/wRDR_max
N_C = 7.06e-004 # [N.m[]
f = 1.21e-006*9.5492965964254 # [N.m/rad/s]

# Parâmetros Orbita
r_SCGI = np.array([2.25526213722520e+006, -3.00492371279401e+006, -5.84397331427593e+006]) # m
v_SCGI = np.array([-5.19923341417592e+003, 3.82519438208177e+003, -3.97333292224794e+003]) # m/s

mu_Terra = 3.98600441e+014
[a, e, i, RAAN, omega, f] = osv2eko(r_SCGI, v_SCGI)
movimentoMedio = np.sqrt(mu_Terra/a**3) # [rad/s]
w_O_I_O = movimentoMedio

# Projeto de controle

states = ['phi', 'psi', 'dotphi', 'dotpsi']
inputs = ['dotomega_rdr_x', 'dotomega_rdr_z']

A_Plant = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [1/Ixx*w_O_I_O**2*(Izz-Iyy), 0, 0, 1/Ixx*w_O_I_O*(Ixx+Izz-Iyy)],
    [0, 1/Izz*-w_O_I_O**2*(Iyy-Ixx), 1/Izz*-w_O_I_O*(Izz-Iyy+Ixx),0]
])

B_Plant = np.array([
    [0, 0],
    [0, 0],
    [I_rdr/Ixx, 0],
    [ 0, I_rdr/Izz],
])
num_inputs  = B_Plant.shape[1]
num_outputs = A_Plant.shape[0]
C_Plant = np.eye(num_outputs)
D__Plant = np.zeros([num_outputs,num_inputs])

sys_Plant = control.StateSpace(A_Plant, B_Plant, C_Plant,D__Plant)
poles_Plant = sys_Plant.poles()



print("Os polos da planta em malha aberta são:")
print (poles_Plant)
print("Em Hz:")
print(f"{poles_Plant/(2*np.pi)} Hz")

plt.figure()
plt.title("Alocação de Polos")
plt.plot(poles_Plant.real, poles_Plant.imag, 'kx', label=r'$Poles \; Plant$')

plt.xlabel("Imaginário")
plt.ylabel("Real")
plt.grid(True)

P = np.array([
  -0.080479,
  -0.049499,
  -0.077369,
  -0.048302,
])

print(A_Plant)

fsf = signal.place_poles(A_Plant, B_Plant, P, method='YT')
plt.plot(fsf.computed_poles.real, fsf.computed_poles.imag, 'bx', label=r'$Poles \; Alocated$')
plt.legend()


print("Plant Poles")
print(poles_Plant)
print("")
print("Alocated Poles")
print(fsf.computed_poles)
print("")
print("Gain Matrix")
print(fsf.gain_matrix)
print("")

plt.show()