# Utilização da técnica de controle de lugar das raizes
# para parte do sistema acoplada, sistema SISO
# sendo os estado theta, dottheta
# e entradas omega_rdr_y, dotomega_rdr_y

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

# Definindo constantes
velo_varr_mini = 3  # deg/s
velo_varr_reco = 7  # deg/s
desv_regi_esta_mini = 3/60  # deg/s
desv_regi_esta_reco = 1/60  # deg/s
erro_regi_esta_mini = 0.5  # deg
erro_regi_esta_reco = 0.1  # deg
temp_esta_5_por_cent_mini = 5*60  # s
temp_esta_5_por_cent_reco = 2*60  # s
temp_esta_2_por_cent_mini = 7*60  # s
temp_esta_2_por_cent_reco = 3*60  # s
temp_subi_mini = 5*60  # s
temp_subi_reco = 2*60  # s
maxi_sobr_sina_mini = 0.5 
maxi_sobr_sina_reco = 0.25

# Cálculos para requisitos mínimos
zeta_mini = -np.log(maxi_sobr_sina_mini)/np.sqrt(np.pi**2 + (np.log(maxi_sobr_sina_mini))**2)
omega_n_mini_5_por_cent = 3/(temp_esta_5_por_cent_mini*zeta_mini)  # Corrigido: inversão da fórmula
beta_mini_5_por_cent = np.arctan(np.sqrt(1 - zeta_mini**2)/zeta_mini)  # Simplificado

# Cálculos para requisitos recomendados
zeta_reco = -np.log(maxi_sobr_sina_reco)/np.sqrt(np.pi**2 + (np.log(maxi_sobr_sina_reco))**2)
omega_n_reco_5_por_cent = 3/(temp_esta_5_por_cent_reco*zeta_reco)  # Corrigido: inversão da fórmula
beta_reco_5_por_cent = np.arctan(np.sqrt(1 - zeta_reco**2)/zeta_reco)  # Simplificado

# For a CubeSat 6U
cubeSatMass =   5.0 # [kg] 
cubeSatLength = 0.1 # [m] -> x
cubeSatWidth =  0.2 # [m] -> y
cubeSatHeight = 0.3 # [m] -> z
Iyy = cubeSatMass/12 * (cubeSatLength**2 + cubeSatHeight**2) # kg *sqm

# Parâmetros Roda de Reação 
m_RDR = 310e-3 # [kg]
r_RDR = 66e-3 # [m]
I_rdr = 0.5*m_RDR*r_RDR**2 # [kg*sqm]

# Parâmetros Orbita
r_SCGI = np.array([2.25526213722520e+006, -3.00492371279401e+006, -5.84397331427593e+006]) # m
v_SCGI = np.array([-5.19923341417592e+003, 3.82519438208177e+003, -3.97333292224794e+003]) # m/s

mu_Terra = 3.98600441e+014
[a, e, i, RAAN, omega, f] = osv2eko(r_SCGI, v_SCGI)
movimentoMedio = np.sqrt(mu_Terra/a**3) # [rad/s]
w_O_I_O = movimentoMedio

states = ['theta', 'dottheta']
inputs = [ 'dotomega_rdr_y']

A_Plant = np.array([
    [0, 1],
    [0, 0],
])

B_Plant = np.array([
    [0],
    [I_rdr/Iyy],
])

num_inputs  = B_Plant.shape[1]
num_outputs = A_Plant.shape[0]
C_Plant= np.eye(num_outputs)
D_Plant = np.zeros([num_outputs,num_inputs])

sys_Plant= control.StateSpace(A_Plant,B_Plant,C_Plant,D_Plant)
poles_Plant = sys_Plant.poles()

print("Os polos da planta em malha aberta são:")
print (poles_Plant)

plt.figure()
plt.title("Lugar das raízes")
plt.plot(poles_Plant.real, poles_Plant.imag, 'kx', label=r'$Poles \; Plant$')
plt.xlabel("Imaginário")
plt.ylabel("Real")
plt.grid(True)

#%% Control design
K_dottheta = 1.75

K = np.zeros([1,num_outputs])
output_index = states.index('dottheta')
K[0][output_index] = K_dottheta

# Define the closed-loop system state-space matrices

# Inverted because feedback is inverted!!
input_index = inputs.index('dotomega_rdr_y')
B_rdr = B_Plant[:,input_index:input_index+1]

A_Inner = A_Plant - K*B_rdr

num_outputs = A_Plant.shape[0]
C_open_loop = np.eye(num_outputs)
C_dottheta = C_Plant[output_index,:]

sys_Inner = control.StateSpace(A_Inner, B_rdr, C_dottheta, 0)
control.root_locus(sys_Inner, plot=True, xlim=[-0.03, 0], ylim=[-0.04, 0.04])
poles_malha_interna = sys_Inner.poles()

print("Os polos da planta malha interna são:")
print (poles_Plant)

K_theta = 0.5

K = np.zeros([1,num_outputs])
output_index = states.index('theta')
K[0][output_index] = K_dottheta

A_Outter = A_Inner - K*B_rdr

num_outputs = A_Plant.shape[0]
C_open_loop = np.eye(num_outputs)
C_theta = C_Plant[output_index,:]

sys_Outter = control.StateSpace(A_Outter, B_rdr, C_theta, 0)
control.root_locus(sys_Outter)
poles_malha_externa = sys_Outter.poles()
fig, ax = plt.subplots()
plt.title("Lugar das raízes")

circle_5_mini = plt.Circle((0, 0), omega_n_mini_5_por_cent*zeta_mini, color='b', fill=False)
ax.add_patch(circle_5_mini)
x_vals = np.linspace(-omega_n_mini_5_por_cent*zeta_mini, omega_n_mini_5_por_cent*zeta_mini, 100)
y_vals = np.tan(beta_mini_5_por_cent) * x_vals
ax.plot(-abs(x_vals), y_vals, color='b', linestyle='--',)

circle_5_reco = plt.Circle((0, 0), omega_n_reco_5_por_cent*zeta_reco, color='r', fill=False)
ax.add_patch(circle_5_reco)
x_vals = np.linspace(-omega_n_reco_5_por_cent*zeta_reco, omega_n_reco_5_por_cent*zeta_reco, 100)
y_vals = np.tan(beta_reco_5_por_cent) * x_vals
ax.plot(-abs(x_vals), y_vals, color='r', linestyle='--',)
# Configurar os rótulos dos eixos

plt.plot(poles_Plant.real, poles_Plant.imag, 'xb', label=r'$Sys Open Loop$')
plt.plot(poles_malha_interna.real, poles_malha_interna.imag, 'xr', label=r'$Sys Malha Interna$')
plt.plot(poles_malha_externa.real, poles_malha_externa.imag, 'xk', label=r'$Sys Malha Externa$')
ax.set_xlabel(r"Real")
ax.set_ylabel(r"Imaginário")
ax.set_xlabel(r"Real")
ax.set_ylabel(r"Imaginário")
plt.legend()
ax.grid(True)
plt.show()
