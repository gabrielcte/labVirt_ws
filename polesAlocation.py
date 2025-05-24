import matplotlib.pyplot as plt
import numpy as np
import control
from scipy import signal

def rpm2rads(rpm):
    """Converte RPM para rad/s"""
    return rpm * (2 * np.pi) / 60

def osv2eko(r_SCGI, v_SCGI):
    """Converte posição e velocidade para elementos orbitais keplerianos"""
    mu_Terra = 3.98600441e+14
    vec_k = np.array([0, 0, 1])
    vec_i = np.array([1, 0, 0])
    
    h = np.cross(r_SCGI, v_SCGI)
    p = np.dot(h, h)/mu_Terra
    B = np.cross(v_SCGI, h) - mu_Terra * r_SCGI / np.linalg.norm(r_SCGI)
    e = B/mu_Terra
    a = p/(1-np.dot(e, e))
    vec_N = np.cross(vec_k, h)
    i = np.arccos(np.dot(vec_k, h)/np.linalg.norm(h))
    RAAN = np.arccos(np.dot(vec_i, vec_N)/np.linalg.norm(vec_N))
    omega = np.arccos(np.dot(e, vec_N)/(np.linalg.norm(e)*np.linalg.norm(vec_N)))
    f = np.real(np.arccos(np.dot(r_SCGI, e)/(np.linalg.norm(e)*np.linalg.norm(r_SCGI))))
    return [a, e, i, RAAN, omega, f]

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

# Parâmetros do CubeSat 6U
cubeSatMass = 5  # [kg]
cubeSatLength, cubeSatWidth, cubeSatHeight = 0.1, 0.2, 0.3  # [m]
Ixx = 1/12 * cubeSatMass * (cubeSatWidth**2 + cubeSatHeight**2)
Iyy = 1/12 * cubeSatMass * (cubeSatLength**2 + cubeSatHeight**2)
Izz = 1/12 * cubeSatMass * (cubeSatWidth**2 + cubeSatLength**2)

# Parâmetros das Rodas de Reação
m_RDR, r_RDR = 310e-3, 66e-3  # [kg], [m]
I_rdr = 0.5 * m_RDR * r_RDR**2  # [kg*m²]

# Parâmetros Orbitais
r_SCGI = np.array([2.25526213722520e6, -3.00492371279401e6, -5.84397331427593e6])  # [m]
v_SCGI = np.array([-5.19923341417592e3, 3.82519438208177e3, -3.97333292224794e3])  # [m/s]
mu_Terra = 3.98600441e14
a, e, i, RAAN, omega, f = osv2eko(r_SCGI, v_SCGI)
w_O_I_O = np.sqrt(mu_Terra/a**3)  # [rad/s]

# Modelo de Espaço de Estados
A_Plant = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [w_O_I_O**2*(Izz-Iyy)/Ixx, 0, 0, w_O_I_O*(Ixx+Izz-Iyy)/Ixx],
    [0, -w_O_I_O**2*(Iyy-Ixx)/Izz, -w_O_I_O*(Izz-Iyy+Ixx)/Izz, 0]
])

B_Plant = np.array([
    [0, 0],
    [0, 0],
    [I_rdr/Ixx, 0],
    [0, I_rdr/Izz]
])

C_Plant = np.eye(4)
D_Plant = np.zeros((4, 2))

sys_Plant = control.StateSpace(A_Plant, B_Plant, C_Plant, D_Plant)
poles_Plant = sys_Plant.poles()

# Alocação de Polos
P1 =  -0.025+0.05j
P2 =  -0.025-0.05j
P3 =  -0.026-0.05j  
P4 =  -0.026+0.05j

P = np.array([P1, P2, P3, P4])

fsf = signal.place_poles(A_Plant, B_Plant, P, method='YT')

# Requisitos de Desempenho
velo_varr_mini = 3  # deg/s
velo_varr_reco = 7  # deg/s
maxi_sobr_sina_mini, maxi_sobr_sina_reco = 0.5, 0.25
temp_esta_5_por_cent_mini, temp_esta_5_por_cent_reco = 5*60, 2*60  # s

# Cálculos de Zeta e Omega
zeta_mini = -np.log(maxi_sobr_sina_mini)/np.sqrt(np.pi**2 + (np.log(maxi_sobr_sina_mini))**2)
omega_n_mini_5_por_cent = 3/(temp_esta_5_por_cent_mini*zeta_mini)
beta_mini_5_por_cent = np.arctan(np.sqrt(1 - zeta_mini**2)/zeta_mini)

zeta_reco = -np.log(maxi_sobr_sina_reco)/np.sqrt(np.pi**2 + (np.log(maxi_sobr_sina_reco))**2)
omega_n_reco_5_por_cent = 3/(temp_esta_5_por_cent_reco*zeta_reco)
beta_reco_5_por_cent = np.arctan(np.sqrt(1 - zeta_reco**2)/zeta_reco)

# Plot Integrado
plt.figure(figsize=(12, 6))


# Plot das Regiões de Desempenho
plt.title("Regiões de Desempenho")
circle_mini = plt.Circle((0, 0), omega_n_mini_5_por_cent*zeta_mini, color='b', fill=False, linestyle='--', label='Mínimo (5%)')
plt.gca().add_patch(circle_mini)
x_vals = np.linspace(-omega_n_mini_5_por_cent*zeta_mini, 0, 100)
y_vals = np.tan(beta_mini_5_por_cent) * x_vals
plt.plot(x_vals, y_vals, 'b--')
plt.plot(x_vals, -y_vals, 'b--')

circle_reco = plt.Circle((0, 0), omega_n_reco_5_por_cent*zeta_reco, color='r', fill=False, label='Recomendado (5%)')
plt.gca().add_patch(circle_reco)
x_vals = np.linspace(-omega_n_reco_5_por_cent*zeta_reco, 0, 100)
y_vals = np.tan(beta_reco_5_por_cent) * x_vals
plt.plot(x_vals, y_vals, 'r-')
plt.plot(x_vals, -y_vals, 'r-')

plt.plot(poles_Plant.real, poles_Plant.imag, 'kx', markersize=10, label='Polos Originais')
plt.plot(fsf.computed_poles.real, fsf.computed_poles.imag, 'bx', markersize=8, label='Polos Alocados')
plt.xlabel("Parte Real")
plt.ylabel("Parte Imaginária")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.legend()
plt.tight_layout()

# Resultados Numéricos
print("\nResultados da Análise:")
print("Polos da Planta (malha aberta):\n", poles_Plant)
print("\nPolos Alocados:\n", fsf.computed_poles)
print("\nMatriz de Ganho K:\n", fsf.gain_matrix)

plt.show()