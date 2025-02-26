# This scrip is for the calculation to set the cubesat configuration.
# Imports
import matplotlib.pyplot as plt
import numpy as np
import control

# Parâmetros da Simulação
sim_stps = 100

# Parâmetros Roda de Reação 
R = 166.66 # [OHN]
V_RDR_Idle = 0.225 # [V]
V_RDR_Operacao = 5 # [V]
wRDR_max = 8000/9.5492965964254 # [rad/s]
wRDR_otm = 1000/9.5492965964254 # [rad/s]
alpha = 1-wRDR_otm/wRDR_max
r = 1
No = 3.84e-003 # [N.m]
N_C = 7.06e-004 # [N.m]
f = 1.21e-008*9.5492965964254 # [N.m/rad/s]
m_RDR = 0.137 # [kg]
r_RDR = 0.0435 # [m]
J_axial_RDR = 0.5*m_RDR*r_RDR**2 # [kg*m²]
alpha = 1-wRDR_otm/wRDR_max
N_C = 7.06e-004 # [N.m[]
f = 1.21e-006*9.5492965964254 # [N.m/rad/s]
# Alocando Variáveis
Xdc =  np.zeros((sim_stps)) 
N_Friccao = np.zeros((sim_stps)) # [N.m]
N_em = np.zeros((2,  sim_stps)) # [N.m]
N = np.zeros((2,  sim_stps)) # [N.m]


Ke = wRDR_otm*f*R/V_RDR_Operacao

states = ['omega_rdr']
inputs = [ 'V_in']

A_Plant = np.array([-f])

B_Plant = np.array([Ke/R])

num_inputs  = B_Plant.shape[0]
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
K_rdr = -1.0

K = np.zeros([1,num_outputs])
output_index = states.index('omega_rdr')
K[0][output_index] = K_rdr

# Define the closed-loop system state-space matrices

# Inverted because feedback is inverted!!
input_index = inputs.index('V_in')
B_V_in = B_Plant

A_SAS = A_Plant + K*B_V_in

num_outputs = A_Plant.shape[0]
C_open_loop = np.eye(num_outputs)
C_rdr = C_Plant[output_index,:]

sys_SAS = control.StateSpace(A_SAS, B_V_in, C_rdr, 0)
control.root_locus(sys_SAS)

plt.figure()
plt.title("Lugar das raízes")
poles_SAS = sys_SAS.poles()
print (poles_SAS)

print (poles_SAS/poles_Plant)

plt.plot(poles_Plant.real, poles_Plant.imag, 'b*', label=r'$Sys Open Loop$')
plt.plot(poles_SAS.real, poles_SAS.imag, 'r*', label=r'$Sys Augmented$')
plt.legend()
plt.xlabel("Imaginário")
plt.ylabel("Real")
plt.grid(True)
plt.show()
