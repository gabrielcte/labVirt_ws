'''
R01: A taxa de variação do comando da velocidade angular não deve exceder a aceleração angular máxima permitida da roda de reação.
R02: A velocidade angular comandada não deve exceder a velocidade angular máxima permitida da roda de reação.
R03: A precisão do apontamento deve ser de pelo menos 1 grau como limite mínimo e 0,08 graus como objetivo.
R04: A faixa de apontamento no eixo Z deve ser de -180 a 180 graus.
R05: A faixa de apontamento no eixo Y deve ser de -90 a 90 graus.
R06: A faixa de apontamento no eixo X deve ser de -180 a 180 graus.
R07: A taxa de varredura deve ser maior que 3 deg/s como limite mínimo e maior que 7 deg/s como objetivo.
R08: Após a estabilização, a taxa de desvio deve ser menor que 3 deg/min como limite mínimo e menor que 1 deg/min como objetivo.
R09: Após a estabilização, o desvio total deve ser <= ±0,5 graus como limite mínimo e <= ±0,1 graus como objetivo.
R10: O tempo de estabilização de 5% deve ser <= 5 minutos como limite mínimo e <=2 minutos como objetivo.
R11: O tempo de estabilização de 2% deve ser <= 7 minutos como limite mínimo e <=3 minutos como objetivo.
R12: O tempo de subida deve ser <= 5 minutos como limite mínimo e <=2 minutos como objetivo.
R13: O percentual de ultrapassagem deve ser <= 50% como limite mínimo e <= 25% como objetivo.
'''

import numpy as np
import matplotlib.pyplot as plt

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
fig, ax = plt.subplots()
plt.title('Pole Placing')

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
ax.set_xlabel(r"Real")

ax.set_ylabel(r"Imaginário")
# Configurar a grade e proporções iguais
ax.set_xlim([-0.1, 0])

ax.grid(True)

P1 =  -0.025+0.05j
P2 =  -0.025-0.05j
P3 =  -0.026-0.05j  
P4 =  -0.026+0.05j

ax.plot(P1.real, P1.imag, 'xk')
ax.plot(P2.real, P2.imag, 'xk')
ax.plot(P3.real, P3.imag, 'xk')
ax.plot(P4.real, P4.imag, 'xk')
ax.set_aspect("equal")
# Exibir o gráfico
plt.show()