import matplotlib.pyplot as plt
import numpy as np

def rpm2radps(rpm):
    return rpm * (2 * np.pi) / 60

def osv2eko(r_SCGI, v_SCGI):
    global mu_Terra
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

mu_Terra = 3.98600441e+014
r_SCGI = np.array([2.25526213722520e+006, -3.00492371279401e+006, -5.84397331427593e+006]) # m
v_SCGI = np.array([-5.19923341417592e+003, 3.82519438208177e+003, -3.97333292224794e+003]) # m/s
[a, e, i, RAAN, omega, f] = osv2eko(r_SCGI, v_SCGI)

movimentoMedio = np.sqrt(mu_Terra/a**3) # [rad/s]
w_O_I_O = movimentoMedio

dt = 0.01
samples = int(7*2*60/dt)

time = np.zeros((samples, 1))
phi = np.zeros((samples, 1))
theta = np.zeros((samples, 1))
psi = np.zeros((samples, 1))
dotphi = np.zeros((samples, 1))
dottheta = np.zeros((samples, 1))
dotpsi = np.zeros((samples, 1))
ddotphi = np.zeros((samples, 1))
ddottheta = np.zeros((samples, 1))
ddotpsi = np.zeros((samples, 1))
w_rdr_x = np.zeros((samples, 1))
w_rdr_y = np.zeros((samples, 1))
w_rdr_z = np.zeros((samples, 1))
dotw_rdr_x = np.zeros((samples, 1))
dotw_rdr_y = np.zeros((samples, 1))
dotw_rdr_z = np.zeros((samples, 1))
T_x = np.zeros((samples, 1))
T_y = np.zeros((samples, 1))
T_z = np.zeros((samples, 1))

phi[0] = np.deg2rad(-1)
theta[0] = np.deg2rad(-1)
psi[0] = np.deg2rad(-1)

dotphi[0] = np.deg2rad(0)
dottheta[0] = np.deg2rad(0)
dotpsi[0] = np.deg2rad(0)

# For a CubeSat 6U
cubeSatMass = 5 #  [kg] 
cubeSatLength = 0.1 # [m] -> x
cubeSatWidth =  0.2 # [m] -> y
cubeSatHeight = 0.3 # [m] -> z

Ixx = 1 / 12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatHeight ** 2))  # slug * ft ^ 2
Iyy = 1 / 12 * cubeSatMass * ((cubeSatLength ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
Izz = 1 / 12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatLength ** 2))  # slug * ft ^ 2

# For a RDR
# Parâmetros Roda de Reação 
m_RDR = 310e-3 # [kg]
r_RDR = 66e-3 # [m]
I_rdr = 0.5*m_RDR*r_RDR**2 # [kg*sqm]
w_rdr_max = rpm2radps(10000) # [rad/s]
T_rdr_max = 16e-3 # [Nm] 
dotw_rdr_max = T_rdr_max/I_rdr # [rad/sec^2]

K = np.array(
 [[ 2.52713974e-01, -2.04575076e-03,  4.09150153e+00, -2.65105846e-02],
 [-7.86827216e-04,  9.72336472e-02, -8.45709048e-02,  1.57365443e+00]]
)

K_dottheta = 1.75
K_theta =  0.25

for i in range(len(time)-1):
    time[i+1] = time[i]+dt 

    # Controle
    dotw_rdr_x[i+1] = -phi[i]*K[0][0]-psi[i]*K[0][1]-dotphi[i]*K[0][2]-dotpsi[i]*K[0][3]
    dotw_rdr_y[i+1] = -theta[i]*K_theta-dottheta[i]*K_dottheta
    dotw_rdr_z[i+1] = -phi[i]*K[1][0]-psi[i]*K[1][1]-dotphi[i]*K[1][2]-dotpsi[i]*K[1][3]

    if dotw_rdr_x[i+1]>dotw_rdr_max or dotw_rdr_y[i+1]>dotw_rdr_max or dotw_rdr_z[i+1]>dotw_rdr_max:
        print('Aceleração Máxima Ultrapassada')
        exit()

    w_rdr_x[i+1] = w_rdr_x[i]+dotw_rdr_x[i+1]*dt
    w_rdr_y[i+1] = w_rdr_y[i]+dotw_rdr_y[i+1]*dt     
    w_rdr_z[i+1] = w_rdr_z[i]+dotw_rdr_z[i+1]*dt

    if w_rdr_x[i+1]>w_rdr_max or w_rdr_y[i+1]>w_rdr_max or w_rdr_z[i+1]>w_rdr_max:
        print('Velocidade Máxima Ultrapassada')
        exit()

    T_x[i+1] = I_rdr*dotw_rdr_x[i+1]
    T_y[i+1] = I_rdr*dotw_rdr_y[i+1]
    T_z[i+1] = I_rdr*dotw_rdr_z[i+1]

    # Planta
    ddotphi[i+1] = 1/Ixx*(w_O_I_O**2*(Izz-Iyy)*phi[i]+w_O_I_O*(Ixx+Izz-Iyy)*dotpsi[i]+T_x[i+1])
    ddottheta[i+1] = T_y[i+1]/Iyy
    ddotpsi[i+1] = 1/Izz*(-w_O_I_O**2*(Iyy-Ixx)*psi[i]-w_O_I_O*(Izz-Iyy+Ixx)*dotphi[i]+T_z[i+1])

    dotphi[i+1] = dotphi[i]+ddotphi[i+1]*dt
    dottheta[i+1] = dottheta[i]+ddottheta[i+1]*dt
    dotpsi[i+1] = ddotpsi[i]+ddotpsi[i+1]*dt

    phi[i+1] = phi[i]+dotphi[i+1]*dt
    theta[i+1] = theta[i]+dottheta[i+1]*dt
    psi[i+1] = psi[i]+dotpsi[i+1]*dt 

# R01: A taxa de variação do comando da velocidade angular não deve exceder a aceleração angular máxima permitida da roda de reação.
# R02: A velocidade angular comandada não deve exceder a velocidade angular máxima permitida da roda de reação.

# R03: A precisão do apontamento deve ser de pelo menos 1 grau como limite mínimo e 0,08 graus como objetivo.

# R04: A faixa de apontamento no eixo Z deve ser de -180 a 180 graus.
# R05: A faixa de apontamento no eixo Y deve ser de -90 a 90 graus.
# R06: A faixa de apontamento no eixo X deve ser de -180 a 180 graus.

# R07: A taxa de varredura deve ser maior que 3 deg/s como limite mínimo e maior que 7 deg/s como objetivo

# R08: Após a estabilização, a taxa de desvio deve ser menor que 3 deg/min como limite mínimo e menor que 1 deg/min como objetivo.

# R09: Após a estabilização, o desvio total deve ser <= $\pm$ 0,5 graus como limite mínimo e <= $\pm$ 0,1 graus como objetivo.
# R10: O tempo de estabilização de 5%deve ser <= 5 minutos como limite mínimo e <=2 minutos como objetivo.
# R11: O tempo de estabilização de 2%deve ser <= 7 minutos como limite mínimo e <=3 minutos como objetivo.
# R12: O tempo de subida deve ser <= 5 minutos como limite mínimo e <=2 minutos como objetivo.
# R13: O percentual de ultrapassagem deve ser <= 50% como limite mínimo e <= 25% como objetivo.

velo_varr_mini = 3 # deg/s
velo_varr_reco = 7 # deg/s
desv_regi_esta_mini = 3/60 # deg/s
desv_regi_esta_reco = 1/60 # deg/s
erro_regi_esta_mini = 1 # deg
erro_regi_esta_reco = 0.08 # deg
temp_esta_5_por_cent_mini = 5*60 # s
temp_esta_5_por_cent_reco = 2*60 # s
temp_esta_2_por_cent_mini = 7*60 # s
temp_esta_2_por_cent_reco = 3*60 # s
temp_subi_mini = 5*60 # s
temp_subi_reco = 2*60 # s
maxi_sobr_sina_mini = 0.5 
maxi_sobr_sina_reco = 0.25



plt.figure()
plt.title('Evolução Aceleração Angular RDR')
plt.plot(time,np.rad2deg(dotw_rdr_x),'b',label=r'$\dot{\omega_{rdr_x}}$')
plt.plot(time,np.rad2deg(dotw_rdr_y),'r',label=r'$\dot{\omega_{rdr_y}}$')
plt.plot(time,np.rad2deg(dotw_rdr_z),'g',label=r'$\dot{\omega_{rdr_z}}$')
plt.axhline(y=np.rad2deg(dotw_rdr_max), color='k', linestyle='--', label='R01')
plt.axhline(y=-np.rad2deg(dotw_rdr_max), color='k', linestyle='--', label='R01')
# Configurar os rótulos dos eixos
plt.xlabel(r"Tempo [s]")
plt.ylabel(r"Aceleração Angular RDR [$deg/s^2$]")
# Configurar a grade e proporções iguais
plt.axis('on')
plt.legend()
plt.grid(True)

plt.figure()
plt.title('Evolução Velocidade Angular RDR')
plt.plot(time,np.rad2deg(w_rdr_x),'b',label=r'$\omega_{rdr_x}$')
plt.plot(time,np.rad2deg(w_rdr_y),'r',label=r'$\omega_{rdr_y}$')
plt.plot(time,np.rad2deg(w_rdr_z),'g',label=r'$\omega_{rdr_z}$')
plt.axhline(y=np.rad2deg(w_rdr_max), color='k', linestyle='--', label='R02')
plt.axhline(y=-np.rad2deg(w_rdr_max), color='k', linestyle='--', label='R02')
# Configurar os rótulos dos eixos
plt.xlabel(r"Tempo [s]")
plt.ylabel(r"Velocidade Angular RDR [$deg/s$]")
# Configurar a grade e proporções iguais
plt.axis('on')
plt.legend()
plt.grid(True)


# Resultado Torque por Velocidade de Rotação
plt.figure()
plt.title('Evolução Aceleração Angular')
plt.plot(time,np.rad2deg(ddotphi),'b',label=r'$\ddot{\phi}$')
plt.plot(time,np.rad2deg(ddottheta),'r',label=r'$\ddot{\theta}$')
plt.plot(time,np.rad2deg(ddotpsi),'g',label=r'$\ddot{\psi}$')
# Configurar os rótulos dos eixos
plt.xlabel(r"Tempo [s]")
plt.ylabel(r"Aceleração Angular [$deg/s^2$]")
# Configurar a grade e proporções iguais
plt.axis('on')
plt.legend()
plt.grid(True)

plt.figure()
plt.title('Evolução Velocidade Angular')
plt.plot(time,np.rad2deg(dotphi),'b',label=r'$\dot{\phi}$')
plt.plot(time,np.rad2deg(dottheta),'r',label=r'$\dot{\theta}$')
plt.plot(time,np.rad2deg(dotpsi),'g',label=r'$\dot{\psi}$')
plt.axhline(y=desv_regi_esta_reco, color='k', linestyle='--', label='R08')
plt.axhline(y=-desv_regi_esta_reco, color='k', linestyle='--', label='R08')
plt.axhline(y=desv_regi_esta_mini, color='k', linestyle='--', label='R08')
plt.axhline(y=-desv_regi_esta_mini, color='k', linestyle='--', label='R08')
# Configurar os rótulos dos eixos
plt.xlabel(r"Tempo [s]")
plt.ylabel(r"Velocidade Angular [$deg/s$]")
# Configurar a grade e proporções iguais
plt.axis('on')
plt.legend()
plt.grid(True)

plt.figure()
plt.title('Evolução Atitude')
plt.plot(time,np.rad2deg(phi),'b',label=r'$\phi$')
plt.plot(time,np.rad2deg(theta),'r',label=r'$\theta$')
plt.plot(time,np.rad2deg(psi),'g',label=r'$\psi$')
plt.axhline(y=erro_regi_esta_reco, color='k', linestyle='--', label='R03')
plt.axhline(y=-erro_regi_esta_reco, color='k', linestyle='--', label='R03')
plt.axhline(y=erro_regi_esta_mini, color='k', linestyle='--', label='R03')
plt.axhline(y=-erro_regi_esta_mini, color='k', linestyle='--', label='R03')
plt.axvline(x=temp_esta_5_por_cent_reco, color='m', linestyle='--', label='R10')
plt.axvline(x=temp_esta_5_por_cent_mini, color='m', linestyle='--', label='R10')
plt.axvline(x=temp_esta_2_por_cent_reco, color='y', linestyle='--', label='R11')
plt.axvline(x=temp_esta_2_por_cent_mini, color='y', linestyle='--', label='R11')
plt.axhline(y=0.5, color='c', linestyle='--', label='R12')
plt.axhline(y=-0.5, color='c', linestyle='--', label='R12')
plt.axhline(y=0.25, color='orange', linestyle='--', label='R13')
plt.axhline(y=-0.25, color='orange', linestyle='--', label='R13')
# Configurar os rótulos dos eixos
plt.xlabel(r"Tempo [s]")
plt.ylabel(r"Atitude [$deg$]")
# Configurar a grade e proporções iguais
plt.axis('on')
plt.grid(True)
plt.legend()
plt.show()