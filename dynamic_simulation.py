#
# dynamic_simulation
#

import jsbsim
import time
from pathlib                import Path
import numpy                as np
from enum                   import Enum
import matplotlib.pyplot    as plt
import matplotlib.image     as mpimg
import pandas               as pd

def ft2m(feet_value):
    meter_value = feet_value*0.3048
    return meter_value

def m2ft(meter_value):
    feet_value = meter_value/0.3048
    return feet_value

def Nm2Lbft(Nm):
    Lbft = Nm*0.7375621493
    return Lbft

class FlightStages(Enum):
    flight_stage_no_control           = 0
    flight_stage_control              = 1


if __name__ == '__main__':
    
    # Simulação
    realtime     = False
    num_steps = 300000
    frame_time   = 0
    dt = 0.01
    frame_period = dt
    flight_stage_current = FlightStages.flight_stage_no_control
    no_control_duration = 0
    control_duration = 10000    

    # Set jsbsim and flightgear
    aircraft_model='cubesat'
    aircraft_path=(Path('.')).resolve()

    fdm = jsbsim.FGFDMExec(str(aircraft_path))
    #fdm.set_output_directive(str(aircraft_path/'fg_conn.xml'))
    fdm.set_debug_level(0)
    fdm.load_model(aircraft_model)
    fdm.set_dt(dt)                                            # Define o passo da simulação (s)

    # Initial Conditions
    # Position
    fdm['ic/lat-geod-rad'] = np.deg2rad(-82.365)     # Latitude (rad)
    fdm['ic/long-gc-rad'] = np.deg2rad(-41.076)      # Longitude (rad)
    fdm['ic/h-sl-ft'] = m2ft(569366.9993)            # ft

    # Attitude
    fdm['ic/phi-rad'] =         np.deg2rad(15)   # Roll (rad)
    fdm['ic/theta-rad'] =       np.deg2rad(15)   # Pitch (rad)   
    fdm['ic/psi-true-rad'] =    np.deg2rad(15)  # Yaw (rad)


     # Linear Velocities
    fdm['ic/u-fps'] = m2ft(-5.19923341417592e+003)
    fdm['ic/v-fps'] = m2ft(3.82519438208177e+003)
    fdm['ic/w-fps'] = m2ft(-3.97333292224794e+003)

    # Angular Velocities
    fdm['ic/p-rad_sec'] =   np.deg2rad(0.01)                                    
    fdm['ic/q-rad_sec'] =   np.deg2rad(0.01)                                     
    fdm['ic/r-rad_sec'] =   np.deg2rad(0.01)         


    fdm.run_ic()

    # For a CubeSat 6U
    cubeSatMass = 6 #  [kg] 
    cubeSatLength = 0.1 # [m] -> x
    cubeSatWidth = 0.2 # [m] -> y
    cubeSatHeight = 0.3 # [m] -> z

    Ixx = 1 / 12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
    Iyy = 1 / 12 * cubeSatMass * ((cubeSatLength ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
    Izz = 1 / 12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatLength ** 2)) # slug * ft ^ 2

    # Parâmetros Roda de Reação
    V_RDR_Idle = 0.225 # [V]
    V_RDR_Operacao = 4.5 # [V]
    wRDR_max = 8000/9.5492965964254 # [rad/s]
    wRDR_Otima = 1000/9.5492965964254; # [rad/s]
    alpha = 1-wRDR_Otima/wRDR_max
    r = 1
    No = 3.84e-003 # [N.m]
    N_C = 7.06e-004 # [N.m]
    f = 1.21e-008*9.5492965964254 # [N.m/rad/s]
    m_rdr = 0.137 # [kg]
    r_rdr = 0.0435 # [m]
    I_rdr = 0.5*m_rdr*r_rdr**2 # [kg*m²]
    R = 166.66 # [OHN]
    Ke = wRDR_Otima*f*R/V_RDR_Operacao
    T_max = 3.2e-3 # [N*m]

    # Declarando Variáveis
    w_rdr_x = np.zeros((num_steps, 1))
    w_rdr_y = np.zeros((num_steps, 1))
    w_rdr_z = np.zeros((num_steps, 1))
    dotw_rdr_x = np.zeros((num_steps, 1))
    dotw_rdr_y = np.zeros((num_steps, 1))
    dotw_rdr_z = np.zeros((num_steps, 1))
    con_sig_dotw_rdr_x = np.zeros((num_steps, 1))
    con_sig_dotw_rdr_y = np.zeros((num_steps, 1))
    con_sig_dotw_rdr_z = np.zeros((num_steps, 1))
    T_x = np.zeros((num_steps, 1))
    T_y = np.zeros((num_steps, 1))
    T_z = np.zeros((num_steps, 1))

    # Ganhos Controlador
    K = np.array([
    [1.61301684e+00, 5.15160536e-02, 5.34167090e+01, 1.17972689e+00],
    [1.98138668e-02, 6.20578432e-01, 6.63276596e-02, 2.05448881e+01]
    ])

    K_phi_rdr_x =       -K[0][0]*0-1
    K_psi_rdr_x =       -K[0][1]*0 
    K_dotphi_rdr_x =    -K[0][2]*0-0.5
    K_dotpsi_rdr_x =    -K[0][3]*0-0.5

    K_phi_rdr_z =       -K[1][0]*0
    K_psi_rdr_z =       -K[1][1]*0-1
    K_dotphi_rdr_z =    -K[1][2]*0-0.5
    K_dotpsi_rdr_z =    -K[1][3]*0-0.5

    K_theta =           -0.5
    K_dottheta =        -1


    # Data frame
    data = []
        
    for i in range(num_steps):
        psi = fdm['attitude/psi-rad']  # Yaw
        x = np.cos(psi)
        y = np.sin(psi)
        psi_calc = np.arctan2(y, x)

        if flight_stage_current == FlightStages.flight_stage_no_control:

            if fdm.get_sim_time() > no_control_duration:
                print('ACTIVATE CONTROL')
                flight_stage_current = FlightStages.flight_stage_control
                #break

        elif flight_stage_current == FlightStages.flight_stage_control:

            # Controle

            con_sig_dotw_rdr_x[i] = K_phi_rdr_x*fdm['attitude/phi-rad']+K_psi_rdr_x*psi_calc+K_dotphi_rdr_x*fdm['velocities/phidot-rad_sec']+K_dotpsi_rdr_x*fdm['velocities/psidot-rad_sec']
            con_sig_dotw_rdr_y[i] = K_theta*fdm['attitude/theta-rad']+K_dottheta*fdm['velocities/thetadot-rad_sec']  
            con_sig_dotw_rdr_z[i] = K_phi_rdr_z*fdm['attitude/phi-rad']+K_psi_rdr_z*psi_calc+K_dotphi_rdr_z*fdm['velocities/phidot-rad_sec']+K_dotpsi_rdr_z*fdm['velocities/psidot-rad_sec']
            
            # ==== Adicionando dinâmica do atuador ===== #            
            constante_tempo = 1/8.31849893e-05 /6 # => 6x a planta
            ganho_unit = 1 # de angulo da tubeira para angulo da tubeira
            dotw_rdr_x[i] = ganho_unit*dt*(con_sig_dotw_rdr_x[i]+con_sig_dotw_rdr_x[i-1])/(2*constante_tempo+dt)+dotw_rdr_x[i-1]
            dotw_rdr_y[i] = ganho_unit*dt*(con_sig_dotw_rdr_y[i]+con_sig_dotw_rdr_y[i-1])/(2*constante_tempo+dt)+dotw_rdr_y[i-1]
            dotw_rdr_z[i] = ganho_unit*dt*(con_sig_dotw_rdr_z[i]+con_sig_dotw_rdr_z[i-1])/(2*constante_tempo+dt)+dotw_rdr_z[i-1]
            # ========================================== #

            w_rdr_x[i] = w_rdr_x[i]+dotw_rdr_x[i]*dt
            w_rdr_y[i] = w_rdr_y[i]+dotw_rdr_y[i]*dt     
            w_rdr_z[i] = w_rdr_z[i]+dotw_rdr_z[i]*dt


            # ==== Saturação atuador ===== #
            if abs(w_rdr_x[i])>wRDR_max:
                w_rdr_x[i] = np.sign(w_rdr_x[i])*wRDR_max

            if abs(w_rdr_y[i])>wRDR_max:
                w_rdr_y[i] = np.sign(w_rdr_y[i])*wRDR_max

            if abs(w_rdr_z[i])>wRDR_max:
                w_rdr_z[i] = np.sign(w_rdr_z[i])*wRDR_max
            # ============================ #
            


            T_x[i] = dotw_rdr_x[i]*I_rdr-f*w_rdr_x[i] 
            T_y[i] = dotw_rdr_y[i]*I_rdr-f*w_rdr_y[i] 
            T_z[i] = dotw_rdr_z[i]*I_rdr-f*w_rdr_z[i]



            if abs(T_x[i])>T_max:
                T_x[i] = np.sign(T_x[i])*T_max

            if abs(T_y[i])>T_max:
                T_y[i+1] = np.sign(T_y[i])*T_max

            if abs(T_z[i])>T_max:
                T_z[i] = np.sign(T_z[i])*T_max 

            # Aplicação do torque de controle na planta
            fdm['actuator/RDR-x'] =  T_x[i]
            fdm['actuator/RDR-y'] =  T_y[i]
            fdm['actuator/RDR-z'] = -T_z[i]

        if fdm.get_sim_time() > control_duration:
            break                                            
            
       
        new_data = {'sim-time-sec' : fdm.get_sim_time(),
                    'position/lat-geod-deg' : fdm['position/lat-geod-deg'],
                    'position/long-gc-deg' : fdm['position/long-gc-deg'],
                    'position/geod-alt-ft' : fdm['position/geod-alt-ft'],                    
                    'attitude/phi-rad' : fdm['attitude/phi-rad'],
                    'attitude/theta-rad' : fdm['attitude/theta-rad'],   
                    'attitude/psi-rad' : psi_calc,
                    'velocities/p-rad_sec' : fdm['velocities/p-rad_sec'],
                    'velocities/q-rad_sec': fdm['velocities/q-rad_sec'],
                    'velocities/r-rad_sec': fdm['velocities/q-rad_sec'],
                    'velocities/phidot-rad_sec' : fdm['velocities/phidot-rad_sec'],                                
                    'velocities/thetadot-rad_sec' : fdm['velocities/thetadot-rad_sec'],                                     
                    'velocities/psidot-rad_sec' : fdm['velocities/psidot-rad_sec'],
                    }
        
        data.append(new_data)       


        print(f"Time: {fdm.get_sim_time():.2f} s\
                Atuator X: {w_rdr_x[i]} rad/sec\
                Atuator Y: {w_rdr_y[i]} rad/sec\
                Atuator Z: {w_rdr_z[i]} rad/sec", end='\r', flush=True)
        
        fdm.run()

        if realtime:
            
            if fdm.get_sim_time() > frame_time:
                frame_time += frame_period
                time.sleep(frame_period)

    df = pd.DataFrame(data, columns=[
                    'sim-time-sec',
                    'position/lat-geod-deg',
                    'position/long-gc-deg',
                    'position/geod-alt-ft',                        
                    'attitude/phi-rad',
                    'attitude/theta-rad',   
                    'attitude/psi-rad',
                    'velocities/p-rad_sec',
                    'velocities/q-rad_sec',
                    'velocities/r-rad_sec',
                    'velocities/phidot-rad_sec',                                
                    'velocities/thetadot-rad_sec',                                     
                    'velocities/psidot-rad_sec',
            ])   

    plt.figure(1)
    plt.title('Ground Track')
    plt.plot(df['position/long-gc-deg'], df['position/lat-geod-deg'],':b')

    # Ler a imagem

    contour = mpimg.imread('mapaContorno.jpg')
    # Configurar a exibição da imagem

    plt.imshow(contour, extent=[-180, 180, -90, 90], cmap='gray')

    # Configurar os rótulos dos eixos
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")

    # Definir limites dos eixos
    plt.xlim([-180, 180])
    plt.ylim([-90, 90])

    # Definir as marcações dos eixos x
    plt.xticks([-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 180],
            ['180W', '150W', '120W', '90W', '60W', '30W', '0', '30E', '60E', '90E', '120E', '150E', '180E'])

    # Definir as marcações dos eixos y
    plt.yticks([90, 75, 60, 45, 30, 15, 0, -15, -30, -45, -60, -75, -90],
            ['90N', '75N', '60N', '45N', '30N', '15N', '0', '15S', '30S', '45S', '60S', '75S', '90S'])

    # Configurar a grade e proporções iguais
    plt.axis('on')
    plt.axis('equal')
    plt.grid(True)


    plt.figure(2)
    # Plotar as curvas de atitude
    plt.plot(df['sim-time-sec'], np.rad2deg(df['attitude/phi-rad']), 'b', label=r'$\phi$')
    plt.plot(df['sim-time-sec'], np.rad2deg(df['attitude/theta-rad']), 'r', label=r'$\theta$')
    plt.plot(df['sim-time-sec'], np.rad2deg(df['attitude/psi-rad']), 'g', label=r'$\psi$')
    plt.legend()
    plt.xlabel('Time [sec]')
    plt.ylabel('Attitude [Deg]')
    plt.title('Evolução da Atitude do Cubesat 6U')
    plt.grid()

    plt.figure(3)
    plt.plot(df['sim-time-sec'], np.rad2deg(df['velocities/phidot-rad_sec']),'b', label=r'$\dot{\phi}$')
    plt.plot(df['sim-time-sec'], np.rad2deg(df['velocities/thetadot-rad_sec']),'r', label=r'$\dot{\theta}$')
    plt.plot(df['sim-time-sec'], np.rad2deg(df['velocities/psidot-rad_sec']),'g', label=r'$\dot{\psi}$')
    plt.legend()
    plt.xlabel('Time [sec]')
    plt.ylabel('Velocidade Angular [Deg/sec]')
    plt.title('Velocidade Angular do Cubesat 6U')
    plt.grid()



    # Mostrar a imagem
    plt.show()

    # Salva data frame como csv
    df.to_csv('cubesat_dataframe.csv', index=False)
