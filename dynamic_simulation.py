import sys

from pathlib import Path
from enum import Enum
import csv
import jsbsim_utils as jsbsu
import cubesat_plots as plots
import jsbsim
import navpy
import numpy as np
import time



class FlightStages(Enum):
    flight_stage_cruise = 0
    flight_stage_turning = 1
    flight_stage_falling = 2
    flight_stage_trajectory_hold = 3


def ramp_function(x0, xf, t0, delta_t, tk):
    tf = t0 + delta_t

    # Verifica se o tempo atual excede o tempo total da rampa
    if tk > t0 + delta_t:
        return xf  # Retorna o valor final se o tempo exceder a rampa
    elif tk < t0:
        return x0

    # Calcula o valor da rampa linearmente com base no tempo atual
    xk = x0 + (xf - x0) * (tk - t0) / delta_t

    return xk


root = (Path('.')).resolve()  # Define o caminho absoluto para o diretório "aircrafts"
fdm = jsbsim.FGFDMExec(
    str(root))  # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
fdm.set_debug_level(0)

# Aircraft model
aircraft = 'cubesat'
fdm.load_model(aircraft)

# Simulação
filename_con = 'fg_conn.xml'  # Arquivo de conexão com o FlightGear
filename_log = 'simulation_data.csv'  # Arquivo de conexão com o FlightGear
realtime = False
sim_dt = 0.004
sim_period = 10000

# Localização inicial: Aeroporto do Qatar, Umm Garn airport.
h0 = 34.000366848  # Elevação aproximada do terreno (m)
lat0_deg = 25.7532  # Latitude (graus)
lon0_deg = 51.2926  # Longitude (graus)

# Rampa e Missão
ramp_init_val = 0.00
ramp_final_val = np.deg2rad(-80)
ramp_time = 12.00
max_elevator_angle = np.deg2rad(18)
pitch_final_val = np.deg2rad(-70)
target_pitch = ramp_init_val

# PID
kp_alfa = -0.8
ki_alfa = -0.04
kd_alfa = -1
kd_filter_fc_alfa = 80.0



output_path = Path(
    '.').resolve() / filename_con  # Define o caminho para o arquivo de saída (fg_conn.xml na mesma pasta do script)

# Cria uma instância do FGFDMExec usando o caminho absoluto para o diretório "aircrafts"
fdm = jsbsim.FGFDMExec(str(root))

fdm.load_model(aircraft)
fdm.set_output_directive(str(output_path))
fdm.set_dt(sim_dt)  # Define o passo da simulação (s)

jsbsu.set_location(fdm, lat0_deg, lon0_deg, h0)  # Aplicar a localização inicial
jsbsu.set_ic0(fdm)

fdm['ic/h-sl-ft'] = 400000  # Define a altitude inicial (pés)
fdm['ic/phi-rad'] = 0
fdm['ic/psi-true-rad'] = 0
fdm['ic/alpha-rad'] = 0
fdm['ic/beta-rad'] = 0
fdm['ic/gamma-deg'] = 0

# Liga os motores
prop = fdm.get_propulsion()
prop.init_running(0)

# Superfícies de controle


# Potência dos motores
fdm['fcs/throttle-cmd-norm[0]'] = 1

# Abre o arquivo CSV para escrita
csvfile = open(filename_log, 'w', newline='')
writer = csv.writer(csvfile)
writer.writerow(['Time (s)',
                 'Latitude (rad)', 'Longitude (rad)', 'Altitude (m)', 'Height (m)',
                 'Pos N (m)', 'Pos E (m)', 'Pos D (m)',
                 'Vel X (m/s)', 'Vel Y (m/s)', 'Vel Z (m/s)', 'Vel Module (m/s)',
                 'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)', 'Target Pitch (rad)',
                 'Angle of Attack (rad)', 'Angle of Sideslip (rad)',
                 'Elevator CMD (rad)', 'Elevator Actuator (rad)', 'Elevator Control (deg)',
                 'Rudder CMD (rad)', 'Rudder Actuator (rad)', 'Rudder Control (deg)',
                 'Aileron CMD (rad)', 'Aileron Actuator (rad)', 'Aileron Control (deg)',
                 'Throttle CMD'])

# Variáveis do loop de simulação
dt = fdm.get_delta_t()
initial_seconds = time.time()
result = True
sim_time = fdm.get_sim_time()
frame_time = 0
frame_period = 0.03

# pid_alfa = dsp.PIDController(kp_alfa, ki_alfa, kd_alfa, kd_filter_fc_alfa, dt)

flight_stage_current = FlightStages.flight_stage_cruise

k_q = 0.656
k_theta = 1.7
cruise_duration = 60
turn_duration = 60

try:

    # Inicialize o FDM com as configurações iniciais
    fdm.run_ic()

    while result and sim_time <= sim_period:

        sim_data = jsbsu.get_simulation_data(fdm)

        lla_deg = [np.rad2deg(sim_data['lat']), np.rad2deg(sim_data['lon']), sim_data['alt']]
        xyz = navpy.lla2ned(lla_deg[0], lla_deg[1], lla_deg[2], lat0_deg, lon0_deg, h0)
        vel_module = np.sqrt(sim_data['vel_x'] ** 2 + sim_data['vel_y'] ** 2 + sim_data['vel_z'] ** 2)

        if flight_stage_current == FlightStages.flight_stage_cruise:

            op_cruise['ic/theta-rad'] = op_cruise['ic/gamma-rad'] + op_cruise['ic/alpha-rad']
            elevator_cmd = op_cruise['fcs/elevator-cmd-norm']
            elevator_cmd += k_q * fdm['velocities/q-rad_sec']
            elevator_cmd += k_theta * (fdm['attitude/theta-rad'] - op_cruise['ic/theta-rad'])
            elevator_cmd *= max_elevator_angle
            flight_stage_current = FlightStages.flight_stage_turning
            if sim_data['sim_time'] > cruise_duration:
                print('### TIME REACHED!!')
                # break

        if flight_stage_current == FlightStages.flight_stage_turning:

            aileron_cmd = op_turn['fcs/elevator-cmd-norm']
            aileron_cmd *= max_elevator_angle

            elevator_cmd = op_turn['fcs/elevator-cmd-norm']
            elevator_cmd *= max_elevator_angle

            rudder_cmd = op_turn['fcs/elevator-cmd-norm']
            rudder_cmd *= max_elevator_angle

            if sim_data['sim_time'] > turn_duration:
                flight_stage_current = FlightStages.flight_stage_falling
                print('### TIME REACHED!!')
                # break

        elif flight_stage_current == FlightStages.flight_stage_falling:

            target_pitch = ramp_function(ramp_init_val, ramp_final_val, cruise_duration, ramp_time,
                                         sim_data['sim_time'])

            elevator_cmd = op_terminal_dive['fcs/elevator-cmd-norm']
            elevator_cmd += k_q * (fdm['velocities/q-rad_sec'] - op_terminal_dive['ic/q-rad_sec'])
            elevator_cmd += k_theta * (fdm['attitude/theta-rad'] - target_pitch)
            elevator_cmd *= max_elevator_angle

            if sim_data['pitch'] < pitch_final_val:
                flight_stage_current = FlightStages.flight_stage_trajectory_hold
                print('### ANGLE REACHED!!')
                # break

        elif flight_stage_current == FlightStages.flight_stage_trajectory_hold:

            op_terminal_lock['ic/theta-rad'] = op_terminal_lock['ic/gamma-rad'] + op_terminal_lock['ic/alpha-rad']
            elevator_cmd = op_terminal_lock['fcs/elevator-cmd-norm']
            elevator_cmd += k_q * fdm['velocities/q-rad_sec']
            elevator_cmd += k_theta * (fdm['attitude/theta-rad'] - op_terminal_lock['ic/theta-rad'])
            elevator_cmd *= max_elevator_angle

        else:
            raise Exception('### ERROR: undefined flight stage!')

        fdm['fcs/elevator-cmd-norm'] = (elevator_cmd / max_elevator_angle)
        fdm['fcs/rudder-cmd-norm'] = (rudder_cmd / max_elevator_angle)
        fdm['fcs/aileron-cmd-norm'] = (aileron_cmd / max_elevator_angle)

        writer.writerow([sim_data['sim_time'],
                         sim_data['lat'], sim_data['lon'], sim_data['alt'], sim_data['height'],
                         xyz[0], xyz[1], xyz[2],
                         sim_data['vel_x'], sim_data['vel_y'], sim_data['vel_z'], vel_module,
                         sim_data['roll'], sim_data['pitch'], sim_data['yaw'], target_pitch,
                         sim_data['alpha'], sim_data['beta'],
                         elevator_cmd, sim_data['ele_act'], sim_data['ele_ctrl'],
                         rudder_cmd, sim_data['rud_act'], sim_data['rud_ctrl'],
                         aileron_cmd, sim_data['ail_act'], sim_data['ail_ctrl'],
                         fdm['fcs/throttle-cmd-norm[0]']])

        result = fdm.run()

        print(f"Time: {sim_data['sim_time']:.2f} s\
                   H: {sim_data['height']:.2f} m\
               Vel X: {sim_data['vel_x']:.2f} m/s\
             Atuador: {np.rad2deg(elevator_cmd):.2f} deg\
               Alpha: {np.rad2deg(sim_data['alpha']):.2f} deg\
               Pitch: {np.rad2deg(sim_data['pitch']):.2f} deg", end='\r', flush=True)

        if sim_data['height'] < 0.1:
            break

        if realtime:

            if sim_data['sim_time'] > frame_time:
                frame_time += frame_period
                time.sleep(frame_period)

except ValueError as ve:
    print(f"Erro de valor encontrado: {ve}")

except KeyError as ke:
    print(f"Chave não encontrada no dicionário: {ke}")

except FileNotFoundError as fe:
    print(f"Arquivo não encontrado: {fe}")

except Exception as e:
    print(f"A simulação encontrou um erro: {type(e).__name__}: {e}")

finally:

    # Fecha o arquivo de log
    csvfile.close()

    # Plota os gráficos com os dados salvos
    plots.plot_simulation_data(filename_log)