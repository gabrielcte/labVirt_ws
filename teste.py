import sys
sys.path.insert(0, '.\\aerospace_ctrl_toolkit')
import matplotlib.pyplot as plt
from pathlib           import Path
from enum              import Enum
import csv
import dsp           
import jsbsim_utils    as jsbsu
import aatd_plots      as plots   
import jsbsim 
import navpy
import numpy           as np
import time
import trim
import pandas as pd

class FlightStages(Enum):
    flight_stage_cruise          = 0
    flight_stage_falling         = 1
    flight_stage_trajectory_hold = 2

root = (Path('.')).resolve()              # Define o caminho absoluto para o diretório "aircrafts"
fdm  = jsbsim.FGFDMExec(str(root))        # The path supplied to FGFDMExec is the location of the folders "aircraft", "engines" and "systems"
fdm.set_debug_level(0)


# Aircraft model
aircraft = 'AATD_Config010'

fdm.load_model(aircraft)

# Loads trim settings
op_cruise = trim.trim_wings_level_flight(
    fdm = fdm,
    ic_h_sl_ft = 10000,
    ic_mach = 0.7,
    fuel_content=1000
)


# Simulação
filename_con = 'fg_conn.xml'                                     # Arquivo de conexão com o FlightGear
filename_log = 'simulation_data.csv'                             # Arquivo de conexão com o FlightGear
realtime     = False
sim_dt       = 0.004
sim_period   = 10

# Localização inicial: Aeroporto do Qatar, Umm Garn airport.
h0        = 34.000366848                                         # Elevação aproximada do terreno (m)
lat0_deg  = 25.7532                                              # Latitude (graus)
lon0_deg  = 51.2926                                              # Longitude (graus)

# Trimming values
h_sl_ft_0    = op_cruise['ic/h-sl-ft']
v_mach_0     = op_cruise['ic/mach']
phi0         = op_cruise['ic/phi-rad']
psi0         = op_cruise['ic/psi-true-rad']
alpha0       = op_cruise['ic/alpha-rad']
beta0        = op_cruise['ic/beta-rad']
gamma0       = op_cruise['ic/gamma-rad']     
aileron_cmd  = op_cruise['fcs/aileron-cmd-norm'] 
elevator_cmd = op_cruise['fcs/elevator-cmd-norm']
rudder_cmd   = op_cruise['fcs/rudder-cmd-norm']
throttle_cmd = op_cruise['fcs/throttle-cmd-norm[0]']

output_path = Path('.').resolve()/filename_con                   # Define o caminho para o arquivo de saída (fg_conn.xml na mesma pasta do script)

# Cria uma instância do FGFDMExec usando o caminho absoluto para o diretório "aircrafts"
fdm = jsbsim.FGFDMExec(str(root))

fdm.load_model(aircraft)
fdm.set_output_directive(str(output_path))
fdm.set_dt(sim_dt)                                               # Define o passo da simulação (s)

jsbsu.set_location(fdm, lat0_deg, lon0_deg, h0)                  # Aplicar a localização inicial
jsbsu.set_ic0(fdm)

fdm['ic/h-sl-ft']      = h_sl_ft_0                               # Define a altitude inicial (pés)
fdm['ic/mach']         = v_mach_0
fdm['ic/phi-rad']      = phi0
fdm['ic/psi-true-rad'] = psi0
fdm['ic/alpha-rad']    = alpha0
fdm['ic/beta-rad']     = beta0
fdm['ic/gamma-deg']    = gamma0

# Liga os motores
prop = fdm.get_propulsion()
prop.init_running(0)

# Superfícies de controle 
fdm['fcs/aileron-cmd-norm']  = (aileron_cmd)
fdm['fcs/elevator-cmd-norm'] = (elevator_cmd)
fdm['fcs/flap-cmd-norm']     = 0
fdm['fcs/rudder-cmd-norm']   = (rudder_cmd)

# Potência dos motores
fdm['fcs/throttle-cmd-norm[0]'] = throttle_cmd
data = []



# Variáveis do loop de simulação
dt              = fdm.get_delta_t()
initial_seconds = time.time()
result          = True
sim_time        = fdm.get_sim_time()
frame_time      = 0
frame_period    = 0.03

#pid_alfa = dsp.PIDController(kp_alfa, ki_alfa, kd_alfa, kd_filter_fc_alfa, dt)

flight_stage_current = FlightStages.flight_stage_cruise

k_q = 0.656
k_theta = 1.7
cruise_duration = 10

try:

    # Inicialize o FDM com as configurações iniciais
    fdm.run_ic()
    while fdm.get_sim_time() <= sim_period:

        new_data = {'sim-time-sec': fdm.get_sim_time(),}
        data.append(new_data)

        fdm.run()

                
except ValueError as ve:
    print(f"Erro de valor encontrado: {ve}")

except KeyError as ke:
    print(f"Chave não encontrada no dicionário: {ke}")

except FileNotFoundError as fe:
    print(f"Arquivo não encontrado: {fe}")

except Exception as e:
    print(f"A simulação encontrou um erro: {type(e).__name__}: {e}")

finally:


    df = pd.DataFrame(data, columns=['sim-time-sec',
                    'position/lat-geod-rad', 'position/long-gc-rad', 'position/geod-alt-ft', 'position/h-agl-ft',
                    'velocities/u-fps', 'velocities/v-fps', 'velocities/w-fps',
                    'attitude/phi-rad', 'attitude/theta-rad', 'attitude/psi-true-rad',
                    'aero/alpha-rad', 'aero/beta-rad',
                    'fcs/elevator-cmd-norm', 'fcs/elevator-actuator', 'fcs/elevator-control',
                    'fcs/rudder-cmd-norm', 'fcs/rudder-actuator', 'fcs/rudder-control',
                    'fcs/aileron-cmd-norm', 'fcs/aileron-actuator', 'fcs/aileron-actuator',
                    'fcs/throttle-cmd-norm[0]'])

        # Plota os gráficos com os dados salvos
    df.to_csv('teste_dataframe.csv', index=False)

df2 = pd.read_csv('teste_dataframe.csv')

plt.figure(figsize=(10, 5))
plt.plot(df2['sim-time-sec'])
plt.title('Plot da sim_time')
plt.xlabel('Índice')
plt.ylabel('sim_time sec')
plt.show()