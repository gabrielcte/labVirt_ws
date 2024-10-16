import jsbsim
import jsbsim_utils as jsbsu
from pathlib        import Path
import time
import numpy as np
from enum              import Enum

class FlightStages(Enum):
    flight_stage_cruise            = 0


if __name__ == '__main__':
    
    aircraft_model='c172p'
    aircraft_path=(Path('.')).resolve()

    fdm = jsbsim.FGFDMExec(str(aircraft_path))
    fdm.set_output_directive(str(aircraft_path/'fg_conn.xml'))
    fdm.set_debug_level(0)
    fdm.load_model(aircraft_model)
    fdm.set_dt(0.005)                                            # Define o passo da simulação (s)

    # Initial Conditions
    fdm['ic/lat-geod-rad'] = np.deg2rad(-15.7584639398734650)    # Latitude (rad)
    fdm['ic/long-gc-rad'] = np.deg2rad(-47.221525849944896)      # Longitude (rad)c
    fdm['ic/geod-alt-ft'] = 3003.0                               # ft
    fdm['ic/h-agl-ft'] = 10000                               # ft
    fdm['ic/mach'] = 0.2
    fdm['ic/phi-rad'] =  0.0                                     # Roll (rad)
    fdm['ic/theta-rad'] = 0.0                                    # Pitch (rad)   
    fdm['ic/psi-true-rad'] = np.deg2rad(180)                     # Yaw (rad)
    fdm['velocities/phidot-rad_sec'] =  0.0                                     # Roll (rad)
    fdm['velocities/thetadot-rad_sec'] = 0.0                                    # Pitch (rad)   
    fdm['velocities/psidot-rad_sec'] = 0                                      # Yaw (rad)

    # Initialization

    fdm['propulsion/tank[0]/contents-lbs'] = 3000
    fdm['forces/hold-down'] = 0    
    fdm['fcs/mixture-cmd-norm'] = 0.87
    fdm['propulsion/magneto_cmd'] = 3
    fdm['propulsion/starter_cmd'] = 1


    fdm.run_ic()

    # Simulação
    realtime     = True
    sim_period   = 3600
    frame_time   = 0
    frame_period = 0.005
    flight_stage_current = FlightStages.flight_stage_cruise
    cruise_duration = 500

    try:

        fdm.get_propulsion().init_running(0)


        while fdm.get_sim_time() <= sim_period:

            if flight_stage_current == FlightStages.flight_stage_cruise:  # Initialization
                fdm['fcs/throttle-cmd-norm'] = 0.65

                if fdm.get_sim_time() > cruise_duration:
                    print('### Time REACHED!!')
                    break

                if fdm['position/h-agl-ft'] < 1000*3.281 :
                    print('Crash!')
                    break
            

            else:
                raise Exception('### ERROR: undefined flight stage!')        

            print(f"Time: {fdm.get_sim_time():.2f} s\
                        H: {fdm['position/h-agl-ft']/3.281:.2f} m\
                        Vel X: {fdm['velocities/u-fps']/3.281:.2f} m/s\
                        Atuador: {np.rad2deg(fdm['fcs/elevator-cmd-norm']):.2f} deg\
                        Alpha: {np.rad2deg(fdm['aero/alpha-rad']):.2f} deg\
                        Pitch: {np.rad2deg(fdm['attitude/theta-rad']):.2f} deg", end='\r', flush=True)
            
            fdm.run()

            if realtime:
                
                if fdm.get_sim_time() > frame_time:
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
        print('END')