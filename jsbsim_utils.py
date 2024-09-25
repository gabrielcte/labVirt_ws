from pathlib             import Path
import control           as ctrl
import jsbsim
import numpy             as np
import scipy.optimize
from math import pi


###############################################################################################################################################################
#                                                                    Simulation Functions                                                                     #
###############################################################################################################################################################

def get_simulation_data(fdm):


    sim_time = fdm.get_sim_time()
    lat      = fdm['position/lat-geod-rad']
    lon      = fdm['position/long-gc-rad']
    alt      = fdm['position/h-sl-meters']
    height   = fdm['position/h-agl-ft']/3.281
    vel_x    = fdm['velocities/u-fps']/3.281
    vel_y    = fdm['velocities/v-fps']/3.281
    vel_z    = fdm['velocities/w-fps']/3.281
    roll     = fdm['attitude/phi-rad']
    pitch    = fdm['attitude/theta-rad']
    yaw      = fdm['attitude/psi-rad']
    alpha    = fdm['aero/alpha-rad']
    beta     = fdm['aero/beta-rad']
    ele_act  = fdm['fcs/elevator-actuator']
    ele_ctrl = fdm['fcs/elevator-control']
    rud_act  = fdm['fcs/rudder-actuator']
    rud_ctrl = fdm['fcs/rudder-control']
    ail_act  = fdm['fcs/left-aileron-actuator']
    ail_ctrl = fdm['fcs/left-aileron-control']

    sim_data = {key: value for key, value in locals().items() if key in [
        'sim_time',
        'lat', 'lon', 'alt', 'height',
        'vel_x', 'vel_y', 'vel_z',
        'roll', 'pitch', 'yaw',
        'alpha', 'beta',
        'ele_act', 'ele_ctrl', 'rud_act', 'rud_ctrl', 'ail_act', 'ail_ctrl',
    ]}

    return sim_data


def set_ic0(fdm):

    # Valores par a simulação de Monte Carlo
    fdm['montecarlo/bL'] = 0.0
    fdm['montecarlo/kL'] = 1.0
    fdm['montecarlo/bD'] = 0.0
    fdm['montecarlo/kD'] = 1.0
    fdm['montecarlo/bM'] = 0.0
    fdm['montecarlo/kM'] = 1.0
    fdm['montecarlo/bY'] = 0.0
    fdm['montecarlo/kY'] = 1.0
    fdm['montecarlo/bN'] = 0.0
    fdm['montecarlo/kN'] = 1.0
    fdm['montecarlo/bR'] = 0.0
    fdm['montecarlo/kR'] = 1.0

    # Valores para a atmosfera e vento
    #fdm['atmosphere/delta-T']       = 66.6                 # Especificação encontrada no script de Trimagem da PH
    fdm['atmosphere/wind-north-fps'] = 0.0
    fdm['atmosphere/wind-east-fps']  = 0.0
    fdm['atmosphere/wind-down-fps']  = 0.0

    # Desligando as estruturas de PA
    fdm['ap/airspeed_hold']     = 0
    fdm['ap/airspeed_setpoint'] = 0
    fdm['ap/attitude_hold']     = 0
    fdm['ap/autopilot-roll-on'] = 0
    fdm['ap/heading_hold']      = 0
    fdm['forces/hold-down']     = 0

    # Wing opened
    fdm['wing/wing_ang_pos'] = 0
    
    # Booster separation
    fdm['inertia/pointmass-weight-lbs[1]'] = 0
    fdm['propulsion/tank[1]/contents-lbs'] = 0
    # fdm['propulsion/tank[0]/contents-lbs'] = 1000           # Considera como 0 a massa de combustível


def set_location(fdm, lat, lon, h):

    fdm['ic/terrain-elevation-ft'] = h*3.281                # Elevação aproximada do terreno (pés)
    fdm['ic/lat-geod-deg']         = lat                    # Latitude (graus)
    fdm['ic/long-gc-deg']          = lon                    # Longitude (graus)

###############################################################################################################################################################


###############################################################################################################################################################
#                                                                      Control Functions                                                                      #
###############################################################################################################################################################

def clean_tf(G, tol=1e-5):

    num = G.num
    den = G.den

    for poly in num, den:
        for i in range(len(poly)):
            for j in range(len(poly[i])):
                poly[i][j] = np.where(np.abs(poly[i][j]) < tol, 0, poly[i][j])

    return ctrl.tf(num, den)

###############################################################################################################################################################
'''
print('### INPUT: ', state)
print('### CG X:', fdm['inertia/cg-x-in'] * 0.0833333)
print('### Lever Arm', abs(fdm['inertia/cg-x-in']-fdm['metrics/aero-rp-x-in']))
print('### CM_A_M_1:', start['aero/coefficient/CM_A_M'])
print('### CM_A_M_2:', fdm['aero/coefficient/CM_A_M'])
print('### CL_A_M1:', start['aero/coefficient/CL_A_M'])
print('### CL_A_M2:', fdm['aero/coefficient/CL_A_M'])
print('### dCM_A_M_with_Booster_1:', start['aero/coefficient/dCM_A_M_with_Booster'])
print('### dCM_A_M_with_Booster_2:', fdm['aero/coefficient/dCM_A_M_with_Booster'])
print('### dCM_B_M_1:', start['aero/coefficient/dCM_B_M'])
print('### dCM_B_M_2:', fdm['aero/coefficient/dCM_B_M'])
print('### dCM_B_with_booster_1:', start['aero/coefficient/dCM_B_with_booster'])
print('### dCM_B_with_booster_2:', fdm['aero/coefficient/dCM_B_with_booster'])
print('### dCM_A_Elev_M_1:', start['aero/coefficient/dCM_A_Elev_M'])
print('### dCM_A_Elev_M_2:', fdm['aero/coefficient/dCM_A_Elev_M'])
print('### dCMq_1:', start['aero/coefficient/dCMq'])
print('### dCMq_2:', fdm['aero/coefficient/dCMq'])
print('### Qdot1:', start['accelerations/qdot-rad_sec2'])
print('### Qdot2:', fdm['accelerations/qdot-rad_sec2'])
print('### alpha_dot: ', fdm['aero/alphadot-rad_sec'])

d        = fdm['metrics/cbarw-ft']
Iyy      = 1630.6023833                                          
Sref     = fdm['metrics/Sw-sqft']
Vm       = fdm['velocities/u-fps']                          
rho      = fdm['atmosphere/rho-slugs_ft3']                  
Q        = fdm['aero/qbar-psf']
Q_calc   = 0.5*rho*Vm**2
CMyalpha = fdm['aero/coefficient/CM_A_M']
#CMyalpha = -13.724364
Malpha   = (Q*Sref*d/Iyy)*CMyalpha

d2       = 0.6                                      # m
Iyy2     = 3600
Sref2    = (np.pi*(d2**2))/4
#Vm2     = (0.7*343)                                # ms
Vm2      = 253                                      # Valor Edmundo
rho2     = 0.79
#Q2      = 0.5*ro*V_m**2
Q2       = 25214.445                                # Valor Edmundo
CMyalpha2 = -13.724364
Malpha2  = (Q2*Sref2*d2/Iyy2)*CMyalpha2

print(f"\nWing cord: {d}")
print(f"Wing area: {Sref}")
print(f"Velocity: {Vm}")
print(f"Rho: {rho}")
print(f"Q: {Q}")
print(f"Q Calculated: {Q_calc}")
print(f"Cmyalpha: {CMyalpha}")
print(f"M alpha: {Malpha}")
print(f"M alpha 2: {Malpha2}\n")
'''
