import csv
import matplotlib.pyplot as plt
import numpy             as np

# Função para plotar os gráficos com os dados da simulação
def plot_simulation_data(csv_file):
    # Carrega os dados do arquivo CSV
    time, lat, lon, alt, pos_x, pos_y, pos_z, height, vel_x, vel_y, vel_z, vel_module, roll, pitch, yaw, target_pitch, alpha, beta, elevator_cmd, ele_act, ele_ctrl, rudder_cmd, rud_act, rud_ctrl, aileron_cmd, ail_act, ail_ctrl, throttle_cmd  = \
      [],  [],  [],  [],    [],    [],    [],     [],    [],    [],    [],         [],   [],    [],  [],           [],    [],   [],           [],      [],       [],         [],      [],       [],          [],      [],       [],           []
    with open(csv_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            time.append(float(row['Time (s)']))
            lat.append(float(row['Latitude (rad)']))
            lon.append(float(row['Longitude (rad)']))
            alt.append(float(row['Altitude (m)']))
            height.append(float(row['Height (m)']))
            pos_x.append(float(row['Pos N (m)']))
            pos_y.append(float(row['Pos E (m)']))
            pos_z.append(float(row['Pos D (m)']))
            vel_x.append(float(row['Vel X (m/s)']))
            vel_y.append(float(row['Vel Y (m/s)']))
            vel_z.append(float(row['Vel Z (m/s)']))
            vel_module.append(float(row['Vel Module (m/s)']))
            roll.append(float(row['Roll (rad)']))
            pitch.append(float(row['Pitch (rad)']))
            yaw.append(float(row['Yaw (rad)']))
            target_pitch.append(float(row['Target Pitch (rad)']))
            alpha.append(float(row['Angle of Attack (rad)']))
            beta.append(float(row['Angle of Sideslip (rad)']))
            elevator_cmd.append(float(row['Elevator CMD (rad)']))
            ele_act.append(float(row['Elevator Actuator (rad)']))
            ele_ctrl.append(float(row['Elevator Control (deg)']))
            rudder_cmd.append(float(row['Rudder CMD (rad)']))
            rud_act.append(float(row['Rudder Actuator (rad)']))
            rud_ctrl.append(float(row['Rudder Control (deg)']))
            aileron_cmd.append(float(row['Aileron CMD (rad)']))
            ail_act.append(float(row['Aileron Actuator (rad)']))
            ail_ctrl.append(float(row['Aileron Control (deg)']))
            throttle_cmd.append(float(row['Throttle CMD']))

    lat_deg          = np.rad2deg(np.array(lat))
    lon_deg          = np.rad2deg(np.array(lon))
    pos_x            = np.array(pos_x)
    pos_y            = np.array(pos_y)
    roll_deg         = np.rad2deg(np.array(roll))
    pitch_deg        = np.rad2deg(np.array(pitch))
    yaw_deg          = np.rad2deg(np.array(yaw))
    target_pitch_deg = np.rad2deg(np.array(target_pitch))
    alpha_deg        = np.rad2deg(np.array(alpha))
    beta_deg         = np.rad2deg(np.array(beta))
    elevator_cmd_deg = np.rad2deg(np.array(elevator_cmd))
    ele_act_deg      = np.rad2deg(np.array(ele_act))
    ele_ctrl_deg     = np.rad2deg(np.array(ele_ctrl))
    rudder_cmd_deg   = np.rad2deg(np.array(rudder_cmd))
    rud_act_deg      = np.rad2deg(np.array(rud_act))
    rud_ctrl_deg     = np.rad2deg(np.array(rud_ctrl))
    aileron_cmd_deg  = np.rad2deg(np.array(aileron_cmd))
    ail_act_deg      = np.rad2deg(np.array(ail_act))
    ail_ctrl_deg     = np.rad2deg(np.array(ail_ctrl))
    throttle_cmd     = np.array(throttle_cmd)

    # LLA vs. Tempo
    '''plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time, lat_deg)
    plt.xlabel('Time (s)')
    plt.ylabel('Latitude (deg)')
    plt.title('Latitude vs. Time')
    plt.subplot(3, 1, 2)
    plt.plot(time, lon_deg)
    plt.xlabel('Time (s)')
    plt.ylabel('Longitude (deg)')
    plt.title('Longitude vs. Time')
    plt.subplot(3, 1, 3)
    plt.plot(time, alt)
    plt.xlabel('Time (m)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs. Time')
    plt.tight_layout()'''

    # NED vs. Tempo
    plt.figure(figsize=(12, 8))
    plt.subplot(4, 1, 1)
    plt.plot(time, pos_x)
    plt.xlabel('Time (s)')
    plt.ylabel('Position N (m)')
    plt.title('Position N vs. Time')
    plt.subplot(4, 1, 2)
    plt.plot(time, pos_y)
    plt.xlabel('Time (s)')
    plt.ylabel('Position E (m)')
    plt.title('Position E vs. Time')
    plt.subplot(4, 1, 3)
    plt.plot(time, pos_z)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Z (m)')
    plt.title('Position Z vs. Time')
    plt.subplot(4, 1, 4)
    plt.plot(np.sqrt(pos_x ** 2 + pos_y ** 2), height)
    plt.xlabel('Distance traveled (m)')
    plt.ylabel('Height (m)')
    plt.title('Distance Traveled vs Height')
    plt.tight_layout()

    # Vel NED vs. Tempo
    plt.figure(figsize=(12, 8))
    plt.subplot(4, 1, 1)
    plt.plot(time, vel_x)
    plt.xlabel('Time (s)')
    plt.ylabel('Vel X (m/s)')
    plt.title('Vel X vs. Time')
    plt.subplot(4, 1, 2)
    plt.plot(time, vel_y)
    plt.xlabel('Time (s)')
    plt.ylabel('Vel Y (m/s)')
    plt.title('Vel Y vs. Time')
    plt.subplot(4, 1, 3)
    plt.plot(time, vel_z)
    plt.xlabel('Time (s)')
    plt.ylabel('Vel Z (m/s)')
    plt.title('Vel Z vs. Time')
    plt.subplot(4, 1, 4)
    plt.plot(time, vel_module)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity Module (m/s)')
    plt.title('Velocity Module vs. Time')
    plt.tight_layout()

    # Ângulos de Euler vs. Tempo
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time, roll_deg)
    plt.xlabel('Time (s)')
    plt.ylabel('Roll (deg)')
    plt.title('Roll vs. Time')
    plt.subplot(3, 1, 2)
    plt.plot(time, pitch_deg)
    plt.plot(time, target_pitch_deg)
    plt.legend(['Pitch (deg)', 'Target Pitch (deg)'])
    plt.xlabel('Time (s)')
    plt.ylabel('Pitch (deg)')
    plt.title('Pitch vs. Time')
    plt.subplot(3, 1, 3)
    plt.plot(time, yaw_deg)
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (deg)')
    plt.title('Yaw vs. Time')
    plt.tight_layout()

    # Plota os gráficos
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    # plt.plot(time, elevator_cmd_deg)
    # plt.plot(time, ele_ctrl_deg)
    plt.plot(time, ele_act_deg)
    # plt.plot(time, rudder_cmd_deg)
    # plt.plot(time, rud_ctrl_deg)
    plt.plot(time, rud_act_deg)
    # plt.plot(time, aileron_cmd_deg)
    # plt.plot(time, ail_ctrl_deg)
    plt.plot(time, ail_act_deg)
    plt.xlabel('Time (s)')
    plt.ylabel('Commands (deg)')
    plt.legend([
        # 'Python Elevator Command', 'JSBSim Elevator Control', 'JSBSim Elevator Actuator',\
        # 'Python Rudder Command', 'JSBSim Rudder Control', 'JSBSim Rudder Actuator',\
        # 'Python Aileron Command', 'JSBSim Aileron Control', 'JSBSim Aileron Actuator',\
        # 'Python Elevator Command', 'Python Rudder Command', 'Python Aileron Command',\
        # 'JSBSim Elevator Control', 'JSBSim Rudder Control', 'JSBSim Aileron Control',\
        'JSBSim Elevator Actuator', 'JSBSim Rudder Actuator', 'JSBSim Aileron Actuator',\
    ])
    plt.title('Elevator Commands')
    plt.subplot(3, 1, 2)
    plt.plot(time, alpha_deg)
    plt.plot(time, beta_deg)
    plt.legend(['alpha','beta'])
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Angle of Attack vs. Time')
    plt.subplot(3, 1, 3)
    plt.plot(time, throttle_cmd*100)
    plt.xlabel('Time (s)')
    plt.ylabel('Throttle (%)')
    plt.title('Throttle vs. Time')
    plt.tight_layout()



    # syntax for 3-D projection
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Trajetória AATD')

    # defining all 3 axes
    z = height
    x = pos_x
    y = pos_y

    # plotting
    ax.plot3D(x, y, z, 'b')

    # Adding labels to the axes
    ax.set_xlabel('N [m]')
    ax.set_ylabel('E [m]')
    ax.set_zlabel('H [m]')
    ax.set_ylim([-1500, 1500])
    # Exibe os gráficos
    plt.show()


    # Fecha o arquivo de log
    csvfile.close()
