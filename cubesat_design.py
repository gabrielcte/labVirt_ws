# This scrip is for the calculation to set the cubesat configuration.
# Imports
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg

# Variáveis globais
mu_Terra = 3.98600441e+014 # [m^3/s^2]
raioEquatorialTerra = 6.378136e+6 # [m]
raioPolarTerra = 6.356753e+6 # [m]
sequencia_rotacao = np.array([3, 2, 1])

def rad2deg(angle_rad):
    angle_deg = angle_rad/np.pi*180
    return angle_deg

def eulerAngles2C (theta_1, theta_2, theta_3):
    global sequencia_rotacao
    angles = np.array([theta_1, theta_2, theta_3])
    C = np.eye(3)
    for seq in range(len(sequencia_rotacao)):
        if sequencia_rotacao[seq] == 1:
            e_x = angles[seq]
            rot_x = np.array([[1, 0, 0], [0, np.cos(e_x), np.sin(e_x)], [0, -np.sin(e_x), np.cos(e_x)]])
            C = np.matmul(C, rot_x)
        elif sequencia_rotacao[seq] == 2:
            e_y = angles[seq]
            rot_y = np.array([[np.cos(e_y), 0, -np.sin(e_y)], [0, 1, 0], [np.sin(e_y), 0, np.cos(e_y)]])
            C = np.matmul(C, rot_y)
        elif sequencia_rotacao[seq] == 3:
            e_z = angles[seq]
            rot_z = np.array([[np.cos(e_z), np.sin(e_z), 0], [-np.sin(e_z), np.cos(e_z), 0], [0, 0, 1]])
            C = np.matmul(C, rot_z)
        else:
            print('Erro em calcular a matriz de rotação')
    return C

def osv2eko(r_SCGI, v_SCGI):
    global mu_Terra
    # versores sistema de coordenadas inercial centrado na terra J2000
    vec_k = np.array([0, 0, 1])
    vec_i = np.array([1, 0, 0])
    # Elementos orbitais clássicos
    h = np.cross(r_SCGI, v_SCGI) # [m^2/s]
    p = np.dot(h, h)/mu_Terra # [m]
    B = np.cross(v_SCGI,h) - mu_Terra * r_SCGI / np.linalg.norm(r_SCGI) # [m^3/s^2]
    e = B/mu_Terra
    a = p/(1-np.dot(e, e)) # [m]
    vec_N = np.cross(vec_k, h) # [m^2/s]
    i = np.arccos(np.dot(vec_k, h)/np.linalg.norm(h))
    RAAN = np.arccos(np.dot(vec_i, vec_N)/np.linalg.norm(vec_N)) # [rad]
    omega = np.arccos(np.dot(e, vec_N)/(np.linalg.norm(e)*np.linalg.norm(vec_N))) # [rad]
    f = np.real(np.arccos(np.dot(r_SCGI, e)/(np.linalg.norm(e)*np.linalg.norm(r_SCGI)))) # [rad]
    return  [a, e, i, RAAN, omega, f]

def eko2osv(a, e, i, RAAN, omega, f):
    global mu_Terra
    e = np.linalg.norm(e)
    p = a*(1-e**2)
    T = np.zeros((3, 3))
    u = omega+f
    T[0][0] = np.cos(RAAN)*np.cos(u)-np.sin(RAAN)*np.sin(u)*np.cos(i)
    T[0][1] = np.sin(RAAN)*np.cos(u)+np.cos(RAAN)*np.sin(u)*np.cos(i)
    T[0][2] = np.sin(u)*np.sin(i)
    T[1][0] = -np.cos(RAAN)*np.sin(u)-np.sin(RAAN)*np.cos(u)*np.cos(i)
    T[1][1] = -np.sin(RAAN)*np.sin(u)+np.cos(RAAN)*np.cos(u)*np.cos(i)
    T[1][2] = np.cos(u)*np.sin(i)
    T[2][0] = np.sin(RAAN)*np.sin(i)
    T[2][1] = -np.cos(RAAN)*np.sin(i)
    T[2][2] = np.cos(i)
    r_SCGI = np.matmul(np.transpose(T), np.array([[p/(1+e*np.cos(f))], [0], [0] ]))
    v_SCGI = np.matmul(math.sqrt(mu_Terra/p)*np.transpose(T),np.array([[e*np.sin(f)], [(1+e*np.cos(f))],[0]]))
    return [r_SCGI, v_SCGI]



def trajetoria(e, a, i, RAAN, omega, f, num_Orbitas, dt):
    x_scgi = np.zeros((num_Orbitas, 1))
    y_scgi = np.zeros((num_Orbitas, 1))
    z_scgi = np.zeros((num_Orbitas, 1))
    lat = np.zeros((num_Orbitas, 1))
    lon = np.zeros((num_Orbitas, 1))
    aux = np.zeros((num_Orbitas, 1))
    r_polar = np.zeros((num_Orbitas, 1))
    e_x_scgi = np.zeros((num_Orbitas, 1))
    e_y_scgi = np.zeros((num_Orbitas, 1))
    e_z_scgi = np.zeros((num_Orbitas, 1))
    e_polar = np.zeros((3, 1))
    e_xpolar = np.zeros((num_Orbitas, 1))
    e_ypolar = np.zeros((num_Orbitas, 1))
    e_zpolar = np.zeros((num_Orbitas, 1))
    p = a * (1 - e ** 2)
    p = np.linalg.norm(p)
    C = eulerAngles2C (-RAAN, -i, -omega)
    for stps in range(num_Orbitas):
        E = omega+f
        x_scgi[stps] = (p/(1+np.linalg.norm(e)*np.cos(f)))*(np.cos(RAAN)*np.cos(E)-np.sin(RAAN)*np.sin(E)*np.cos(i))
        y_scgi[stps] = (p/(1+np.linalg.norm(e)*np.cos(f)))*(np.sin(RAAN)*np.cos(E)+np.cos(RAAN)*np.sin(E)*np.cos(i))
        z_scgi[stps] = (p/(1+np.linalg.norm(e)*np.cos(f)))*(np.sin(E)*np.sin(i))

        # Trajetória Polar
        aux[stps]= f
        r_polar[stps] = a*(1-np.dot(e,e))/(1+np.linalg.norm(e)*np.cos(aux[stps]))
        e_xpolar[stps] = r_polar[stps] * np.cos(f)
        e_ypolar[stps] = r_polar[stps] * np.sin(f)
        e_zpolar[stps] = 0
        f += dt

        # Trajetória SGI

        e_polar[0][0] = e_xpolar[stps]
        e_polar[1][0] = e_ypolar[stps]
        e_polar[2][0] = e_zpolar[stps]
        e_scgi = np.matmul(C, e_polar)
        e_x_scgi[stps] = e_scgi[0][0]
        e_y_scgi[stps] = e_scgi[1][0]
        e_z_scgi[stps] = e_scgi[2][0] ##

        # Ground Track
        lat[stps]= rad2deg(np.arctan2(e_z_scgi[stps],np.sqrt(e_x_scgi[stps]**2+e_y_scgi[stps]**2)))
        lon[stps]= rad2deg(np.arctan2(e_y_scgi[stps],e_x_scgi[stps]))
        if lon[stps]< 0:
            lon[stps] = lon[stps]
    return [x_scgi, y_scgi, z_scgi, lat, lon]


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    m2ft = 3.2808399
    kg2slug = 0.06852177

    # For a CubeSat 6U
    cubeSatMass = 5.18350019*kg2slug #  [SLUG]
    cubeSatLength = 0.1*m2ft # [FT] -> x
    cubeSatWidth = 0.2*m2ft # [FT] -> y
    cubeSatHeight = 0.3*m2ft # [FT] -> z

    print(f'Cubesat mass: {cubeSatMass:.2f} slug')
    print(f'Cubesat length: {cubeSatLength:.2f} ft')
    print(f'Cubesat width: {cubeSatWidth:.2f} ft')
    print(f'Cubesat height: {cubeSatHeight:.2f} ft')

    Ixx = 1 /12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
    Iyy = 1 / 12 * cubeSatMass * ((cubeSatLength ** 2) + (cubeSatHeight ** 2)) # slug * ft ^ 2
    Izz = 1 / 12 * cubeSatMass * ((cubeSatWidth ** 2) + (cubeSatLength ** 2)) # slug * ft ^ 2

    I = np.diag([Ixx, Iyy, Izz])

    print('Matrix of Inertia')
    print(I)

    # Determinando Órbita
    r_SCGI = np.array([2.25526213722520e+006, -3.00492371279401e+006, -5.84397331427593e+006]) # m
    v_SCGI = np.array([-5.19923341417592e+003, 3.82519438208177e+003, -3.97333292224794e+003]) # m/s

    [a, e, i, RAAN, omega, f] = osv2eko(r_SCGI, v_SCGI)
    print('Elementos Clássicos de Órbita:')
    print(f'Semi-eixo maior: {a} [m]')
    print(f'e: {e}')
    print(f'Inclinação: {i}')
    print(f'lon do nodo ascendente: {RAAN}')
    print(f'Argumento do perigeo: {omega}')
    print(f'Anomalia verdadeira: {f}')

    [r_SCGI, v_SCGI]= eko2osv(a, e, i, RAAN, omega, f)

    print('Vetores de estado da órbita:')
    print(f'distancia no SCGI: {r_SCGI} [m]')
    print(f'velocidade no SCGI: {v_SCGI} [m/s]')

    [x_scgi, y_scgi, z_scgi, lat, lon]= trajetoria(e=e, a=a, i=i, RAAN=RAAN, omega=omega, f=f, num_Orbitas=720, dt=1)

    plt.figure(1)
    plt.title('Ground Track')
    plt.plot(lon, lat,'.b')

    # Ler a imagem

    contour = mpimg.imread('mapaContorno.jpg')

    # Configurar a exibição da imagem
    plt.imshow(contour, extent=[-180, 180, -90, 90])

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
    plt.yticks([-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90],
               ['90N', '75N', '60N', '45N', '30N', '15N', '0', '15S', '30S', '45S', '60S', '75S', '90S'])

    # Configurar a grade e proporções iguais
    plt.axis('on')
    plt.axis('equal')
    plt.grid(True)


    # syntax for 3-D projection
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('3D Orbit')

    # Carregar a imagem do mapa de topografia
    topomapa = mpimg.imread('topomapa.jpg')

    # Definir os parâmetros da Terra

    divisoes = 100

    # Criar o ellipsoide
    phi = np.linspace(0, 2 * np.pi, divisoes)
    theta = np.linspace(0, np.pi, divisoes)
    phi, theta = np.meshgrid(phi, theta)

    x = raioEquatorialTerra * np.sin(theta) * np.cos(phi)
    y = raioEquatorialTerra * np.sin(theta) * np.sin(phi)
    z = raioPolarTerra * np.cos(theta)

    # Configurar a figura e o eixo 3D
    ax = fig.add_subplot(111, projection='3d')

    # Mapear a imagem no ellipsoide
    ax.plot_surface(x, y, z, facecolors=plt.cm.gray(topomapa), edgecolor='none', linewidth=0.1, linestyle=':')
    # defining all 3 axes
    z = z_scgi
    x = x_scgi
    y = y_scgi

    # Adding labels to the axes

    # Configurar os eixos
    ax.set_xlim([-raioEquatorialTerra, raioEquatorialTerra])
    ax.set_ylim([-raioEquatorialTerra, raioEquatorialTerra])
    ax.set_zlim([-raioPolarTerra, raioPolarTerra])

    # plotting
    ax.plot3D(x, y, z, '.b')

    # Configurar a visualização
    ax.view_init(30, -30)
    plt.axis('on')
    plt.axis('equal')
    plt.grid(True)


# Mostrar a imagem
    plt.show()