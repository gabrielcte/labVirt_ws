# This scrip is for the calculation to set the cubesat configuration.
# Imports
import numpy as np
import math
# Funções
mu_Terra = 3.98600441e+014 # [m^3/s^2]

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
    T[0,0] = np.cos(RAAN)*np.cos(u)-np.sin(RAAN)*np.sin(u)*np.cos(i)
    T[0,1] = np.sin(RAAN)*np.cos(u)+np.cos(RAAN)*np.sin(u)*np.cos(i)
    T[0,2] = np.sin(u)*np.sin(i)
    T[1,0] = -np.cos(RAAN)*np.sin(u)-np.sin(RAAN)*np.cos(u)*np.cos(i)
    T[1,1] = -np.sin(RAAN)*np.sin(u)+np.cos(RAAN)*np.cos(u)*np.cos(i)
    T[1,2] = np.cos(u)*np.sin(i)
    T[2,0] = np.sin(RAAN)*np.sin(i)
    T[2,1] = -np.cos(RAAN)*np.sin(i)
    T[2,2] = np.cos(i)
    r_SCGI = np.matmul(np.transpose(T), np.array([[p/(1+e*np.cos(f))], [0], [0] ]))
    v_SCGI = np.matmul(math.sqrt(mu_Terra/p)*np.transpose(T),np.array([[e*np.sin(f)], [(1+e*np.cos(f))],[0]]))
    return [r_SCGI, v_SCGI]

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
    print(f'Excentricidade: {e}')
    print(f'Inclinação: {i}')
    print(f'Longitude do nodo ascendente: {RAAN}')
    print(f'Argumento do perigeo: {omega}')
    print(f'Anomalia verdadeira: {f}')

    [r_SCGI, v_SCGI]= eko2osv(a, e, i, RAAN, omega, f)

    print('Vetores de estado da órbita:')
    print(f'distancia: {r_SCGI} [m]')
    print(f'velocidade: {v_SCGI} [m/s]')

