# This scrip is for the calculation to set the cubesat configuration.
# Imports
import numpy as np

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    m2ft = 3.2808399
    kg2slug = 0.06852177



    # For a CubeSat 6U
    cubeSatMass = 6*kg2slug #  [SLUG]
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

