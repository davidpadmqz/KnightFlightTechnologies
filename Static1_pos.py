#################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 3
# Fecha de última modificación: Semana 4
# Descripción: Código para mantener un vuelo de un dron, en una altura de 1 metro y una X y Y fijas
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Angel Sanchez 

#Código basado en https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/positioning/initial_position.py
#  Copyright (C) 2019 Bitcraze AB

#################################################################################################################


import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

# Change the sequence according to your setup
#             x    y    z
sequence = [
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 0.2),
]

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    #A partir de aquí se implementa un filtro estimador que calcula la posicion del dron 
    #Esta funcion fue desarrollada por los fabricantes, véase en el link de referencia
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500) # Se establece el tiempo de ejecución de la función 
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break
            
            #Fin del filtro


def set_initial_position(scf, x, y, z, yaw_deg): #Se establecen las posiciones iniciales
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians) #Se establece el ángulo inicial en radianes


def reset_estimator(scf): #Este reset estimator lo requiere el filtro predictor creado por el fabricante 
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw): #Comienza la funcion que se mandará al dron 
    cf = scf.cf

    for position in sequence: #Se ejecutan todas las posiciones definidas, en este caso en X y Y se manda 0 y solo cambia las de Z
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            cf.commander.send_position_setpoint(x, y, z, yaw) #Commnader es una funcion que manda las posiciones al dron realizando un control de posicion 
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Estas son las coordenadas en las que se coloca el dron en el suelo, basado en coordenadas del LPS
    initial_x = 1.5
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    #Funcion principal que se manda al Dron, el dron recibe las posiciones, X, Y, X y ángulo al que debe llegar
    #Todo se encuentra en un control 
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf, sequence,
                     initial_x, initial_y, initial_z, initial_yaw)