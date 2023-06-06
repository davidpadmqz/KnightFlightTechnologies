##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 7  
# Fecha de última modificación: Semana 8
# Descripción: Código para volar 2 drones en modo estático, el LPS debe estar en TDoA2
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Angel Sanchez

##################################################################################################################################################



import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702') #Uris modificadas en CfClient

# Change the sequence according to your setup
#             x    y    z
#Las coordenadas son absolutas a la posicion del dron 
sequence1 = [
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 0.2),
]

sequence2 = [
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 0.2),
]

#El filtro estimador de Kalman que ya se ha implementado anteriormente
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
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

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def set_initial_position(scf, x, y, z, yaw_deg): #Esta funcion servirá para ambos drones al igual que la de reset
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Se colocan las posiciones iniciales de ambos drones
    initial_x1 = 1.475
    initial_y1 = 4.725
    initial_z1 = 0.0
    initial_yaw1 = 90  # In degrees

    initial_x2 = 1.475
    initial_y2 = 1.575
    initial_z2 = 0.0
    initial_yaw2 = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf: #Funcion para el primer dron 
        set_initial_position1(scf, initial_x1, initial_y1, initial_z1, initial_yaw1)
        reset_estimator(scf)
        print('Running sequence 1')
        run_sequence1(scf, sequence1,
                     initial_x1, initial_y1, initial_z1, initial_yaw1)
        
    with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2: #Funcion para el segundo dron 
        set_initial_position2(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2)
        reset_estimator(scf2)
        print('Running Sequence 2')
        run_sequence2(scf2, sequence2,
                     initial_x2, initial_y2, initial_z2, initial_yaw2)
    