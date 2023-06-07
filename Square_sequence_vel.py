##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 8
# Fecha de última modificación: Semana 8
# Descripción: Código para realizar una secuencia de cuadrado a una altura de un metro con el dron Crazyflie implementando un filtro de Kalman 
# para las señales recibidas del sistema Loco para la estimación de la posición realizando un control mediante velocidades
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Misael Cruz

##################################################################################################################################################

#  Copyright (C) 2019 Bitcraze AB

import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import keyboard as kb

import pandas

#Variables globales para saber la posicion
X1 = 0
Y1 = 0
Z1 = 0
#Variables globales para la velocidad
VX1 = 0
VY1 = 0
VZ1 = 0

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# Change the sequence according to your setup
#             x    y    z

FINAL_ARRAY = []

def pos_Callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global X1
    global Y1
    global Z1

 #   print("ES DATA")
  #  print(data)

    X1 = data['stateEstimate.x']
    Y1 = data['stateEstimate.y']
    Z1 = data['stateEstimate.z']

    FINAL_ARRAY.append(dic_temp)

def dataExcelLogs(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global VX1
    global VY1
    global VZ1
   
 #   print("ES DATA")
  #  print(data)

    VX1 = data['stateEstimate.vx']
    VY1 = data['stateEstimate.vy']
    VZ1 = data['stateEstimate.vz']
    
    FINAL_ARRAY.append(dic_temp)

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

def set_initial_position(scf, x, y, z, yaw_deg):
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

def run_sequence(scf, base_x, base_y, logconf, logconf2):
    cf = scf.cf

    cf.log.add_config(logconf)
    cf.log.add_config(logconf2)
    
    logconf.data_received_cb.add_callback(dataExcelLogs)
    logconf2.data_received_cb.add_callback(pos_Callback)
    
    logconf.start()
    logconf2.start()
    
    global X1
    global Y1
    global Z1
    global VX1
    global VY1
    global VZ1
    #Valores de ganancias proporcional y derivatiba
    kpx = 1.0
    kdx = 0.16

    kpy = 1.0
    kdy = 0.16

    kpz = 0.5 #0.5
    kdz = 0.03 #0.03

    #se genera la secuencia de puntos de forma absoluta
            #centro                 esquina superior izq                esquina superior derecha       esquina inf der           esquina inf izq                        centro              centro atterizar
    points = [[base_x, base_y, 1],[base_x + 0.7, base_y + 0.7, 1],[base_x + 0.7, base_y - 0.7, 1],[base_x - 0.7, base_y - 0.7, 1], [base_x - 0.7, base_y + 0.7, 1], [base_x, base_y, 1], [base_x, base_y, 0.0], [base_x, base_y, 0.0]]
    #Error de posicion definido en metros
    error_tolerado = 0.05
    contador = 1
    #Valor de velocidad maximo, m/s
    saturacion_error = 0.5
    for point in points:

        x_deseado = point[0]
        y_deseado = point[1]
        z_deseado = point[2]

        print(x_deseado, y_deseado, z_deseado)

        error_x = abs(X1 - x_deseado)
        error_y = abs(Y1 - y_deseado)
        error_z = abs(Z1 - z_deseado)

        print("Iteracion: " + str(contador))

        while ( (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado) ):
            #Como medida de seguridad, si se preciona la letra q, el programa termina
            if kb.is_pressed("q"):
                print("q")
                break

            vx_send = -kpx*(X1 - x_deseado) - kdx*(VX1)
            vy_send = -kpy*(Y1 - y_deseado) - kdy*(VY1)
            vz_send = -kpz*(Z1 - z_deseado) - kdz*(VZ1)

            #SATURACIONES
            #En caso de que la velocidad calculada sea mayor al valor definido, la velociadd máxima sera el valor de saturacion_error
            if(vx_send > saturacion_error):
                vx_send = saturacion_error
            elif(vx_send < -saturacion_error):
                vx_send = -saturacion_error

            if(vy_send > saturacion_error):
                vy_send = saturacion_error
            elif(vy_send < -saturacion_error):
                vy_send = -saturacion_error
            
            if(vz_send > saturacion_error):
                vz_send = saturacion_error
            elif(vz_send < -saturacion_error):
                vz_send = -saturacion_error
        
            #se envian las velocidades calculadas
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0) #vx, vy, vz, yawRate
            time.sleep(0.1)

            error_x = abs(X1 - x_deseado)
            error_y = abs(Y1 - y_deseado)
            error_z = abs(Z1 - z_deseado)

            #print("X: ", X1, "Y: ", Y1, "Z: ", Z1)
            #print("VX: ", VX1, "VY: ", VY1, "VZ: ", VZ1)
            #print("VXsend: ", vx_send, "VYsend: ", vy_send, "VZsend: ", vz_send)
        contador+=1

        #aterrizaje 
    #cf.commander.send_position_setpoint(base_x, base_y, 0.2, yaw)
        #time.sleep(0.1)

    #time.sleep(2)

    logconf.stop()
    logconf2.stop()

    cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
    time.sleep(0.1)

    generateExcelData()
# Función para generar el excel con todos los datos
def generateExcelData():
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("./logs_pruebas1dron_31Mayo/31_05_23_Prueba018_3_Punto3Dcon1dron_TDOA2.csv")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 1.47 
    initial_y = 3.15 #3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    #####variables de posicion y angulos########################################################
    # Con este comando se mandamos a llamar las variables que queremos leer del dron. El nombre
    # de la variable se obtiene desde el CFClient
    # NOTA: No se pueden agregar mas de 6 variables a medir por cada lg_stb
    lg_stab = LogConfig(name='Stabilizer1', period_in_ms=100)

    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimate.vx', 'float')
    lg_stab.add_variable('stateEstimate.vy', 'float')
    lg_stab.add_variable('stateEstimate.vz', 'float')
    ##########pos log######################################################################
    lg_stab2 = LogConfig(name='Stabilizer2', period_in_ms=100)

    lg_stab2.add_variable('stateEstimate.x', 'float')
    lg_stab2.add_variable('stateEstimate.y', 'float')
    lg_stab2.add_variable('stateEstimate.z', 'float')
    #######################################################################################

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf, initial_x, initial_y, lg_stab, lg_stab2)