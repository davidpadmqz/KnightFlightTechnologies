##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 8
# Descripción: Script para secuencia de cuadrados con 2 drones
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

#  Copyright (C) 2019 Bitcraze AB

##################################################################################################################################################


import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import keyboard as kb
import threading



import pandas


#Variables globales para saber la posicion drone 1
X1 = 0
Y1 = 0
Z1 = 0

VX1 = 0
VY1 = 0
VZ1 = 0

#Variables globales para saber la posicion drone 2
X2 = 0
Y2 = 0
Z2 = 0

VX2 = 0
VY2 = 0
VZ2 = 0


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703') # Address drone 3
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702') # Address drone 2
# Change the sequence according to your setup
#             x    y    z


FINAL_ARRAY = []
FINAL_ARRAY2 = []

def pos_angCallback(timestamp, data, logconf):
    
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global X1
    global Y1
    global Z1


    X1 = data['stateEstimate.x']
    Y1 = data['stateEstimate.y']
    Z1 = data['stateEstimate.z']

    FINAL_ARRAY.append(dic_temp)
   

def velCallback(timestamp, data, logconf):

    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global VX1
    global VY1
    global VZ1

    VX1 = data['stateEstimate.vx']
    VY1 = data['stateEstimate.vy']
    VZ1 = data['stateEstimate.vz']

    FINAL_ARRAY.append(dic_temp)

def quaternionCallback(timestamp, data, logconf):

    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

def anglesRatesCallback(timestamp, data, logconf):

    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

#############drone 2 callbacks###################

def pos_angCallback2(timestamp, data, logconf):
    
    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global X2
    global Y2
    global Z2



    X2 = data['stateEstimate.x']
    Y2 = data['stateEstimate.y']
    Z2 = data['stateEstimate.z']

    FINAL_ARRAY2.append(dic_temp)
   

def velCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    global VX2
    global VY2
    global VZ2

    VX2 = data['stateEstimate.vx']
    VY2 = data['stateEstimate.vy']
    VZ2 = data['stateEstimate.vz']

    FINAL_ARRAY2.append(dic_temp)

def quaternionCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY2.append(dic_temp)

def anglesRatesCallback2(timestamp, data, logconf):

    global FINAL_ARRAY2
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY2.append(dic_temp)

###########################################

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


def run_sequence(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):
    cf = scf.cf

    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback)
    logconf_vel.data_received_cb.add_callback(velCallback)
    logconf_quat.data_received_cb.add_callback(quaternionCallback)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback)


    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    
    global X1
    global Y1
    global Z1
    global VX1
    global VY1
    global VZ1

    kpx = 1.0  # Ganancia proporcional en X
    kdx = 0.16 # Ganancia derivativa en X

    kpy = 1.0 # Ganancia proporcional en Y
    kdy = 0.16 # Ganancia derivativa en Y

    kpz = 0.5 # Ganancia proporcional en Z
    kdz = 0.03 # Ganancia derivativa en Z

            #centro                 esquina superior izq                esquina superior derecha       esquina inf der           esquina inf izq                        centro              centro aterizar
    points = [[base_x, base_y, 1],[base_x + 0.7, base_y + 0.7, 1],[base_x + 0.7, base_y - 0.7, 1],[base_x - 0.7, base_y - 0.7, 1], [base_x - 0.7, base_y + 0.7, 1], [base_x, base_y, 1], [base_x, base_y, 0.0], [base_x, base_y, 0.0]]
    
    

    error_tolerado = 0.10
    contador = 1

    saturacion_error = 0.5  # Velocidad de saturación [m/s]
    for point in points: # Ciclo for para iterar las listas que contienen las coordenadas en X, Y, Z para realizar la secuencia de cuadrado

        x_deseado = point[0] # Toma el primer valor de la lista que corresponde a X deseado
        y_deseado = point[1] # Toma el segundo valor de la lista que corresponde a Y deseado
        z_deseado = point[2] # Toma el tercer valor de la lista que corresponde a Z deseado

        print(x_deseado, y_deseado, z_deseado) # Imprime las coordenadas en X, Y, Z deseadas, al punto correspondiente de la secuencia del cuadrado

        error_x = abs(X1 - x_deseado) # Se obtiene el error en el eje X, restando la posición actual en X menos la posición deseada en X
        error_y = abs(Y1 - y_deseado) # Se obtiene el error en el eje Y, restando la posición actual en X menos la posición deseada en Y
        error_z = abs(Z1 - z_deseado) # Se obtiene el error en el eje Z, restando la posición actual en X menos la posición deseada en Z

        print("Iteracion: " + str(contador)) # Imprime en terminal la esquina en la cual se encuentra el drone
        while ( (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado) ): 
            
            
            if kb.is_pressed("q"): # Linea para romper la ejecución del ciclo while mediante input del teclado presionando la tecla "q"
                print("q")
                break

            vx_send = -kpx*(X1 - x_deseado) - kdx*(VX1) # Control PD de velocidad en X
            vy_send = -kpy*(Y1 - y_deseado) - kdy*(VY1) # Control PD de velocidad en Y
            vz_send = -kpz*(Z1 - z_deseado) - kdz*(VZ1) # Control PD de velocidad en Z

            #SATURACIONES

            if(vx_send > saturacion_error): # Se satura con la velocidad de saturación si la velocidad en X es mayor que la velocidad de saturación en dirección positiva
                vx_send = saturacion_error
            elif(vx_send < -saturacion_error): # Se satura con la velocidad de saturación si la velocidad en X es mayor que la velocidad de saturación en dirección negativa
                vx_send = -saturacion_error

            if(vy_send > saturacion_error): # Se satura con la velocidad de saturación si la velocidad en Y es mayor que la velocidad de saturación en dirección positiva
                vy_send = saturacion_error
            elif(vy_send < -saturacion_error): # Se satura con la velocidad de saturación si la velocidad en Y es mayor que la velocidad de saturación en dirección negativa
                vy_send = -saturacion_error
            
            if(vz_send > saturacion_error): # Se satura con la velocidad de saturación si la velocidad en Z es mayor que la velocidad de saturación en dirección positiva
                vz_send = saturacion_error
            elif(vz_send < -saturacion_error): # Se satura con la velocidad de saturación si la velocidad en Z es mayor que la velocidad de saturación en dirección negativa
                vz_send = -saturacion_error
        
            #se mandan las velocidades calculadas
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0) # Velocidad en X, Velocidad en Y, Velocidad en Z, Yaw rate
            time.sleep(0.1)

            error_x = abs(X1 - x_deseado)
            error_y = abs(Y1 - y_deseado)
            error_z = abs(Z1 - z_deseado)


        contador+=1
        time.sleep(0.5)

        #aterrizaje 
    #cf.commander.send_position_setpoint(base_x, base_y, 0.2, yaw)
        #time.sleep(0.1)

    #time.sleep(2)

    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
    time.sleep(0.1)
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)

    df.to_csv("Data.csv")

def run_sequence2(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):
    cf = scf.cf

    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback2)
    logconf_vel.data_received_cb.add_callback(velCallback2)
    logconf_quat.data_received_cb.add_callback(quaternionCallback2)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback2)


    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    
    global X2
    global Y2
    global Z2
    global VX2
    global VY2
    global VZ2

    kpx = 1.0 # Ganancia proporcional en X drone 2
    kdx = 0.16 # Ganancia derivativa en X drone 2

    kpy = 1.0 # Ganancia proporcional en Y drone 2
    kdy = 0.16 # Ganancia derivativa en Y drone 2

    kpz = 0.5 # Ganancia proporcional en Z drone 2
    kdz = 0.03 # Ganancia derivativa en Z drone 2

    
            #centro                 esquina superior izq                esquina superior derecha       esquina inf der           esquina inf izq                        centro              centro atterizar
    points = [[base_x, base_y, 1],[base_x + 0.7, base_y + 0.7, 1],[base_x + 0.7, base_y - 0.7, 1],[base_x - 0.7, base_y - 0.7, 1], [base_x - 0.7, base_y + 0.7, 1], [base_x, base_y, 1], [base_x, base_y, 0.0], [base_x, base_y, 0.0]]
    

    error_tolerado = 0.10
    contador = 1

    saturacion_error = 0.5
    for point in points:

        x_deseado = point[0]
        y_deseado = point[1]
        z_deseado = point[2]

        print(x_deseado, y_deseado, z_deseado)

        error_x = abs(X2 - x_deseado)
        error_y = abs(Y2 - y_deseado)
        error_z = abs(Z2 - z_deseado)

        print("Iteracion: " + str(contador))
        while ( (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado) ):
            
            
            if kb.is_pressed("q"):
                print("q")
                break

            vx_send = -kpx*(X2 - x_deseado) - kdx*(VX2)
            vy_send = -kpy*(Y2 - y_deseado) - kdy*(VY2)
            vz_send = -kpz*(Z2 - z_deseado) - kdz*(VZ2)

            #SATURACIONES

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
        
            #se mandan las calculadas
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0) #vx, vy, vz, yawRate
            time.sleep(0.1)

            error_x = abs(X2 - x_deseado)
            error_y = abs(Y2 - y_deseado)
            error_z = abs(Z2 - z_deseado)


        contador+=1
        time.sleep(0.5)



    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
    time.sleep(0.1)
    global FINAL_ARRAY2
    df = pandas.DataFrame(FINAL_ARRAY2)

    df.to_csv("Data.csv")

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 1.47 
    initial_y = 1.5 #3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees

    initial_x2 = 1.47
    initial_y2 = 4.7
    initial_z2 = 0.0
    initial_yaw2 = 90
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    #####variables de posicion y angulos########################################################
    lg_stab_pos_ang = LogConfig(name='Stabilizer1', period_in_ms=100)

    lg_stab_pos_ang.add_variable('stabilizer.roll', 'float')
    lg_stab_pos_ang.add_variable('stabilizer.pitch', 'float')
    lg_stab_pos_ang.add_variable('stabilizer.yaw', 'float')
    
    lg_stab_pos_ang.add_variable('stateEstimate.x', 'float')
    lg_stab_pos_ang.add_variable('stateEstimate.y', 'float')
    lg_stab_pos_ang.add_variable('stateEstimate.z', 'float')
#############Velocidades en x,y,z#####################################################################

    lg_stab_vel = LogConfig(name='Stabilizer2', period_in_ms=100)
    
    lg_stab_vel.add_variable('stateEstimate.vx', 'float')
    lg_stab_vel.add_variable('stateEstimate.vy', 'float')
    lg_stab_vel.add_variable('stateEstimate.vz', 'float')

###################### QUATERNIONS PART#############################################################################

    lg_stab_quat = LogConfig(name='Stabilizer3', period_in_ms=100)
    
    lg_stab_quat.add_variable('stateEstimate.qx', 'float')
    lg_stab_quat.add_variable('stateEstimate.qy', 'float')
    lg_stab_quat.add_variable('stateEstimate.qz', 'float')
    lg_stab_quat.add_variable('stateEstimate.qw', 'float')

###################################################################################################

    lg_stab_angles_rates = LogConfig(name='Stabilizer4', period_in_ms=100)
    
    lg_stab_angles_rates.add_variable('controller.r_pitch', 'float')
    lg_stab_angles_rates.add_variable('controller.r_roll', 'float')
    lg_stab_angles_rates.add_variable('controller.r_yaw', 'float')

##############################################################################################
####################Instancias para el dron 2##################################################
    lg_stab_pos_ang2 = LogConfig(name='Stabilizer5', period_in_ms=100)

    lg_stab_pos_ang2.add_variable('stabilizer.roll', 'float')
    lg_stab_pos_ang2.add_variable('stabilizer.pitch', 'float')
    lg_stab_pos_ang2.add_variable('stabilizer.yaw', 'float')
    
    lg_stab_pos_ang2.add_variable('stateEstimate.x', 'float')
    lg_stab_pos_ang2.add_variable('stateEstimate.y', 'float')
    lg_stab_pos_ang2.add_variable('stateEstimate.z', 'float')
#############Velocidades en x,y,z#####################################################################

    lg_stab_vel2 = LogConfig(name='Stabilizer6', period_in_ms=100)
    
    lg_stab_vel2.add_variable('stateEstimate.vx', 'float')
    lg_stab_vel2.add_variable('stateEstimate.vy', 'float')
    lg_stab_vel2.add_variable('stateEstimate.vz', 'float')

###################### QUATERNIONS PART#############################################################################

    lg_stab_quat2 = LogConfig(name='Stabilizer7', period_in_ms=100)
    
    lg_stab_quat2.add_variable('stateEstimate.qx', 'float')
    lg_stab_quat2.add_variable('stateEstimate.qy', 'float')
    lg_stab_quat2.add_variable('stateEstimate.qz', 'float')
    lg_stab_quat2.add_variable('stateEstimate.qw', 'float')

###################################################################################################

    lg_stab_angles_rates2 = LogConfig(name='Stabilizer8', period_in_ms=100)
    
    lg_stab_angles_rates2.add_variable('controller.r_pitch', 'float')
    lg_stab_angles_rates2.add_variable('controller.r_roll', 'float')
    lg_stab_angles_rates2.add_variable('controller.r_yaw', 'float')



################################################################################################

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:

        

            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)

            set_initial_position(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2)
            reset_estimator(scf2)

            hilo_dron1 = threading.Thread(target=run_sequence, args=(scf, initial_x, initial_y, initial_z, initial_yaw,
                                                                    lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates))

            hilo_dron2 = threading.Thread(target=run_sequence2, args=(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2,
                                                                    lg_stab_pos_ang2, lg_stab_vel2, lg_stab_quat2, lg_stab_angles_rates2))

            hilo_dron1.start()
            hilo_dron2.start()

            #aqui se pose el codigo del lider#########################################

            ##########################################################################

            hilo_dron1.join()
            hilo_dron2.join()
        