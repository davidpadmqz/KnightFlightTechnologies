#################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 8
# Fecha de última modificación: Semana 8
# Descripción: Código para mandar 1 dron a una coordenada (x,y) y mantenerse a una altura de 1 metro y finalmente volver a aterrizar
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Adrian Lara Guzman

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
import keyboard as kb
import pandas


#VVariables globales para saber la posicion
X1 = 0
Y1 = 0
Z1 = 0

VX1 = 0
VY1 = 0
VZ1 = 0


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# Change the sequence according to your setup
#             x    y    z


FINAL_ARRAY = []

#############drone 1 callbacks################################################################################3
def pos_angCallback(timestamp, data, logconf):
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
   
def velCallback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
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
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

def anglesRatesCallback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)
##########################################################################################################333

##################################Funcion para implementar el filtro de kalman##########################################################
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

###############Funcion que recibe las posiciones iniciales del dron  y utiliza el filtro del Kalman#######################################
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

###################Funcion que se reinicia los valores anteriores almacenados en el filtro de kalman#######################################
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

################Funcion que manda llamar la sequencia de puntos que debe seguir el dron##################################################
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

    kpx = 0.5
    kdx = 0.02

    kpy = 0.5
    kdy = 0.02

    kpz = 0.5
    kdz = 0.05 #0.05

    x_deseado = base_x
    y_deseado = base_y
    z_deseado = 1.0
            #centro                 ezquina superior izq                centro              centro atterizar
    points = [[base_x, base_y, 1],[base_x + 0.7, base_y + 0.7, 1], [base_x, base_y, 1], [base_x, base_y, 0.1]]
    error_tolerado = 0.10
    contador = 1

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
            
            
            #if kb.is_pressed("q"):
             #   print("q")
              #  break

            vx_send = -kpx*(X1 - x_deseado) - kdx*(VX1)
            vy_send = -kpy*(Y1 - y_deseado) - kdy*(VY1)
            vz_send = -kpz*(Z1 - z_deseado) - kdz*(VZ1)

            #SATURACIONES

            if(vx_send > 0.5):
                vx_send = 0.5
            elif(vx_send < -0.5):
                vx_send = -0.5

            if(vy_send > 0.5):
                vy_send = 0.5
            elif(vy_send < -0.5):
                vy_send = -0.5
            
            if(vz_send > 0.5):
                vz_send = 0.5
            elif(vz_send < -0.5):
                vz_send = -0.5
        
            #se mandan las calculadas
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
    df.to_csv("DataSquare.csv")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 1.47
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
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

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf,
                     initial_x, initial_y, initial_z, initial_yaw,
                     lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates)