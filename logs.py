##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 4  
# Fecha de última modificación: Semana 5
# Descripción: Código para almacenar los datos del dron mediante la configuración de los LOGS
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Cristóbal Padilla

##################################################################################################################################################

import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

import pandas

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')


#Array donde se almacenaran los datos recopilados a lo largo de la ejecución del código
FINAL_ARRAY = []

####################CALLBACKS para cada configuración###################################################

def pos_angCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)
   

def velCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

def quaternionCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

def anglesRatesCallback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global FINAL_ARRAY
    dic_temp = data
    dic_temp.update({"time": timestamp})

    FINAL_ARRAY.append(dic_temp)

#################Fin de funciones de callbacks###########################################


#Se manda a ejecutar el código donde se almacenan los datos
def run_sequence(scf, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):

    cf = scf.cf

    #Las configuraciones de los logs se añaden al dron
    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    #Se anclan las funciones que funcionarán como callbacks
    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback)
    logconf_vel.data_received_cb.add_callback(velCallback)
    logconf_quat.data_received_cb.add_callback(quaternionCallback)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback)

    #se inician la recopilación de información para cada configuración
    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    
    #Durante 10 segundos se estarán llamando los callbacks correspondientes y almacenando los datos
    time.sleep(10)

    #Se terminan los 10 segundos y se terminan los callbacks
    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    #Con este comando se termina de volar el dron parando los motores
    cf.commander.send_stop_setpoint()
    

    #Se genera el excel con los datos recopilados
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("Data.csv")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # posición del dron cuando inicia y orientación en yaw
    initial_x = 1.5
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # En grados


    ###Configuración de los logs##################
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

############################ Rate de los angulos #######################################################################

    lg_stab_angles_rates = LogConfig(name='Stabilizer4', period_in_ms=100)
    
    lg_stab_angles_rates.add_variable('controller.r_pitch', 'float')
    lg_stab_angles_rates.add_variable('controller.r_roll', 'float')
    lg_stab_angles_rates.add_variable('controller.r_yaw', 'float')

    #Conexión con el crazy fly 
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        #se ejecuta la secuencia para almacenar LOGS
        run_sequence(scf, lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates)