##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 1
# Descripción: Código para conectar y verificar la conexión del dorn con el CrazyRadio
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Carlos Flores

##################################################################################################################################################


import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper



# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E702'

def simple_connect():

    print("Yeah, I'm connected! :D") # Mensaje de que se conecta
    time.sleep(3)
    print("Now I will disconnect :'(") #Mensaje para indicar que ha terminado

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf: #Se crea una instancia del dron 

        simple_connect()