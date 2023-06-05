##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 3  
# Fecha de última modificación: Semana 3
# Descripción: Código para realizar un despegue del dron a 1 metro de altura y mantenerla durante 15s con el sistema CrazyRadio sin movimientos en (x,y)
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: Misael Cruz

##################################################################################################################################################

#  Copyright (C) 2019 Bitcraze AB



import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# Change the sequence according to your setup
#             x    y    z
#Secuencia para mantener la altura a 1 metro dependiendo de dónde inicie el dron, descritos en formato (x,y,z)
sequence = [
    (0, 0, 1), #Mantener altura
    (0, 0, 1), #Mantener altura
    (0, 0, 1), #Mantener altura
    (0, 0, 0.2),  #Bajar a zona segura
]



#Se manda a ejecutar el código donde se mantendrá la altura 
def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):

    cf = scf.cf


    #para cada posición en el arreglo que se mandó se calcula la posición en X,Y,Z 
    for position in sequence:
        print('Setting position {}'.format(position))

        #La secuencia se creo relativa al dron, aquí se realiza la modificación para que sea absoluta al sistema Loco
        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(50):
            #se manda el comando de la posición (x,y,z) y ángulo (yaw) que debe tener el dron
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)
  

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Cambiar valores de posiicón inicial dependiendo de dónde se coloca el dron en mts
    initial_x = 1.5
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # En grados



    #Se inicia lo comunicación con el dron
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #se manda a ejecutar la secuencia para mantener la altura de Z a 1 metro, se envia la secuencia a seguir y las posiciones inciales del dron
        run_sequence(scf, sequence,
                     initial_x, initial_y, initial_z, initial_yaw)