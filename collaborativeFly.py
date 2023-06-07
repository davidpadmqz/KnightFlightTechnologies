##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 8  
# Fecha de última modificación: Semana 9
# Descripción: Código para mediante un lider virtual (con ruta senoidal) se genere un vuelo coordinado entre dos drones
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

import matplotlib.pyplot as plt
import numpy as np

# URI de los drones a los que se van a conectar
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')




#función para hacer el calculo de la estimacion una vez se reinicia
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


#funcion para establecer la posicion inicial del dron y hacer estimaciones correctas
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


#funcion para resetear el estimador del dron
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def run_sequence(scf, base_x, base_y, base_z, yaw, base_x2, base_y2, base_z2, scf2):

    #Se establecen las instancias del dron y el dron 2 
    cf = scf.cf
    cf2 = scf2.cf

    # Se establecen los ejes para la grafica
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('center')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # Se hacen los vectores para la función senoidal a seguir y se grafica en azul
    x1 = np.linspace(0, 1.5*np.pi, 25) + 0.5 
    y1 = np.sin(x1)+1.5
    line1, = ax.plot(y1, x1, 'b')

    # SE GRAFICAN LOS LIMITES DEL ÁREA DE TRABAJO EN VERDE########################################
    leftLine_y = np.linspace(0, 6.3, 2)
    leftLine_x = 0.0 + leftLine_y*0
    leftLine, = ax.plot(leftLine_x, leftLine_y, 'g')

    rightLine_y = np.linspace(0, 6.3, 2)
    rightLine_x = 2.95 + rightLine_y*0
    rightLine, = ax.plot(rightLine_x, rightLine_y, 'g')

    topLine_y = np.linspace(0.0, 2.95, 2)
    topLine_x = 6.3 + topLine_y*0
    topLine, = ax.plot(topLine_y, topLine_x, 'g')

    botLine_y = np.linspace(0.0, 2.95, 2)
    botLine_x = 0 + botLine_y*0
    botLine, = ax.plot(botLine_y, botLine_x, 'g')
    #################################################################################################

    # Se establecen los límites de los ejes en X y Y
    ax.set_xlim(-3, 2*np.pi)
    ax.set_ylim(-3, 7)

    # Se generan vectores vacios donde se irá graficando en tiempo real el movimiento del lider virtual en rojo
    x2 = []
    y2 = []
    line2, = ax.plot([], [], 'r')


    #------------------------------------------------------------------------

    #Iniciar variables para el lider virtual

    #posicion lider virtual
    x_lider = 0.5
    y_lider = 2.0

    #Variables del carro y tiempo
    l = 0.2
    t = 0
    Tf = 0.1
    dt = 0.01

    #Angulo inicial del lider y constantes de control
    theta =  0.7854
    kt = 10
    kr = 100

    #distancias conocidad para la referencia geometrica
    dc = 0.25


    #------------------------------------------------------------------------

    #Son los puntos deseados a seguir que genera el lider virtual
    punto_deseado1, = ax.plot(0, 0, marker='*', color='red')
    punto_deseado2, = ax.plot(0, 0, marker='*', color='red')

    #despegue de drones donde inicien y mantenerlos a 1 metro de altura
    cf.commander.send_position_setpoint(base_x, base_y, 1.0, yaw)
    cf2.commander.send_position_setpoint(base_x2, base_y2, 1.0, yaw)
    time.sleep(0.1)


    # Inicia el loop inicial con el movimiento del lider 
    for i in range(len(x1)):

        #punto deseado del lider virtual
        xd = x1[i]
        yd = y1[i]
        
        #puntos deseados en X y Y para el primer punto deseado
        p1x = x_lider - dc*np.sin(theta)
        p1y = y_lider + dc*np.cos(theta)

        #puntos deseados en X y Y para el segundo punto deseado
        p2x = x_lider + dc*np.sin(theta)
        p2y = y_lider - dc*np.cos(theta)

        #error actual del punto deseado del lider y posicion actual
        xe = x_lider - xd
        ye = y_lider - yd 

        #ciclo que se ejecuta hasta tener un error menor al 5% en el lider en X y Y
        while (abs(xe)>0.05 or abs(ye)>0.05 ):

            #calculo del error
            xe = x_lider - xd
            ye = y_lider - yd 

            #se calcula el angulo deseado y el error del lider virtual
            theta_d = math.atan2(yd - y_lider, xd - x_lider)
            theta_e = theta - theta_d

            #se calcula la velocidad del lider asi como la velocidad angular
            v = kt*math.sqrt(xe**2 + ye**2)
            omega = -kr*theta_e

            #saturacion de velocidades
            if(v>1):
                v = 1
            if(omega>(np.pi/2)):
                omega = np.pi/2        
            if(omega<(-np.pi/2)):
                omega = -np.pi/2   

            #calculo de velocidades por llanta
            vr = v + 0.5*l*omega
            vl = v - 0.5*l*omega

            #calculo de velocidades completas de lineal y angular 
            v = (vr + vl)/2
            omega = (vr - vl)/l

            #se calcula la derivada de la posicion en X , Y y theta.
            xp = v*math.cos(theta)
            yp = v*math.sin(theta)
            thetap = omega

            #Integrar para obtener posicion del lider
            x_lider = x_lider + xp*dt
            y_lider = y_lider + yp*dt 
            theta = theta + thetap*dt

            t = t + dt

            #---------------------------------------------------------
            #GRAFICAR, BLOQUE

            # Se añade a las listas vacias la posicion del lider
            x2.append(x_lider)
            y2.append(y_lider)
            
            # se actualiza en la grafica la posicion del lider recorrida en rojo
            line2.set_xdata(y2)
            line2.set_ydata(x2)

            #Actualizar dato nuevo en gráfica para los puntos deseados calculados
            punto_deseado1.set_data(p1y, p1x)
            punto_deseado2.set_data(p2y, p2x)
            
            # Se dibuja la grafica completa con la ruta a seguir, el lider y puntos deseados
            plt.draw()
            
            # Pausa para graficar bien los datos
            plt.pause(0.01) 
            #---------------------------------------------------------
            #Se calculan las posiciones a mandar para cada drone dependiendo de donde inicio y los puntos
            #deseados calculados por el lider
            x_drone1 = p1y + base_x
            y_drone1 = p1x + base_y
            z_drone1 = 1.0

            x_drone2 = p2y + base_x2
            y_drone2 = p2x + base_y2
            z_drone2 = 1.0
        
        #Se mandan los puntos calculados a cada dron
        cf.commander.send_position_setpoint(x_drone2, y_drone2, z_drone2, yaw)
        cf2.commander.send_position_setpoint(x_drone1, y_drone1, z_drone1, yaw)
        time.sleep(0.1)

    # Se muestra la grafica final
        plt.show()

    #Se mandan los drones a una altura baja para aterrizar
    cf.commander.send_position_setpoint(x_drone1, y_drone1, 0.15, yaw)
    cf2.commander.send_position_setpoint(x_drone2, y_drone2, 0.15, yaw)

    time.sleep(2)

    #Se termina el vuelo y se frenan los motores
    cf.commander.send_stop_setpoint()
    cf2.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Posicion del dron 1 en X,Y,Z y orientación en yaw
    initial_x = 2.70
    initial_y = 3.15
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    
    # Posicion del dron 2 en X,Y,Z y orientación en yaw
    initial_x2 = 0.2
    initial_y2 = 3.15
    initial_z2 = 0.0



    #Conexión a dron 1  
    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:

        #establecer posicion inicial del dron 1
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        #reiniciar estimador para el dron 1 
        reset_estimator(scf)
        
        #conexión al dron 2
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:

            #establecer posición inicial del dron 2
            set_initial_position(scf2, initial_x2, initial_y2, initial_z2, initial_yaw)
            #reiniciar el estimador para el dron 2
            reset_estimator(scf2)

            #iniciar la secuencia donde los drones siguen al lider virtual
            run_sequence(scf, initial_x, initial_y, initial_z, initial_yaw, initial_x2, initial_y2, initial_z2, scf2)