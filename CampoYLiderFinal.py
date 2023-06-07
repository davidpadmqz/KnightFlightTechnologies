##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 9  
# Fecha de última modificación: Semana 9
# Descripción: Código para mediante un lider virtual (con ruta senoidal) se genere un vuelo coordinado entre dos drones con hilos y campos potenciales
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
import keyboard as kb
import threading

import pandas

import matplotlib.pyplot as plt
import numpy as np


#Se establecen los puntos deseados donde deberán llegar los drones inicialmente, en X y Y son donde pongas el dron en el mapa al inicio
# y en Z es la altura a la que quieres que llegue al inicio, para establizarse primero.

#posicion dron 1
puntoDeseado_x_dron1 = 1.0
puntoDeseado_y_dron1 = 0.2
puntoDeseado_z_dron1 = 1.0

#posicion dron 2
puntoDeseado_x_dron2 = 2.0
puntoDeseado_y_dron2 = 0.2
puntoDeseado_z_dron2 = 1.0

#bandera para determinar si el lider virtual sigue corriendo o ya terminó su ejecución
liderAunCorreFlag = True

#VVariables globales para saber la posicion drone 1 con los LOGS
X1 = 0
Y1 = 0
Z1 = 0

VX1 = 0
VY1 = 0
VZ1 = 0

#VVariables globales para saber la posicion drone 2 con los LOGS
X2 = 0
Y2 = 0
Z2 = 0

VX2 = 0
VY2 = 0
VZ2 = 0


# URI de los crazyfly a los que se conectará. Lo que corresponde al dron 1 corresponde a la URI normal, lo 
# del dron2 corresponde a la uri2.
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')


#Listas donde se almacenarán los datos del dron1 y el dron2
FINAL_ARRAY = []
FINAL_ARRAY2 = []


#función que manda a llamar al lider Virtual y sus graficas con los puntos deseados y posiciones de drones en tiempo real
#NOTA: Esta función tiene que correrse en el hilo principal del código, pues las librerías de matplotlib así lo requiere.
def liderVirtual():


    # Se configuran los ejes de la gráfica 
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('center')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    # Se genera el vector con la función senoidal que deberá seguir el lider virtual y se grafica en azul.
    x1 = np.linspace(0, 1.5*np.pi, 50) + 0.5 
    y1 = 0.5*np.sin(x1)+1.5
    line1, = ax.plot(y1, x1, 'b')

    ########### SE GENERA EL CUADRADO (área de trabajo) DE LOS DRONES EN VERDE ################################
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

    ################ FIN DE GENERACIÓN DE ÁREA DE TRABAJO ################################ 


    # Se definen los límites de la gráfica (solo para visualización)
    ax.set_xlim(-3, 2*np.pi)
    ax.set_ylim(-3, 7)

    # Se generan las listas vacias que almacenarán la ruta seguida por el lider virtual en rojo
    x2 = []
    y2 = []
    line2, = ax.plot([], [], 'r')


    #------------------------------------------------------------------------

    #Iniciar variables para el lider virtual

    #posicion inicial del lider virtual
    x_lider = 0.5
    y_lider = 1.72

    #Variables del carro y tiempo
    l = 0.2
    t = 0
    Tf = 0.1
    dt = 0.01

    #Angulo inicial del lider y constantes de control
    theta =  0.7854
    kt = 5 
    kr = 100

    #distancias conocidad para la referencia geometrica, del centro al punto deseado
    dc = 0.5


    #------------------------------------------------------------------------

    #se crean las instancias para graficar los puntos deseados en tiempo real en rojo.
    punto_deseado1, = ax.plot(0, 0, marker='*', color='red')
    punto_deseado2, = ax.plot(0, 0, marker='*', color='red')

    #se crean las instacias para graficar la posicion de los drones en tiempo real en azul.
    dron1_posActual, = ax.plot(0, 0, marker='*', color='blue')
    dron2_posActual, = ax.plot(0, 0, marker='*', color='blue')

    #Se toman las variables globales de los puntos deseados para cada dron
    global puntoDeseado_x_dron1
    global puntoDeseado_y_dron1

    global puntoDeseado_x_dron2
    global puntoDeseado_y_dron2

    #Se toman la posicion de los drones de las variables globales que se actualizan en los logs
    global X1
    global Y1
    global X2
    global Y2

    #tiempo para que los drones vuelen tranquilos, se estabilizen y luego comience el lider virtual
    time.sleep(5)

    # Inicia la secuencia para el líder virtual, se generan los puntos deseados y los drones se mueven en formación.
    for i in range(len(x1)):

        #se toma el siguiente punto deseado para el lider virtual
        xd = x1[i]
        yd = y1[i]

        #se reinica el tiempo a 0 cada que se mueve a un punto deseado
        t = 0

        #se calculan los puntos deseados en formación geométrica para el dron 2
        p2x = x_lider - dc*np.sin(theta)
        p2y = y_lider + dc*np.cos(theta)

        #se calculan los puntos deseados en formación geométrica para el dron 1
        p1x = x_lider + dc*np.sin(theta)
        p1y = y_lider - dc*np.cos(theta)

        #las variables globales de puntos deseados son actualizadas con los cálculos del lider virtual
        puntoDeseado_x_dron1 = p1y
        puntoDeseado_y_dron1 = p1x
        puntoDeseado_x_dron2 = p2y
        puntoDeseado_y_dron2 = p2x

        #error de posicion del lider virtual
        xe = x_lider - xd
        ye = y_lider - yd 

        
        #ciclo while para definir cuando el lider virtual llegue a un punto deseado
        while (abs(xe)>0.05 or abs(ye)>0.05 ):

            #calculo del error
            xe = x_lider - xd
            ye = y_lider - yd 

            #se calcula el angulo deseado y el error
            theta_d = math.atan2(yd - y_lider, xd - x_lider)
            theta_e = theta - theta_d

            #se calcula la velocidad del lider asi como su velocidad angular
            v = kt*math.sqrt(xe**2 + ye**2)
            omega = -kr*theta_e

            #se saturan ambas velocidades para que no vaya muy rapido y no gire muy brusco
            if(v>1):
                v = 1
            if(omega>(np.pi/2)):
                omega = np.pi/2        
            if(omega<(-np.pi/2)):
                omega = -np.pi/2   

            #calculo de velocidades en cada llanta para el lider virtual
            vr = v + 0.5*l*omega
            vl = v - 0.5*l*omega

            #calculo de las velocidades generales
            v = (vr + vl)/2
            omega = (vr - vl)/l

            #calculo de las derivadas de posicion y ángulo
            xp = v*math.cos(theta)
            yp = v*math.sin(theta)
            thetap = omega

            #Integrar para obtener posicion del lider
            x_lider = x_lider + xp*dt
            y_lider = y_lider + yp*dt 
            theta = theta + thetap*dt

            #se acumula el tiempo de simulación
            t = t + dt

            #---------------------------------------------------------
            #############################PARTE PARA GRAFICAR LOS DATOS EN TIEMPO REAL###############################

            # Se añaden los datos de posicion del lider virtual para ser graficados
            x2.append(x_lider)
            y2.append(y_lider)
            
            # Se actualiza la grafica del lider virtual con su posicion y trayectoria recorrida en rojo
            line2.set_xdata(y2)
            line2.set_ydata(x2)

            #Se actualizan los datos de los puntos deseados y de los drones en tiempo real
            
            punto_deseado1.set_xdata([p1y])
            punto_deseado1.set_ydata([p1x])

            punto_deseado2.set_xdata([p2y])
            punto_deseado2.set_ydata([p2x])

            dron1_posActual.set_xdata([X1])
            dron1_posActual.set_ydata([Y1])

            dron2_posActual.set_xdata([X2])
            dron2_posActual.set_ydata([Y2])
            
            # Se dibuja la grafica con todos los datos
            plt.draw()
            
            # Pausa para actualizar los datos de manera correcta
            plt.pause(0.01) 
            #---------------------------------------------------------
            #################################### FIN DEL BLOQUE DE GRAFICAR#############################################

   
    #se toman los puntos deseados para z de ambos drones y se bajan a 0 por 5 segundos.
    #Esto se hace para que estén en una posición segura antes de aterrizar.
    global puntoDeseado_z_dron1
    global puntoDeseado_z_dron2

    puntoDeseado_z_dron1 = 0.0
    puntoDeseado_z_dron2 = 0.0

    #print para saber que ya está aterrizando ambos drones
    print("Aterrizando . . .")
    time.sleep(5)

    #se toma la bandera de si el lider sigue corriendo para ponerla en falsa.
    #Esto genera que ambos hilos (dron 1 y dron 2) terminen su ejecución y por ende termine el codigo
    global liderAunCorreFlag
    liderAunCorreFlag = False

############SECCIÓN DE CALLBACKS PARA LOS DRONES ##################################
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

############# FIN SECCIÓN DE CALLBACKS PARA LOS DRONES##############################

# Función para estimar la posicion del dron en cuestión
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

#función para indicar la posicion inical del dron y se estimen las posiciones de mejor manera
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

#funcion para resetear el estimador de la posicion del dron en cuestion
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

#funcion que será anclada al HILO 1 para el dron 1, control por velocidades con campos potenciales.
def run_sequence(scf, base_x, base_y, base_z, yaw, logconf_pos_ang, logconf_vel, logconf_quat, logconf_angles_rates):

    #instancia de control
    cf = scf.cf

    #se añaden todas las configuraciones de los logs creadas para el dron 1
    cf.log.add_config(logconf_pos_ang)
    cf.log.add_config(logconf_vel)
    cf.log.add_config(logconf_quat)
    cf.log.add_config(logconf_angles_rates)

    #se anclan las funciones que serviran como callbacks para el dron 1
    logconf_pos_ang.data_received_cb.add_callback(pos_angCallback)
    logconf_vel.data_received_cb.add_callback(velCallback)
    logconf_quat.data_received_cb.add_callback(quaternionCallback)
    logconf_angles_rates.data_received_cb.add_callback(anglesRatesCallback)

    #se inica los logs y recopilacion de datos para el dron 1
    logconf_pos_ang.start()
    logconf_vel.start()
    logconf_quat.start()
    logconf_angles_rates.start()

    #Se toman las variables de posicion y velocidad del dron 1 
    global X1
    global Y1
    global Z1
    global VX1
    global VY1
    global VZ1

    #Se toman las posiciones del dron vecino, en este caos el 2
    global X2
    global Y2


    #constantes de control de velocidad para dron 1
    kpx = 1.0
    kdx = 0.16

    kpy = 1.0
    kdy = 0.16

    kpz = 0.5 
    kdz = 0.03 

    #se toman los puntos deseados para el dron 1 calculados en la funcion del lider virtual (se actualizan constantemente)
    global puntoDeseado_x_dron1
    global puntoDeseado_y_dron1
    global puntoDeseado_z_dron1
    #se toma la bandera para saber cuando el lider haya terminando y terminar con el hilo
    global liderAunCorreFlag

    #se asigan estas posiciones a los puntos deseados para el algoritmo de control
    x_deseado = puntoDeseado_x_dron1
    y_deseado = puntoDeseado_y_dron1
    z_deseado = puntoDeseado_z_dron1

    #error tolerado para algoritmod e control por velocidad
    error_tolerado = 0.12
    #contador para saber la iteracion en la que vamos(no afecta en nada al control se puede quitar) 
    contador = 1

    #saturacion máxima permitida para las velocidades resultantes calculades en X, Y, Z
    saturacion_error = 0.5

    ##ganancias para los campos potenciales y distancia de seguridad
    kp_potencial = 1.0
    kt_potencial = 70 
    kr_potencial = 1   
    distancia_seguridad = 0.40
    #############################
    

    print("Iteracion: " + str(contador))

    #ciclo while principal para saber cuando terminar el codigo
    while liderAunCorreFlag:

        #se toman nuevamente los puntos deseados en los tres ejes
        x_deseado = puntoDeseado_x_dron1
        y_deseado = puntoDeseado_y_dron1
        z_deseado = puntoDeseado_z_dron1

        #se estima el error
        error_x = abs(X1 - x_deseado)
        error_y = abs(Y1 - y_deseado)
        error_z = abs(Z1 - z_deseado)

        #ciclo while para algoritmo de control por velocidades mediante error en posicion
        while ( (error_x > error_tolerado) or (error_y > error_tolerado) or (error_z > error_tolerado) ):
                
            #calculo para controlar por velocidad a los drones
            vx_send = -kpx*(X1 - x_deseado) - kdx*(VX1)
            vy_send = -kpy*(Y1 - y_deseado) - kdy*(VY1)
            vz_send = -kpz*(Z1 - z_deseado) - kdz*(VZ1)


            ##########CAMPOS POTENCIALES######################################################

            #se calculan los vectores necesarios para los calculos de campos potenciales
            punto_deseado_dron1 = np.array([[x_deseado],[y_deseado]])
            punto_actual_dron1 = np.array([[X1],[Y1]])
            punto_vecino = np.array([[X2],[Y2]])
            norma_pv_pa = np.linalg.norm(np.subtract(punto_vecino, punto_actual_dron1))
            
            #se divide la formula en tres secciones
            vel_campos_potenciales_parte_izq = -kp_potencial*(np.subtract(punto_actual_dron1, punto_deseado_dron1))
            vel_campos_potenciales_parte_der = -(np.subtract(punto_vecino, punto_actual_dron1) / norma_pv_pa)
            coeficienteDeControl_parte_der = (-0.5*np.tanh(kt_potencial*(norma_pv_pa - distancia_seguridad)) + 0.5) * (kr_potencial / norma_pv_pa)

            #se juntan las tres secciones para obtener el vector con las velocidades resultantes
            velocidades_resultantes_campos_potenciales = vel_campos_potenciales_parte_izq + vel_campos_potenciales_parte_der * coeficienteDeControl_parte_der

            #se obtienen los valores resultantes de los campos potenciales para los ejes X , Y
            vx_campo_potencial = velocidades_resultantes_campos_potenciales[0,0]
            vy_campo_potencial = velocidades_resultantes_campos_potenciales[1,0]

            #a la velocidad que se había calculado previamente se le suma lo calculado en los campos potenciales
            vx_send = vx_send + vx_campo_potencial
            vy_send = vy_send + vy_campo_potencial

            ###################################################################################

            #SATURACIONES de las velocidades##############################################################

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
            
            ########### FIN SATURACIONES ##########################################################

            #Se mandan las velocidades calculadas 
            cf.commander.send_velocity_world_setpoint(vx_send, vy_send, vz_send, 0.0) #vx, vy, vz, yawRate
            time.sleep(0.1)

            #se calcula el error
            error_x = abs(X1 - x_deseado)
            error_y = abs(Y1 - y_deseado)
            error_z = abs(Z1 - z_deseado)

            #se vuelve a obtener el siguiente punto deseado
            x_deseado = puntoDeseado_x_dron1
            y_deseado = puntoDeseado_y_dron1
            z_deseado = puntoDeseado_z_dron1

            
    contador+=1


    #Se terminan los logs
    logconf_pos_ang.stop()
    logconf_vel.stop()
    logconf_quat.stop()
    logconf_angles_rates.stop()

    #se detienen los motores
    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

    #se genera el excel con los datos recopilados del dron
    global FINAL_ARRAY
    df = pandas.DataFrame(FINAL_ARRAY)
    df.to_csv("./campos/06_06_23_Prueba029_3_LiderConCampos_drone1.csv")


#funcion que será anclada al HILO 2 para el dron 2, control por velocidades con campos potenciales.
#NOTA: esta funcion no esta comentada porque funciona exactamente igual que la funcipon del dron 1, solo cambia
# que se toman las posiciones, velocidades etc... del dron 2, pero funcionan igual.
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

    kpx = 1.0
    kdx = 0.16

    kpy = 1.0
    kdy = 0.16

    kpz = 0.5 
    kdz = 0.03 

    global puntoDeseado_x_dron2
    global puntoDeseado_y_dron2
    global puntoDeseado_z_dron2
    global liderAunCorreFlag

    x_deseado = puntoDeseado_x_dron2
    y_deseado = puntoDeseado_y_dron2
    z_deseado = puntoDeseado_z_dron2

    error_tolerado = 0.12
    contador = 1

    ##ganancias campos potenciales
    kp_potencial = 1.0
    kt_potencial = 70 
    kr_potencial = 1   
    distancia_seguridad = 0.40
    #############################

    saturacion_error = 0.5
    while liderAunCorreFlag:



        x_deseado = puntoDeseado_x_dron2
        y_deseado = puntoDeseado_y_dron2
        z_deseado = puntoDeseado_z_dron2

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

            ##########CAMPOS POTENCIALES######################################################


            punto_deseado_dron1 = np.array([[x_deseado],[y_deseado]])
            punto_actual_dron1 = np.array([[X2],[Y2]])
            punto_vecino = np.array([[X1],[Y1]])
            norma_pv_pa = np.linalg.norm(np.subtract(punto_vecino, punto_actual_dron1))
            

            vel_campos_potenciales_parte_izq = -kp_potencial*(np.subtract(punto_actual_dron1, punto_deseado_dron1))
            vel_campos_potenciales_parte_der = -(np.subtract(punto_vecino, punto_actual_dron1) / norma_pv_pa)
            coeficienteDeControl_parte_der = (-0.5*np.tanh(kt_potencial*(norma_pv_pa - distancia_seguridad)) + 0.5) * (kr_potencial / norma_pv_pa)

            velocidades_resultantes_campos_potenciales = vel_campos_potenciales_parte_izq + vel_campos_potenciales_parte_der * coeficienteDeControl_parte_der

            vx_campo_potencial = velocidades_resultantes_campos_potenciales[0,0]
            vy_campo_potencial = velocidades_resultantes_campos_potenciales[1,0]

            vx_send = vx_send + vx_campo_potencial
            vy_send = vy_send + vy_campo_potencial

            ###################################################################################

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

            x_deseado = puntoDeseado_x_dron2
            y_deseado = puntoDeseado_y_dron2
            z_deseado = puntoDeseado_z_dron2

    
    contador+=1



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
    df.to_csv("./campos/06_06_23_Prueba029_3_LiderConCampos_drone2.csv")

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    #posiciones del dron 1 iniciales
    initial_x = puntoDeseado_x_dron1 
    initial_y = puntoDeseado_y_dron1 
    initial_z = 0.0
    initial_yaw = 90  # en grados

    #posiciones del dron 2
    initial_x2 = puntoDeseado_x_dron2
    initial_y2 = puntoDeseado_y_dron2
    initial_z2 = 0.0
    initial_yaw2 = 90
    

    #####CONFIGURACION DE LOS LOGS ####################################
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

    #Conexion con los drones 
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with SyncCrazyflie(uri2, cf=Crazyflie(rw_cache='./cache')) as scf2:

        
            #indicar posicion inicial de los drones y reiniciar estimadores
            set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
            reset_estimator(scf)

            set_initial_position(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2)
            reset_estimator(scf2)

            #se crea el hilo del dron 1
            hilo_dron1 = threading.Thread(target=run_sequence, args=(scf, initial_x, initial_y, initial_z, initial_yaw,
                                                                    lg_stab_pos_ang, lg_stab_vel, lg_stab_quat, lg_stab_angles_rates))

            #se crea el hilo del dron 2
            hilo_dron2 = threading.Thread(target=run_sequence2, args=(scf2, initial_x2, initial_y2, initial_z2, initial_yaw2,
                                                                    lg_stab_pos_ang2, lg_stab_vel2, lg_stab_quat2, lg_stab_angles_rates2))

            #se ejecutan ambos hilos, inician vuelo ambos drones
            hilo_dron1.start()
            hilo_dron2.start()

            #Inicia funcion del lider virtual SIN HILO para que este en el hilo principal #########################################
            liderVirtual()
            ##########################################################################

            #se espera a que se terminen los hilos para terminar el codigo
            hilo_dron1.join()
            hilo_dron2.join()