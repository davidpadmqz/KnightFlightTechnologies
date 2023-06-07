##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 9
# Descripción: Gráfica el movimiento de dos drones para análisis de datos
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

##################################################################################################################################################

from matplotlib import pyplot as plt
import pandas as pd

def graph(df, df1):

    ############## 3D Graph ##################

    plt.figure('3D trajectory mapping') # Crea una figura nueva con el nombre 3D trajectory mapping
    ax = plt.axes(projection ="3d") # Se generan 3 ejes para graficar en 3D
    plot = ax.scatter3D(df['stateEstimate.x'],df['stateEstimate.y'],df['stateEstimate.z'], c = df['time']/10000 ,cmap = 'plasma') # Se grafica las posiciones en X,Y,Z del drone 1 con respecto al tiempo, el tiempo se encuentra representado mediante un mapa de calor
    plot1 = ax.scatter3D(df1['stateEstimate.x'],df1['stateEstimate.y'],df1['stateEstimate.z'], c =df1['time']/10000, cmap = 'plasma') # Se grafica las posiciones en X,Y,Z del drone 2 con respecto al tiempo, el tiempo se encuentra representado mediante un mapa de calor
    ax.set_xlabel('position in X [m]') # Se nombra al eje x como position in X [m]
    ax.set_ylabel('Position in Y [m]') # Se nombra al eje y como position in Y [m]
    ax.set_zlabel('Position in Z [m]') # Se nombra al eje z como position in Z [m]
    plt.colorbar(plot,cmap=plt.cm.magma,label = 'Time [s]') # Se incluye la barra de color con la finalidad de interpretar el mapa de color, la escala esta en segundos
    plt.title('3D trajectory mapping') # Se agrega un titulo para la grafica 

    ############## XY Plane ##################

    plt.figure('XY Plane') # Crea una figura nueva con el nombre XY Plane
    plot = plt.scatter(df['stateEstimate.x'], df['stateEstimate.y'], c = df['time']/10000, cmap='plasma') # Se grafica las posiciones en X,Y del drone 1 con respecto al tiempo, el tiempo se encuentra representado mediante un mapa de calor
    plot1 = plt.scatter(df1['stateEstimate.x'] ,df1['stateEstimate.y'],c = df1['time']/10000, cmap='plasma') # Se grafica las posiciones en X,Y del drone 2 con respecto al tiempo, el tiempo se encuentra representado mediante un mapa de calor
    plt.colorbar(plot, cmap=plt.cm.magma, label = 'Time [s]') # Se incluye la barra de color con la finalidad de interpretar el mapa de color, la escala esta en segundos
    # circle1 = plt.Circle((2, 4), 0.2, color='black', fill = False, label = 'Error zone')          # Comentar o descomentar para graficar un circulo con centro en las coordenadas XY de radio de 0.2
    # circle2 = plt.Circle((1.47, 3.15), 0.2, color='black', fill = False, label = 'Error zone')    # Comentar o descomentar para graficar un circulo con centro en las coordenadas XY de radio de 0.2
    # plt.gca().add_patch(circle1)
    # plt.gca().add_patch(circle2)   
    plt.xlabel('Position in X [m]') # Se nombra al eje x como position in X [m]
    plt.ylabel('Position in Y [m]') # Se nombra al eje y como position in Y [m]
    plt.title('XY Plane') # Se agrega un titulo para la grafica 
    plt.grid() # Se agrega una rejilla en el fondo de la gráfica

    ############## Height vs Time ##################

    plt.figure('Height vs Time') # Crea una figura nueva con el nombre Height vs Time
    plt.scatter(df['time']/10000, df['stateEstimate.z'], label = 'Measures height drone 1') # Se grafica la posición de la altura del drone 1 contra el tiempo
    plt.scatter(df1['time']/10000, df1['stateEstimate.z'], label = 'Real height drone 2', color = 'green') # Se grafica la posición de la altura del drone 2 contra el tiempo
    plt.grid() # Se agrega una rejilla en el fondo de la gráfica
    plt.axhline(1.2, color ='red', label = 'Max') # Grafica una linea horizontal para representar el limite superior de los requerimientos propuestos
    plt.axhline(0.8, color = 'black', label = 'Min') # Grafica una linea horizontal para representar el limite inferior de los requerimientos propuestos
    plt.xlabel('Time [s]') # Se nombra al eje x como Time [s]
    plt.ylabel('Height [m]') # Se nombra el eje y como Height [m]
    plt.legend() # Se agrega la leyenda para identificar los datos de la gráfica
    plt.title('Height vs Time') # Se agrega un titulo para la gráfica 
    plt.show() # Se despliegan las gráficas generadas

file = r'D:\Downloads\01_06_23_Prueba027_3_LiderCon2Drones_drone1.csv'
file1 = r'D:\Downloads\01_06_23_Prueba027_3_LiderCon2Drones_drone2.csv'
df = pd.read_csv(file, index_col= 0)
df1 = pd.read_csv(file1, index_col= 0)

graph(df,df1)