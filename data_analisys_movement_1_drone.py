##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 9
# Descripción: Gráfica el movimiento de un drone para análisis de datos
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

##################################################################################################################################################

from matplotlib import pyplot as plt
import pandas as pd

def graph(df):

    ############## 3D Graph ##################
    
    plt.figure('3D trajectory mapping')
    ax = plt.axes(projection ="3d")
    plot = ax.scatter3D(df['stateEstimate.x'],df['stateEstimate.y'],df['stateEstimate.z'], c = df['time']/10000 ,cmap = 'plasma')    
    ax.set_xlabel('position in X [m]')
    ax.set_ylabel('Position in Y [m]')
    ax.set_zlabel('Position in Z [m]')   
    plt.colorbar(plot,cmap=plt.cm.magma,label = 'Time [s]')
    plt.title('3D trajectory mapping')

    ############## XY Plane ##################

    plt.figure('XY Plane')
    plot = plt.scatter(df['stateEstimate.x'], df['stateEstimate.y'], c = df['time']/10000, cmap='plasma')
    circle1 = plt.Circle((2, 4), 0.2, color='black', fill = False, label = 'Error zone') 
    plt.gca().add_patch(circle1) 
    plt.colorbar(plot, cmap=plt.cm.magma, label = 'Time [s]')    
    plt.xlabel('Position in X [m]')
    plt.ylabel('Position in Y [m]')
    plt.title('XY Plane')
    plt.legend()
    plt.grid()

    ############## Height vs Time ##################

    plt.figure('Height vs Time')
    plt.scatter(df['time']/10000, df['stateEstimate.z'], label = 'Measures height drone')  
    plt.grid()
    plt.axhline(1.2, color ='red', label = 'Max')
    plt.axhline(0.8, color = 'black', label = 'Min')
    plt.xlabel('Time [s]')
    plt.ylabel('Height [m]')
    plt.legend()
    plt.title('Height vs Time')
    plt.show()

file = r'D:\Downloads\logs_pruebas1dron_31Mayo\31_05_23_Prueba018_3_Punto3Dcon1dron_TDOA2.csv'
df = pd.read_csv(file, index_col= 0)
graph(df)