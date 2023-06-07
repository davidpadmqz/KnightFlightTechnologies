##################################################################################################################################################
# Autores: Equipo Knight Flight Technologies
# Fecha de creación: Semana 1  
# Fecha de última modificación: Semana 9
# Descripción: Obtiene el valor máximo y minimo de algunos párametros para fines de verificación y validación
# Formato de nombrar variables y funciones: Minúsculas y descriptivas
# Comentado por: David Padilla

##################################################################################################################################################

import os
import pandas as pd

path = "D:\Downloads\Logs_pruebas" # Crea la ruta para abrir el folder que posteriormente definiremos como dir
dir = os.listdir( path ) # Abre la dirección definida en path
df1 = pd.DataFrame() # Crea un dataframe vacio

for file in dir: # Itera en los diferentes archivos guardados dentro de la carpeta dir
    df = pd.read_csv(path + r'\\' + file, index_col = 0)  # Se leen los diferentes archivos .csv y los convierte a un Dataframe de pandas  
    df1 =df1.append(df,ignore_index = True) # Se agregan los dataframes en cada iteración al dataframe que definimos antes del ciclo for

print('La velocidad máxima en x es:', df1['stateEstimate.vx'].max()) # Despliega en terminal el valor máximo de la velocidad en el eje X [m/s]
print('La velocidad máxima en y es:', df1['stateEstimate.vy'].max()) # Despliega en terminal el valor máximo de la velocidad en el eje Y [m/s]
print('La velocidad máxima en z es:', df1['stateEstimate.vz'].max()) # Despliega en terminal el valor máximo de la velocidad en el eje Z [m/s]
print('El ángulo máximo en yaw es:', df1['stabilizer.yaw'].max()) # Despliega en terminal el valor máximo del ángulo en yaw [°]
print('El ángulo minimo en yaw es:', df1['stabilizer.yaw'].min()) # Despliega en terminal el valor minimo del ángulo en yaw [°]

