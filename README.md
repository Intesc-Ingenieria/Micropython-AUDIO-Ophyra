# Micropython-AUDIO-Ophyra
Repositorio que integra librerías escritas en Micropython  para el funcionamiento del  microfono MP45DT02-PDM y el amplicicador de Audio de la tarjeta.


Este trabajo comprende los primeros pasos para la utilización del micrófono en la tarjeta Ophyra. 
A partir de julio de 2021, se agregó la clase I2S como Tech preview, por lo que los métodos utilizados en esta versión del código están sujetos a cambios. 
Esta clase está presente en Micropython 1.16, por lo que será necesario utilizar la última versión del firmware, así como realizar cambios en los archivos de compilación para activar el bus I2S. Información más detallada se encuentra dentro del reporte. 

# Códigos 
Este repositorio cuenta con 3 scripts de Python distintos. 
fir.py corresponde a un módulo diseñado para Micropython con el que se pueden implementar Filtros de Respuesta Finita. Se utiliza en el script audio_I2S.py.
audio_I2S.py es el código donde se realiza toda la recolección y procesamiento de audio. 

Por último, playWAVToDAC.py es un programa demo que permite reproducir un archivo WAV presente en la memoria de la tarjeta a través del DAC para garantizar el funcionamiento del amplificador de audio. 

# Reporte
El reporte presentado en este repositorio muestra a detalle el proceso de modificación de archivos desde la compilación, hasta un desglose del algoritmo pensado para el procesamiento de datos.

