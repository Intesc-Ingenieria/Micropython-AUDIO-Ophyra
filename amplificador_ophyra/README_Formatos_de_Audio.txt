- Para tener una lectura correcta en el programa de prueba es necesario configurar 
correctamente los parametros: Bits de resolución y Tasa de muestreo.

IMPORTANTE que los archivos tengan 8 bits de resolución y esten en formato mono canal,
la frecuencia de muestreo puede cambiar, pero se recomienda dejarla en 8 KHz o 16 KHz.

----------------------------------------
|          ARCHIVOS DE AUDIO           |
| - test1 ---> (8 bits, 8 KHz, mono)   |
| - test2 ---> (8 bits, 16 KHz, mono)  |
----------------------------------------

NOTA: la tasa de muestreo se cambia en el archivo "wav_ophyra.py" en "SAMPLE_RATE_IN_HZ"
