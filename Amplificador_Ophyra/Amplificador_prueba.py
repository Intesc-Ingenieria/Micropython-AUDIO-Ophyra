from pyb import DAC
from pyb import delay

dac = DAC(1, bits=8)          #Inicialización del DAC1 en modo de 8 bits

#-------------CONFIGURACIÓN DE AUDIO---------------
WAV_FILE = "test.wav"         #Archivo .WAV a reproducir
WAV_SAMPLE_SIZE_IN_BITS = 8   #Bits de resolución
SAMPLE_RATE_IN_HZ = 8000      #Tasa de muestreo de 8KHz
seg = 120                     #Segundos que tendra la reproducción del archivo .WAV


"""El Bucle While se integra únicamente para repetir el audio después de transcurrir el tiempo que el usuario ingreso.  
"""

print("-------- INICIO DE LA REPRODUCCIÓN --------")
n = 0
while n < 1:
    lon = 44 #Se ignora el encabezado del .WAV
    for i in range(seg):
        wav = open(WAV_FILE, "rb") #Apertura del archivo de audio 
        pos = wav.seek(lon)

        #---- Declaración de buffers de datos ----
        wav_samples = bytearray(8000) #Matriz de bytes equivalentes a 1 seg de reproducción
        wav_samples_mv = memoryview(wav_samples)

        #---- Inicio de reproducción del archivo ----
        try:
            num_read =wav.readinto(wav_samples_mv) #Se escriben los datos del .WAV al buffer de datos
            if num_read == 0:
                pos = wav.seek(lon) 
            else:
                dac.write_timed(wav_samples_mv[:num_read], SAMPLE_RATE_IN_HZ, mode=DAC.CIRCULAR) #Se escriben los datos del buffer al DAC
        except (KeyboardInterrupt, Exception) as e:
            print("caught exception {} {}".format(type(e).name, e))

        wav.close()                   #Se cierra el archivo de audio
        print("Reproduciendo..." )
        lon = lon + 8000              #Se mueve el cursor para leer el siguiente segundo del archivo .WAV
        delay(1000)                   #Se espera 1 seg, tiempo para reproducir el sonido en el amplificador
