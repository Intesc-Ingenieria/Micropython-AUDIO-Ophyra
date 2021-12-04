"""
----- REPRODUCTOR DE AUDIO WAV -----

- WAV_FILE ---> Nombre del archivo .wav
- seg --------> Tiempo en segundos para reproducir el audio
"""

from pyb import DAC
from pyb import delay

class wav:

    def play(WAV_FILE, seg, canal, muestreo):
        #-------------CONFIGURACIÓN DE AUDIO---------------
        dac = DAC(canal, bits=8)        #Inicialización del DAC en modo de 8 bits / 8 bits de resolución.
        SAMPLE_RATE_IN_HZ = muestreo    #Tasa de muestreo en Hz

        #------- Inicio de reproducción -------
        ini = 44  #Se ignora el encabezado del .WAV
        for i in range(seg+1):
            wav = open(WAV_FILE, "rb") #Apertura del archivo de audio 
            pos = wav.seek(ini)

            #---- Declaración de buffers de datos ----
            wav_samples = bytearray(SAMPLE_RATE_IN_HZ)  #Matriz de bytes equivalentes a 1 seg de reproducción
            wav_samples_mv = memoryview(wav_samples)

            #---- Escritura en buffer DAC ----
            try:
                num_read =wav.readinto(wav_samples_mv) #Se escriben los datos del .WAV al buffer de datos
                if num_read == 0:
                    pos = wav.seek(ini) #Se ignora el encabezado del .WAV
                else:
                    dac.write_timed(wav_samples_mv[:num_read], SAMPLE_RATE_IN_HZ, mode=DAC.CIRCULAR) #Se escriben los datos del buffer al DAC
            except (KeyboardInterrupt, Exception) as e:
                print("caught exception {} {}".format(type(e).name, e))

            print("Reproduciendo... " + str(i) +" seg")
            ini = ini + SAMPLE_RATE_IN_HZ  #Se mueve el cursor para leer el siguiente segundo del archivo .WAV
            wav.close() #Se cierra el archivo de audio
            delay(1000)  #Espera 1 seg, tiempo para reproducir el sonido en el amplificador
        dac.deinit()  #Se desactiva el DAC
