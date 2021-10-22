from pyb import DAC

dac = DAC(1, bits=8) #Inicialización del DAC_1 en modo de 8 bits

#-------------CONFIGURACIÓN DE AUDIO---------------
WAV_FILE = "harmony.wav" #Archivo .WAV a reproducir
WAV_SAMPLE_SIZE_IN_BITS = 8

SAMPLE_RATE_IN_HZ = 8000 #Tasa de muestreo de 8KHz

wav = open(WAV_FILE, "rb") #Apertura del archivo de audio 
pos = wav.seek(44)

#Declaración de buffers de datos
wav_samples = bytearray(65000)
wav_samples_mv = memoryview(wav_samples)

#Inicio de reproducción del archivo
print("-------- INICIO DE LA REPRODUCCIÓN --------")
try:
    num_read =wav.readinto(wav_samples_mv) #Se escriben los datos del .WAV al buffer de datos
    if num_read == 0:
        pos = wav.seek(44) #Se ignora el encabezado del .WAV
    else:
        dac.write_timed(wav_samples_mv[:num_read], SAMPLE_RATE_IN_HZ, mode=DAC.CIRCULAR) #Se escriben los datos del buffer al DAC
except (KeyboardInterrupt, Exception) as e:
    print("caught exception {} {}".format(type(e).name, e))

wav.close() #Se cierra el archivo de audio
print("Done")
