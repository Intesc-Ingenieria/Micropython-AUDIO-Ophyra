import wav_ophyra  #Importamos la libreria del reproductor WAV

#-------------CONFIGURACIÓN DE AUDIO---------------
WAV_FILE = "test1.wav"        #Archivo .WAV a reproducir
seg = 120                    #Segundos que tendra la reproducción del archivo .WAV

#Agregamos un bucle while para ciclar la reproducción del audio
n = 0
while n < 1:
    wav_ophyra.play(WAV_FILE,seg)

