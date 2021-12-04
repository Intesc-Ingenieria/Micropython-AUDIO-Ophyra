from wav_ophyra import wav 

print("   ---------------------------------")
print("   | REPRODUCTOR DE AUDIO - OPHYRA |")
print("   ---------------------------------")

#-------------CONFIGURACIÓN DE AUDIO---------------
archivo = "test2.wav"  #Archivo .WAV a reproducir
seg = 120                 #Segundos que tendra la reproducción del archivo .WAV
canal = 1                #Inicialización del DAC (DAC1 o DAC2)
muestreo = 16000          #Tasa de muestreo en Hz (recomendado a 8 KHz o 16 KHz)

#Agregamos un bucle while para ciclar la reproducción del audio
while 1:
    print("\n -------- INICIO DE AUDIO --------")
    wav.play(archivo,seg,canal,muestreo)
    print("--------- FIN DE AUDIO ---------")
