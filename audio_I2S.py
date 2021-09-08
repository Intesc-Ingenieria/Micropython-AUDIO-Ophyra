from pyb import Pin
from machine import I2S 
from pyb import delay
from pyb import DAC
from fir import fir
import math
import array
import gc


# =================== Uso de memoria en bytes ================================= #
#El buffer interno es 4 veces más grande que el tamaño declarado, debido a que la API requiere esto para hacer conversión en palabras de 16 o 32 bits.
#Dos bytearrays son declarados para guardar las muestras. El filteredValueBuff debe ser más grande que mic_samples,
#debido a que hay más muestras que se están procesando durante el diezmado. 
#La str_chain guarda los bits en forma de una cadena de texto, lo que requiere cierta memoria. Es, aproximadamente, 6 veces el tamaño de mic_samples


#El microcontrolador tiene una memoria total disponible de aproximadamente 100Kbytes

# NOMBRE DE BUFFER O VARIABLE            TAMAÑO DECLARADO en bytes           TAMAÑO REAL en bytes
# Internal buffer                             5,000                              20,000
# filtered buffer                             10,000                             10,000
# mic_samples                                 5,000                              5,000
# str_chain                                 mic_samples *8                       40,000***
#______________________________________________________________________________________________
#USO TOTAL DE MEMORIA                                                           70,500 bytes 

#El garbage collector (gc) es usado para evitar problemas de fuga de memoria cuando se desecha la cadena de texto. 
# =================== CONFIGURACIÓN de PARÁMETROS DE AUDIO ======================== #
WAV_FILE = "mic.wav"
WAV_SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 44100
BUFFER_LENGTH_IN_BYTES = 5000

# ====================== CONFIGURACIÓN DE I2S  ================================== #

sck_pin = Pin("B13")
ws_pin = Pin("B12")
sd_pin = Pin("C3")
I2S_ID = 2

audio_in = I2S(2,sck = sck_pin, ws = ws_pin, sd = sd_pin, mode = I2S.RX, bits = WAV_SAMPLE_SIZE_IN_BITS, format = I2S.MONO, rate = SAMPLE_RATE_IN_HZ, ibuf = BUFFER_LENGTH_IN_BYTES)

mic_samples = bytearray(5000)
str_chain = ""

dac = DAC(1, bits = 8)

# ======================= COEFICIENTES DE FILTRO PASABAJAS Y DATA ================================#
#Filter desgined for pass band at 20 to 6.5kHz, stopband at 18kHz
#Filtro diseñado para una banda de paso ubicada de 20Hz a 6.5kHz. La banda de paro comienza a los 18kHz. Diseñado para una frecuencia de muestreo de 44100 Hz
#Todos los coeficientes deben declararse en formato entero, no decimal y es el único parámetro que deberá modificarse.
#Las demás variables se calculan en función de los datos en el array.

coeffs = array.array('i', (-131139,-648174,-921587,919196,4878777,7109966,4878777,919196,-921587,-648174,-131139))
#coeffs = array.array('i', (72, 47, 61, 75, 90, 105, 119, 132, 142, 149, 152, 149,
#  140, 125, 102, 71, 33, -12, -65, -123, -187, -254, -322, -389, -453, -511, -561,
#  -599, -622, -628, -615, -579, -519, -435, -324, -187, -23, 165, 375, 607, 855,
#  1118, 1389, 1666, 1941, 2212, 2472, 2715, 2938, 3135, 3303, 3437, 3535, 3594,
#  3614, 3594, 3535, 3437, 3303, 3135, 2938, 2715, 2472, 2212, 1941, 1666, 1389,
#  1118, 855, 607, 375, 165, -23, -187, -324, -435, -519, -579, -615, -628, -622,
#  -599, -561, -511, -453, -389, -322, -254, -187, -123, -65, -12, 33, 71, 102, 125,
#  140,  149, 152, 149, 142, 132, 119, 105, 90, 75, 61, 47, 72))
ncoeffs = len(coeffs)
data = array.array('i', [0]*(ncoeffs +3)) # Scratchpad must be three larger than coeffs
data[0] = ncoeffs
data[1] = 16
filteredValueBuff = bytearray(10000)
# ============================================================================================ #

k = 0

while True:
  audio_in.readinto(mic_samples) #Lectura del micrófono al buffer
  for i in range(len(mic_samples)):
    str_chain += bin(mic_samples[i]).replace("0b","")#Se separa cada byte y se representa en su cadena de bits en formato de texto. 
    #Se elimina el '0b' presente en cada cadena de bits
  for j in range(len(str_chain)): #Se recorre la cadena de bits
    if(j % 64 == 0):
      continue
      #Por cada 64 bits, se ignora todo un byte de datos
    else: 
      if(len(str_chain[j:j+8]) == 8): #Asegura que el tamaño sea de 8 bits
        decimalValue = int("0b"+str_chain[j:j+8]) #Convierte el byte a un valor decimal
        if(k > 2000): #Condición no necesaria, se utiliza con fines de debugeo para no saturar memoria
          break
        else:
          filteredValueBuff[k] = fir(data, coeffs, decimalValue)#Envío de los valores decimales al filtro pasabajas, el cual retorna  un valor decimal y lo guarda en el buffer.
          j+=1
          k+=1

      else:
        continue
  
  #Escritura de datos filtrados al DAC    
  dac.write_timed(filteredValueBuff, SAMPLE_RATE_IN_HZ, mode = DAC.NORMAL)
  
  #Se devuelve el tamaño en bytes que ocupa la cadena de texto. Línea con propósitos de debuggeo.
  print(len(str_chain.encode('utf-8')))
  
  str_chain = "" #Se limpia la cadena de bits
  gc.collect() #Se limpian los datos y objetos no usados para no saturar memoria

audio_in.deinit() #Desactivación del bus I2S