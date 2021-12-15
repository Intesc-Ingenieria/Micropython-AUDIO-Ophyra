# main.py -- put your code here!
# Date: 14 December 2021 
# Written by: Dionicio Meza Solano.

from ophyra_mp45dt02 import MP45DT02    #SE IMPORTA LA LIBRERIA DEL MICROFONO 
import time
from machine import Pin


WAV_FILE = "mic.wav"            # NOMBRE QUE RECIBIRA EL ARCHIVO DE GRABACION 
WAV_SAMPLE_SIZE_IN_BITS = 16    # TAMAÑO DE CADA MUESTRA EN BITS
SAMPLE_RATE_IN_HZ = 22000       # VELOCIDAD DE MUESTREO
NUM_CHANNELS = 1                # NUMERO DE CANALES (1 -> MONO)

RECORD = 0                      # ESTADO - GRABAR 
PAUSE = 1                       # ESTADO - PAUSA
RESUME = 2                      # ESTADO - REANUDAR
STOP = 3                        # ESTADO - DETENER

WAV_SAMPLE_SIZE_IN_BYTES = WAV_SAMPLE_SIZE_IN_BITS // 8                         # TAMAÑO DE CADA MUESTRA EN BYTES


def create_wav_header(sampleRate, bitsPerSample, num_channels, num_samples):    # ENCABEZADO DEL ARCHIVO .wav
    datasize = num_samples * num_channels * bitsPerSample // 8                     
    o = bytes("RIFF", "ascii")                                                  # (4byte) Marks file as RIFF
    o += (datasize + 36).to_bytes(4, "little")                                  # (4byte) File size in bytes excluding this and RIFF marker
    o += bytes("WAVE", "ascii")                                                 # (4byte) File type
    o += bytes("fmt ", "ascii")                                                 # (4byte) Format Chunk Marker
    o += (16).to_bytes(4, "little")                                             # (4byte) Length of above format data
    o += (1).to_bytes(2, "little")                                              # (2byte) Format type (1 - PCM)
    o += (num_channels).to_bytes(2, "little")                                   # (2byte)
    o += (sampleRate).to_bytes(4, "little")                                     # (4byte)
    o += (sampleRate * num_channels * bitsPerSample // 8).to_bytes(4, "little") # (4byte)
    o += (num_channels * bitsPerSample // 8).to_bytes(2, "little")              # (2byte)
    o += (bitsPerSample).to_bytes(2, "little")                                  # (2byte)
    o += bytes("data", "ascii")                                                 # (4byte) Data Chunk Marker
    o += (datasize).to_bytes(4, "little")                                       # (4byte) Data size in bytes
    return o


def mic_callback_rx(arg):                                                       # DEVOLUCION DE LLAMADA .irq() (BUFFER PROPORCIONADO LLENO)
    global state                                                                # SE IMPORTAN LAS VARIABLES GLOBALES
    global num_sample_bytes_written_to_wav
    global mic_samples_mv
    global num_read

    if state == RECORD:                                                         # -GRABAR- 
        num_bytes_written = wav.write(mic_samples_mv[:num_read])                # SE ESCRIBEN LAS MUESTRAS OBTENIDAS EN EL ARCHIBO .wav 
        num_sample_bytes_written_to_wav += num_bytes_written                    # NUMERO DE BYTES ESCRITOS EN EL ARCHIVO
        num_read = mic.readinto(mic_samples_mv)                                 # SE LLENA EL BUFFER DE MUESTRAS
    
    elif state == RESUME:                                                       # -REANUDAR-
        state = RECORD                                                          # EL ESTADO VULVE A -GRABAR-            
        num_read = mic.readinto(mic_samples_mv)                                 # SE LLENA EL BUFFER DE MUESTRAS (NO SE ESCRIBEN EN EL ARCHIVO .wav)

    elif state == PAUSE:                                                        # -PAUSA-
        num_read = mic.readinto(mic_samples_mv)                                 # SE LLENA EL BUFFER DE MUESTRAS (NO SE ESCRIBEN EN EL ARCHIVO .wav)

    elif state == STOP:                                                         # -DETENER-
        wav_header = create_wav_header(                                         # SE CREA EL ENCABEZADO DEL DEL ARCHIVO .wav
            SAMPLE_RATE_IN_HZ,
            WAV_SAMPLE_SIZE_IN_BITS,
            NUM_CHANNELS,
            num_sample_bytes_written_to_wav // (WAV_SAMPLE_SIZE_IN_BYTES * NUM_CHANNELS),
        )
        _ = wav.seek(0)                                                         # POSICION INICIAL DEL ARCHIVO
        num_bytes_written = wav.write(wav_header)                               # SE ESCRIBE EL ENCABEZADO DEL ARCHIVO .wav
        
        wav.close()                                                             # SE CIERRA EL ARCHIVO .wav
        
        mic.deinit()                                                            # SE DETIENEN EL FUNCIONAMIENTO DEL MICROFONO
        print("Done")
    else:
        print("Not a valid state.  State ignored")


wav = open(WAV_FILE, "wb")                          # SE CREA EL ARCHIVO .wav PARA ESCRITURA
pos = wav.seek(44)                                  # POSICION 44 DONDE INICIAN LOS DATOS DE AUDIO 

mic = MP45DT02()                                    # SE CREA EL OBJETO MICROFONO

mic.irq(mic_callback_rx)                            # ESTA INTERRUPCION INDICA QUE EL BUFFER PROPORCIONADO ESTÁ LLENO

mic_samples = bytearray(15000)                      # BUFFER QUE ALMACENA LOS DATOS DE AUDIO
mic_samples_mv = memoryview(mic_samples)            # PERMITE ACCEDER AL BUFFER SIN CREAR UNA COPIA

num_sample_bytes_written_to_wav = 0

state = PAUSE                                       # ESTADO INICIAL -PAUSA-

num_read = mic.readinto(mic_samples_mv)             # SE REALIZA LA PRIMERA LECTURA DEL MICROFONO, CUANDO EL BUFFER SE LLENA SE CREA LA INTERRUPCION
                                                    # SE CONTINUA EN SEGUNDO PLANO


print("starting recording for 15s")
state = RECORD                                      # SE COMIENZA LA GRABACION
time.sleep(15)                                      # EL TIEMPO ESTABLECIDO DETERMINA EL TIEMPO DE GRABACION
print("stopping recording and closing WAV file")
state = STOP                                        # SE TERMINA LA GRABACION
