from machine import I2S
from machine import Pin
from pyb import delay

#Congiguración de audio
WAV_SAMPLE_SIZE_IN_BITS = 32
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 22050

#Configuración de Pines I2S2 (No se conocen las conexiones a los otros pines)
SCK_PIN = "B13"
SD_PIN = "C3"
WS_PIN = "B12"
I2S_ID = 2
BUFFER_LENGTH_IN_BYTES = 40000

sck_pin = Pin(SCK_PIN)
sd_pin = Pin(SD_PIN)
ws_pin = Pin(WS_PIN)

#################### Callback para llenar el buffer en un proceso secundario (Método que no bloquea)################################
def i2s_callback_rx(arg):
	global mic_samples
	global num_read
	num_read = audio_in.readinto(mic_samples)
############################################################################
audio_in = I2S(2,sck = sck_pin, ws = ws_pin,sd = sd_pin, mode = I2S.RX, bits = WAV_SAMPLE_SIZE_IN_BITS, format = FORMAT, rate = SAMPLE_RATE_IN_HZ, ibuf = BUFFER_LENGTH_IN_BYTES)


mic_samples = bytearray(10000)
#mic_samples_mv = memoryview(mic_samples)


for i in range(10):
	num_read = audio_in.readinto(mic_samples)
	print (num_read)
	delay(1)


#Se debe cerrar el I2S, de lo contrario causa conflictos con la comunicación serial con Putty y se reinicia la tarjeta
audio_in.deinit()