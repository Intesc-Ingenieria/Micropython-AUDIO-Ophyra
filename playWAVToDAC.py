from machine import I2S
from machine import Pin
from pyb import DAC

dac = DAC(1, bits=8)

# ======= AUDIO CONFIGURATION =======
WAV_FILE = "a-team_crazy_fool_x.wav"
WAV_SAMPLE_SIZE_IN_BITS = 8

SAMPLE_RATE_IN_HZ = 8000
# ======= AUDIO CONFIGURATION =======


wav = open(WAV_FILE, "rb")
pos = wav.seek(44)  # advance to first byte of Data section in WAV file

# allocate sample array
# memoryview used to reduce heap allocation
wav_samples = bytearray(40000)
wav_samples_mv = memoryview(wav_samples)

# continuously read audio samples from the WAV file
# and write them to an I2S DAC
print("==========  START PLAYBACK ==========")
try:
        num_read = wav.readinto(wav_samples_mv)
        # end of WAV file?
        if num_read == 0:
            # end-of-file, advance to first byte of Data section
            pos = wav.seek(44)
        else:
            dac.write_timed(wav_samples_mv[:num_read], SAMPLE_RATE_IN_HZ, mode=DAC.CIRCULAR)

except (KeyboardInterrupt, Exception) as e:
    print("caught exception {} {}".format(type(e).name, e))

# cleanup
wav.close()
print("Done")