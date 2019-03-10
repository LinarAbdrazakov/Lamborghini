import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)

CH1 = 13
CH2 = 19
CH3 = 26
CH4 = 21
CH5 = 20
CH6 = 16

GPIO.setup(CH1, GPIO.IN)
GPIO.setup(CH2, GPIO.IN)
GPIO.setup(CH3, GPIO.IN)
GPIO.setup(CH4, GPIO.IN)
GPIO.setup(CH5, GPIO.IN)
GPIO.setup(CH6, GPIO.IN)


def read_channel(channel):
    while GPIO.input(channel) == 0:
        pass
    start = time.time()
    while GPIO.input(channel) == 1:
        pass
    stop = time.time()
    delay = (stop - start) * 1000

    return delay


def receive():
        delay_1 = read_channel(CH1)
        delay_2 = read_channel(CH2)
        delay_3 = read_channel(CH3)
        delay_4 = read_channel(CH4)
        delay_5 = read_channel(CH5)
        delay_6 = read_channel(CH6)

        return [delay_1, delay_2, delay_3, delay_4, delay_5, delay_6]


try:
    while True:
        print([int(1000*d) for d in receive()])

finally:
    GPIO.cleanup()
