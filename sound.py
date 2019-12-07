import RPi.GPIO as GPIO
import time

channel = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.IN)

def callback(channel):
    print(GPIO.input(channel))
    if GPIO.input(channel):
        print("Sound detected")
    else:
        print("Sound not detected")

GPIO.add_event_detect(channel, GPIO.BOTH, bouncetime=300)
GPIO.add_event_callback(channel, callback)

while 1:
    time.sleep(1)