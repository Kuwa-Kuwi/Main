from datetime import datetime
from time import sleep
import serial
import RPi.GPIO as GPIO
import requests

BUZZER_PIN = 4
SENSITIVITY_THRESHOLD = 30
# GPIO Init
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
#
# Set serial mode
ser = serial.Serial('/dev/ttyACM0', 9600)
try:
	while True:
		read_serial = ser.readline().decode('utf-8')
		print((int(read_serial)))
		sleep(0.1)
		if int(read_serial) > SENSITIVITY_THRESHOLD:
			GPIO.output(BUZZER_PIN, GPIO.LOW)
			sleep(0.5)
			GPIO.output(BUZZER_PIN, GPIO.HIGH)
			sleep(2)

			# KIRIM INFO KE WEB APP
			timestamp = datetime.now()
			data = {
				'timestamp': timestamp,
				'decibel': int(read_serial),
				}
			requests.post(
				'http://192.168.4.1:8000/noise-log/create-noise-log/',
				data
			) 
		else:
			GPIO.output(BUZZER_PIN, GPIO.HIGH)
except KeyboardInterrupt:
	GPIO.cleanup()
finally:
	GPIO.cleanup()

#import RPi.GPIO as GPIO
#from time import sleep
##Disable warnings (optional)
#GPIO.setwarnings(False)
##Select GPIO mode
#GPIO.setmode(GPIO.BCM)
##Set buzzer - pin 23 as output
#buzzer=17
#GPIO.setup(buzzer,GPIO.OUT)
##Run forever loop
#
#try:
#	while True:
#    		GPIO.output(buzzer,GPIO.HIGH)
#    		#print ("Beep")
#    		sleep(2) # Delay in seconds
#    		GPIO.output(buzzer,GPIO.LOW)
#    		#print ("No Beep")
#    		sleep(2)
#except KeyboardInterrupt:
#	GPIO.cleanup()

