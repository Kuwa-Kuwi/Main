import RPi.GPIO as GPIO
from time import sleep
#Disable warnings (optional)
GPIO.setwarnings(False)
#Select GPIO mode
GPIO.setmode(GPIO.BCM)
#Set buzzer - pin 23 as output
buzzer=19
GPIO.setup(buzzer, GPIO.OUT)
#Run forever loop
flag_beep = False
try:
	while True:
		flag_beep = not flag_beep
		GPIO.output(buzzer, GPIO.LOW)
		sleep(1)
    		#print ("Beep")
		GPIO.output(buzzer, GPIO.HIGH)
    		#print ("No Beep")
		sleep(3)
		print(flag_beep)
except KeyboardInterrupt:
	GPIO.cleanup()

finally:
	GPIO.cleanup()

