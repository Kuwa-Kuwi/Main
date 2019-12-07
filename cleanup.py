import RPi.GPIO as GPIO


LED_GREEN_PINS = [26, 25]
LED_YELLOW_PINS = [16, 27]
LED_RED_PINS = [23, 24]
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_GREEN_PINS, GPIO.OUT)
GPIO.setup(LED_YELLOW_PINS, GPIO.OUT)
GPIO.setup(LED_RED_PINS, GPIO.OUT)

GPIO.cleanup()