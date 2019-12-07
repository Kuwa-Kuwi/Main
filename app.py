from datetime import datetime
from time import sleep
import serial
import RPi.GPIO as GPIO
import smbus
import requests

# PINS
BUZZER_PIN = 4
LED_GREEN_PINS = [26, 25]
LED_YELLOW_PINS = [16, 22]
LED_RED_PINS = [23, 24]

# SOUND INTENSITIES SETTING
LOW_INTENSITY = 30
MEDIUM_INTENSITY = 35
HIGH_INTENSITY = 40

# GPIO Init
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_GREEN_PINS, GPIO.OUT)
GPIO.setup(LED_YELLOW_PINS, GPIO.OUT)
GPIO.setup(LED_RED_PINS, GPIO.OUT)

# URL
WEB_URL = 'http://192.168.4.1:8000/noise-log/create-noise-log/'

class LCD:
    
    # Define some device parameters
    I2C_ADDR  = 0x3f # I2C device address, if any error, change this address to 0x3f
    LCD_WIDTH = 16   # Maximum characters per line

    # Define some device constants
    LCD_CHR = 1 # Mode - Sending data
    LCD_CMD = 0 # Mode - Sending command

    LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
    LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
    LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
    LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

    LCD_BACKLIGHT  = 0x08  # On
    #LCD_BACKLIGHT = 0x00  # Off

    ENABLE = 0b00000100 # Enable bit

    # Timing constants
    E_PULSE = 0.0005
    E_DELAY = 0.0005

    #Open I2C interface
    #bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
    bus = smbus.SMBus(1) # Rev 2 Pi uses 1

    def __init__(self):
        self.lcd_init()

    def lcd_init(self):
        # Initialise display
        self.lcd_byte(0x33, self.LCD_CMD) # 110011 Initialise
        self.lcd_byte(0x32, self.LCD_CMD) # 110010 Initialise
        self.lcd_byte(0x06, self.LCD_CMD) # 000110 Cursor move direction
        self.lcd_byte(0x0C, self.LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
        self.lcd_byte(0x28, self.LCD_CMD) # 101000 Data length, number of lines, font size
        self.lcd_byte(0x01, self.LCD_CMD) # 000001 Clear display
        sleep(self.E_DELAY)

    def lcd_byte(self, bits, mode):
        # Send byte to data pins
        # bits = the data
        # mode = 1 for data
        #        0 for command

        bits_high = mode | (bits & 0xF0) | self.LCD_BACKLIGHT
        bits_low = mode | ((bits<<4) & 0xF0) | self.LCD_BACKLIGHT

        # High bits
        self.bus.write_byte(self.I2C_ADDR, bits_high)
        self.lcd_toggle_enable(bits_high)

        # Low bits
        self.bus.write_byte(self.I2C_ADDR, bits_low)
        self.lcd_toggle_enable(bits_low)
    
    def lcd_toggle_enable(self, bits):
        # Toggle enable
        sleep(self.E_DELAY)
        self.bus.write_byte(self.I2C_ADDR, (bits | self.ENABLE))
        sleep(self.E_PULSE)
        self.bus.write_byte(self.I2C_ADDR,(bits & ~self.ENABLE))
        sleep(self.E_DELAY)
    
    def lcd_string(self, message, line):
        # Send string to display
        message = message.ljust(self.LCD_WIDTH, " ")
        self.lcd_byte(line, self.LCD_CMD)
        for i in range(self.LCD_WIDTH):
            self.lcd_byte(ord(message[i]), self.LCD_CHR)                        

# Set serial mode
ser = serial.Serial('/dev/ttyACM0', 9600)
lcd = LCD()
try:
    while True:
        read_serial = int(ser.readline().decode('utf-8'))
        print(read_serial)
        sleep(0.1)

        if read_serial < LOW_INTENSITY or LOW_INTENSITY <= read_serial < MEDIUM_INTENSITY:
            GPIO.output(LED_GREEN_PINS, GPIO.HIGH)
            GPIO.output(LED_YELLOW_PINS, GPIO.LOW)
            GPIO.output(LED_RED_PINS, GPIO.LOW)
            lcd.lcd_string(f"DECIBEL: {str(read_serial)}", lcd.LCD_LINE_1)
            lcd.lcd_string(f"CATEGORY: LOW", lcd.LCD_LINE_2)

        elif MEDIUM_INTENSITY <= read_serial < HIGH_INTENSITY:
            GPIO.output(LED_GREEN_PINS, GPIO.HIGH)
            GPIO.output(LED_YELLOW_PINS, GPIO.HIGH)
            GPIO.output(LED_RED_PINS, GPIO.LOW)
            lcd.lcd_string(f"DECIBEL: {str(read_serial)}", lcd.LCD_LINE_1)
            lcd.lcd_string(f"CATEGORY: MEDIUM", lcd.LCD_LINE_2)
            
        elif read_serial >= HIGH_INTENSITY:
            GPIO.output(LED_GREEN_PINS, GPIO.HIGH)
            GPIO.output(LED_YELLOW_PINS, GPIO.HIGH)
            GPIO.output(LED_RED_PINS, GPIO.HIGH)
            lcd.lcd_string("A", lcd.LCD_LINE_1)
            lcd.lcd_string("B", lcd.LCD_LINE_2)

            GPIO.output(BUZZER_PIN, GPIO.LOW)
            sleep(0.5)

            timestamp = datetime.now()
            data = {
                'timestamp': timestamp,
                'decibel': read_serial,
            }
            requests.post(WEB_URL, data)

            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            sleep(2)
        else:
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
except KeyboardInterrupt:
    GPIO.cleanup()
    lcd.lcd_byte(0x01, lcd.LCD_CMD)
finally:
    GPIO.cleanup()
    lcd.lcd_byte(0x01, lcd.LCD_CMD)



