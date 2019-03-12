import smbus
import time
import math
import RPi.GPIO as GPIO

from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

from time import sleep, strftime
from datetime import datetime

address = 0x48	#default address of PCF8591
bus=smbus.SMBus(1)
cmd=0x40		#command

PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.

def setup():
	GPIO.setmode(GPIO.BOARD)

def analogRead(chn):#read ADC value,chn:0,1,2,3
	value = bus.read_byte_data(address,cmd+chn)
	return value

def analogWrite(value):#write DAC value
	bus.write_byte_data(address,cmd,value)

def displayVoltage():
	value = analogRead(0)	#read the ADC value of channel 0
	analogWrite(value)		#write the DAC value
	voltage = value / 255.0 * 3.3  #calculate the voltage value
	print ('ADC Value : %d, Voltage : %.2f'%(value,voltage))
	return '{:.2f}'.format( float(voltage)/1000 ) + ' V'


def displayTemperature():
	value = analogRead(1)		#read A1 pin
	voltage = value / 255.0 * 3.3		#calculate voltage
	Rt = 10 * voltage / (3.3 - voltage)	#calculate resistance value of thermistor
	tempK = 1/(1/(273.15 + 25) + math.log(Rt/10)/3950.0) #calculate temperature (Kelvin)
	tempC = tempK -273.15		#calculate temperature (Celsius)
	print ('ADC Value : %d, Voltage : %.2f, Temperature : %.2f'%(value,voltage,tempC))
	return '{:.2f}'.format( float(tempC)/1000 ) + ' C'



def loop(caption):
	mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2)     # set number of LCD lines and columns

	while True:
		lcd.setCursor(0,0)  # set cursor position
		if caption == 'Voltage':
			lcd.message( caption + ': ' + displayVoltage() +'\n' )# displays the caption and value
		elif caption == 'Temp':
			lcd.message( caption + ': ' + displayTemperature() +'\n' )# displays the caption and value
        sleep(1)


def destroy():
	lcd.clear()
	GPIO.cleanup()
	bus.close()

# Create PCF8574 GPIO adapter.
try:
	mcp = PCF8574_GPIO(PCF8574_address)
except:
	try:
		mcp = PCF8574_GPIO(PCF8574A_address)
	except:
		print ('I2C Address Error !')
		exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)


if __name__ == '__main__':
	print ('Program is starting ... ')
	try:
		loop() #put whether to display temerature or voltage
	except KeyboardInterrupt:
		destroy()
