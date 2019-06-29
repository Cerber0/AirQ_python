#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____
#   / _ \/ _ \(_) __/__  __ __
#  / , _/ ___/ /\ \/ _ \/ // /
# /_/|_/_/  /_/___/ .__/\_, /
#                /_/   /___/
#
#           bme280.py
#  Read data from a digital pressure sensor.
#
#  Official datasheet available from :
#  https://www.bosch-sensortec.com/bst/products/all_products/bme280
#
# Author : Matt Hawkins
# Date   : 21/01/2018
#
# https://www.raspberrypi-spy.co.uk/
#
#--------------------------------------
import smbus
import time
import thingspeak
import sys
import Adafruit_DHT
import RPi.GPIO as GPIO
import numpy as np
#import pigpio

smoothingWindowLength = 400

from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte

DEVICE = 0x76 # Default device I2C address
THINGSPEAKKEY = '0UJ71LEQ5KZJ7PA5'
THINGSPEAKURL = 'https://api.thingspeak.com'
PARTCPIN = 27 
#particulas = 0
bus = smbus.SMBus(1) # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1
                     # Rev 1 Pi uses bus 0

inPINS = [PARTCPIN]

def getTimex():
  return time.time()

GPIO.setmode(GPIO.BCM)
GPIO.setup(PARTCPIN, GPIO.IN)
upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]

def my_callback1():
#  i = inPINS.index(channel)
  v = GPIO.input(inPINS[0])

  print ("jhhj")
  if (v==0):
    downTimes[0].append(getTimex())
#    print('Down={0:0.4f}'.format(downTimes[i]))
    if len(downTimes[0])>smoothingWindowLength: del downTimes[0][0]
  else:
    upTimes[0].append(getTimex())
#    print('Up={0:0.4f}'.format(upTimes[i]))
    if len(upTimes[0])>smoothingWindowLength: del deltaTimes[0][0]
  deltaTimes[0].append((downTimes[0][-1]-upTimes[0][-2]/upTimes[0][-1]-downTimes[0][-1]))
  if len(deltaTimes[0])>smoothingWindowLength: del deltaTimes[0][0]

def sendData(url,key,DHT22_temp,DHT22_humidity,BME_temp,BME_press,DSM501_particulas):
  """
  Send event to internet site
  """

  values = {'field1' : DHT22_temp,'field2' : DHT22_humidity,'field3' : BME_temp,'field4' : BME_press,'field5' : DSM501_particulas}

  log = time.strftime("%d-%m-%Y,%H:%M:%S") + ", "
  log = log + "{:.2f}C".format(DHT22_temp) + ", "
  log = log + "{:.2f}%".format(DHT22_humidity) + ", "
  log = log + "{:.2f}C".format(BME_temp) + ", "
  log = log + "{:.2f}hPa".format(BME_press) + ", "
  log = log + "{:.2f}".format(DSM501_particulas)

  print (log)

  ch = thingspeak.Channel(788430, key, 'json',10,url)
  ch.update(values)



def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index+1] << 8) + data[index]).value

def getUShort(data, index):
  # return two bytes from data as an unsigned 16-bit value
  return (data[index+1] << 8) + data[index]

def getChar(data,index):
  # return one byte from data as a signed char
  result = data[index]
  if result > 127:
    result -= 256
  return result

def getUChar(data,index):
  # return one byte from data as an unsigned char
  result =  data[index] & 0xFF
  return result

def readBME280ID(addr=DEVICE):
  # Chip ID Register Address
  REG_ID     = 0xD0
  (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
  return (chip_id, chip_version)

def readBME280All(addr=DEVICE):
  # Register Addresses
  REG_DATA = 0xF7
  REG_CONTROL = 0xF4
  REG_CONFIG  = 0xF5

  REG_CONTROL_HUM = 0xF2
  REG_HUM_MSB = 0xFD
  REG_HUM_LSB = 0xFE

  # Oversample setting - page 27
  OVERSAMPLE_TEMP = 2
  OVERSAMPLE_PRES = 2
  MODE = 1

  # Oversample setting for humidity register - page 26
  OVERSAMPLE_HUM = 2
  bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)

  control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
  bus.write_byte_data(addr, REG_CONTROL, control)

  # Read blocks of calibration data from EEPROM
  # See Page 22 data sheet
  cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
  cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
  cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)

  # Convert byte data to word values
  dig_T1 = getUShort(cal1, 0)
  dig_T2 = getShort(cal1, 2)
  dig_T3 = getShort(cal1, 4)

  dig_P1 = getUShort(cal1, 6)
  dig_P2 = getShort(cal1, 8)
  dig_P3 = getShort(cal1, 10)
  dig_P4 = getShort(cal1, 12)
  dig_P5 = getShort(cal1, 14)
  dig_P6 = getShort(cal1, 16)
  dig_P7 = getShort(cal1, 18)
  dig_P8 = getShort(cal1, 20)
  dig_P9 = getShort(cal1, 22)

  dig_H1 = getUChar(cal2, 0)
  dig_H2 = getShort(cal3, 0)
  dig_H3 = getUChar(cal3, 2)

  dig_H4 = getChar(cal3, 3)
  dig_H4 = (dig_H4 << 24) >> 20
  dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)

  dig_H5 = getChar(cal3, 5)
  dig_H5 = (dig_H5 << 24) >> 20
  dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

  dig_H6 = getChar(cal3, 6)

  # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
  wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575)
  time.sleep(wait_time/1000)  # Wait the required time  

  # Read temperature/pressure/humidity
  data = bus.read_i2c_block_data(addr, REG_DATA, 8)
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
  hum_raw = (data[6] << 8) | data[7]

  #Refine temperature
  var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
  var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
  t_fine = var1+var2
  temperature = float(((t_fine * 5) + 128) >> 8);

  # Refine pressure and adjust for temperature
  var1 = t_fine / 2.0 - 64000.0
  var2 = var1 * var1 * dig_P6 / 32768.0
  var2 = var2 + var1 * dig_P5 * 2.0
  var2 = var2 / 4.0 + dig_P4 * 65536.0
  var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
  var1 = (1.0 + var1 / 32768.0) * dig_P1
  if var1 == 0:
    pressure=0
  else:
    pressure = 1048576.0 - pres_raw
    pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
    var1 = dig_P9 * pressure * pressure / 2147483648.0
    var2 = pressure * dig_P8 / 32768.0
    pressure = pressure + (var1 + var2 + dig_P7) / 16.0

  # Refine humidity
  humidity = t_fine - 76800.0
  humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
  humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
  if humidity > 100:
    humidity = 100
  elif humidity < 0:
    humidity = 0

  return temperature/100.0,pressure/100.0,humidity

def readDHT22(sensor,pin):
  # Parse command line parameters.
  #sensor_args = { '11': Adafruit_DHT.DHT11,
  #                '22': Adafruit_DHT.DHT22,
  #                '2302': Adafruit_DHT.AM2302 }
  #if len(sys.argv) == 3 and sys.argv[1] in sensor_args:
  #    sensor = sensor_args[sys.argv[1]]
  #    pin = sys.argv[2]
  #else:
  #    print('Usage: sudo ./Adafruit_DHT.py [11|22|2302] <GPIO pin number>')
  #    print('Example: sudo ./Adafruit_DHT.py 2302 4 - Read from an AM2302 connected to GPIO pin #4')
  #    sys.exit(1)

  # Try to grab a sensor reading.  Use the read_retry method which will retry up
  # to 15 times to get a sensor reading (waiting 2 seconds between each retry).
  humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

  # Un-comment the line below to convert the temperature to Fahrenheit.
  # temperature = temperature * 9/5.0 + 32

  # Note that sometimes you won't get a reading and
  # the results will be null (because Linux can't
  # guarantee the timing of calls to read the sensor).
  # If this happens try again!
  if humidity is not None and temperature is not None:
#      print('Temp={0:0.1f}C  Humidity={1:0.1f}%'.format(temperature, humidity))
      return temperature,humidity  
  else:
      print('Failed to get reading. Try again!')
      sys.exit(1)

#sys.path.append("/home/pi/BME280")
#import PPD42NS

def readParticulas():
  GPIO.add_event_detect(inPINS[0], GPIO.BOTH, callback=my_callback1)
  #try:
  #  while True:
  ov1 = deltaTimes[0][-smoothingWindowLength:]
  #ov = sorted(ov1)[len[ov1] // 2]
  ov = np.mean(ov1)
  #print ('jj={0:0.7f} '.format(ov))
  return ov
  #time.sleep(0.1)
  #except KeyboardInterrupt:
  ##GPIO.cleanup()
  #pi = pigpio.pi()
  #s = PPD42NS.sensor(pi, PARTCPIN)
  #g, r, c = s.read()
  #print("gpio={} ratio={:.1f} conc={} pcs per 0.01 cubic foot".format(g, r, int(c)))
  #pi.stop()
  #return c


  #pi.set_mode(PARTCPIN,pigpio.INPUT)
  #range = pi.get_PWM_dutycycle(PARTCPIN)
  #freq = pi.get_PWM_pulsewidth(PARTCPIN)
  #print("range={0:0.2f} freq={0:0.2f}".format(range,freq))
  #return freq

def main():

#  (chip_id, chip_version) = readBME280ID()
#  print ("Chip ID     :", chip_id)
#  print ("Version     :", chip_version)
  temperature,pressure,humidity = readBME280All()
  temperature2,humidity2 = readDHT22(22,22)
  particulas = readParticulas()

  sendData(THINGSPEAKURL,THINGSPEAKKEY,temperature2,humidity2,temperature,pressure,particulas)

if __name__=="__main__":
   main()
