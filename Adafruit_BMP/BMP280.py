# 
# Author: Jason Byars
#
# Adapted from Tony DiCola's work on BMP085.py
# Calcuations come from BMP280 datasheet version 1.14
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import time


# BMP280 default address.
BMP280_I2CADDR           = 0x77

# Operating Modes
BMP280_ULTRALOWPOWER     = 0b00100100 # p x1 t x1
BMP280_LOWPOWER          = 0b00101000 # p x2 t x 1
BMP280_STANDARD          = 0b00101100 # p x4 t x1
BMP280_HIGHRES           = 0b00110000 # p x8 t x1
BMP280_ULTRAHIGHRES      = 0b01010100 # p x16 t x2

BMP280_SLEEPMODE         = 0
BMP280_FORCEDMODE        = 1
BMP280_NORMALMODE        = 3

# BMP280 Registers
BMP280_DIG_T1            = 0x88  # R   Calibration data (16 bits)
BMP280_DIG_T2            = 0x8A  # R   Calibration data (16 bits)
BMP280_DIG_T3            = 0x8C  # R   Calibration data (16 bits)

BMP280_DIG_P1            = 0x8E  # R   Calibration data (16 bits)
BMP280_DIG_P2            = 0x90  # R   Calibration data (16 bits)
BMP280_DIG_P3            = 0x92  # R   Calibration data (16 bits)
BMP280_DIG_P4            = 0x94  # R   Calibration data (16 bits)
BMP280_DIG_P5            = 0x96  # R   Calibration data (16 bits)
BMP280_DIG_P6            = 0x98  # R   Calibration data (16 bits)
BMP280_DIG_P7            = 0x9A  # R   Calibration data (16 bits)
BMP280_DIG_P8            = 0x9C  # R   Calibration data (16 bits)
BMP280_DIG_P9            = 0x9E  # R   Calibration data (16 bits)

BMP280_CHIPID            = 0xD0
BMP280_VERSION           = 0xD1
BMP280_SOFTRESET         = 0xE0
BMP280_REGISTER_CAL      = 0xE1

BMP280_STATUS            = 0xF3
BMP280_CONTROL           = 0xF4
BMP280_CONFIG            = 0xF5
BMP280_TEMPDATA          = 0xFA
BMP280_PRESSUREDATA      = 0xF7

# Commands
BMP280_READTEMPCMD       = 0x2E
BMP280_READPRESSURECMD   = 0x34


class BMP280(object):
	def __init__(self, mode=BMP280_STANDARD, address=BMP280_I2CADDR, i2c=None, **kwargs):
		self._logger = logging.getLogger('Adafruit_BMP.BMP280')
		#logging.basicConfig(level=logging.DEBUG)
		# Check that mode is valid.
		if mode not in [BMP280_ULTRALOWPOWER, BMP280_LOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, BMP280_ULTRAHIGHRES]:
			raise ValueError('Unexpected mode value {0}.  Set mode to one of BMP280_ULTRALOWPOWER, BMP280_LOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, or BMP280_ULTRAHIGHRES'.format(mode))
		self._mode = mode
		# Create I2C device.
		if i2c is None:
			import Adafruit_GPIO.I2C as I2C
			i2c = I2C
		self._device = i2c.get_i2c_device(address, **kwargs)
		# Load calibration values.
		self._load_calibration()
		#self._load_datasheet_calibration()

	def _load_calibration(self):
		self.dig_T1 = self._device.readU16LE(BMP280_DIG_T1)     # UINT16
		self.dig_T2 = self._device.readS16LE(BMP280_DIG_T2)     # INT16
		self.dig_T3 = self._device.readS16LE(BMP280_DIG_T3)     # INT16
		self.dig_P1 = self._device.readU16LE(BMP280_DIG_P1)     # UINT16
		self.dig_P2 = self._device.readS16LE(BMP280_DIG_P2)     # INT16
		self.dig_P3 = self._device.readS16LE(BMP280_DIG_P3)     # INT16
		self.dig_P4 = self._device.readS16LE(BMP280_DIG_P4)     # INT16
		self.dig_P5 = self._device.readS16LE(BMP280_DIG_P5)     # INT16
		self.dig_P6 = self._device.readS16LE(BMP280_DIG_P6)     # INT16
		self.dig_P7 = self._device.readS16LE(BMP280_DIG_P7)     # INT16
		self.dig_P8 = self._device.readS16LE(BMP280_DIG_P8)     # INT16
		self.dig_P9 = self._device.readS16LE(BMP280_DIG_P9)     # INT16
		self._logger.debug('dig_T1 = {0:6d}'.format(self.dig_T1))
		self._logger.debug('dig_T2 = {0:6d}'.format(self.dig_T2))
		self._logger.debug('dig_T3 = {0:6d}'.format(self.dig_T3))
		self._logger.debug('dig_P1 = {0:6d}'.format(self.dig_P1))
		self._logger.debug('dig_P2 = {0:6d}'.format(self.dig_P2))
		self._logger.debug('dig_P3 = {0:6d}'.format(self.dig_P3))
		self._logger.debug('dig_P4 = {0:6d}'.format(self.dig_P4))
		self._logger.debug('dig_P5 = {0:6d}'.format(self.dig_P5))
		self._logger.debug('dig_P6 = {0:6d}'.format(self.dig_P6))
		self._logger.debug('dig_P7 = {0:6d}'.format(self.dig_P7))
		self._logger.debug('dig_P8 = {0:6d}'.format(self.dig_P8))
		self._logger.debug('dig_P9 = {0:6d}'.format(self.dig_P9))

	def _load_datasheet_calibration(self):
		# Set calibration from values in the datasheet example.  Useful for debugging the
		# temp and pressure calculation accuracy.
		self.dig_T1 = 27504
		self.dig_T2 = 26435
		self.dig_T3 = -1000
		self.dig_P1 = 36477
		self.dig_P2 = -10685
		self.dig_P3 = 3024
		self.dig_P4 = 2855
		self.dig_P5 = 140
		self.dig_P6 = -7
		self.dig_P7 = 15500
		self.dig_P8 = 14600
		self.dig_P9 = 6000

        def read_raw(self, reg):
                """Really should figure out how to do the burst read described in the mfg docs."""
		self._device.write8(BMP280_CONTROL, self._mode + BMP280_FORCEDMODE)
		if self._mode == BMP280_ULTRALOWPOWER:
			time.sleep(0.007)
		elif self._mode == BMP280_LOWPOWER:
                        time.sleep(0.009)
                elif self._mode == BMP280_STANDARD:
                        time.sleep(0.014)
		elif self._mode == BMP280_HIGHRES:
			time.sleep(0.023)
		elif self._mode == BMP280_ULTRAHIGHRES:
			time.sleep(0.044)
		else:
			raise ValueError('Unexpected mode {0}. Set mode to one of BMP280_ULTRALOWPOWER, BMP280_LOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, or BMP280_ULTRAHIGHRES'.format(mode))
		msb = self._device.readU8(reg)
		lsb = self._device.readU8(reg+1)
		xlsb = self._device.readU8(reg+2)
		raw = ((msb << 16) + (lsb << 8) + xlsb) >> 4
		return raw
	
	def read_raw_temp(self):
		"""Reads the raw (uncompensated) temperature from the sensor."""
		self._device.write8(BMP280_CONTROL, self._mode + BMP280_FORCEDMODE)
                raw = self.read_raw(BMP280_TEMPDATA)
		self._logger.debug('Raw temp 0x{0:X} ({1})'.format(raw & 0xFFFF, raw))
		return raw

	def read_raw_pressure(self):
		"""Reads the raw (uncompensated) pressure level from the sensor."""
		self._device.write8(BMP280_CONTROL, self._mode + BMP280_FORCEDMODE)
                raw = self.read_raw(BMP280_PRESSUREDATA)
		self._logger.debug('Raw pressure 0x{0:04X} ({1})'.format(raw & 0xFFFF, raw))
		return raw

	def read_temperature(self):
		"""Gets the compensated temperature in degrees celsius."""
		# Calculation from section 3.12 of datasheet
		adc_T = self.read_raw_temp()
		var1 = (adc_T/16384.0 - self.dig_T1/1024.0) * self.dig_T2
		var2 = ((adc_T/131072.0 - self.dig_T1/8192.0) * (adc_T/131072.0 - self.dig_T1/8192.0)) * self.dig_T3
		temp = (var1 + var2)/5120.0
		self._logger.debug('var1 {0}'.format(var1))
		self._logger.debug('var2 {0}'.format(var2))
		self._logger.debug('Calibrated temperature {0} C'.format(temp))
		return temp

	def read_pressure(self):
		"""Gets the compensated pressure in Pascals."""
		# Calculation from section 3.12 of datasheet
		adc_P = self.read_raw_pressure()
		adc_T = self.read_raw_temp()
		var1 = (adc_T/16384.0 - self.dig_T1/1024.0) * self.dig_T2
		var2 = ((adc_T/131072.0 - self.dig_T1/8192.0) * (adc_T/131072.0 - self.dig_T1/8192.0)) * self.dig_T3
                t_fine = var1+var2
                var1 = t_fine/2.0 -64000.0
                var2 = var1*var1*self.dig_P6/32768.0
                var2 = var2 + var1*self.dig_P5*2.0
                var2 = (var2/4.0) + (self.dig_P4 * 65536.0) 
                var1 = (self.dig_P3 * var1 * var1/524288.0 + self.dig_P2*var1) / 524288.0
                var1 = (1.0 + var1/32768.0) * float(self.dig_P1)
                p = 1048576.0 - adc_P
		p = (p - (var2/4096.0)) * 6250.0/var1
                var1 = self.dig_P9 * p * p / 2137483648.0
                var2 = p * self.dig_P8 / 32768.0
                p = p + (var1 + var2 + self.dig_P7)/16.0

		self._logger.debug('Pressure {0} Pa'.format(p))
		return p

	def read_altitude(self, sealevel_pa=101325.0):
		"""Calculates the altitude in meters."""
		# Calculation taken straight from section 3.6 of the BMP085 datasheet.
		pressure = float(self.read_pressure())
		altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
		self._logger.debug('Altitude {0} m'.format(altitude))
		return altitude

	def read_sealevel_pressure(self, altitude_m=0.0):
		"""Calculates the pressure at sealevel when given a known altitude in
		meters. Returns a value in Pascals."""
		pressure = float(self.read_pressure())
		p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
		self._logger.debug('Sealevel pressure {0} Pa'.format(p0))
		return p0

	def reset(self):
                """Trigger complete power-on-reset procedure."""
                self._device.write8(BMP280_SOFTRESET, 0xB6)
