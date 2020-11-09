#!/usr/bin/env python
#
#   Copyright 2016 Mauricio Galetto
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

"""

.. moduleauthor:: Mauricio Galetto

Driver for the Love 8C process controller, for communication using the Modbus ASCII protocol.

"""

import minimalmodbus
import argparse

import json

import sys
import glob
import serial

__author__  = "Mauricio Galetto "
__license__ = "Apache License, Version 2.0"


CONTROL_MODES = {
			0: 'PID', 
			1: 'ON/OFF',
			2: 'Manual Tuning'
} 

SENSOR_TYPES = {	
			0: 'K-Type1',
			1: 'K-Type2',
			2: 'J-Type1',
			3: 'J-Type2',
			4: 'T-Type1',
			5: 'T-Type2',
			6: 'E-Type',
			7: 'N-Type',
			8: 'R-Type',
			9: 'S-Type',
			10: 'B-Type',
			11: 'JPt100-1',
			12: 'JPt100-2',
			13: 'Pt100-1',
			14: 'Pt100-2',
			15: 'Pt100-3',
			16: 'L-Type',
			17: 'U-Type',
			18: 'TXK'
}

ALARMS_TYPES = {
			0: 'Alarm function disabled',
			1: 'Deviation upper- and lower-limit',
			2: 'Deviation upper-limit',
			3: 'Deviation lower-limit',
			4: 'Reverse deviation upper- and lower-limit',
			5: 'Absolute value upper- and lower-limit',
			6: 'Absolute value upper-limit',
			7: 'Absolute value lower-limit',
			8: 'Deviation upper- and lower-limit with standby sequence',
			9: 'Deviation upper-limit with standby sequence',
			10: 'Deviation lower-limit with standby sequence',
			11: 'Hysteresis upper-limit alarm output',
			12: 'Hysteresis lower-limit alarm output'
}

STATUS = {
			0: 'Normal operation (No error)',
			1: 'Initial process',
			2: 'Initial status (Temperature is not stable)',
			3: 'Temperature sensor is not connected',
			4: 'Temperature sensor input error',
			5: 'Measured temperature value exceeds the temperature range',
			6: 'No Int. error',
			7: 'EEPROM Error'
}

#WARNING, Not in Docs, From Snifer
LOCK_STATUS_SET = { 
			0: 'Normal', 
			1: 'Lock All', 
			11:'Not SV'
}
LOCK_STATUS_READ = { 
			0: 'Normal', 
			2: 'Lock All', 
			22:'Not SV'
}
"""
Sets				->	Gets (:01034731 0001 83)
:01064731 0000 81 	->	:010302 0000 FA
:01064731 000B 76	->	:010302 0016 E4
:01064731 0001 80	->	:010302 0002 F8
"""


"""
Sniffer PID Offsets
I. Offset :0106470E00178D (Ti <> 0 y set 23, lo pasa a entero 23.5 a 23, de 0 a 100)
PD. Offset: proportional_control_offset_error_value, Sin decimales, Solo si Ti = 0, de 0 a 100 
"""

"""
Sniffer PV tunnig
:01064714001688 temperature_regulation_value

"""

"""
Sniffer Manual 
	Output
	10 -> :0106472A000A7E  (Entero de 0 a 100)

Only available for analog output models (Not 8C):
	Tune Out High
	10 -> :0106470C000A9C (Entero desde -100 hasta 54)
	
	Tune Out Low
	10 -> :0106470D000A9B  (Entero desde -39 hasta 60)
"""

"""
Inegieria Inversa para LEDS
0x471C:
LEDS:
Ordenado			
F				4		1		4
C				8		1		12
Alarm 2			16		1		28
Alarm 1			32		1		60
Output			64		1		124
AT				128		1		252
								252
LEDS = {'AT':0,'Output':0,'Alarm1':0,'Alarm2':0,'C':0,'F':0}
if (ret>128){
	LEDS.AT = 1;
	ret = ret - 128;
}
if (ret>64){
	LEDS.Output = 1;
	ret = ret - 64;
}
if (ret>32){
	LEDS.Alarm1 = 1;
	ret = ret - 32;
}
if (ret>16){
	LEDS.Alarm2 = 1;
	ret = ret - 16;
}
if (ret>8){
	LEDS.C = 1;
	ret = ret - 8;
}
if (ret>4){
	LEDS.F = 1;
	ret = ret - 4;
}
"""

"""Description of the control mode numerical values."""
REGISTER_START = {
			'process_value':	0x4700,
			'set_point':	0x4701,
			'upper_limit_alarm_1':	0x4702,
			'lower_limit_alarm_1':	0x4703,
			'upper_limit_alarm_2':	0x4704,
			'lower_limit_alarm_2':	0x4705,
			'upper_limit_of_temperature_range':	0x4706,
			'lower_limit_of_temperature_range':	0x4707,
			'pb_proportional_band':	0x4708,
			'ti_integral_time':	0x4709,
			'td_derivative_time':	0x470A,
			'heating_cooling_hysteresis':	0x470B,
			'input_temperature_sensor_type':	0x4710,
			'control_method':	0x4711,
			'heating_cooling_control_cycle':	0x4712,
			'proportional_control_offset_error_value':	0x4713,
			'temperature_regulation_value':	0x4714,
			'alarm_1_type':	0x4715,
			'alarm_2_type':	0x4716,
			'temperature_unit_display_selection':	0x4717,
			'heating_cooling_control_selection':	0x4718,
			'control_run_stop_setting':	0x4719,
			'communication_write_in_selection':	0x471A,
			'software_version':	0x471B,
			'at_setting':	0x4729,
			'status':	0x472B,
			'lock_status': 0x4731,
			'i_offset': 0x470E,
			'output': 0x472A,
			'control_output': 0x471E,
			'leds': 0x471C
}

#read_register(registeraddress, numberOfDecimals=0, functioncode=3, signed=False):		
#numberOfDecimals	functioncode	signed
#numberOfDecimals, functioncode,signed
REGISTER_READ_DETAIL = {
			'process_value': [1, 3, 1],
			'set_point': [1, 3, 1],
			'upper_limit_alarm_1': [1, 3, 1],
			'lower_limit_alarm_1': [1, 3, 1],
			'upper_limit_alarm_2': [1, 3, 1],
			'lower_limit_alarm_2': [1, 3, 1],
			'upper_limit_of_temperature_range': [1, 3, 1],
			'lower_limit_of_temperature_range': [1, 3, 1],
			'pb_proportional_band': [1, 3, 0],
			'ti_integral_time': [0, 3, 0],
			'td_derivative_time': [0, 3, 0],
			'heating_cooling_hysteresis': [1, 3, 0],
			'input_temperature_sensor_type': [0, 3, 0],
			'control_method': [0, 3, 0],
			'heating_cooling_control_cycle': [0, 3, 0],
			'proportional_control_offset_error_value': [0, 3, 0],
			'temperature_regulation_value': [1, 3, 1],
			'alarm_1_type': [0, 3, 0],
			'alarm_2_type': [0, 3, 0],
			'temperature_unit_display_selection': [0, 3, 0],
			'heating_cooling_control_selection': [0, 3, 0],
			'control_run_stop_setting': [0, 3, 0],
			'communication_write_in_selection': [0, 3, 0],
			'software_version': [0, 3, 0],
			'at_setting': [0, 3, 0],
			'status': [0, 3, 0],
			'lock_status': [0, 3, 0],
			'i_offset': [0, 3, 0],
			'output': [0, 3, 0],
			'control_output': [0, 3, 0],
			'leds': [0, 3, 0]
}

#write_register(registeraddress, value, numberOfDecimals=0, functioncode=16, signed=False)
#tipo, numberOfDecimals, functioncode,signed, min, max
REGISTER_WRITE_DETAIL = {
			'set_point': ["int", 1, 6, 1, -273, 999],
			'upper_limit_alarm_1': ["int", 1, 6, 1, -999, 999],
			'lower_limit_alarm_1': ["int", 1, 6, 1, -999, 999],
			'upper_limit_alarm_2': ["int", 1, 6, 1, -999, 999],
			'lower_limit_alarm_2': ["int", 1, 6, 1, -999, 999],
			'upper_limit_of_temperature_range': ["int", 1, 6, 1, -200, 1800],
			'lower_limit_of_temperature_range': ["int", 1, 6, 1, -200, 1800],
			'pb_proportional_band': ["int", 1, 6, 0, 0.1, 999.9],
			'ti_integral_time': ["int", 0, 6, 0, 0, 9999],
			'td_derivative_time': ["int", 0, 6, 0, 0, 9999],
			'heating_cooling_hysteresis': ["int", 1, 6, 0, 0, 999],
			'input_temperature_sensor_type': ["int", 0, 6, 0, 0, 18],
			'control_method': ["int", 0, 6, 0, 0, 2],
			'heating_cooling_control_cycle': ["int", 0, 6, 0, 1, 99],
			'proportional_control_offset_error_value': ["int", 0, 6, 1, 0, 100],
			'temperature_regulation_value': ["int", 1, 6, 1, -99, 99],
			'alarm_1_type': ["int", 0, 6, 0, 0, 12],
			'alarm_2_type': ["int", 0, 6, 0, 0, 12],
			'temperature_unit_display_selection': ["int", 0, 6, 0, 0, 1],
			'heating_cooling_control_selection': ["int", 0, 6, 0, 0, 1],
			'control_run_stop_setting': ["int", 0, 6, 0, 0, 1],
			'communication_write_in_selection': ["int", 0, 6, 0, 0, 1],
			'at_setting': ["int", 0, 6, 0, 0, 1],
			'lock_status': ["int", 0, 6, 0, 0, 11],
			'i_offset': ["int", 0, 6, 0, 0, 100],
			'output': ["int", 0, 6, 0, 0, 100]
}




REGISTER_LABELS = {
			'process_value':	'Process value',
			'set_point':	'Set point',
			'upper_limit_alarm_1':	'Upper-limit alarm 1',
			'lower_limit_alarm_1':	'Lower-limit alarm 1',
			'upper_limit_alarm_2':	'Upper-limit alarm 2',
			'lower_limit_alarm_2':	'Lower-limit alarm 2',
			'upper_limit_of_temperature_range':	'Upper-limit of temperature range',
			'lower_limit_of_temperature_range':	'Lower-limit of temperature range',
			'pb_proportional_band':	'PB Proportional band',
			'ti_integral_time':	'Ti Integral time',
			'td_derivative_time':	'Td Derivative time',
			'heating_cooling_hysteresis':	'Heating/Cooling hysteresis',
			'input_temperature_sensor_type':	'Input temperature sensor type',
			'control_method':	'Control method',
			'heating_cooling_control_cycle':	'Heating/Cooling control cycle',
			'proportional_control_offset_error_value':	'Proportional control offset error value',
			'temperature_regulation_value':	'Temperature regulation value (PV Offset)',
			'alarm_1_type':	'Alarm 1 type',
			'alarm_2_type':	'Alarm 2 type',
			'temperature_unit_display_selection':	'Temperature unit display selection',
			'heating_cooling_control_selection':	'Heating/Cooling control Selection',
			'control_run_stop_setting':	'Control Run/Stop setting',
			'communication_write_in_selection':	'Communication write-in selection',
			'software_version':	'Software version',
			'at_setting':	'Auto Tune Setting',
			'status':	'Status',
			'lock_status': 'Lock Status',
			'i_offset': 'PID i. Offset',
			'output': 'Output(%)',
			'control_output': 'Control Output (%)',
			'leds': 'Actual LEDs'
}


REGISTER_DETAILS = {
			'process_value':	'Measuring unit is 0.1, updated one time in 0.5 second.',
			'set_point':	'Unit is 0.1, oC or oF',
			'upper_limit_alarm_1':	'',
			'lower_limit_alarm_1':	'',
			'upper_limit_alarm_2':	'',
			'lower_limit_alarm_2':	'',
			'upper_limit_of_temperature_range':	'The data content should not be higher than the temperature range',
			'lower_limit_of_temperature_range':	'The data content should not be lower than the temperature range',
			'pb_proportional_band':	'0.1 to 999.9, unit is 0.1',
			'ti_integral_time':	'0 to 9999',
			'td_derivative_time':	'0 to 9999',
			'heating_cooling_hysteresis':	'0 to 9999',
			'input_temperature_sensor_type':	'Please refer to the contents of the -Temperature Sensor Type and Temperature Range- for detail',
			'control_method':	'0: PID (default), 1: ON/OFF, 2: manual tuning',
			'heating_cooling_control_cycle':	'1 to 99 second',
			'proportional_control_offset_error_value':	'PD. Offset, 0% to 100%',
			'temperature_regulation_value':	'-999 ~ 999, unit: 0.1',
			'alarm_1_type':	'Please refer to the contents of the -Alarm Outputs- for detail',
			'alarm_2_type':	'Please refer to the contents of the -Alarm Outputs- for detail',
			'temperature_unit_display_selection':	'oC: 1 (default), oF: 0',
			'heating_cooling_control_selection':	'Heating: 0 (default), Cooling: 1',
			'control_run_stop_setting':	'Run: 1 (default), Stop: 0',
			'communication_write_in_selection':	'Communication write in disabled: 0 (default), Communication write in enabled: 1',
			'software_version':	'W1.00 indicates 0 x 100',
			'at_setting':	'OFF: 0 (default), ON:1',
			'status':	'Code 0 Normal operation (No error), Code 1 Initial process, Code 2 Initial status (Temperature is not stable), Code 3 Temperature sensor is not connected, Code 4 Temperature sensor input error, Code 5 Measured temperature value exceeds, the temperature range, Code 6 No Int. error, Code 7 EEPROM Error',
			'lock_status': '',
			'i_offset': '',
			'output': '',
			'control_output': '',
			'leds': ''
}



class Love8C( minimalmodbus.Instrument ):
	"""Instrument class for Love 8C process controller. 
	
	Communicates via Modbus ASCII protocol (via RS485), using the :mod:`minimalmodbus` Python module.
	
	This driver is intended to enable control of the Love8C controller from the command line.
	
	Args:
		* portname (str): port name
		
		* examples:
		* OS X: '/dev/tty.usbserial'
		* Linux: '/dev/ttyUSB0'
		* Windows: '/com3'
		
		* slaveaddress (int): slave address in the range 1 to 247 (in decimal)
			
	Implemented with these function codes (in decimal):
		
	==================  ====================
	Description         Modbus function code
	==================  ====================
	Read registers      3
	Write one register  6
	==================  ====================
	
	"""
	
	def __init__(self, portname, slaveaddress):
		self.handle_local_echo = False
		self.close_port_after_each_call = True
		minimalmodbus.Instrument.__init__(self, portname, slaveaddress, minimalmodbus.MODE_ASCII)
		self.serial.baudrate = 9600
		self.serial.bytesize = 7
		self.serial.parity = minimalmodbus.serial.PARITY_EVEN
		self.serial.stopbits = 1
		self.serial.timeout  = 0.5
		#self.debug = True
	
	def get_register(self, name):
		signed = False
		decimals = REGISTER_READ_DETAIL[name][0]
		funct = REGISTER_READ_DETAIL[name][1]
		if (REGISTER_READ_DETAIL[name][2]==1):
			signed = True
		return self.read_register( REGISTER_START[name], decimals, funct, signed)
	
	def set_register(self, name, setpointvalue):
		tipoPermitido = REGISTER_WRITE_DETAIL[name][0]
		minimo = REGISTER_WRITE_DETAIL[name][4]
		maximo = REGISTER_WRITE_DETAIL[name][5]
		funct = REGISTER_WRITE_DETAIL[name][2]
		signed = False
		decimals = REGISTER_WRITE_DETAIL[name][1]
		if (REGISTER_WRITE_DETAIL[name][3]==1):
			signed = True
		if (checkNumerical(setpointvalue, minvalue=minimo, maxvalue=maximo) ):
			self.write_register( REGISTER_START[name], setpointvalue, decimals, funct, signed)
			return True
		else:
			return False


def checkNumerical(inputvalue, minvalue=None, maxvalue=None):
	if not isinstance(inputvalue, (int, long, float)):
		raise TypeError('The {0} must be numerical. Given: {1!r}'.format(description, inputvalue))
	if not isinstance(minvalue, (int, float, long, type(None))):
		raise TypeError('The minvalue must be numeric or None. Given: {0!r}'.format(minvalue))
	if not isinstance(maxvalue, (int, float, long, type(None))):
		raise TypeError('The maxvalue must be numeric or None. Given: {0!r}'.format(maxvalue))
	if (not minvalue is None) and (not maxvalue is None):
		if maxvalue < minvalue:
			raise ValueError('The maxvalue must not be smaller than minvalue. Given: {0} and {1}, respectively.'.format(maxvalue, minvalue))
	if not minvalue is None:
		if inputvalue < minvalue:
			return False
	if not maxvalue is None:
		if inputvalue > maxvalue:
			return False
	return True

def serial_ports():
	if sys.platform.startswith('win'):
		ports = ['COM%s' % (i + 1) for i in range(256)]
	elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
		# this excludes your current terminal "/dev/tty"
		ports = glob.glob('/dev/tty[A-Za-z]*')
	elif sys.platform.startswith('darwin'):
		ports = glob.glob('/dev/tty.*')
	else:
		raise EnvironmentError('Unsupported platform')
	result = []
	for port in ports:
		try:
			s = serial.Serial(port)
			s.close()
			result.append(port)
		except (OSError, serial.SerialException):
			pass
	return result


########################
## Testing the module ##
########################

if __name__ == '__main__':
	#PORTNAME = 'COM3'
	#PORTNAME = '/dev/ttyUSB0'
	#ADDRESS = 1
	
	parser = argparse.ArgumentParser(description='Love 8C controller.')
	#for name in REGISTER_READ_DETAIL:
	#	parser.add_argument('get_'+name, help=REGISTER_LABELS[name])
	parser.add_argument('-get', metavar='var_name', nargs=1, help='The Value Name, Example: set_point')
	parser.add_argument('-set', metavar='var_name', nargs=1, help='Example: set_point')
	parser.add_argument('-set_value', metavar='value', nargs=1, type=float, help='Example: 15.5')
	parser.add_argument('-port', metavar='port', nargs=1, help='Example: COM3')
	parser.add_argument('-address', metavar='deviceNro', nargs=1, type=int, help='Device address for RS485, Example: 1')
	parser.add_argument("-j", "--json", action="count", default=0, help='Encode out to json')
	parser.add_argument("-t", "--test", action="count", default=0)
	parser.add_argument("-e", "--emu", action="count", default=0)
	
	
	args = parser.parse_args()
	
	if (args.port == None):
		data = {}
		data["ports"] = serial_ports()
		if (args.json):
			if (args.emu):
				minimalmodbus._print_out("{\"ports\":[\"COM99\"]}")
				exit()
			json_data = json.dumps(data)
			minimalmodbus._print_out(json_data)
		else:
			minimalmodbus._print_out("Need set the port, Ex: -port COM3")
			minimalmodbus._print_out("Avaible:")
			ports_data = json.dumps(data["ports"])
			minimalmodbus._print_out(ports_data)
		exit()
	if (args.address == None):
		minimalmodbus._print_out("Need set the address, Ex: -address 1")
		exit()
	
	PORTNAME = args.port[0]
	#PORTNAME = '/dev/ttyUSB0'
	ADDRESS = args.address[0]
	
	try:
		if (args.set):
			name = args.set[0]
			if (args.set_value):
				if (args.emu):
					exit()
				setval = args.set_value[0]
				instr = Love8C(PORTNAME, ADDRESS)
				instr.set_register(name, setval)
				instr.serial.close()
			else:
				minimalmodbus._print_out("Need set the value with -set_value")
		
		if (args.get):
			data = {}
			if (args.emu):
				emu = '{"i_offset":0,"output":0, "control_output": 0,"process_value": 17.4, "upper_limit_alarm_1": 2.0, "temperature_unit_display_selection": 1, "at_setting": 0, "heating_cooling_hysteresis": 0.1, "alarm_2_type": 1, "temperature_regulation_value": 0.0, "ti_integral_time": 10, "alarm_1_type": 1, "lower_limit_alarm_2": 3.0, "lower_limit_alarm_1": 2.0, "control_method": 1, "td_derivative_time": 41, "status": 0, "lower_limit_of_temperature_range": -20.0, "software_version": 1056, "upper_limit_of_temperature_range": 500.0, "communication_write_in_selection": 1, "heating_cooling_control_cycle": 22, "heating_cooling_control_selection": 1, "control_run_stop_setting": 0, "set_point": 18.5, "proportional_control_offset_error_value": 0.0, "upper_limit_alarm_2": 3.0, "input_temperature_sensor_type": 14, "pb_proportional_band": 2.0, "leds": 84,"lock_status":0}'
				if (ADDRESS==2):
					emu = '{"i_offset":0,"output":0, "control_output": 0,"process_value": 1.4, "upper_limit_alarm_1": 2.0, "temperature_unit_display_selection": 1, "at_setting": 0, "heating_cooling_hysteresis": 0.1, "alarm_2_type": 1, "temperature_regulation_value": 0.0, "ti_integral_time": 10, "alarm_1_type": 1, "lower_limit_alarm_2": 3.0, "lower_limit_alarm_1": 2.0, "control_method": 1, "td_derivative_time": 41, "status": 0, "lower_limit_of_temperature_range": -20.0, "software_version": 1056, "upper_limit_of_temperature_range": 500.0, "communication_write_in_selection": 1, "heating_cooling_control_cycle": 22, "heating_cooling_control_selection": 1, "control_run_stop_setting": 1, "set_point": 1.5, "proportional_control_offset_error_value": 0.0, "upper_limit_alarm_2": 3.0, "input_temperature_sensor_type": 14, "pb_proportional_band": 2.0, "leds": 84,"lock_status":0}'
				if (args.get[0] == "all"):
					minimalmodbus._print_out(emu)
					exit()			
				fake_data = json.loads(emu)
				names = [x.strip() for x in args.get[0].split(",")] #obtengo array de props sin espacios
				for propName in names:
					data[propName] = 'N/D'
					if (propName in fake_data):
						data[propName] = fake_data[propName]
					else:
						minimalmodbus._print_out("No " + propName )
					if (args.json == 0):
						minimalmodbus._print_out(propName + ":" + str(data[propName]))
				if (args.json):
					json_data = json.dumps(data)
					minimalmodbus._print_out(json_data)
				exit()
			
			instr = Love8C(PORTNAME, ADDRESS)
			if (args.get[0] == "all"):
				for name in REGISTER_READ_DETAIL:
					data[name] = instr.get_register(name)
				if (args.json == 0):
					minimalmodbus._print_out( str(REGISTER_LABELS[name]) + ': ' + str(data[name]))
			else:
				names = [x.strip() for x in args.get[0].split(",")] #obtengo array de props sin espacios
				for propName in names:
					data[propName] = instr.get_register(propName)
					if (args.json == 0):
						minimalmodbus._print_out(propName + ":" + str(data[propName]))
			instr.serial.close()
			if (args.json):
				json_data = json.dumps(data)
				minimalmodbus._print_out(json_data)
		
		if (args.test):
			minimalmodbus._print_out('TESTING LOVE 8C MODBUS MODULE')
			minimalmodbus._print_out( 'Port: ' +  str(PORTNAME) + ', Address: ' + str(ADDRESS) )
			instr = Love8C(PORTNAME, ADDRESS)
			
			data = {}
			#json_data = json.dumps(data)
			for name in REGISTER_READ_DETAIL:
				data[name] = instr.get_register(name)
				minimalmodbus._print_out( str(REGISTER_LABELS[name]) + ': ' + str(data[name]))
			minimalmodbus._print_out('DONE!')
			instr.serial.close()
	except (OSError, serial.SerialException) as ex:
		data = {}
		data["error"] = "port"
		data["msg"] = ex.__str__()
		errcode = 0
		if "WindowsError(2," in data["msg"]:
			errcode = 2 #No existe el puerto
		if "WindowsError(5," in data["msg"]:
			errcode = 5 #En uso
		data["code"] = errcode
		json_data = json.dumps(data)
		minimalmodbus._print_out(json_data)
	except Exception as e:
		data = {}
		data["error"] = "unknow"
		data["code"] = 0
		data["msg"] = e.__str__()
		json_data = json.dumps(data)
		minimalmodbus._print_out(json_data)

pass
"""
E:\Tau\Love8C>love8c.py -get all -port COM3 -addres 1 -j
{"process_value": 17.0, "upper_limit_alarm_1": 2.0, "temperature_unit_display_selection": 1, "at_setting": 0, "heating_cooling_hysteresis": 0.1, "alarm_2_type":
 1, "temperature_regulation_value": 0.0, "ti_integral_time": 10, "alarm_1_type": 1, "lower_limit_alarm_2": 3.0, "lower_limit_alarm_1": 2.0, "control_method": 1,
 "td_derivative_time": 41, "status": 0, "lower_limit_of_temperature_range": -20.0, "software_version": 1056, "upper_limit_of_temperature_range": 500.0, "communi
cation_write_in_selection": 1, "heating_cooling_control_cycle": 22, "heating_cooling_control_selection": 1, "control_run_stop_setting": 1, "set_point": 18.5, "p
roportional_control_offset_error_value": 0.0, "upper_limit_alarm_2": 3.0, "input_temperature_sensor_type": 14, "pb_proportional_band": 2.0}
"""
