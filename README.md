# Love8C Controller Modbus
Driver for the Love 8C process controller, for communication using the Modbus ASCII protocol.

# Usage
> love8c.py [-h] [-get var_name] [-set var_name] [-set_value value] [-port port] [-address deviceNro] [-j] [-t]
```
Arguments:
  -h, --help          show this help message and exit
  -get var_name       The Value Name, Example: set_point
  -set var_name       Example: set_point
  -set_value value    Example: 15.5
  -port port          Example: COM3
  -address deviceNro  Device address for RS485, Example: 1
  -j, --json          Encode out to json
```

## List ports:
> love8c.py -j

## Get All from Device 1 on port COM3:
> love8c.py -get all -port COM3 -addres 1 -j

```
{"process_value": 17.0, "upper_limit_alarm_1": 2.0, "temperature_unit_display_selection": 1, "at_setting": 0, "heating_cooling_hysteresis": 0.1, "alarm_2_type":
 1, "temperature_regulation_value": 0.0, "ti_integral_time": 10, "alarm_1_type": 1, "lower_limit_alarm_2": 3.0, "lower_limit_alarm_1": 2.0, "control_method": 1,
 "td_derivative_time": 41, "status": 0, "lower_limit_of_temperature_range": -20.0, "software_version": 1056, "upper_limit_of_temperature_range": 500.0, "communi
cation_write_in_selection": 1, "heating_cooling_control_cycle": 22, "heating_cooling_control_selection": 1, "control_run_stop_setting": 1, "set_point": 18.5, "p
roportional_control_offset_error_value": 0.0, "upper_limit_alarm_2": 3.0, "input_temperature_sensor_type": 14, "pb_proportional_band": 2.0}
```

# Default serial controller settings
```
baudrate = 9600
bytesize = 7
parity   = EVEN
stopbits = 1
```

# Dependence
minimalmodbus
