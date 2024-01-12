import u6
from math import log
from time import sleep, time
from rtk_gps import reach_rover
from pathlib import Path
from utils import threaded
from threading import Event
from typing import Dict
from datetime import datetime

#calibration coefficients mc2,mc1,mc0,bc2,bc1,bc0
units_cc = { 
    '1137':[99751.8,8975590,1758570000,5101.13,177434,-9899180],
    '1138':[99635.4,9224420,1777020000,3108.49,134684,-2025390],
    '1139':[106007,9349540,1810330000,4577.95,196889,1718420],
    '1140':[89738.8,8845740,1678800000,4171.33,146896,-4733060],
    '1141':[93426.8,8872840,1731910000,4695.67,274791,1591430],
    '1142':[95912.5,9357680,1815870000,3248.41,220819,3119340] 
    }

# irr_unit_T = Dict[str,str,str,int,int,int,enumerate,list]

def get_temp(irr_unit:dict ,sensor_mV,object_mV,voltaje_divider = False):
    A = 1.129241e-3 
    B = 2.341077e-4
    C = 8.775468e-8
    fix_resistor = 24900 #ohms
    
    if voltaje_divider == True: #with voltaje divider of 2.5 V in
        Rt = fix_resistor * (2500/sensor_mV - 1) 
    else: #with constant current source of 10 uA
        Rt = sensor_mV/0.010 - fix_resistor
    #temperature detected by thermistor also known as sensor body temperature
    sensor_body_t_C = 1/(A+B*log(Rt) + C*log(Rt)**3 ) - 273.15

    cc = irr_unit['cc']
    m = cc[0]*sensor_body_t_C**2 + cc[1]*sensor_body_t_C + cc[2]
    b = cc[3]*sensor_body_t_C**2 + cc[4]*sensor_body_t_C + cc[5]

    target_t_C = ( (sensor_body_t_C + 273.15)**4 + m*object_mV + b )**0.25 -273.5
    return (sensor_body_t_C,target_t_C)


def get_irr_array_temperatures(d:u6.U6,irr_array:list):
    # read voltajes in every thermistor and correct for series resistor
    series_resistor_volt = [d.getAIN(IRR['thermistor_ain'],resolutionIndex = IRR['res_index'],gainIndex = IRR['gain_index'])*1000 for IRR in irr_array ]
    # calculate individual thermistor voltages for resistance calculation
    diff_list = [series_resistor_volt[i] - series_resistor_volt[i+1] for i in range(len(series_resistor_volt)-1)]
    diff_list.append(series_resistor_volt[-1])
    thermistor_voltage = diff_list
    thermopile_voltage = [d.getAIN(IRR['thermopile_ain'],resolutionIndex=6,gainIndex=3,differential=True)*1000 for IRR in irr_array]

    temp_array_C = [get_temp(x[0],x[1],x[2]) for x in zip(irr_array,thermistor_voltage,thermopile_voltage)]
    #{'sensor_id':,
    # 'sensor_position':,
    # 'sensor_body_t_C':,
    # 'target_t_C':}
    result = [{'sensor_id':x[0]['unit'],'sensor_position':x[0]['position'],'sensor_body_t_C':x[1][0],'target_t_C':x[1][1]} for x in zip(irr_array,temp_array_C)]
    return(result)

def get_temperature_with_coordinates(d:u6.U6,irr_array:list,GPS_rover:reach_rover=None):
    sleep(0.8)
    # read voltajes in every thermistor and correct for series resistor
    series_resistor_volt = [d.getAIN(IRR['thermistor_ain'],resolutionIndex = IRR['res_index'],gainIndex = IRR['gain_index'])*1000 for IRR in irr_array ]
    # calculate individual thermistor voltages for resistance calculation
    diff_list = [series_resistor_volt[i] - series_resistor_volt[i+1] for i in range(len(series_resistor_volt)-1)]
    diff_list.append(series_resistor_volt[-1])
    thermistor_voltage = diff_list
    thermopile_voltage = [d.getAIN(IRR['thermopile_ain'],resolutionIndex=6,gainIndex=3,differential=True)*1000 for IRR in irr_array]

    timestamp = time()
    temp_array_C = [get_temp(x[0],x[1],x[2]) for x in zip(irr_array,thermistor_voltage,thermopile_voltage)]
    if GPS_rover:
        coordinates_with_meta = GPS_rover.coordinates
        lat,long,alt = coordinates_with_meta['coordinates']
        datetime_iso,quality_fix = coordinates_with_meta['metadata']

        result = [{'timestamp':timestamp,
                    'datetime_iso':datetime_iso,
                    'quality_fix':quality_fix,
                    'lat': lat,
                    'long': long,
                    'alt': alt,
                    'sensor_id':x[0]['unit'],
                    'sensor_position':x[0]['position'],
                    'sensorbody_temp_C':x[1][0],
                    'target_temp_C':x[1][1]} for x in zip(irr_array,temp_array_C)]
    else:
        result = [{'timestamp':timestamp,
                    'datetime_iso':0,
                    'quality_fix':0,
                    'lat': 0,
                    'long': 0,
                    'alt': 0,
                    'sensor_id':x[0]['unit'],
                    'sensor_position':x[0]['position'],
                    'sensorbody_temp_C':x[1][0],
                    'target_temp_C':x[1][1]} for x in zip(irr_array,temp_array_C)]

    return(result)

@threaded
def log_temperatures(txt_path:Path,irr_list:list[Dict],stop_event:Event,gps:reach_rover=None):
    u6_device = u6.U6()
    u6_device.getCalibrationData()
    stop_event = stop_event
    try:
        for irr in irr_list:
            irr['cc'] = units_cc[irr['unit']]
        with txt_path.open('w',encoding='utf-8') as f:
            header = 'timestamp,datetime_iso,quality_fix,lat,long,alt,sensor_id,sensor_position,sensorbody_temp_C,target_temp_C\n'
            f.write(header)
            while not stop_event.is_set():
                sleep(0.6) #0.6 1H1 step response time
                # read voltajes in every thermistor and correct for series resistor
                series_resistor_volt = [u6_device.getAIN(IRR['thermistor_ain'],resolutionIndex = IRR['res_index'],gainIndex = IRR['gain_index'])*1000 for IRR in irr_list ]
                # calculate individual thermistor voltages for resistance calculation
                diff_list = [series_resistor_volt[i] - series_resistor_volt[i+1] for i in range(len(series_resistor_volt)-1)]
                diff_list.append(series_resistor_volt[-1])
                thermistor_voltage = diff_list
                thermopile_voltage = [u6_device.getAIN(IRR['thermopile_ain'],resolutionIndex=6,gainIndex=3,differential=True)*1000 for IRR in irr_list]

                timestamp = time()
                if gps:
                    coordinates_with_meta = gps.coordinates_with_meta
                else:
                    coordinates_with_meta = {'latitude':'0.0',
                                             'longitude':'0.0',
                                             'altitude':'0.0',
                                             'datetime_iso':'' }
                    coordinates_with_meta = None
                for irr,thermistor_V,thermopile_V in zip(irr_list,thermistor_voltage,thermopile_voltage):
                    sensorbody_t,target_t = get_temp(irr,thermistor_V,thermopile_V)
                    irr['sensorbody_t_C'] = sensorbody_t
                    irr['target_t_C'] = target_t
                    print(f"{irr['unit']}: {sensorbody_t},{target_t}")
                    if coordinates_with_meta is None:
                        timestamp_iso = datetime.fromtimestamp(timestamp).isoformat(' ','milliseconds')
                        line =','.join([f"{timestamp:.6f},{timestamp_iso},0",
                                "0.0,0.0,0.0",
                                f"{irr['unit']},{irr['position'].name.upper()},{sensorbody_t:.6f},{target_t:.6f}"]) + '\n'
                    else:
                        line =','.join([f"{timestamp:.6f},{coordinates_with_meta['datetime_iso']},{coordinates_with_meta['quality_fix']}",
                                f"{coordinates_with_meta['latitude']},{coordinates_with_meta['longitude']},{coordinates_with_meta['altitude']}",
                                f"{irr['unit']},{irr['position'].name.upper()},{sensorbody_t:.6f},{target_t:.6f}"]) + '\n'
                    f.write(line)
    except ZeroDivisionError as e:
        u6_device.close()
        print(e)
    except Exception as e:
        u6_device.close()
        print(e)
    # finally:
        # raise

# def append_calib_coeffs(irr_array:list):
#     #get calibration coefficients for each IRR
#     for IRR in irr_array:
#         IRR['cc'] = units_cc[IRR['unit']]
#     return irr_array

