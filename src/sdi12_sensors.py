from abc import ABC, abstractmethod
import serial
import time
import re
from enum import Enum,auto
from rtk_gps import reach_rover
from threading import Event
from utils import threaded
from pathlib import Path
from utils import SensorOrientation,SensorPosition
from datetime import datetime

class SDI12_sensor(ABC):
    '''	abstract class for all sdi12 sensors '''
    def __init__(self, id:str, position:SensorPosition, orientation:SensorOrientation, com_interface:serial.Serial):
        self.id = id
        self.position = position
        self.orientation = orientation
        self.com_interface = com_interface

    @abstractmethod
    def call_concurrent_measurement(self):
        pass

    @abstractmethod
    def parse_response(self):
        pass

class Dualband_sensor(SDI12_sensor):
    '''	implementation of a dualband sensor '''
    def __init__(self, id:str, position:SensorPosition, orientation:SensorOrientation, com_interface:serial.Serial):
        super().__init__(id, position, orientation, com_interface)
        self.lower_band = 0.0
        self.upper_band = 0.0
        self._last_update = 0.0
        self.re_exp = re.compile('([a-zA-Z0-9])([+|-][0-9]\.[0-9]+)([+|-][0-9]\.[0-9]+)([+|-][0-9]*)?') #type: ignore

    def call_concurrent_measurement(self):
        cmd = f'{self.id}C!\r\n'.encode('ascii')
        self.com_interface.write(cmd)
        time.sleep(0.84)
        self.com_interface.read_until(b'\n')

    def parse_response(self):
        '''
        Parse response from sensor containing id,lower band, upper_band, <orientation>.
        Calibrateed upper and lower band values are in Watts*m^-2
        '''
        cmd = f'{self.id}D0!\r\n'.encode('ascii')
        self.com_interface.write(cmd)
        # time.sleep()
        response = self.com_interface.read_until(b'\n').replace(b'\x00',b'')
        # print(response)
        m = self.re_exp.match(response.decode('ascii').strip())
        if m:
            #(sensor_id,sensor_position,lower_band, upper_band)
            id = m.groups()[0]
            self.lower_band = float(m.groups()[1])
            self.upper_band = float(m.groups()[2])
            assert self.id == id, f'Error: sensor id {self.id} does not match response id'
            return(True)
        else:
            print(f'Error: could not parse response: {response.decode("ascii")}')
            return(False)
    @property
    def last_update(self):
        return self._last_update
    
    @last_update.setter
    def last_update(self,value:float):
        self._last_update = value
        
## to do: a GPS receiver could be added to the Dualband_sensor class to log the location of the measurement
class Dualband_sensor_pair():
    def __init__(self, downlooking_sensor: Dualband_sensor, uplooking_sensor: Dualband_sensor,wait_for_update_s:int=20,GPS_receiver:reach_rover=None):
        self.downlooking_sensor = downlooking_sensor
        self.uplooking_sensor = uplooking_sensor
        self.succesful_measurement = False
        # self.start_time = start_time
        self.timestamp = 0.0
        self.wait_for_update = wait_for_update_s
        self.lower_band_reflectance = 0.0
        self.upper_band_reflectance = 0.0
        self.GPS_receiver = GPS_receiver
        self.coordinates_with_meta = None#{'coordinates':(0,0,0),'metadata':(0,0)}

    def update_reflectance_values(self):
        self.downlooking_sensor.call_concurrent_measurement()
        self.timestamp = time.time()
        if self.GPS_receiver:
            self.coordinates_with_meta = self.GPS_receiver.coordinates_with_meta
        else:
            datetime_iso = datetime.fromtimestamp(self.timestamp).isoformat(' ','milliseconds')
            self.coordinates_with_meta = {'latitude':0.0,'longitude':0.0,'altitude':0.0,'datetime_iso':datetime_iso,'quality_fix':0}
        downlooking_success = self.downlooking_sensor.parse_response()
        elapsed_time = self.timestamp - self.uplooking_sensor.last_update
        if ((elapsed_time) > self.wait_for_update) or (self.uplooking_sensor.lower_band is None):
            self.uplooking_sensor.call_concurrent_measurement()
            print(f"updating uplooking values: {elapsed_time} s")
            uplooking_success = self.uplooking_sensor.parse_response()
            self.uplooking_sensor.last_update = self.timestamp
            if not uplooking_success:
                print('Warning: uplooking values cannot be updated') 
        valid_data_is_available = downlooking_success and self.uplooking_sensor.lower_band
        if valid_data_is_available:
            try:
                self.lower_band_reflectance = self.downlooking_sensor.lower_band/self.uplooking_sensor.lower_band
                self.upper_band_reflectance = self.downlooking_sensor.upper_band/self.uplooking_sensor.upper_band
            except ZeroDivisionError:
                print('Invalid reflectance')
                self.lower_band_reflectance = 0.0
                self.upper_band_reflectance = 0.0
        return(valid_data_is_available)


class NDVI_pair(Dualband_sensor_pair):
    def get_NDVI(self):
        ''' compute ndvi from lower and upper band values '''
        if self.update_reflectance_values():
            p_650 = self.lower_band_reflectance# pRED
            p_810 = self.upper_band_reflectance # pNIR
            try:
                ndvi = (p_810 - p_650)/(p_810 + p_650)
            except ZeroDivisionError as e:
                print(e)
                ndvi = 0.0
        else:
            print('Warning: No succesful measurement')
            ndvi = 0.0
        return (ndvi)        

class PRI_pair(Dualband_sensor_pair):
    def get_PRI(self):
        ''' compute pri from lower and upper band values '''
        if self.update_reflectance_values():
            p_532 = self.lower_band_reflectance# pGREEN
            p_570 = self.upper_band_reflectance # pYELLOW
            try:
                pri = (p_532 - p_570)/(p_532 + p_570)
            except ZeroDivisionError as e:
                print(e)
                pri = 0.0
        else:
            print('Warning: No succesful measurement')
            pri = 0.0
        return (pri)
    
class MultisensorCart_SDI12():
    def __init__(self,serial_port:'serial.Serial',ndvi_units:list[dict],pri_units:list[dict],rover:reach_rover) -> None:
        self.uplooking_ndvi_sensor = Dualband_sensor(ndvi_units[0]['id'],ndvi_units[0]['position'],ndvi_units[0]['orientation'],serial_port)
        self.uplooking_pri_sensor = Dualband_sensor(pri_units[0]['id'],pri_units[0]['position'],pri_units[0]['orientation'],serial_port)
        self.ndvi_pair_array = [NDVI_pair(Dualband_sensor(unit['id'],unit['position'],unit['orientation'],serial_port),self.uplooking_ndvi_sensor,GPS_receiver=rover) for unit in ndvi_units[1:]]
        self.pri_pair_array = [PRI_pair(Dualband_sensor(unit['id'],unit['position'],unit['orientation'],serial_port),self.uplooking_pri_sensor,GPS_receiver=rover) for unit in pri_units[1:]]


    def get_values(self):
        ndvi_values = [ndvi_pair.get_NDVI() for ndvi_pair in self.ndvi_pair_array]
        pri_values = [pri_pair.get_PRI() for pri_pair in self.pri_pair_array]

        ndvi_values = [{'timestamp':ndvi_pair[0].timestamp,
                        'datetime_iso':ndvi_pair[0].coordinates_with_meta['metadata'][0],
                        'quality_fix':ndvi_pair[0].coordinates_with_meta['metadata'][1],
                        'lat':ndvi_pair[0].coordinates_with_meta['coordinates'][0],
                        'long':ndvi_pair[0].coordinates_with_meta['coordinates'][1],
                        'alt':ndvi_pair[0].coordinates_with_meta['coordinates'][2],
                        'position':ndvi_pair[0].downlooking_sensor.position.name.upper(),
                        'sensor_id':ndvi_pair[0].downlooking_sensor.id,
                        'type': 'NDVI',	
                        'index_value': ndvi_pair[1]}
                        for ndvi_pair in zip(self.ndvi_pair_array,ndvi_values)]
        pri_values = [{'timestamp':pri_pair[0].timestamp,
                        'datetime_iso':pri_pair[0].coordinates_with_meta['metadata'][0],
                        'quality_fix':pri_pair[0].coordinates_with_meta['metadata'][1],
                        'lat':pri_pair[0].coordinates_with_meta['coordinates'][0],
                        'long':pri_pair[0].coordinates_with_meta['coordinates'][1],
                        'alt':pri_pair[0].coordinates_with_meta['coordinates'][2],
                        'position':pri_pair[0].downlooking_sensor.position.name.upper(),
                        'sensor_id':pri_pair[0].downlooking_sensor.id,
                        'type': 'PRI',
                        'index_value': pri_pair[1]}
                        for pri_pair in zip(self.pri_pair_array,pri_values)]
        return(ndvi_values+pri_values)

def make_ndvi_pairs(uplooking_sensor:dict,downlooking_sensors:list[dict],serial_if:serial.Serial,gps:reach_rover=None) -> list[NDVI_pair]:
    uplooking = Dualband_sensor(uplooking_sensor['id'],uplooking_sensor['position'],uplooking_sensor['orientation'],serial_if)
    downlooking = [Dualband_sensor(sensor['id'],sensor['position'],sensor['orientation'],serial_if) for sensor in downlooking_sensors]
    ndvi_modules = [NDVI_pair(d,uplooking,GPS_receiver=gps) for d in downlooking]
    return ndvi_modules

def make_pri_pairs(uplooking_sensor:dict,downlooking_sensors:list[dict],serial_if:serial.Serial,gps:reach_rover=None) -> list[PRI_pair]:
    uplooking = Dualband_sensor(uplooking_sensor['id'],uplooking_sensor['position'],uplooking_sensor['orientation'],serial_if)
    downlooking = [Dualband_sensor(sensor['id'],sensor['position'],sensor['orientation'],serial_if) for sensor in downlooking_sensors]
    pri_modules = [PRI_pair(d,uplooking,GPS_receiver=gps) for d in downlooking]
    return pri_modules

@threaded
def log_ndvi_pri(txt_path:Path,ndvi_units:'list[NDVI_pair]',pri_units:list[PRI_pair],stop_event:Event):
    stop = stop_event
    with txt_path.open('w',encoding='utf-8') as f:
        header = "timestamp,datetime_iso,quality_fix,latitude,longitude,altitude,sensor_id,sensor_position,type,index_value\n"
        f.write(header)
        while not stop.is_set():
            ndvi_values = [ndvi_pair.get_NDVI() for ndvi_pair in ndvi_units]
            for sensor_pair,ndvi in zip(ndvi_units,ndvi_values):
                print(f"ID: {sensor_pair.downlooking_sensor.id}, {ndvi}")
                line = ','.join([f"{sensor_pair.timestamp:.6f},{sensor_pair.coordinates_with_meta['datetime_iso']},{sensor_pair.coordinates_with_meta['quality_fix']}",
                                    f"{sensor_pair.coordinates_with_meta['latitude']},{sensor_pair.coordinates_with_meta['longitude']},{sensor_pair.coordinates_with_meta['altitude']}",
                                    f"{sensor_pair.downlooking_sensor.id},{sensor_pair.downlooking_sensor.position.name.upper()},NDVI,{ndvi}"]) + '\n'
                f.write(line)
            pri_values = [pri_pair.get_PRI() for pri_pair in pri_units]
            for sensor_pair,pri in zip(pri_units,pri_values):
                print(f"ID: {sensor_pair.downlooking_sensor.id}, {pri}")
                line = ','.join([f"{sensor_pair.timestamp:.6f},{sensor_pair.coordinates_with_meta['datetime_iso']},{sensor_pair.coordinates_with_meta['quality_fix']}",
                                    f"{sensor_pair.coordinates_with_meta['latitude']},{sensor_pair.coordinates_with_meta['longitude']},{sensor_pair.coordinates_with_meta['altitude']}",
                                    f"{sensor_pair.downlooking_sensor.id},{sensor_pair.downlooking_sensor.position.name.upper()},PRI,{pri}"]) + '\n'
                f.write(line)
