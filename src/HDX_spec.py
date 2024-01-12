import numpy as np
import seabreeze
seabreeze.use('pyseabreeze')
from math import trunc
from pathlib import Path
from datetime import datetime
import time
import itertools
from rtk_gps import reach_rover
from threading import Thread,Event
import tables
from seabreeze.spectrometers import Spectrometer
from utils import SensorOrientation,SensorPosition,get_unique_filepath_from_string

pixel_number = 2068

class SpectrometerTable(tables.IsDescription):
    index = tables.Int32Col(pos=0)
    integration_time_ms = tables.Float32Col(pos=1)
    timestamp = tables.Float64Col(pos=2)
    datetimeiso = tables.StringCol(itemsize=64,dflt=" ",pos=3)
    latitude = tables.Float64Col(pos=4)
    longitude = tables.Float64Col(pos=5)
    altitude = tables.Float64Col(pos=6)
    quality_fix = tables.Int32Col(pos=7)
    spectrum = tables.Float32Col(shape=(pixel_number,),pos=8)

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn,daemon=False, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class HDXXR_spectrometer():
    def __init__(self,spec:Spectrometer,integration_time_ms:int=100,scans_to_average:int=3,boxcar_size:int=0,orientation:SensorOrientation=SensorOrientation.UNDEFINED,position:SensorPosition=SensorPosition.UPSIDE) -> None:
        self.spec = spec
        self.max_integration_time_ms = 10000
        self.min_integration_time_ms = 6
        # self.default_integration_time_ms = integration_time_ms #seabreeze API work with microseconds
        self._optimal_integration_time_us = integration_time_ms*1000 #this line might be redundant
        self.integration_time_ms = integration_time_ms
        self.optimized = False
        self.scans_to_avg = scans_to_average #increases signal to noise ratio
        self.boxcar_size = boxcar_size
        self.orientation = orientation
        self.position = position

    def optimize(self,start_integration_time_ms=None,saturation_counts=52000,max_iterations = 50,max_err = 0.005):        
        '''
        This method will find the optimal integration time for the spectrometer
          saturation_counts: number of counts that saturates the sensor at 85 % of the dynamic range
        '''
        if start_integration_time_ms is None:
            start_integration_time_ms = self.integration_time_ms

        max_count = self.get_max_count(start_integration_time_ms*1000)
        if max_count > saturation_counts: # start integration time already saturates the sensor so optimal value should be on the left side
            a = self.min_integration_time_ms
            b = start_integration_time_ms
        else: #let's try to find the interval where optimal value is located, big O(log(n))
            a = start_integration_time_ms
            b = 2*a
            while True:
                max_count = self.get_max_count(b*1000)
                if max_count >= saturation_counts:
                    print(f'{b} gets {self.get_max_count(b*1000)} counts')
                    break
                else:
                    a = b
                    b = 2*a
                    if b > self.max_integration_time_ms:
                        b = self.max_integration_time_ms
                        print("Warning: max integration time reached")
                        break
        print(f'start bisect with: {a} , {b}')
        optimal_integration_time_ms = self.bisect(a,b,saturation_counts,max_iterations,max_err)
        # self._optimal_integration_time_us = trunc(optimal_integration_time_ms)*1000
        self.integration_time_ms = trunc(optimal_integration_time_ms)
        self.optimized = True
        # self.spec.integration_time_micros(self.integration_time_ms*1000)
        return

    def bisect(self,a,b,f_root,max_iterations, err):
        iteration = itertools.count(1)
        while(True):
            m = (a+b)/2 #m is in milliseconds
            max_count_m = self.get_max_count(trunc(m)*1000)
            error_m = abs((f_root - max_count_m)/f_root)
            print(f"m: {m} ms, {max_count_m}:counts, error:{error_m} ")
            if error_m < err:
                print(f"optimal integration time:{m}, {max_count_m}:counts, error:{error_m} ")
                break
            else:
                if max_count_m > f_root: #root is on the left side
                    a = a
                    b = m
                else: #root is on the right side
                    a = m
                    b = b
            if next(iteration) >= max_iterations:
                print(f"Maximun iterations reached with {max_count_m} at {m} ms ")
                break
        return(m)

    @property
    def spectra(self):
        if self.optimized == False:
            print("Warning: Spectrometer not optimized")
        raw_spectras = np.array([self.spec.intensities(correct_nonlinearity=True) for i in range(self.scans_to_avg)])
        spectra_avg = np.mean(raw_spectras,axis=0)
        spectra_boxcar = np.convolve(spectra_avg, np.ones(self.boxcar_size)/self.boxcar_size, mode='valid')
        return spectra_boxcar
    
    @property
    def wavelengths(self):
        raw_wavelengths = self.spec.wavelengths()
        wavelengths_box = np.convolve(raw_wavelengths, np.ones(self.boxcar_size)/self.boxcar_size, mode='valid')
        return wavelengths_box
    
    @property
    def integration_time_ms(self):
        return self._optimal_integration_time_us/1000
    
    @integration_time_ms.setter
    def integration_time_ms(self,integration_time_ms):
        self._optimal_integration_time_us = integration_time_ms*1000
        self.spec.integration_time_micros(self._optimal_integration_time_us)
    
    def get_max_count(self,integration_time_us:int):
        self.spec.integration_time_micros(integration_time_us)
        max_counts = [self.spec.intensities(correct_nonlinearity=False).max() for i in range(3)] #to do: check how to overcome sensor stabilization
        # spectra = self.spec.intensities(correct_nonlinearity=False)
        max_count = np.array(max_counts)[-1]
        print(max_counts)
        print(f'{integration_time_us/1000} ms gets {max_count} counts')
        return max_count
    
class HDXXR_pair():
    def __init__(self,spec1:HDXXR_spectrometer,spec2:HDXXR_spectrometer) -> None:
        self.uplooking_spec = spec1
        self.downlooking_spec = spec2

        self._band_centers = None

        self.uplooking_white_ref = None
        self.uplooking_dark_ref = None
        self.downlooking_white_ref = None
        self.downlooking_dark_ref = None

        self.correction_factors = None
        self.calibration_panel_reflectance = None

    def set_calibration_panel_reflectance(self,white_cal_wavelengths:np.ndarray,white_cal_reflectance:np.ndarray):
        #to do: this could be changed to properties
        self.white_cal_wavelengths = white_cal_wavelengths
        self.white_cal_reflectance = white_cal_reflectance
        if self.band_centers is None:
            print('Warning: band centers not defined, using downlooking spectrometer wavelengths instead')
            self.calibration_panel_reflectance = np.interp(self.downlooking_spec.wavelengths,white_cal_wavelengths,white_cal_reflectance)
        else:
            self.calibration_panel_reflectance = np.interp(self.band_centers,white_cal_wavelengths,white_cal_reflectance)

    def inter_calibrate(self,optimize_downlooking:bool=True):
        ''' 
        This function gets the white reference and dark reference for both spectrometers
        uplooking spectrometer has a cosine corrector
        downlooking spectrometer uses a white reflectance standard

        optimize: if True, it will optimize the integration time for both spectrometers
        '''
        input("Place the uplooking spectrometer under direct sunlight and press enter")
        self.uplooking_white_ref = self.uplooking_spec.spectra
        input(f"Place the {self.downlooking_spec.position.name} downlooking spectrometer on the white reflectance standard and press enter")
        if optimize_downlooking:
            self.downlooking_spec.optimize()
        self.downlooking_white_ref = self.downlooking_spec.spectra
        input("Please cover the uplooking spectrometer and press enter")
        self.uplooking_dark_ref = self.uplooking_spec.spectra
        input(f"Please cover the {self.downlooking_spec.position.name} downlooking spectrometer and press enter")
        self.downlooking_dark_ref = self.downlooking_spec.spectra

        cal_incident_irradiance = self.uplooking_white_ref - self.uplooking_dark_ref
        cal_upwelling_radiance = self.downlooking_white_ref - self.downlooking_dark_ref

        if self.band_centers is None:
            print('Warning: band centers not defined, using downlooking spectrometer wavelengths instead')
            cal_incident_irradiance = np.interp(self.downlooking_spec.wavelengths,self.uplooking_spec.wavelengths,cal_incident_irradiance)
        else:
            cal_incident_irradiance = np.interp(self.band_centers,self.uplooking_spec.wavelengths,cal_incident_irradiance)
            cal_upwelling_radiance = np.interp(self.band_centers,self.downlooking_spec.wavelengths,cal_upwelling_radiance)

        self.correction_factors = cal_incident_irradiance/cal_upwelling_radiance
        print("Calibration complete")
    
    #just for visualization purposes
    @property
    def reflectance_spectra(self):
        if self.correction_factors is None:
            print("Please run inter_calibrate first")
            return
        
        upwelling_radiance = self.downlooking_spec.spectra-self.downlooking_dark_ref
        incident_irradiance = self.uplooking_spec.spectra-self.uplooking_dark_ref

        if self.band_centers is None:
            print('Warning: band centers not defined, using downlooking spectrometer wavelengths instead')
            incident_irradiance = np.interp(self.downlooking_spec.wavelengths,self.uplooking_spec.wavelengths,incident_irradiance)
        else:
            upwelling_radiance = np.interp(self.band_centers,self.downlooking_spec.wavelengths,upwelling_radiance)
            incident_irradiance = np.interp(self.band_centers,self.uplooking_spec.wavelengths,incident_irradiance)

        reflectance_spectra = (upwelling_radiance/incident_irradiance)*(self.correction_factors)*100#*self.calibration_panel_reflectance*100
        print("Reflectance spectra calculated")
        return(reflectance_spectra)
    
    @property
    def band_centers(self):
        if self._band_centers is None:
            return self.downlooking_spec.wavelengths
        else:
            return self._band_centers
    
    @band_centers.setter
    def band_centers(self,band_centers:np.ndarray):
        self._band_centers = band_centers
    

class HDX_reflectance_module(HDXXR_pair):
    def __init__(self, spec1: HDXXR_spectrometer, spec2: HDXXR_spectrometer,rtk_rover:reach_rover=None,position:SensorPosition=None) -> None:
        super().__init__(spec1, spec2)
        self.GPS = rtk_rover
        self.Position = position
        # self.Orientation = orientation

@threaded
def save_raw_spectra(file:Path,stop_event:Event=None,reflectance_modules:list[HDX_reflectance_module]=None,gps:reach_rover=None):
    stop = stop_event 
    with tables.open_file(file,'w') as f:
        reference_panel_group = f.create_group(f.root,'reference_panel','Standard reflectance panel data')
        f.create_array(reference_panel_group,'wavelengths',reflectance_modules[0].white_cal_wavelengths)
        f.create_array(reference_panel_group,'reflectance',reflectance_modules[0].white_cal_reflectance)
        calibration_group = f.create_group(f.root,'calibration','Inter calibration data')
        # reflectance_modules_groups = [f.create_group(calibration_group,module.Position.name.upper(),f'{calibration_group,module.Position.name.upper()} spectrometer calibration data')
        #                               for module in reflectance_modules]
        for module in reflectance_modules:
            module_position = module.Position.name.upper()
            module_group = f.create_group(calibration_group,module_position,f'{module_position} spectrometer calibration data')
            f.create_array(module_group,'integration_times',[module.uplooking_spec.integration_time_ms,module.downlooking_spec.integration_time_ms],'uplooking and downlooking integration times')
            f.create_array(module_group,'uplooking_spec_wavelengths',module.uplooking_spec.wavelengths)
            f.create_array(module_group,'downlooking_spec_wavelengths',module.downlooking_spec.wavelengths)
            f.create_array(module_group,'incident_irrandiance',module.uplooking_white_ref)
            f.create_array(module_group,'calibration_panel_radiance',module.downlooking_white_ref)
            f.create_array(module_group,'uplooking_dark_reference',module.uplooking_dark_ref)
            f.create_array(module_group,'downlooking_dark_reference',module.downlooking_dark_ref)
        #start logging raw data
        uplooking_spec = reflectance_modules[0].uplooking_spec
        downlooking_spec = [x.downlooking_spec for x in reflectance_modules]
        spectrometers_group = f.create_group(f.root,'spectrometers','Raw spectrometer data')
        uplooking_spec_group = f.create_group(spectrometers_group,f'{uplooking_spec.orientation.name.upper()}','Uplooking spectrometer data')
        downlooking_spec_group = [f.create_group(spectrometers_group,f'{module.Position.name.upper()}',
                                                    f'{module.Position.name.upper()} spectrometer data') 
                                                    for module in reflectance_modules]
        spectrometers_list = [uplooking_spec]+downlooking_spec
        spectrometers_group_list = [uplooking_spec_group]+downlooking_spec_group
        for spec,spec_group in zip(spectrometers_list,spectrometers_group_list):
            f.create_array(spec_group,'wavelengths',spec.wavelengths)

        raw_data_tables = [f.create_table(group,'raw',SpectrometerTable,f'{spec.orientation.name.upper()}_{spec.position.name.upper()}_raw_data')
                            for spec,group in zip(spectrometers_list,spectrometers_group_list)]
        rows = [table.row for table in raw_data_tables]
        index = itertools.count()
        while not stop.is_set():
            for row,spec in zip(rows,spectrometers_list):
                timestamp = time.time()
                if gps:
                    coordinates_with_meta = gps.coordinates_with_meta
                else:
                    datetime_iso = datetime.fromtimestamp(timestamp).isoformat(' ','milliseconds')
                    coordinates_with_meta = {'latitude':0.0,'longitude':0.0,'altitude':0.0,'datetime_iso':datetime_iso,'quality_fix':0}
                spectra = spec.spectra
                row['index'] = next(index)
                row['integration_time_ms'] = spec.integration_time_ms
                row['timestamp'] = timestamp
                row['datetimeiso'] = coordinates_with_meta['datetime_iso']
                row['latitude'] = coordinates_with_meta['latitude']
                row['longitude'] = coordinates_with_meta['longitude']
                row['altitude'] = coordinates_with_meta['altitude']
                row['quality_fix'] = coordinates_with_meta['quality_fix']
                row['spectrum'] = spectra
                row.append()
                time.sleep(0.2)
        for table in raw_data_tables:
            table.flush()
        print('Logging stopped')

    

