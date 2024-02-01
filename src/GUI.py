import seabreeze
seabreeze.use('pyseabreeze')
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from seabreeze.spectrometers import Spectrometer
from pathlib import Path
from threading import Event,Lock,Thread
import threading
from rtk_gps import reach_rover
from enum import Enum
from time import sleep
import numpy as np
import serial
from os import environ
from serial.tools.list_ports import comports
from utils import get_unique_filepath_from_string,SensorOrientation,SensorPosition
from HDX_spec import HDXXR_spectrometer,HDX_reflectance_module,save_raw_spectra
from IRR_labjack import log_temperatures
from sdi12_sensors import make_ndvi_pairs,make_pri_pairs,log_ndvi_pri


class colors(Enum):
    RED = 0 
    GREEN = 1 #fix
    YELLOW = 2 #float
    LIGHTBLUE = 4 #DGPS
    ORANGE = 5 #Single
    LIGHTGREY = 6 #waiting

class MainApp(tk.Frame):
    def __init__(self,rtk_rover:reach_rover|None,irr_u:'list[dict]',ndvi_u,pri_u,hdx_up,hdx_down,root:tk.Tk=None): #type: ignore
        super().__init__(root)
        self.root = root
        self.gps = rtk_rover
        self.irr_units = irr_u
        self.ndvi_units = ndvi_u
        self.pri_units = pri_u
        self.hdx_uplooking = hdx_up
        self.hdx_downlooking = hdx_down
        self.hdx_modules = None
        self.irr_stop_event = Event()
        self.sdi12_stop_event = Event()
        self.spec_stop_event = Event()
        self.wd = Path()
        self.root.title('Phenocart Multisensor')
        ttk.Button(self,text='Seleccionar carpeta',command=self.select_directory).grid(row=0,column=0)
        self.calibrate_bttn = ttk.Button(self,text='Calibrar spec',command=self.calibrate_hdx_modules)
        self.calibrate_bttn.grid(row=4,column=0)
        self.start_temp_bttn = ttk.Button(self,text='Start temp',command=self.call_log_temperatures)
        self.start_temp_bttn.grid(row=4,column=1)
        self.start_sdi12_bttn = ttk.Button(self,text='Start NDVI/PRI',command=self.call_log_sdi12)
        self.start_sdi12_bttn.grid(row=5,column=0)
        self.start_sdi12_bttn['state'] = 'disabled'
        self.start_spec_bttn = ttk.Button(self,text='Start spec',command=self.call_log_spec)
        self.start_spec_bttn.grid(row=5,column=1)
        self.start_spec_bttn['state'] = 'disabled'
        self.gps_frame_container = ttk.LabelFrame(self,text='GPS')#,sticky=(tk.E,tk.W)) #type: ignore
        self.gps_frame_container.grid(row=3,column=1,sticky=(tk.W,tk.E) ) #type: ignore
        ttk.Label(self,text='Puerto COM').grid(row=2,column=0)
        self.com_port_str = tk.StringVar()
        self.COM_port_combo = ttk.Combobox(self,textvariable=self.com_port_str,values=[x.name for x in comports()],state='readonly')
        self.COM_port_combo.bind('<<ComboboxSelected>>',self.select_comport_callback)
        self.COM_port_combo.grid(row=2,column=1,sticky=tk.W)
        self.connect_gps_bttn = ttk.Button(self.gps_frame_container,text='Conectar GPS',command=self.gps_connect_callback)
        self.connect_gps_bttn.grid(row=0,column=0)
        ttk.Label(self.gps_frame_container,text='Fix Quality Status:').grid(row=0,column=1)
        self.status_color_label = ttk.Label(self.gps_frame_container,text='          ',background=colors(6).name)
        self.status_color_label.grid(row=0,column=2)
        self.gps_connected = False
        self.current_folder_text = tk.StringVar()
        self.name_suffix = tk.StringVar()
        self.wd_label = ttk.Label(self,textvariable=self.current_folder_text).grid(row=0,column=1)
        self.current_folder_text.set(str(self.wd.resolve()))
        ttk.Label(self,text = 'Nombre de ensayo').grid(row=1,column=0)
        ttk.Entry(self,textvariable = self.name_suffix).grid(row=1,column=1,sticky=(tk.W,tk.E)) #type: ignore
        self.temp_logging = False
        self.sdi12_logging = False
        self.spec_logging= False
        self.spec_modules_created = False
        self.spec_calibrated = False
        self.bind('<Destroy>',self.safe_exit) 

        for child in self.winfo_children():
            child.grid_configure(padx=5,pady=5)

    def select_directory(self):
        d = filedialog.askdirectory(initialdir=Path(environ['USERPROFILE']))
        if d is None:
            p = Path('.')
        else:
            p = Path(d)
        self.wd = p
        self.current_folder_text.set( str(self.wd.resolve()) )

    def select_comport_callback(self,event):
        #print(self.com_port_str.get())
        self.start_sdi12_bttn['state'] = 'normal'
        self.root.focus()


    def gps_connect_callback(self):
        if not self.gps_connected and not (self.gps is None):
            try:
                tf = self.gps.spin()
                f = Thread(target=self.seek_gps_status,daemon=True,args=(tf,) )
                f.start()
                self.connect_gps_bttn.config(text='Desconectar GPS')
                self.gps_connected = True
            except Exception as e:
                self.gps.stop()
                self.gps_connected = False
                self.connect_gps_bttn.config(text='Conectar GPS')
                self.status_color_label.config(background=colors(0).name)
        else:
            if self.gps is None:
                print("No gps to connect")
            else:
                self.gps.stop()
                self.connect_gps_bttn.config(text='Conectar GPS')
                self.status_color_label.config(background='grey')
                self.gps_connected = False
    
    def seek_gps_status(self,tf):
        while True:
            if tf.is_alive():
                c = self.gps.coordinates
                if c['metadata'][1] is None:
                    self.status_color_label.config( background=colors(0).name )
                else:
                    # print(c['coordinates'])
                    color = int( c['metadata'][1] )
                    self.status_color_label.config( background=colors(color).name )
            else:
                self.status_color_label.config( background=colors(6).name )
                self.connect_gps_bttn.config(text='Conectar GPS')
                break
            sleep(1)

    def call_log_temperatures(self):
        if not self.temp_logging:
            try:
                self.irr_stop_event.clear()
                trial_name = self.name_suffix.get()
                filepath = get_unique_filepath_from_string(self.wd,trial_name,'temp','.txt')
                log_temperatures(filepath,self.irr_units,self.irr_stop_event,self.gps)
                self.temp_logging = True
                self.start_temp_bttn.config(text="Stop temp")
                print("Logging temperature")
            except:
                self.irr_stop_event.set()
                print("Error logging temperature")
                self.temp_logging = False
                self.start_temp_bttn.config(text="Start temp")
        else:
            self.irr_stop_event.set()
            self.temp_logging = False
            print("Stop logging temperature")
            self.start_temp_bttn.config(text="Start temp")

    def call_log_sdi12(self):
        serial_port = serial.Serial()
        if not self.sdi12_logging:
            try:
                self.sdi12_stop_event.clear()
                # serial_port = serial.Serial('COM15',19200,timeout=5)
                serial_port.baudrate = 19200
                serial_port.port = self.com_port_str.get()
                serial_port.timeout = 5
                serial_port.open()
                ndvi_list = make_ndvi_pairs(self.ndvi_units[0],self.ndvi_units[1:],serial_port,self.gps)
                pri_list = make_pri_pairs(self.pri_units[0],self.pri_units[1:],serial_port,self.gps)
                trial_name = self.name_suffix.get()
                filepath = get_unique_filepath_from_string(self.wd,trial_name,'SDI12','.txt')
                log_ndvi_pri(filepath,ndvi_list,pri_list,self.sdi12_stop_event)
                self.sdi12_loggin = True
                self.start_sdi12_bttn.config(text="Stop SDI12")
            except:
                self.sdi12_stop_event.set()
                print("Error logging NDVI/PRI")
                self.sdi12_logging = False
                self.start_sdi12_bttn.config(text="Start SDI12")
                if serial_port.is_open:
                    serial_port.close()
                    
        else:
            self.sdi12_stop_event.set()
            self.sdi12_logging = False
            self.start_sdi12_bttn.config(text="Start SDI12")
            if serial_port.is_open:
                serial_port.close()

    def calibrate_hdx_modules(self):
        try:
            if not self.spec_modules_created:
                HDX_uplooking = HDXXR_spectrometer(Spectrometer.from_serial_number(self.hdx_uplooking['serial_number']),integration_time_ms = 25,boxcar_size=1,position=self.hdx_uplooking['position'],orientation=self.hdx_uplooking['orientation'])
                HDX_downlooking = [HDXXR_spectrometer(Spectrometer.from_serial_number(device['serial_number']),integration_time_ms = 6,boxcar_size=1,position=device['position'],orientation=device['orientation'])
                                    for device in self.hdx_downlooking]
                
                self.hdx_modules = [HDX_reflectance_module(HDX_uplooking,
                                                                    spec,
                                                                    position=device_info['position'])
                                                for spec,device_info in zip(HDX_downlooking,self.hdx_downlooking)]
                # self.calibrate_bttn.config(bg='green')
                self.spec_modules_created = True
            else:
                pass
            if self.spec_modules_created:
                white_panel_wavelengths,white_panel_reflectance = np.loadtxt(Path.cwd()/'white_panel_reflectance.csv',delimiter=',',skiprows=1,unpack=True)
                for m in self.hdx_modules:
                    m.set_calibration_panel_reflectance(white_panel_wavelengths,white_panel_reflectance)
                    m.inter_calibrate(optimize_downlooking=False)
                self.start_spec_bttn['state'] = 'normal'
                self.spec_calibrated = True
            else:
                pass
        except Exception as e:
            print(f"Error {e}")
    
    def call_log_spec(self):
        #to do: check that hdx modules is not None
        if not self.spec_logging and self.spec_calibrated:
            try:
                self.spec_stop_event.clear()
                trial_name = self.name_suffix.get()
                filepath = get_unique_filepath_from_string(self.wd,trial_name,'spec','.h5')
                save_raw_spectra(filepath,self.spec_stop_event,self.hdx_modules,self.gps)
                self.spec_logging = True
                self.start_spec_bttn.config(text="Stop spec")
                self.calibrate_bttn['state'] = 'disabled'
            except:
                self.spec_stop_event.set()
                self.spec_logging = False
                self.start_spec_bttn.config(text="Start spec")
                self.calibrate_bttn['state'] = 'normal'
                # self.calibrate_bttn.config(bg='light grey')
        else:
            self.spec_stop_event.set()
            self.spec_logging = False
            self.start_spec_bttn.config(text="Start spec")
            self.calibrate_bttn['state'] = 'normal'
            # self.calibrate_bttn.config(bg='light grey')
    
    def safe_exit(self,event):
        self.irr_stop_event.set()
        self.sdi12_stop_event.set()
        self.spec_stop_event.set()

        for thread in threading.enumerate():
            if thread != threading.current_thread():
                thread.join(1)
        print("Done")

                

if __name__ == '__main__':
    shared_lock = Lock()

    ## RTK-GPS configuration
    ## TCP socket streams data in LLH format
    ip4 = '192.168.42.1'
    port = 9001
    rtk_rover = reach_rover(ip4,port,shared_lock)

    ## Infrared radiometers configuration for temperature measurements
    ## Labjack U6 PRO + Apogee 1H1-series IRR
    # AIN13 thernistor unit 1142
    # AIN12 thernistor unit 1140
    # AIN11 thernistor unit 1141
    # gainIndex, the gain index.  0=x1, 1=x10, 2=x100, 3=x1000, 15 = autorange
    # resolution index: 1-8 high-speed, 9-12 high-res,default = 0
    IRR1 = {'unit':'1141','thermistor_ain':13,'thermopile_ain':10,'res_index':12,'gain_index':0,'position':SensorPosition.RIGHT,'cc':None} 
    IRR2 = {'unit':'1142','thermistor_ain':9,'thermopile_ain':6,'res_index':12,'gain_index':0,'position':SensorPosition.CENTER,'cc':None} 
    IRR3 = {'unit':'1140','thermistor_ain':5,'thermopile_ain':2,'res_index':8,'gain_index':1,'position':SensorPosition.LEFT,'cc':None}

    ## NDVI and PRI sensors configuration
    ## Tekbox + Apogee NDVI digital sensor + Meter PRI digital sensor
    ## SDI-12 protocol
    ndvi_units = [
    {'id':'1','position':SensorPosition.CENTER,'orientation':SensorOrientation.UPLOOKING},
    {'id':'2','position':SensorPosition.RIGHT,'orientation':SensorOrientation.DOWNLOOKING},
    {'id':'3','position':SensorPosition.CENTER,'orientation':SensorOrientation.DOWNLOOKING},
    {'id':'4','position':SensorPosition.LEFT,'orientation':SensorOrientation.DOWNLOOKING}
    ]

    pri_units = [
        {'id':'a','position':SensorPosition.CENTER,'orientation':SensorOrientation.UPLOOKING},
        {'id':'b','position':SensorPosition.RIGHT,'orientation':SensorOrientation.DOWNLOOKING},
        {'id':'c','position':SensorPosition.CENTER,'orientation':SensorOrientation.DOWNLOOKING},
        {'id':'d','position':SensorPosition.LEFT,'orientation':SensorOrientation.DOWNLOOKING}
    ]

    ## HDX-XR spectrometers configuration
    ## pySeabreeze Open source USB driver
    downlooking_devices = [
        {'position':SensorPosition.RIGHT,'orientation':SensorOrientation.DOWNLOOKING,'serial_number':'HDX01033'},
        {'position':SensorPosition.CENTER,'orientation':SensorOrientation.DOWNLOOKING,'serial_number':'HDX01034'},
        {'position':SensorPosition.LEFT,'orientation':SensorOrientation.DOWNLOOKING,'serial_number':'HDX01032'}
    ]
    uplooking_device = {'position':SensorPosition.CENTER,'orientation':SensorOrientation.UPLOOKING,'serial_number':'HDX01010'}
    try:
        app = tk.Tk()
        f = MainApp(rtk_rover,[IRR1,IRR2,IRR3],ndvi_units,pri_units,uplooking_device,downlooking_devices,app).grid()
        app.mainloop()
    except Exception as e:
        print("Bad initialization")
        raise(e)
    finally:
        if not(rtk_rover is None):
            rtk_rover.stop()
