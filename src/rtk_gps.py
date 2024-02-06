import socket
from io import BytesIO
from threading import Event,Lock
from utils import threaded

class reach_rover():
    '''This class is used to connect to a rtk rover and get the coordinates in a thread safe way'''
    def __init__(self,ip:str,port:int,lock:Lock) -> None:
        self.ip = ip
        self.port = port
        self.lock = lock 
        self.loop_event_ctrl = Event()
        self.current_coordinates = {'coordinates':(None,None,None), #lat,long,alt
                                    'metadata':(None,None)} #timestamp, quality_fix
        self._coordinates = None

    def parse_stream(self,line):
        if len(line) > 0:
            line_elements = line.split()
            coordinates = tuple(line_elements[2:5])
            date = line_elements[0].replace('/','-')
            time = line_elements[1]
            date_time_iso = date + ' ' + time
            metadata = (date_time_iso,line_elements[5]) #timestamp, quality_fix
            output_dict = {'coordinates':coordinates,'metadata':metadata}
            self.coordinates = output_dict
            self.coordinates_with_meta = {'latitude':coordinates[0],
                                          'longitude':coordinates[1],
                                          'altitude':coordinates[2],
                                          'datetime_iso':metadata[0],
                                          'quality_fix':metadata[1]}

    @threaded
    def spin(self):
        '''This method is used to connect to the rtk rover and start a loop that will poll the socket byte per byte and parse the stream'''
        bytes_buffer = BytesIO()
        self.loop_event_ctrl.clear()
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
                    sock.connect((self.ip,self.port))
                    sock.settimeout(5)
                    while not self.loop_event_ctrl.is_set():
                        c = sock.recv(1)
                        if (c == b'\n' or c == b'\r'):
                            self.parse_stream(bytes_buffer.getvalue().strip().decode('ascii'))
                            bytes_buffer.close()
                            bytes_buffer = BytesIO()
                        else:
                            bytes_buffer.write(c)
        except ConnectionRefusedError:
            print('target machine refused connection')
            self.stop()
        except TimeoutError:
            print('Timeout')
            self.stop()
        print('Closing socket')
    @property  
    def coordinates(self): #{'coordinates':(lat,long,alt),'metadata':(iso timestamp,quality fix)}
        with self.lock:
            result = self.current_coordinates
        return(result)
    
    @coordinates.setter
    def coordinates(self,value):
        with self.lock:
            self.current_coordinates = value

    @property
    def coordinates_with_meta(self):
        with self.lock:
                return self._coordinates
        
    @coordinates_with_meta.setter
    def coordinates_with_meta(self,value):
        with self.lock:
            self._coordinates = value
    
    def stop(self):
        self.loop_event_ctrl.set()




