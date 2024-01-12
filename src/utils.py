from enum import Enum,auto
from pathlib import Path
from threading import Thread
from datetime import datetime


class SensorPosition(Enum):
    CENTER = auto()
    LEFT = auto()
    RIGHT = auto()
    UPSIDE = auto()

class SensorOrientation(Enum):
    UPLOOKING = auto()
    DOWNLOOKING = auto()
    UNDEFINED = auto()

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn,daemon=False, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

def get_unique_filepath_from_path(folder:Path,file_extension:str='.txt') -> Path:
    """_summary_

    Args:
        folder (Path): _description_
        file_extension (str, optional): _description_. Defaults to '.txt'.

    Returns:
        Path: Unique file path for the specified folder
    """
    fill_zeros = 3 #1000 files per folder limitation
    files  = [f for f in list(folder.glob('*'+file_extension)) if f.name.startswith(folder.name)]
    if len(files) == 0:
        new_file_name = folder.name + '_' + str(1).zfill(fill_zeros) + file_extension
    else:
        #to do:create a new folder if there are more than 1000 files in the folder
        files = [f for f in files if f.stem[-fill_zeros:].isdigit()]
        numbers = [int(f.stem[-fill_zeros:]) for f in files]
        highest_number = max(numbers)
        new_file_name = folder.name + '_' + str(highest_number+1).zfill(fill_zeros) + file_extension
    return(folder / new_file_name)

def get_unique_filepath_from_string(root_path:Path,trial:str,folder_content:str,file_ext:str='.txt') -> Path:
    """Returns a unique file path in the asigned folder
    folder name follows: [YYYYMMdd] _ [trialname] _ [sensortype]

    Parameters
    ----------
    trial : str
        trial name
    file_extension : str, optional
        for temp,ndvi and pri data, by default '.txt'
        other file format like .h5 is supported for spectrometer data
    Returns
    -------
    Path
        unique file path with consecuvite number
    """    
    today_date = datetime.now().strftime("%Y%m%d")
    folder_name = today_date+'_'+trial+'_'+folder_content
    folder_path = root_path/folder_name
    folder_path.mkdir(parents=True,exist_ok=True)
    return (get_unique_filepath_from_path(folder_path,file_ext) )
