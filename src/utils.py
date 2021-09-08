import os
from pathlib import Path
import cv2 as cv

class Utils:

    @staticmethod
    def root():
        return Path(__file__).parent.parent
    
    @staticmethod
    def absolute_path(rel_path:str) -> str:
        return os.path.join(Utils.root(), rel_path.replace('/', os.sep))

    @staticmethod
    def load_classes(path:str) -> list:
        labels = []
        with open(Utils.absolute_path(path), "r") as f:
            labels = [cname.strip() for cname in f.readlines()]
        return labels

    @staticmethod
    def is_cuda_cv() -> bool:
        try:
            count = cv.cuda.getCudaEnabledDeviceCount()
            if count > 0:
                return True
            else:
                return False
        except:
            return False
