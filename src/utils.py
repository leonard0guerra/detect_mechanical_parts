import os
from pathlib import Path

class Utils:

    @staticmethod
    def root():
        return Path(__file__).parent.parent
    
    @staticmethod
    def absolutePath(rel_path:str) -> str:
        return os.path.join(Utils.root(), rel_path.replace('/', os.sep))

    @staticmethod
    def load_classes(path:str) -> list:
        labels = []
        with open(Utils.absolutePath(path), "r") as f:
            labels = [cname.strip() for cname in f.readlines()]
        return labels