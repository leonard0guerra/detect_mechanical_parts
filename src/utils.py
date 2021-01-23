import os
from pathlib import Path

class Utils:

    @staticmethod
    def root():
        return Path(__file__).parent.parent
    
    @staticmethod
    def absolutePath(rel_path:str) -> str:
        return os.path.join(Utils.root(), rel_path.replace('/', os.sep))