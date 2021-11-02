from enum import Enum

class ObjType(Enum):
    Background = 0
    Rigid = 1
    Interactive = 2
    ConceptNode = 3


    def __str__(self):
        return self.name