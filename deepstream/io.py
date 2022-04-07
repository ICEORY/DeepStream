
__all__ = ["DataCache"]

class DataCache(object):
    def __init__(self, data=None):
        self.data = data
        self.update_state = False

    def writeData(self, data):
        self.update_state = True
        self.data = data

    def readData(self):
        self.update_state = False
        return self.data

    def readDataOnly(self):
        #self.update_state = False
        return self.data

    def needRefresh(self):
        self.update_state = True
