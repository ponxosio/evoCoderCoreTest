import time
from odsensor import OdSensor

class EvoprogOdSensor(OdSensor):

	def __init__(self, params):
		self.pinNumber = int(params["pinNumber"])

	def getInstructions(self):
		return "";

	def readOD(self, communications):
		command = "READS " + str(self.pinNumber)
		
		communications.sendString(command)
		time.sleep(0.05)
		dataRead = communications.readUntil('\n');
		
		value = ''.join(e for e in dataRead if e.isalnum())
		return float(value)

	@classmethod
	def getParamsType(cls):
		return {"pinNumber":"int"}