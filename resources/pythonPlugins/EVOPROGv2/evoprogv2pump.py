import time
from extractor import Extractor

class EvoprogV2Pump(Extractor):
	def __init__(self, params):
		"""constructor"""
		self.address = int(params["address"]);
		self.dir = int(params["direction"]);
		self.config = False;

	@classmethod
	def getParamsType(cls):
		"""must return a list with the types expected at the params variable in the init function"""
		return {"address":"int", "direction":"int"}

	def getInstructions(self):
		""" must return a string with the instructions to make this component"""
		return ""

	def getMovementType(self):
		""" must return the movement type of this extractor:
			or Extractor.continuous or Extractor.discrete or Extractor.irrelevant
		"""
		return Extractor.continuous

	def extractLiquid(self, rate, communications):
		"""must send instructions via communications object to extract liquid from the container
			communications object has the next api:
				*) nbytessend sendString(string) -- send the string to the machine, return the bytes sended;
				*) string receiveString() -- receive and returns a string from the machine (stops when \n is received), can block;
				*) string readUntil(endCharacter) -- returns a string received from the machine, stops when the endCharacter arrives;
				*) void synch() -- synchronize with the machine, not always necesary, only for protocols compatibles;
		"""
		if self.config == False:
			self.config = True;
			command = "oDir " +  str(self.address) + " " + str(self.dir) + "\r"
			communications.sendString(command)
			print command
			communications.synch()
		
		if rate > 0:
			command = "P " + str(self.address) + " " + str(rate) + "\r"
			communications.sendString(command)
			print command
			communications.synch()
		else:
			command = "pPump " + str(self.address) + "\r"
			communications.sendString(command)
			print command
			communications.synch()